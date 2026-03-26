// lidar_localization/src/obstacle_tracker_node.cpp
//
// Persistent obstacle tracker: detects static obstacles via lidar clustering,
// adds/removes them from the OpenMower map via the xbot_rpc map.replace call.
//
// Map service architecture:
//   - Obstacles are stored as separate areas with type="obstacle" in map.json
//   - GetMowingAreaSrv returns all obstacle areas in response.area.obstacles
//   - To add/remove obstacles we read map.json, modify it, and publish via RPC
//   - The map.replace RPC triggers saveMapToFile() + buildMap() in mower_map_service

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/AddMowingAreaSrv.h"

#include "xbot_rpc/RpcRequest.h"

#include "cluster_tracker.h"

#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <vector>

class ObstacleTrackerNode {
public:
    // A polygon representing a mowing or navigation area
    struct AreaPolygon {
        std::string id;
        std::string type;  // "mow" or "nav"
        std::vector<std::pair<double, double>> vertices;
    };

    ObstacleTrackerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : tf_buffer_(), tf_listener_(tf_buffer_)
    {
        // Parameters
        lidar_localization::ClusterTracker::Config cfg;
        pnh.param("cluster_merge_distance", cfg.cluster_merge_distance, 0.3);
        pnh.param("min_radius", cfg.min_radius, 0.15);
        pnh.param("max_radius", cfg.max_radius, 2.0);
        pnh.param("confirm_duration", cfg.confirm_duration, 60.0);
        pnh.param("remove_duration", cfg.remove_duration, 600.0);
        pnh.param("observation_ratio", cfg.observation_ratio, 0.5);
        pnh.param("max_observation_range", cfg.max_observation_range, 4.0);
        pnh.param("max_scan_range", max_scan_range_, 6.0);
        pnh.param("safety_margin", safety_margin_, 0.15);
        const char* home_env = getenv("HOME");
        std::string home_dir = (home_env && home_env[0] != '\0') ? home_env : "/root";
        pnh.param<std::string>("map_file", map_file_, home_dir + "/.ros/map.json");

        tracker_ = std::make_unique<lidar_localization::ClusterTracker>(cfg);

        // Subscribers
        scan_sub_ = nh.subscribe("scan", 1, &ObstacleTrackerNode::onScan, this);

        // Publishers
        status_pub_ = pnh.advertise<diagnostic_msgs::DiagnosticStatus>("status", 1);
        marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("markers", 1);
        shape_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("shapes", 1);
        rpc_pub_ = nh.advertise<xbot_rpc::RpcRequest>("/xbot/rpc/request", 1);

        // Timer for map service calls (1Hz)
        map_timer_ = nh.createTimer(ros::Duration(1.0), &ObstacleTrackerNode::onMapTimer, this);
        status_timer_ = nh.createTimer(ros::Duration(1.0), &ObstacleTrackerNode::onStatusTimer, this);
        // Refresh area polygons every 30s
        area_refresh_timer_ = nh.createTimer(ros::Duration(30.0), &ObstacleTrackerNode::onAreaRefreshTimer, this);

        ROS_INFO("obstacle_tracker node started (map_file=%s)", map_file_.c_str());
    }

    void loadExistingObstacles() {
        // Read existing obstacles from the map service (GetMowingArea returns all obstacles)
        std::vector<lidar_localization::ClusterTracker::Cluster> existing;

        for (uint32_t i = 0; ; ++i) {
            mower_map::GetMowingAreaSrv srv;
            srv.request.index = i;
            if (!ros::service::call("mower_map_service/get_mowing_area", srv)) {
                break;
            }

            for (const auto& obs_poly : srv.response.area.obstacles) {
                if (obs_poly.points.empty()) continue;

                // Compute centroid and radius from polygon vertices
                double cx = 0, cy = 0;
                for (const auto& pt : obs_poly.points) {
                    cx += pt.x;
                    cy += pt.y;
                }
                cx /= obs_poly.points.size();
                cy /= obs_poly.points.size();

                double max_r = 0;
                for (const auto& pt : obs_poly.points) {
                    double r = std::hypot(pt.x - cx, pt.y - cy);
                    if (r > max_r) max_r = r;
                }

                lidar_localization::ClusterTracker::Cluster c;
                c.cx = cx;
                c.cy = cy;
                c.radius = std::max(0.0, max_r - safety_margin_);
                c.confirmed = true;
                c.obstacle_id = "existing_" + std::to_string(existing.size());
                existing.push_back(c);

                ROS_INFO("Loaded existing obstacle at (%.2f, %.2f) r=%.2f", cx, cy, c.radius);
            }
            // All obstacles are returned with every mowing area, so only need index 0
            break;
        }

        if (!existing.empty()) {
            tracker_->loadExistingObstacles(existing);
            ROS_INFO("Loaded %zu existing obstacles from map", existing.size());
        }
    }

    // Load mowing and navigation area polygons from map.json.
    // GetMowingAreaSrv only returns "mow" type areas, so we read map.json directly
    // to also capture "nav" areas.
    void loadAreaPolygons() {
        std::string json = readMapFile();
        if (json.empty()) return;

        std::vector<AreaPolygon> new_areas;

        // Parse areas from JSON — look for each area entry with type "mow" or "nav"
        size_t search_pos = 0;
        auto areas_pos = json.find("\"areas\"");
        if (areas_pos == std::string::npos) return;

        // Find each area object by looking for "type" field
        while (true) {
            auto type_pos = json.find("\"type\":", search_pos);
            if (type_pos == std::string::npos) break;

            // Extract the type value
            auto quote1 = json.find('"', type_pos + 7);
            if (quote1 == std::string::npos) break;
            auto quote2 = json.find('"', quote1 + 1);
            if (quote2 == std::string::npos) break;
            std::string type_val = json.substr(quote1 + 1, quote2 - quote1 - 1);

            search_pos = quote2 + 1;

            // Only care about mow and nav areas
            if (type_val != "mow" && type_val != "nav") continue;

            // "type" is inside "properties" which is inside the area object.
            // Walk backward TWO '{' levels: first finds properties '{', second finds area '{'
            int levels_found = 0;
            int depth = 0;
            size_t obj_start = type_pos;
            while (obj_start > 0 && levels_found < 2) {
                obj_start--;
                if (json[obj_start] == '}') depth++;
                else if (json[obj_start] == '{') {
                    if (depth == 0) levels_found++;
                    else depth--;
                }
            }

            // Walk forward to find matching '}'
            depth = 1;
            size_t obj_end = obj_start + 1;
            while (obj_end < json.size() && depth > 0) {
                if (json[obj_end] == '{') depth++;
                else if (json[obj_end] == '}') depth--;
                obj_end++;
            }

            std::string obj_str = json.substr(obj_start, obj_end - obj_start);

            // Extract outline vertices
            AreaPolygon area;
            area.type = type_val;

            // Extract id
            auto id_pos = obj_str.find("\"id\":");
            if (id_pos != std::string::npos) {
                auto iq1 = obj_str.find('"', id_pos + 5);
                auto iq2 = obj_str.find('"', iq1 + 1);
                if (iq1 != std::string::npos && iq2 != std::string::npos) {
                    area.id = obj_str.substr(iq1 + 1, iq2 - iq1 - 1);
                }
            }

            // Extract outline points
            auto outline_pos = obj_str.find("\"outline\"");
            if (outline_pos != std::string::npos) {
                size_t pt_search = outline_pos;
                while (true) {
                    auto x_pos = obj_str.find("\"x\":", pt_search);
                    if (x_pos == std::string::npos) break;
                    auto y_pos = obj_str.find("\"y\":", x_pos);
                    if (y_pos == std::string::npos) break;

                    double x = std::stod(obj_str.substr(x_pos + 4));
                    double y = std::stod(obj_str.substr(y_pos + 4));
                    area.vertices.push_back({x, y});
                    pt_search = y_pos + 4;
                }
            }

            if (area.vertices.size() >= 3) {
                new_areas.push_back(std::move(area));
            }
        }

        {
            std::lock_guard<std::mutex> lock(areas_mutex_);
            area_polygons_ = std::move(new_areas);
        }
        ROS_INFO("Loaded %zu mowing/navigation area polygons for containment filtering",
                 area_polygons_.size());
    }

    // Ray-casting point-in-polygon test
    static bool pointInPolygon(double px, double py,
                               const std::vector<std::pair<double, double>>& polygon) {
        bool inside = false;
        size_t n = polygon.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i].first, yi = polygon[i].second;
            double xj = polygon[j].first, yj = polygon[j].second;
            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    // Check if a point is inside any mowing or navigation area
    bool isInsideAnyArea(double x, double y) {
        std::lock_guard<std::mutex> lock(areas_mutex_);
        for (const auto& area : area_polygons_) {
            if (pointInPolygon(x, y, area.vertices)) {
                return true;
            }
        }
        return false;
    }

private:
    void onScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
        geometry_msgs::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("map", msg->header.frame_id,
                                             msg->header.stamp, ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            return;
        }

        std::vector<std::pair<double,double>> points;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r <= 0 || !std::isfinite(r) || r > max_scan_range_) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            geometry_msgs::PointStamped pt_laser, pt_map;
            pt_laser.header = msg->header;
            pt_laser.point.x = r * std::cos(angle);
            pt_laser.point.y = r * std::sin(angle);
            pt_laser.point.z = 0;

            try {
                tf2::doTransform(pt_laser, pt_map, tf);
                points.push_back({pt_map.point.x, pt_map.point.y});
            } catch (...) {
                continue;
            }
        }

        // Get mower position from TF (base_link in map frame)
        double mower_x = tf.transform.translation.x;
        double mower_y = tf.transform.translation.y;
        try {
            auto base_tf = tf_buffer_.lookupTransform("map", "base_link",
                                                       msg->header.stamp, ros::Duration(0.05));
            mower_x = base_tf.transform.translation.x;
            mower_y = base_tf.transform.translation.y;
        } catch (...) {}

        // Per-scan DBSCAN clustering with tight epsilon for real obstacle shapes
        publishScanShapes(points, msg->header.stamp);

        double now = msg->header.stamp.toSec();
        auto result = tracker_->update(points, mower_x, mower_y, now);

        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (const auto& c : result.newly_confirmed) {
            if (isInsideAnyArea(c.cx, c.cy)) {
                pending_additions_.push_back(c);
                ROS_INFO("Obstacle '%s' at (%.2f, %.2f) confirmed inside area — will persist",
                         c.obstacle_id.c_str(), c.cx, c.cy);
            } else {
                ROS_INFO("Obstacle '%s' at (%.2f, %.2f) outside all areas — discarding",
                         c.obstacle_id.c_str(), c.cx, c.cy);
                filtered_count_++;
            }
        }
        for (const auto& c : result.newly_removed) {
            pending_removals_.push_back(c);
        }
    }

    void onMapTimer(const ros::TimerEvent&) {
        std::vector<lidar_localization::ClusterTracker::Cluster> additions, removals;
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            additions.swap(pending_additions_);
            removals.swap(pending_removals_);
        }

        if (additions.empty() && removals.empty()) return;

        // Read current map JSON
        std::string map_json = readMapFile();
        if (map_json.empty()) {
            // Re-queue for retry
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_additions_.insert(pending_additions_.end(), additions.begin(), additions.end());
            pending_removals_.insert(pending_removals_.end(), removals.begin(), removals.end());
            return;
        }

        bool modified = false;

        // Process additions: insert obstacle area entries into map JSON
        for (const auto& c : additions) {
            std::string obstacle_json = makeObstacleAreaJson(c);
            if (insertObstacleIntoJson(map_json, obstacle_json)) {
                ROS_INFO("Added persistent obstacle '%s' at (%.2f, %.2f) r=%.2f",
                         c.obstacle_id.c_str(), c.cx, c.cy, c.radius);
                confirmed_count_++;
                modified = true;
            }
        }

        // Process removals: find and remove matching obstacle entries
        for (const auto& c : removals) {
            if (removeObstacleFromJson(map_json, c.cx, c.cy)) {
                ROS_INFO("Removed persistent obstacle at (%.2f, %.2f)", c.cx, c.cy);
                removed_count_++;
                modified = true;
            }
        }

        if (modified) {
            publishMapReplace(map_json);
        }
    }

    // Read map.json file as a string
    std::string readMapFile() {
        std::ifstream file(map_file_);
        if (!file.is_open()) {
            ROS_WARN("Could not open map file: %s", map_file_.c_str());
            return "";
        }
        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    }

    // Generate a JSON object for an obstacle area entry.
    // Uses the real convex hull shape detected by lidar (inflated by safety_margin)
    // instead of approximating the obstacle as a circle.
    std::string makeObstacleAreaJson(const lidar_localization::ClusterTracker::Cluster& c) {
        std::ostringstream oss;
        oss << std::fixed;
        oss << "{\"id\":\"" << c.obstacle_id << "\",";
        oss << "\"properties\":{\"name\":\"" << c.obstacle_id << "\",\"type\":\"obstacle\"},";
        oss << "\"outline\":[";

        if (c.hull.size() >= 3) {
            // Use real convex hull shape, inflated outward by safety_margin.
            // Inflate each vertex outward from centroid.
            for (size_t i = 0; i < c.hull.size(); ++i) {
                if (i > 0) oss << ",";
                double dx = c.hull[i].first - c.cx;
                double dy = c.hull[i].second - c.cy;
                double dist = std::hypot(dx, dy);
                double inflate = (dist > 1e-6) ? (safety_margin_ / dist) : 0.0;
                double px = c.hull[i].first + dx * inflate;
                double py = c.hull[i].second + dy * inflate;
                oss << std::setprecision(6);
                oss << "{\"x\":" << px << ",\"y\":" << py << "}";
            }
        } else {
            // Fallback: too few hull points — use an 8-point circle
            double r = c.radius + safety_margin_;
            for (int i = 0; i < 8; ++i) {
                double angle = i * M_PI / 4.0;
                if (i > 0) oss << ",";
                oss << std::setprecision(6);
                oss << "{\"x\":" << (c.cx + r * std::cos(angle))
                    << ",\"y\":" << (c.cy + r * std::sin(angle)) << "}";
            }
        }

        oss << "]}";
        return oss.str();
    }

    // Insert an obstacle JSON entry into the areas array in the map JSON string.
    // Finds the last ']' that closes the "areas" array and inserts before it.
    bool insertObstacleIntoJson(std::string& json, const std::string& obstacle) {
        // Find "areas" array — look for the pattern "areas": [...]
        auto areas_pos = json.find("\"areas\"");
        if (areas_pos == std::string::npos) {
            ROS_WARN("Could not find 'areas' in map JSON");
            return false;
        }

        // Find the opening bracket of the array
        auto bracket_start = json.find('[', areas_pos);
        if (bracket_start == std::string::npos) return false;

        // Find the matching closing bracket (handle nested brackets)
        int depth = 1;
        size_t pos = bracket_start + 1;
        while (pos < json.size() && depth > 0) {
            if (json[pos] == '[') depth++;
            else if (json[pos] == ']') depth--;
            if (depth > 0) pos++;
        }

        if (depth != 0) {
            ROS_WARN("Malformed JSON: could not find closing bracket for areas array");
            return false;
        }

        // pos now points to the closing ']' of the areas array
        // Insert ",{obstacle}" before it (or just "{obstacle}" if array is empty)
        // Check if the array has content by looking for a non-whitespace char after bracket_start
        bool array_empty = true;
        for (size_t i = bracket_start + 1; i < pos; ++i) {
            if (json[i] != ' ' && json[i] != '\n' && json[i] != '\r' && json[i] != '\t') {
                array_empty = false;
                break;
            }
        }

        std::string insertion = array_empty ? obstacle : ("," + obstacle);
        json.insert(pos, insertion);
        return true;
    }

    // Remove an obstacle from JSON by matching centroid proximity.
    // Finds obstacle areas by looking for "type":"obstacle" entries and checking centroid.
    bool removeObstacleFromJson(std::string& json, double target_cx, double target_cy) {
        // Find all obstacle entries and check their centroid
        size_t search_pos = 0;
        while (true) {
            auto type_pos = json.find("\"type\":\"obstacle\"", search_pos);
            if (type_pos == std::string::npos) break;

            // Find the enclosing object for this entry
            // Walk backward to find the '{' that starts this area object
            int depth = 0;
            size_t obj_start = type_pos;
            while (obj_start > 0) {
                obj_start--;
                if (json[obj_start] == '}') depth++;
                else if (json[obj_start] == '{') {
                    if (depth == 0) break;
                    depth--;
                }
            }

            // Walk forward to find the matching '}'
            depth = 1;
            size_t obj_end = obj_start + 1;
            while (obj_end < json.size() && depth > 0) {
                if (json[obj_end] == '{') depth++;
                else if (json[obj_end] == '}') depth--;
                obj_end++;
            }

            // Extract this obstacle's outline to compute centroid
            std::string obj_str = json.substr(obj_start, obj_end - obj_start);
            double cx = 0, cy = 0;
            int npts = 0;
            size_t pt_search = 0;
            while (true) {
                auto x_pos = obj_str.find("\"x\":", pt_search);
                if (x_pos == std::string::npos) break;
                auto y_pos = obj_str.find("\"y\":", x_pos);
                if (y_pos == std::string::npos) break;

                double x = std::stod(obj_str.substr(x_pos + 4));
                double y = std::stod(obj_str.substr(y_pos + 4));
                cx += x;
                cy += y;
                npts++;
                pt_search = y_pos + 4;
            }

            if (npts > 0) {
                cx /= npts;
                cy /= npts;

                if (std::hypot(cx - target_cx, cy - target_cy) < 0.5) {
                    // Found the matching obstacle — remove it from JSON
                    // Also remove the leading or trailing comma
                    size_t remove_start = obj_start;
                    size_t remove_end = obj_end;

                    // Check for leading comma
                    size_t before = remove_start;
                    while (before > 0 && (json[before - 1] == ' ' || json[before - 1] == '\n' ||
                                          json[before - 1] == '\r' || json[before - 1] == '\t')) {
                        before--;
                    }
                    if (before > 0 && json[before - 1] == ',') {
                        remove_start = before - 1;
                    } else {
                        // Check for trailing comma
                        size_t after = remove_end;
                        while (after < json.size() && (json[after] == ' ' || json[after] == '\n' ||
                                                       json[after] == '\r' || json[after] == '\t')) {
                            after++;
                        }
                        if (after < json.size() && json[after] == ',') {
                            remove_end = after + 1;
                        }
                    }

                    json.erase(remove_start, remove_end - remove_start);
                    return true;
                }
            }

            search_pos = obj_end;
        }
        return false;
    }

    // Publish map.replace RPC to update the map service
    void publishMapReplace(const std::string& map_json) {
        xbot_rpc::RpcRequest rpc_req;
        rpc_req.method = "map.replace";
        rpc_req.params = "[" + map_json + "]";  // params is a JSON array with the map as first element
        rpc_req.id = "obstacle_tracker_" + std::to_string(ros::Time::now().toNSec());
        rpc_pub_.publish(rpc_req);
        ROS_INFO("Published map.replace RPC to update obstacle map");
    }

    void onAreaRefreshTimer(const ros::TimerEvent&) {
        loadAreaPolygons();
    }

    void onStatusTimer(const ros::TimerEvent&) {
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "obstacle_tracker";
        status.hardware_id = "lidar";
        status.level = diagnostic_msgs::DiagnosticStatus::OK;

        int tracked = 0, confirmed = 0;
        for (const auto& c : tracker_->clusters()) {
            if (c.confirmed) confirmed++;
            else tracked++;
        }

        status.message = "tracking:" + std::to_string(tracked) +
                         " confirmed:" + std::to_string(confirmed);

        diagnostic_msgs::KeyValue kv;
        kv.key = "candidates"; kv.value = std::to_string(tracked);
        status.values.push_back(kv);
        kv.key = "confirmed"; kv.value = std::to_string(confirmed);
        status.values.push_back(kv);
        kv.key = "total_added"; kv.value = std::to_string(confirmed_count_);
        status.values.push_back(kv);
        kv.key = "total_removed"; kv.value = std::to_string(removed_count_);
        status.values.push_back(kv);
        kv.key = "filtered_outside_area"; kv.value = std::to_string(filtered_count_);
        status.values.push_back(kv);

        status_pub_.publish(status);
        publishMarkers();
    }

    // Fast DBSCAN: cluster scan points with tight epsilon, return hulls.
    // O(n^2) — only runs when someone is subscribed to the shapes topic.
    void publishScanShapes(const std::vector<std::pair<double,double>>& points,
                           const ros::Time& stamp)
    {
        if (shape_pub_.getNumSubscribers() == 0) return;

        const double eps = 0.25; // 25cm — adjacent scan hits on same obstacle
        const int min_pts = 3;   // at least 3 points to form a shape

        // Simple DBSCAN: assign points to clusters
        std::vector<int> labels(points.size(), -1);
        int cluster_id = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] >= 0) continue;
            // Find neighbors
            std::vector<size_t> neighbors;
            for (size_t j = 0; j < points.size(); ++j) {
                double d = std::hypot(points[i].first - points[j].first,
                                      points[i].second - points[j].second);
                if (d < eps) neighbors.push_back(j);
            }
            if (static_cast<int>(neighbors.size()) < min_pts) continue;

            // Expand cluster
            for (auto& n : neighbors) labels[n] = cluster_id;
            for (size_t k = 0; k < neighbors.size(); ++k) {
                size_t ni = neighbors[k];
                for (size_t j = 0; j < points.size(); ++j) {
                    if (labels[j] >= 0) continue;
                    double d = std::hypot(points[ni].first - points[j].first,
                                          points[ni].second - points[j].second);
                    if (d < eps) {
                        labels[j] = cluster_id;
                        neighbors.push_back(j);
                    }
                }
            }
            cluster_id++;
        }

        // Build hulls per cluster
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        clear.ns = "scan_shapes";
        markers.markers.push_back(clear);

        for (int c = 0; c < cluster_id; ++c) {
            std::vector<std::pair<double,double>> cpts;
            for (size_t i = 0; i < points.size(); ++i) {
                if (labels[i] == c) cpts.push_back(points[i]);
            }

            auto hull = lidar_localization::ClusterTracker::convexHull(cpts);
            if (hull.size() < 3) continue;

            visualization_msgs::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = stamp;
            m.ns = "scan_shapes";
            m.id = c;
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.03;
            m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.8;
            m.lifetime = ros::Duration(0.5);

            for (const auto& pt : hull) {
                geometry_msgs::Point p;
                p.x = pt.first; p.y = pt.second; p.z = 0.1;
                m.points.push_back(p);
            }
            // Close polygon
            geometry_msgs::Point p;
            p.x = hull[0].first; p.y = hull[0].second; p.z = 0.1;
            m.points.push_back(p);

            markers.markers.push_back(m);
        }

        shape_pub_.publish(markers);
    }

    void publishMarkers() {
        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);

        int id = 0;
        for (const auto& c : tracker_->clusters()) {
            visualization_msgs::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = ros::Time::now();
            m.ns = "obstacle_tracker";
            m.id = id++;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.lifetime = ros::Duration(2.0);

            if (c.confirmed) {
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.7;
            } else {
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5;
            }

            if (c.hull.size() >= 3) {
                // Convex hull polygon
                m.type = visualization_msgs::Marker::LINE_STRIP;
                m.scale.x = 0.03; // line width
                m.pose.position.z = 0.1;
                for (const auto& pt : c.hull) {
                    geometry_msgs::Point p;
                    p.x = pt.first;
                    p.y = pt.second;
                    p.z = 0.1;
                    m.points.push_back(p);
                }
                // Close the polygon
                geometry_msgs::Point p;
                p.x = c.hull[0].first;
                p.y = c.hull[0].second;
                p.z = 0.1;
                m.points.push_back(p);
            } else {
                // Too few points for polygon — fall back to circle
                m.type = visualization_msgs::Marker::CYLINDER;
                m.pose.position.x = c.cx;
                m.pose.position.y = c.cy;
                m.pose.position.z = 0.1;
                m.scale.x = c.radius * 2;
                m.scale.y = c.radius * 2;
                m.scale.z = 0.2;
            }

            markers.markers.push_back(m);
        }

        marker_pub_.publish(markers);
    }

    // ROS
    ros::Subscriber scan_sub_;
    ros::Publisher status_pub_, marker_pub_, shape_pub_, rpc_pub_;
    ros::Timer map_timer_, status_timer_, area_refresh_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Tracker
    std::unique_ptr<lidar_localization::ClusterTracker> tracker_;
    double max_scan_range_ = 6.0;
    double safety_margin_ = 0.15;
    std::string map_file_;

    // Pending map updates
    std::mutex pending_mutex_;
    std::vector<lidar_localization::ClusterTracker::Cluster> pending_additions_;
    std::vector<lidar_localization::ClusterTracker::Cluster> pending_removals_;

    // Area polygons for containment filtering
    std::mutex areas_mutex_;
    std::vector<AreaPolygon> area_polygons_;

    // Counters
    int confirmed_count_ = 0;
    int removed_count_ = 0;
    int filtered_count_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool enable = true;
    pnh.param("enable", enable, true);
    if (!enable) {
        ROS_INFO("obstacle_tracker is disabled. Exiting.");
        return 0;
    }

    ObstacleTrackerNode node(nh, pnh);

    // Wait for map service, then load existing obstacles and area polygons
    if (ros::service::waitForService("mower_map_service/get_mowing_area", ros::Duration(30.0))) {
        node.loadExistingObstacles();
        node.loadAreaPolygons();
    } else {
        ROS_WARN("mower_map_service not available, starting without existing obstacles");
        // Still try to load area polygons from map.json directly
        node.loadAreaPolygons();
    }

    ros::spin();
    return 0;
}
