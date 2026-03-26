// lidar_localization/src/lidar_slam_bridge_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "xbot_msgs/AbsolutePose.h"
#include "scan_filter.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>


// slam_toolbox services
#include <slam_toolbox_msgs/SerializePoseGraph.h>
#include <slam_toolbox_msgs/DeserializePoseGraph.h>
#include <slam_toolbox_msgs/Pause.h>

class LidarSlamBridgeNode {
public:
    LidarSlamBridgeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : tf_buffer_()
        , tf_listener_(tf_buffer_)
    {
        // Parameters
        double max_scan_range, ground_height_threshold;
        pnh.param("max_scan_range", max_scan_range, 6.0);
        pnh.param("ground_height_threshold", ground_height_threshold, -0.1);
        pnh.param("default_lidar_covariance", default_lidar_covariance_, 500.0);
        pnh.param<std::string>("map_file_path", map_file_path_, "/var/lib/lidar_map/lawn_map");
        pnh.param("tilt_suppress_threshold", tilt_suppress_threshold_, 0.35); // ~20 degrees
        pnh.param("yaw_rate_suppress_threshold", yaw_rate_suppress_threshold_, 0.5); // rad/s
        pnh.param("lidar_offset_x", lidar_offset_x_, 0.0);
        pnh.param("lidar_offset_y", lidar_offset_y_, 0.0);
        pnh.param("lidar_offset_z", lidar_offset_z_, 0.18);

        scan_filter_ = std::make_unique<lidar_localization::ScanFilter>(max_scan_range, ground_height_threshold);

        // Subscribers
        std::string scan_topic;
        pnh.param<std::string>("scan_topic", scan_topic, "/scan");
        scan_sub_ = nh.subscribe(scan_topic, 1, &LidarSlamBridgeNode::onScan, this);
        imu_sub_ = pnh.subscribe("imu_in", 10, &LidarSlamBridgeNode::onImu, this);
        // Use RAW GPS for initial position and GPS state — NOT the EKF output.
        // Using EKF would create a feedback loop: EKF → bridge → slam → odom_lidar → EKF
        raw_gps_sub_ = nh.subscribe("/ll/position/gps", 10, &LidarSlamBridgeNode::onRawGps, this);
        map_sub_ = nh.subscribe("/map", 1, &LidarSlamBridgeNode::onMap, this);
        // Subscribe to slam_toolbox pose for covariance (TF carries no covariance)
        std::string slam_pose_topic;
        pnh.param<std::string>("slam_pose_topic", slam_pose_topic, "/slam_toolbox/pose");
        slam_pose_sub_ = nh.subscribe(slam_pose_topic, 10, &LidarSlamBridgeNode::onSlamPoseCovariance, this);
        // Raw wheel odometry for slam_toolbox's odom input.
        // MUST use raw wheels, NOT the EKF, to avoid feedback loop.
        wheel_twist_sub_ = nh.subscribe("/ll/diff_drive/measured_twist", 10,
            &LidarSlamBridgeNode::onWheelTwist, this);

        // Publishers
        filtered_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_lidar", 10);
        // Re-publish slam_toolbox /map with origin in GPS frame so viewers
        // (map_viewer on port 8080) can overlay it on the robot position.
        global_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map_global", 1, true);
        status_pub_ = pnh.advertise<diagnostic_msgs::DiagnosticStatus>("status", 1);

        // Timers
        status_timer_ = nh.createTimer(ros::Duration(1.0), &LidarSlamBridgeNode::publishStatus, this);
        // One-shot timer to load map after GPS arrives (avoids blocking callbacks)
        map_load_timer_ = nh.createTimer(ros::Duration(1.0),
            &LidarSlamBridgeNode::tryLoadMapIfReady, this);
        // Poll slam_toolbox TF at 10Hz to extract pose and publish as Odometry
        tf_poll_timer_ = nh.createTimer(ros::Duration(0.1),
            &LidarSlamBridgeNode::pollSlamTf, this);
        // Publish map->odom identity at 10Hz to maintain valid TF chain.
        // This runs independently of pollSlamTf to ensure the "map" frame
        // always exists, even before slam_toolbox starts.
        map_odom_timer_ = nh.createTimer(ros::Duration(0.1),
            &LidarSlamBridgeNode::publishMapOdomTf, this);

        // Static TF: base_link_odom -> base_laser_slam
        // Mirrors real base_link -> base_laser but in the isolated TF subtree
        // Filtered scans have frame_id remapped to base_laser_slam
        {
            geometry_msgs::TransformStamped static_tf;
            static_tf.header.stamp = ros::Time::now();
            static_tf.header.frame_id = "base_link_odom";
            static_tf.child_frame_id = "base_laser_slam";
            static_tf.transform.translation.x = lidar_offset_x_;
            static_tf.transform.translation.y = lidar_offset_y_;
            static_tf.transform.translation.z = lidar_offset_z_;
            static_tf.transform.rotation.w = 1.0;
            static_tf_broadcaster_.sendTransform(static_tf);
        }

        ROS_INFO("lidar_slam_bridge node started");
    }

    void tryLoadMap() {
        // Wait briefly for slam_toolbox service
        if (!ros::service::waitForService("/slam_toolbox/deserialize_map", ros::Duration(5.0))) {
            ROS_WARN("slam_toolbox deserialize service not available, skipping map load");
            return;
        }

        slam_toolbox_msgs::DeserializePoseGraph srv;
        srv.request.filename = map_file_path_;
        srv.request.match_type = 2; // LOCALIZE_AT_POSE
        srv.request.initial_pose.x = initial_gps_x_;
        srv.request.initial_pose.y = initial_gps_y_;
        srv.request.initial_pose.theta = initial_gps_theta_;

        if (ros::service::call("/slam_toolbox/deserialize_map", srv)) {
            ROS_INFO("Loaded saved map from %s", map_file_path_.c_str());
        } else {
            ROS_WARN("No saved map found at %s, starting fresh", map_file_path_.c_str());
        }
    }

    void saveMap() {
        slam_toolbox_msgs::SerializePoseGraph srv;
        srv.request.filename = map_file_path_;
        if (ros::service::call("/slam_toolbox/serialize_map", srv)) {
            ROS_INFO("Saved map to %s", map_file_path_.c_str());
        } else {
            ROS_WARN("Failed to save map to %s", map_file_path_.c_str());
        }
    }

private:
    void onScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Only feed scans to slam_toolbox when GPS is RTK fixed.
        // This ensures the map is built ONLY with accurate positions —
        // no drift from float GPS or dead reckoning corrupts the map.
        if (!gps_is_fixed_) return;

        // Suppress scans during severe tilt
        if (std::abs(latest_roll_) > tilt_suppress_threshold_ ||
            std::abs(latest_pitch_) > tilt_suppress_threshold_) {
            return;
        }

        // Suppress scans during fast rotation — scan matching fails when
        // consecutive scans are too far apart angularly, corrupting the map
        if (std::abs(latest_yaw_rate_) > yaw_rate_suppress_threshold_) {
            return;
        }

        sensor_msgs::LaserScan filtered = *msg;
        scan_filter_->apply(filtered, latest_roll_, latest_pitch_);
        // Remap frame to isolated SLAM TF subtree
        filtered.header.frame_id = "base_laser_slam";
        filtered_scan_pub_.publish(filtered);
    }

    void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
        latest_yaw_rate_ = msg->angular_velocity.z;

        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        if (q.length2() > 0.5) {
            tf2::Matrix3x3 m(q);
            double yaw;
            m.getRPY(latest_roll_, latest_pitch_, yaw);
            latest_imu_yaw_ = yaw;
            has_imu_yaw_ = true;
        }
    }

    void onRawGps(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
        const bool is_fixed = (msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;

        // Cache GPS position for diagnostics
        if (is_fixed) {
            latest_gps_x_ = msg->pose.pose.position.x;
            latest_gps_y_ = msg->pose.pose.position.y;
            has_gps_pose_ = true;
        }

        // First fix: record initial position from RAW GPS for map loading.
        // We accumulate several samples and average to reduce noise.
        if (is_fixed && !initial_gps_received_) {
            tf2::Quaternion q;
            tf2::fromMsg(msg->pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            init_gps_x_accum_ += msg->pose.pose.position.x;
            init_gps_y_accum_ += msg->pose.pose.position.y;
            // Circular mean for angles: accumulate sin/cos to avoid wrap-around bug
            init_gps_sin_accum_ += std::sin(yaw);
            init_gps_cos_accum_ += std::cos(yaw);
            init_gps_samples_++;

            if (init_gps_samples_ >= 10) {
                initial_gps_received_ = true;
                initial_gps_x_ = init_gps_x_accum_ / init_gps_samples_;
                initial_gps_y_ = init_gps_y_accum_ / init_gps_samples_;
                initial_gps_theta_ = std::atan2(init_gps_sin_accum_, init_gps_cos_accum_);
                ROS_INFO("GPS initial fix at (%.2f, %.2f, %.1f°) from %d raw GPS samples",
                    initial_gps_x_, initial_gps_y_, initial_gps_theta_ * 180.0 / M_PI,
                    init_gps_samples_);
            }
        }

        // GPS state transition: control slam_toolbox pause/unpause.
        if (is_fixed != gps_is_fixed_) {
            gps_is_fixed_ = is_fixed;
            if (is_fixed) {
                setSlamPaused(false);
                ROS_INFO("GPS RTK fixed — slam_toolbox unpaused, mapping resumed");
            } else {
                setSlamPaused(true);
                ROS_WARN("GPS lost RTK fix — slam_toolbox paused, map frozen");
            }
        }
    }

    void setSlamPaused(bool should_pause) {
        // slam_toolbox pause_new_measurements is a toggle, not a set.
        // We track internal state to avoid double-toggling.
        if (should_pause == slam_paused_) return;
        slam_toolbox_msgs::Pause srv;
        if (ros::service::call("/slam_toolbox/pause_new_measurements", srv)) {
            slam_paused_ = should_pause;
            ROS_INFO("slam_toolbox %s", should_pause ? "paused" : "unpaused");
        } else {
            ROS_WARN("Failed to call pause_new_measurements service");
        }
    }

    void onWheelTwist(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // Integrate local dead-reckoning pose for slam_toolbox's odom.
        // Uses absolute IMU yaw for heading (doesn't drift like gyro integration)
        // and wheel linear velocity for distance (approximate with slip,
        // but slam_toolbox's scan matcher corrects residual error).
        const ros::Time now = msg->header.stamp;

        if (!has_wheel_odom_) {
            // Wait for both GPS init and IMU orientation before starting.
            if (!initial_gps_received_ || !has_imu_yaw_) return;
            has_wheel_odom_ = true;
            last_wheel_time_ = now;
            local_odom_x_ = initial_gps_x_;
            local_odom_y_ = initial_gps_y_;
            local_odom_theta_ = latest_imu_yaw_;
            return;
        }

        const double dt = (now - last_wheel_time_).toSec();
        last_wheel_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return;

        const double vx = msg->twist.linear.x;

        // Use absolute IMU yaw for heading. This avoids gyro bias drift
        // that accumulates with yaw rate integration, and prevents the
        // SLAM map from rotating with the robot.
        local_odom_theta_ = latest_imu_yaw_;
        local_odom_x_ += vx * std::cos(local_odom_theta_) * dt;
        local_odom_y_ += vx * std::sin(local_odom_theta_) * dt;

        // Publish slam_odom -> base_link_odom TF for slam_toolbox
        tf2::Quaternion q_local;
        q_local.setRPY(0, 0, local_odom_theta_);

        geometry_msgs::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "slam_odom";
        tf.child_frame_id = "base_link_odom";
        tf.transform.translation.x = local_odom_x_;
        tf.transform.translation.y = local_odom_y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = tf2::toMsg(q_local);

        tf_broadcaster_.sendTransform(tf);
    }

    void onSlamPoseCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // Cache covariance from slam_toolbox's /pose topic (TF carries no covariance)
        cached_covariance_ = msg->pose.covariance;
        has_cached_covariance_ = true;
    }

    void pollSlamTf(const ros::TimerEvent&) {
        // Read slam_toolbox's slam_map->base_link_odom to get SLAM's pose estimate.
        // slam_toolbox uses an isolated TF subtree: slam_map -> slam_odom -> base_link_odom
        geometry_msgs::TransformStamped slam_tf;
        try {
            slam_tf = tf_buffer_.lookupTransform("slam_map", "base_link_odom", ros::Time(0));
        } catch (const tf2::TransformException&) {
            // slam_toolbox TF not yet available — heartbeat only
            if (has_gps_pose_) publishOdomHeartbeat(ros::Time::now());
            return;
        }

        // Check if slam_toolbox has updated since last poll
        geometry_msgs::TransformStamped slam_map_odom;
        try {
            slam_map_odom = tf_buffer_.lookupTransform("slam_map", "slam_odom", ros::Time(0));
        } catch (const tf2::TransformException&) {
            return;
        }
        if (slam_map_odom.header.stamp == last_map_odom_stamp_) return;
        last_map_odom_stamp_ = slam_map_odom.header.stamp;

        has_slam_pose_ = true;
        last_slam_pose_time_ = ros::Time::now();

        // Since local odom now starts in GPS world coordinates (initial_gps_x/y/theta),
        // and the map is loaded with initial_pose in GPS world coords, slam_map
        // frame IS the world frame. slam_map->base_link_odom is already in GPS coords.
        const double slam_x = slam_tf.transform.translation.x;
        const double slam_y = slam_tf.transform.translation.y;
        const tf2::Quaternion q_global(
            slam_tf.transform.rotation.x, slam_tf.transform.rotation.y,
            slam_tf.transform.rotation.z, slam_tf.transform.rotation.w);

        // Compute GPS divergence for diagnostics
        if (has_gps_pose_) {
            const double ex = slam_x - latest_gps_x_;
            const double ey = slam_y - latest_gps_y_;
            slam_gps_divergence_ = std::sqrt(ex * ex + ey * ey);
        }

        // === Heartbeat-only mode ===
        // SLAM odom is always a heartbeat (cov=1e6) that keeps isLidarActive()
        // true in xbot_positioning without influencing the EKF position.
        //
        // Why not use SLAM for position correction during GPS float?
        // - slam_toolbox is paused during float (no scan matching)
        // - The SLAM pose degrades to pure wheel odom + last scan correction
        // - Wheel odom heading drift makes the SLAM position unreliable
        // - Publishing a drifting SLAM pose would hurt more than help
        //
        // SLAM's value in this architecture:
        // 1. Map building for obstacle detection (during GPS fixed)
        // 2. Keeping isLidarActive() true so GPS float updates get applied
        //    with lower covariance (5000 vs 50000 in xbot_positioning)
        // 3. Future: once scan matching runs continuously with a good map,
        //    SLAM can provide real corrections during float
        publishOdomHeartbeat(slam_tf.header.stamp);
    }

    void publishMapOdomTf(const ros::TimerEvent&) {
        publishMapOdomIdentity();
    }

    void publishMapOdomIdentity() {
        // Publish map->odom as identity transform.
        // Since odom->base_link_odom carries the EKF pose in map frame,
        // identity map->odom means map->base_link_odom = EKF pose exactly.
        // This decouples the real TF chain from slam_toolbox drift.
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "odom";
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.w = 1.0;
        tf_broadcaster_.sendTransform(tf);
    }

    void publishOdomHeartbeat(const ros::Time& stamp) {
        // Heartbeat: use GPS position with huge covariance.
        // This is a no-op for the EKF but keeps isLidarActive() true.
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = has_gps_pose_ ? latest_gps_x_ : 0.0;
        odom.pose.pose.position.y = has_gps_pose_ ? latest_gps_y_ : 0.0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.w = 1.0;
        odom.pose.covariance.fill(0);
        odom.pose.covariance[0] = 1e6;
        odom.pose.covariance[7] = 1e6;
        odom.pose.covariance[35] = 1e6;
        odom_pub_.publish(odom);
    }

    void onMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        // Count occupied cells as a proxy for map complexity
        int occupied = 0;
        for (const auto& cell : msg->data) {
            if (cell > 50) ++occupied;
        }
        map_occupied_cells_ = occupied;

        // Re-publish map in "map" frame for web viewer overlay.
        // Since local odom now starts in GPS world coords, slam_map frame
        // is already world-aligned — just re-frame it.
        if (!initial_gps_received_) return;

        nav_msgs::OccupancyGrid global_map = *msg;
        global_map.header.frame_id = "map";
        global_map_pub_.publish(global_map);
    }

    void tryLoadMapIfReady(const ros::TimerEvent&) {
        if (!initial_gps_received_ || map_load_attempted_) return;
        map_load_attempted_ = true;
        map_load_timer_.stop();
        tryLoadMap();
    }

    void publishStatus(const ros::TimerEvent&) {
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "lidar_slam_bridge";
        status.hardware_id = "lidar";

        std::string state;
        if (!initial_gps_received_) state = "WAITING_FOR_GPS";
        else if (gps_is_fixed_) state = "MAPPING";       // building map, heartbeat only to EKF
        else if (has_slam_pose_) state = "CORRECTING";    // GPS float, SLAM correcting EKF
        else state = "PAUSED";

        diagnostic_msgs::KeyValue kv;
        kv.key = "state"; kv.value = state;
        status.values.push_back(kv);

        kv.key = "gps_fixed"; kv.value = gps_is_fixed_ ? "true" : "false";
        status.values.push_back(kv);

        kv.key = "slam_paused"; kv.value = slam_paused_ ? "true" : "false";
        status.values.push_back(kv);

        kv.key = "map_occupied_cells"; kv.value = std::to_string(map_occupied_cells_);
        status.values.push_back(kv);

        double pose_age = has_slam_pose_ ?
            (ros::Time::now() - last_slam_pose_time_).toSec() : -1.0;
        kv.key = "last_pose_age"; kv.value = std::to_string(pose_age);
        status.values.push_back(kv);

        char div_buf[32];
        std::snprintf(div_buf, sizeof(div_buf), "%.3f", slam_gps_divergence_);
        kv.key = "slam_gps_divergence"; kv.value = div_buf;
        status.values.push_back(kv);

        status.level = (state == "MAPPING" && has_slam_pose_ && pose_age < 5.0) ?
            diagnostic_msgs::DiagnosticStatus::OK :
            diagnostic_msgs::DiagnosticStatus::WARN;
        status.message = state;

        status_pub_.publish(status);
    }

    // Subscribers
    ros::Subscriber scan_sub_, imu_sub_, raw_gps_sub_, map_sub_, slam_pose_sub_, wheel_twist_sub_;
    // Publishers
    ros::Publisher filtered_scan_pub_, odom_pub_, global_map_pub_, status_pub_;
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    // Timers
    ros::Timer status_timer_;
    ros::Timer map_load_timer_;
    ros::Timer tf_poll_timer_;
    ros::Timer map_odom_timer_;
    // Scan filter
    std::unique_ptr<lidar_localization::ScanFilter> scan_filter_;
    // State (all callbacks serialized by ros::spin() single-threaded spinner)
    double latest_roll_ = 0.0;
    double latest_pitch_ = 0.0;
    double latest_yaw_rate_ = 0.0;
    double latest_imu_yaw_ = 0.0;
    bool has_imu_yaw_ = false;
    bool initial_gps_received_ = false;
    double initial_gps_x_ = 0.0;
    double initial_gps_y_ = 0.0;
    double initial_gps_theta_ = 0.0;
    bool map_load_attempted_ = false;
    bool has_slam_pose_ = false;
    ros::Time last_slam_pose_time_;
    ros::Time last_map_odom_stamp_;
    int map_occupied_cells_ = 0;
    // Wheel odometry integration (local dead reckoning for slam_toolbox)
    double local_odom_x_ = 0.0;
    double local_odom_y_ = 0.0;
    double local_odom_theta_ = 0.0;
    bool has_wheel_odom_ = false;
    ros::Time last_wheel_time_;
    // GPS init averaging (circular mean for yaw via sin/cos accumulation)
    double init_gps_x_accum_ = 0.0;
    double init_gps_y_accum_ = 0.0;
    double init_gps_sin_accum_ = 0.0;
    double init_gps_cos_accum_ = 0.0;
    int init_gps_samples_ = 0;
    // GPS state
    double latest_gps_x_ = 0.0;
    double latest_gps_y_ = 0.0;
    bool has_gps_pose_ = false;
    bool gps_is_fixed_ = false;
    // slam_toolbox pause state
    bool slam_paused_ = false;
    // Diagnostics
    double slam_gps_divergence_ = 0.0;
    // Cached covariance from slam_toolbox /pose topic
    boost::array<double, 36> cached_covariance_{};
    bool has_cached_covariance_ = false;
    // Config
    double default_lidar_covariance_;
    double tilt_suppress_threshold_;
    double yaw_rate_suppress_threshold_;
    std::string map_file_path_;
    // LiDAR mounting offset from base_link (rotation center at rear wheels)
    double lidar_offset_x_;  // forward
    double lidar_offset_y_;  // lateral
    double lidar_offset_z_;  // height
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_slam_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool enable = false;
    pnh.param("enable", enable, false);

    if (!enable) {
        ROS_INFO("lidar_slam_bridge is disabled (enable=false). Exiting.");
        return 0;
    }

    LidarSlamBridgeNode node(nh, pnh);

    ros::spin();

    // ros::spin() returns after SIGINT/SIGTERM triggers ros::shutdown().
    // ROS services are still callable briefly after spin() exits, so
    // saving the map here is safe (no signal handler needed).
    ROS_INFO("Shutting down, saving map...");
    node.saveMap();

    return 0;
}
