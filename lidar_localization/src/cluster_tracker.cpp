// lidar_localization/src/cluster_tracker.cpp
#include "cluster_tracker.h"
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace lidar_localization {

ClusterTracker::ClusterTracker(const Config& cfg) : cfg_(cfg) {}

double ClusterTracker::distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

bool ClusterTracker::isInRange(double cx, double cy, double mower_x, double mower_y) const {
    return distance(cx, cy, mower_x, mower_y) <= cfg_.max_observation_range;
}

ClusterTracker::UpdateResult ClusterTracker::update(
    const std::vector<std::pair<double,double>>& points_map,
    double mower_x, double mower_y, double now)
{
    UpdateResult result;

    // Reset per-scan state: restore centroid to stable position so point
    // assignment uses the long-term average, not the previous scan's drift.
    for (auto& c : clusters_) {
        c.current_point_count = 0;
        c.cx = c.stable_cx;
        c.cy = c.stable_cy;
    }
    // Temporary storage for points belonging to each cluster
    std::vector<std::vector<std::pair<double,double>>> cluster_points(clusters_.size());

    // Assign each point to nearest cluster or create new candidate
    for (const auto& [px, py] : points_map) {
        double best_dist = cfg_.cluster_merge_distance;
        int best_idx = -1;
        for (int i = 0; i < static_cast<int>(clusters_.size()); ++i) {
            double d = distance(px, py, clusters_[i].cx, clusters_[i].cy);
            if (d < best_dist) {
                best_dist = d;
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            auto& c = clusters_[best_idx];
            c.current_point_count++;
            // Update stable running centroid across ALL observations
            c.total_point_count++;
            c.stable_cx += (px - c.stable_cx) / c.total_point_count;
            c.stable_cy += (py - c.stable_cy) / c.total_point_count;
            // Per-scan centroid for distance checks (reset each scan)
            int n = c.current_point_count;
            c.cx = c.cx + (px - c.cx) / n;
            c.cy = c.cy + (py - c.cy) / n;
            double r = distance(px, py, c.stable_cx, c.stable_cy);
            if (r > c.radius) c.radius = r;
            cluster_points[best_idx].push_back({px, py});
            // Accumulate points for real shape (keep bounded to avoid memory growth)
            if (c.accumulated_points.size() < 500) {
                c.accumulated_points.push_back({px, py});
            }
        } else {
            // New cluster candidate (will be filtered by radius check below)
            Cluster c;
            c.cx = px;
            c.cy = py;
            c.stable_cx = px;
            c.stable_cy = py;
            c.total_point_count = 1;
            c.radius = 0.0;
            c.first_seen = now;
            c.last_seen = now;
            c.last_in_range = now;
            c.current_point_count = 1;
            c.hull = {{px, py}};
            c.accumulated_points = {{px, py}};
            clusters_.push_back(c);
            cluster_points.push_back({{px, py}});
        }
    }

    // Compute convex hull for clusters that received points this scan.
    // For unconfirmed clusters: use current scan points (live view).
    // For confirmed clusters: use accumulated points from all observations
    // (gives the real shape seen from multiple angles, stable across scans).
    for (size_t i = 0; i < clusters_.size() && i < cluster_points.size(); ++i) {
        if (!cluster_points[i].empty()) {
            if (clusters_[i].confirmed) {
                clusters_[i].hull = convexHull(clusters_[i].accumulated_points);
            } else {
                clusters_[i].hull = convexHull(cluster_points[i]);
            }
        }
    }

    // Update observation state for each cluster
    for (auto& c : clusters_) {
        bool in_range = isInRange(c.cx, c.cy, mower_x, mower_y);
        bool observed = c.current_point_count > 0;

        if (observed) {
            c.last_seen = now;
        }

        if (in_range) {
            // Compute time delta before updating last_in_range
            double dt = (c.last_in_range > 0.0) ? (now - c.last_in_range) : 0.0;
            c.last_in_range = now;
            c.in_range_scans++;
            if (observed) {
                c.hit_count++;
                c.not_seen_in_range_time = 0.0;
            } else if (c.confirmed) {
                // Accumulate real elapsed time not seen while in range
                c.not_seen_in_range_time += dt;
            }
        }
    }

    // Remove clusters above max radius (walls/fences — always discard)
    clusters_.erase(
        std::remove_if(clusters_.begin(), clusters_.end(),
            [this](const Cluster& c) {
                return !c.confirmed && c.radius > cfg_.max_radius;
            }),
        clusters_.end());

    // Promotion check (min_radius enforced here, not earlier, so narrow
    // clusters can accumulate points across scans before being judged)
    for (auto& c : clusters_) {
        if (c.confirmed) continue;
        if (c.in_range_scans == 0) continue;

        double elapsed = now - c.first_seen;
        double ratio = static_cast<double>(c.hit_count) / static_cast<double>(c.in_range_scans);

        if (elapsed >= cfg_.confirm_duration && ratio >= cfg_.observation_ratio) {
            if (c.radius < cfg_.min_radius) {
                continue; // too small to be a real obstacle — keep tracking but don't promote
            }
            c.confirmed = true;
            // Lock centroid to the stable running average
            c.cx = c.stable_cx;
            c.cy = c.stable_cy;
            // Compute final hull from all accumulated observations
            if (c.accumulated_points.size() >= 3) {
                c.hull = convexHull(c.accumulated_points);
            }
            // Generate unique obstacle ID (include centroid for uniqueness)
            std::ostringstream oss;
            oss << "lidar_obs_" << std::fixed << std::setprecision(1)
                << c.cx << "_" << c.cy << "_" << std::setprecision(0) << now;
            c.obstacle_id = oss.str();
            result.newly_confirmed.push_back(c);
        }
    }

    // Garbage collect: discard unconfirmed clusters not seen for a long time
    clusters_.erase(
        std::remove_if(clusters_.begin(), clusters_.end(),
            [this, now](const Cluster& c) {
                return !c.confirmed && (now - c.last_seen) > cfg_.confirm_duration * 2;
            }),
        clusters_.end());

    // Removal check: only for confirmed clusters, only when mower is in range
    std::vector<Cluster> to_remove;
    clusters_.erase(
        std::remove_if(clusters_.begin(), clusters_.end(),
            [this, &to_remove, mower_x, mower_y](const Cluster& c) {
                if (!c.confirmed) return false;
                if (!isInRange(c.cx, c.cy, mower_x, mower_y)) return false;
                if (c.not_seen_in_range_time >= cfg_.remove_duration) {
                    to_remove.push_back(c);
                    return true;
                }
                return false;
            }),
        clusters_.end());
    result.newly_removed = to_remove;

    return result;
}

std::vector<std::pair<double,double>> ClusterTracker::convexHull(
    std::vector<std::pair<double,double>> points)
{
    size_t n = points.size();
    if (n < 3) return points;

    std::sort(points.begin(), points.end());

    // Andrew's monotone chain
    std::vector<std::pair<double,double>> hull;
    hull.reserve(2 * n);

    // Lower hull
    for (size_t i = 0; i < n; ++i) {
        while (hull.size() >= 2) {
            auto& a = hull[hull.size() - 2];
            auto& b = hull[hull.size() - 1];
            double cross = (b.first - a.first) * (points[i].second - a.second)
                         - (b.second - a.second) * (points[i].first - a.first);
            if (cross <= 0) hull.pop_back();
            else break;
        }
        hull.push_back(points[i]);
    }

    // Upper hull
    size_t lower_size = hull.size() + 1;
    for (size_t i = n - 1; i-- > 0; ) {
        while (hull.size() >= lower_size) {
            auto& a = hull[hull.size() - 2];
            auto& b = hull[hull.size() - 1];
            double cross = (b.first - a.first) * (points[i].second - a.second)
                         - (b.second - a.second) * (points[i].first - a.first);
            if (cross <= 0) hull.pop_back();
            else break;
        }
        hull.push_back(points[i]);
    }

    hull.pop_back(); // Remove duplicate of first point
    return hull;
}

void ClusterTracker::loadExistingObstacles(const std::vector<Cluster>& existing) {
    for (const auto& e : existing) {
        clusters_.push_back(e);
    }
}

const std::vector<ClusterTracker::Cluster>& ClusterTracker::clusters() const {
    return clusters_;
}

} // namespace lidar_localization
