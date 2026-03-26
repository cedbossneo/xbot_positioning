// lidar_localization/src/cluster_tracker.h
#ifndef LIDAR_LOCALIZATION_CLUSTER_TRACKER_H
#define LIDAR_LOCALIZATION_CLUSTER_TRACKER_H

#include <string>
#include <vector>
#include <cmath>

namespace lidar_localization {

class ClusterTracker {
public:
    struct Config {
        double cluster_merge_distance = 0.3;
        double min_radius = 0.15;
        double max_radius = 2.0;
        double confirm_duration = 60.0;
        double remove_duration = 600.0;
        double observation_ratio = 0.5;
        double max_observation_range = 4.0;
    };

    struct Cluster {
        double cx = 0.0, cy = 0.0;
        double radius = 0.0;
        double first_seen = 0.0;
        double last_seen = 0.0;
        double last_in_range = 0.0;
        double not_seen_in_range_time = 0.0; // accumulated seconds in range without observation
        int hit_count = 0;
        int in_range_scans = 0;
        bool confirmed = false;
        std::string obstacle_id;

        // Number of points absorbed into this cluster (current scan)
        int current_point_count = 0;

        // Running centroid from all observations (stable, doesn't shift per-scan)
        double stable_cx = 0.0, stable_cy = 0.0;
        int total_point_count = 0;

        // Convex hull of scan points from the latest observation.
        // For candidates: updated each scan from current scan points.
        // For confirmed: frozen at confirmation time (accumulated hull).
        std::vector<std::pair<double,double>> hull;

        // Accumulated points across all observations — used to build
        // the real obstacle shape from multiple viewing angles.
        std::vector<std::pair<double,double>> accumulated_points;
    };

    struct UpdateResult {
        std::vector<Cluster> newly_confirmed;
        std::vector<Cluster> newly_removed;
    };

    explicit ClusterTracker(const Config& cfg);

    // mower_x, mower_y: current mower position in map frame
    // now: current time in seconds
    UpdateResult update(const std::vector<std::pair<double,double>>& points_map,
                        double mower_x, double mower_y, double now);

    void loadExistingObstacles(const std::vector<Cluster>& existing);

    const std::vector<Cluster>& clusters() const;

private:
    Config cfg_;
    std::vector<Cluster> clusters_;

    bool isInRange(double cx, double cy, double mower_x, double mower_y) const;
    static double distance(double x1, double y1, double x2, double y2);
public:
    static std::vector<std::pair<double,double>> convexHull(
        std::vector<std::pair<double,double>> points);
private:
};

} // namespace lidar_localization

#endif
