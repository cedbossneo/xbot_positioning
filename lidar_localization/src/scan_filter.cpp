// lidar_localization/src/scan_filter.cpp
#include "scan_filter.h"
#include <cmath>
#include <limits>

namespace lidar_localization {

ScanFilter::ScanFilter(double max_range, double ground_height_threshold)
    : max_range_(max_range), ground_height_threshold_(ground_height_threshold) {}

void ScanFilter::apply(sensor_msgs::LaserScan& scan, double roll, double pitch) const {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const double sin_roll = std::sin(roll);
    const double cos_pitch = std::cos(pitch);
    const double sin_pitch = std::sin(pitch);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];

        // Filter invalid ranges (respect sensor's range_min)
        if (r < scan.range_min || !std::isfinite(r)) {
            scan.ranges[i] = nan;
            continue;
        }

        // Filter beyond max range
        if (static_cast<double>(r) > max_range_) {
            scan.ranges[i] = nan;
            continue;
        }

        // Filter ground reflections using tilt compensation
        double angle = scan.angle_min + i * scan.angle_increment;
        double x_laser = r * std::cos(angle);
        double y_laser = r * std::sin(angle);
        double z_world = -sin_pitch * x_laser + sin_roll * cos_pitch * y_laser;

        if (z_world < ground_height_threshold_) {
            scan.ranges[i] = nan;
        }
    }
}

} // namespace lidar_localization
