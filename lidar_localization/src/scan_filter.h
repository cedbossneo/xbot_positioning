// lidar_localization/src/scan_filter.h
#ifndef LIDAR_LOCALIZATION_SCAN_FILTER_H
#define LIDAR_LOCALIZATION_SCAN_FILTER_H

#include <sensor_msgs/LaserScan.h>

namespace lidar_localization {

class ScanFilter {
public:
    ScanFilter(double max_range, double ground_height_threshold);

    // Modifies scan in-place: sets ranges to NaN for:
    // - ranges beyond max_range
    // - invalid ranges (<= 0, infinite)
    // - ground-reflected beams (computed from roll/pitch)
    void apply(sensor_msgs::LaserScan& scan, double roll, double pitch) const;

private:
    double max_range_;
    double ground_height_threshold_;
};

} // namespace lidar_localization

#endif
