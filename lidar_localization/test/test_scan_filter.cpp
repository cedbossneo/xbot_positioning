// lidar_localization/test/test_scan_filter.cpp
#include <gtest/gtest.h>
#include "../src/scan_filter.h"
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <limits>

namespace {

sensor_msgs::LaserScan makeScan(const std::vector<float>& ranges, float angle_min, float angle_increment) {
    sensor_msgs::LaserScan scan;
    scan.angle_min = angle_min;
    scan.angle_increment = angle_increment;
    scan.range_min = 0.05f;
    scan.range_max = 12.0f;
    scan.ranges = ranges;
    return scan;
}

} // anonymous namespace

TEST(ScanFilterTest, RangesWithinMaxAreKept) {
    lidar_localization::ScanFilter filter(6.0, -0.1);
    auto scan = makeScan({1.0f, 3.0f, 5.9f}, 0.0f, M_PI / 3.0f);
    filter.apply(scan, 0.0, 0.0);
    EXPECT_FLOAT_EQ(scan.ranges[0], 1.0f);
    EXPECT_FLOAT_EQ(scan.ranges[1], 3.0f);
    EXPECT_FLOAT_EQ(scan.ranges[2], 5.9f);
}

TEST(ScanFilterTest, RangesBeyondMaxSetToNaN) {
    lidar_localization::ScanFilter filter(6.0, -0.1);
    auto scan = makeScan({6.1f, 10.0f, 5.0f}, 0.0f, M_PI / 3.0f);
    filter.apply(scan, 0.0, 0.0);
    EXPECT_TRUE(std::isnan(scan.ranges[0]));
    EXPECT_TRUE(std::isnan(scan.ranges[1]));
    EXPECT_FLOAT_EQ(scan.ranges[2], 5.0f);
}

TEST(ScanFilterTest, RangeAtExactMaxIsKept) {
    lidar_localization::ScanFilter filter(6.0, -0.1);
    auto scan = makeScan({6.0f}, 0.0f, 0.1f);
    filter.apply(scan, 0.0, 0.0);
    EXPECT_FLOAT_EQ(scan.ranges[0], 6.0f);
}

TEST(ScanFilterTest, InvalidRangesSetToNaN) {
    lidar_localization::ScanFilter filter(6.0, -0.1);
    auto scan = makeScan({0.0f, -1.0f, std::numeric_limits<float>::infinity(), 5.0f}, 0.0f, M_PI / 4.0f);
    filter.apply(scan, 0.0, 0.0);
    EXPECT_TRUE(std::isnan(scan.ranges[0]));
    EXPECT_TRUE(std::isnan(scan.ranges[1]));
    EXPECT_TRUE(std::isnan(scan.ranges[2]));
    EXPECT_FLOAT_EQ(scan.ranges[3], 5.0f);
}

TEST(ScanFilterTest, TiltedTerrainFiltersGroundHits) {
    lidar_localization::ScanFilter filter(6.0, -0.1);
    // Forward-pointing beam (angle=0) at 5m, with ~30 deg pitch
    // x_laser = 5*cos(0) = 5, y_laser = 5*sin(0) = 0
    // z_world = -sin(0.5) * 5 + 0 = -2.40 -> below -0.1 -> NaN
    auto scan = makeScan({5.0f}, 0.0f, 0.1f);
    filter.apply(scan, 0.0, 0.5);
    EXPECT_TRUE(std::isnan(scan.ranges[0]));
}

TEST(ScanFilterTest, FlatTerrainNoTiltFiltering) {
    lidar_localization::ScanFilter filter(6.0, -0.1);
    auto scan = makeScan({5.0f, 5.0f, 5.0f, 5.0f}, 0.0f, M_PI / 2.0f);
    filter.apply(scan, 0.0, 0.0);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(scan.ranges[i], 5.0f);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
