// lidar_localization/test/test_cluster_tracker.cpp
#include <gtest/gtest.h>
#include "../src/cluster_tracker.h"
#include <cmath>

using lidar_localization::ClusterTracker;

// Helper: generate N points in a circle around (cx, cy) with radius r
std::vector<std::pair<double,double>> circlePoints(double cx, double cy, double r, int n = 8) {
    std::vector<std::pair<double,double>> pts;
    for (int i = 0; i < n; ++i) {
        double a = 2.0 * M_PI * i / n;
        pts.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    return pts;
}

TEST(ClusterTrackerTest, NewPointsFormCluster) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 5.0;
    cfg.remove_duration = 10.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 4.0;
    ClusterTracker tracker(cfg);

    auto pts = circlePoints(5.0, 5.0, 0.2);
    auto result = tracker.update(pts, 0.0, 0.0, 0.0);

    ASSERT_EQ(tracker.clusters().size(), 1u);
    EXPECT_NEAR(tracker.clusters()[0].cx, 5.0, 0.05);
    EXPECT_NEAR(tracker.clusters()[0].cy, 5.0, 0.05);
    EXPECT_FALSE(tracker.clusters()[0].confirmed);
    EXPECT_TRUE(result.newly_confirmed.empty());
    EXPECT_TRUE(result.newly_removed.empty());
}

TEST(ClusterTrackerTest, ClusterPromotedAfterConfirmDuration) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 5.0;
    cfg.remove_duration = 10.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 4.0;
    ClusterTracker tracker(cfg);

    auto pts = circlePoints(2.0, 2.0, 0.2);
    // Simulate 6 seconds of observation at 1Hz, mower at origin (within range)
    ClusterTracker::UpdateResult result;
    for (int t = 0; t <= 6; ++t) {
        result = tracker.update(pts, 0.0, 0.0, static_cast<double>(t));
    }

    ASSERT_EQ(tracker.clusters().size(), 1u);
    EXPECT_TRUE(tracker.clusters()[0].confirmed);
    EXPECT_EQ(result.newly_confirmed.size(), 1u);
    EXPECT_NEAR(result.newly_confirmed[0].cx, 2.0, 0.05);
}

TEST(ClusterTrackerTest, ClusterNotPromotedIfBelowObservationRatio) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 5.0;
    cfg.remove_duration = 10.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 4.0;
    ClusterTracker tracker(cfg);

    auto pts = circlePoints(2.0, 2.0, 0.2);
    // Observe every other scan (50% — borderline)
    // 10 scans over 10 seconds, only 4 have points (40% < 50%)
    for (int t = 0; t < 10; ++t) {
        if (t % 3 == 0) {
            // no points — mower in range but doesn't see obstacle
            tracker.update({}, 0.0, 0.0, static_cast<double>(t));
        } else if (t % 3 == 1) {
            tracker.update({}, 0.0, 0.0, static_cast<double>(t));
        } else {
            tracker.update(pts, 0.0, 0.0, static_cast<double>(t));
        }
    }

    ASSERT_EQ(tracker.clusters().size(), 1u);
    // Only 3-4 hits out of 10 in-range scans — below 50%
    EXPECT_FALSE(tracker.clusters()[0].confirmed);
}

TEST(ClusterTrackerTest, ConfirmedClusterRemovedAfterRemoveDuration) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 2.0;
    cfg.remove_duration = 5.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 4.0;
    ClusterTracker tracker(cfg);

    auto pts = circlePoints(2.0, 2.0, 0.2);
    // Confirm the cluster (3 seconds of observation)
    for (int t = 0; t <= 3; ++t) {
        tracker.update(pts, 0.0, 0.0, static_cast<double>(t));
    }
    ASSERT_TRUE(tracker.clusters()[0].confirmed);

    // Now stop observing for 6 seconds, mower still in range
    ClusterTracker::UpdateResult result;
    for (int t = 4; t <= 10; ++t) {
        result = tracker.update({}, 0.0, 0.0, static_cast<double>(t));
    }

    // Should be removed
    EXPECT_EQ(result.newly_removed.size(), 1u);
}

TEST(ClusterTrackerTest, RemovalPausedWhenMowerOutOfRange) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 2.0;
    cfg.remove_duration = 5.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 4.0;
    ClusterTracker tracker(cfg);

    auto pts = circlePoints(2.0, 2.0, 0.2);
    // Confirm
    for (int t = 0; t <= 3; ++t) {
        tracker.update(pts, 0.0, 0.0, static_cast<double>(t));
    }
    ASSERT_TRUE(tracker.clusters()[0].confirmed);

    // Mower moves far away (100m) — removal clock should pause
    for (int t = 4; t <= 20; ++t) {
        tracker.update({}, 100.0, 100.0, static_cast<double>(t));
    }

    // Cluster should still exist (mower was out of range, removal paused)
    ASSERT_EQ(tracker.clusters().size(), 1u);
    EXPECT_TRUE(tracker.clusters()[0].confirmed);
}

TEST(ClusterTrackerTest, ClusterBelowMinRadiusNotPromoted) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.15;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 2.0;
    cfg.remove_duration = 10.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 4.0;
    ClusterTracker tracker(cfg);

    // Single point — radius essentially 0, below min_radius
    std::vector<std::pair<double,double>> pts = {{3.0, 3.0}};
    // Observe for long enough to pass confirm_duration
    ClusterTracker::UpdateResult result;
    for (int t = 0; t <= 5; ++t) {
        result = tracker.update(pts, 0.0, 0.0, static_cast<double>(t));
    }

    // Cluster exists but is NOT promoted (radius < min_radius)
    EXPECT_TRUE(result.newly_confirmed.empty());
    // Cluster tracked but not confirmed
    bool any_confirmed = false;
    for (const auto& c : tracker.clusters()) {
        if (c.confirmed) any_confirmed = true;
    }
    EXPECT_FALSE(any_confirmed);
}

TEST(ClusterTrackerTest, ClusterAboveMaxRadiusDiscarded) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 5.0;  // large merge distance to force one cluster
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 5.0;
    cfg.remove_duration = 10.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 10.0;
    ClusterTracker tracker(cfg);

    // Points spread over 6m — radius ~3m > max 2.0
    auto pts = circlePoints(5.0, 5.0, 3.0);
    tracker.update(pts, 0.0, 0.0, 0.0);

    EXPECT_EQ(tracker.clusters().size(), 0u);
}

TEST(ClusterTrackerTest, TwoClustersTrackedIndependently) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 5.0;
    cfg.remove_duration = 10.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 10.0;
    ClusterTracker tracker(cfg);

    auto pts1 = circlePoints(2.0, 2.0, 0.2);
    auto pts2 = circlePoints(8.0, 8.0, 0.3);
    auto combined = pts1;
    combined.insert(combined.end(), pts2.begin(), pts2.end());

    tracker.update(combined, 5.0, 5.0, 0.0);

    ASSERT_EQ(tracker.clusters().size(), 2u);
}

TEST(ClusterTrackerTest, LoadExistingObstacles) {
    ClusterTracker::Config cfg;
    cfg.cluster_merge_distance = 0.3;
    cfg.min_radius = 0.10;
    cfg.max_radius = 2.0;
    cfg.confirm_duration = 5.0;
    cfg.remove_duration = 10.0;
    cfg.observation_ratio = 0.5;
    cfg.max_observation_range = 4.0;
    ClusterTracker tracker(cfg);

    // Simulate loading a pre-existing obstacle from the map
    ClusterTracker::Cluster existing;
    existing.cx = 3.0;
    existing.cy = 4.0;
    existing.radius = 0.25;
    existing.confirmed = true;
    existing.obstacle_id = "lidar_obs_123";
    tracker.loadExistingObstacles({existing});

    ASSERT_EQ(tracker.clusters().size(), 1u);
    EXPECT_TRUE(tracker.clusters()[0].confirmed);
    EXPECT_EQ(tracker.clusters()[0].obstacle_id, "lidar_obs_123");

    // Observing near the same spot should not create a duplicate
    auto pts = circlePoints(3.0, 4.0, 0.2);
    tracker.update(pts, 0.0, 0.0, 0.0);

    EXPECT_EQ(tracker.clusters().size(), 1u);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
