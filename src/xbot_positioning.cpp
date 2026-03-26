//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include <cmath>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "SystemModel.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/WheelTick.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/KalmanState.h"
#include "xbot_positioning/SetPoseSrv.h"
#include "xbot_positioning_core.h"

ros::Publisher odometry_pub;
ros::Publisher xbot_absolute_pose_pub;

// Debug Publishers
ros::Publisher kalman_state;
ros::Publisher dbg_expected_motion_vector;

// The kalman filters
xbot::positioning::xbot_positioning_core core{};

// True, if we don't want to do gyro calibration on launch
bool skip_gyro_calibration;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
bool has_gps;
xbot_msgs::AbsolutePose last_gps;

// True, if last_imu is valid and gyro_offset is valid
bool has_gyro;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;

// Current speed calculated by wheel ticks
double vx = 0.0;

// Min speed for motion vector to be fed into kalman filter
double min_speed = 0.0;

// Max position accuracy to allow for GPS updates
double max_gps_accuracy;

// True, if we should publish debug topics (expected motion vector and kalman state)
bool publish_debug;

// Antenna offset (offset between point of rotation and antenna)
double antenna_offset_x, antenna_offset_y;

nav_msgs::Odometry odometry;
xbot_positioning::KalmanState state_msg;
xbot_msgs::AbsolutePose xb_absolute_pose_msg;

bool gps_enabled = true;
int gps_outlier_count = 0;
int valid_gps_samples = 0;
int gps_message_throttle = 1;

ros::Time last_gps_time(0.0);

// TF broadcasting (disable when SLAM provides map->base_link chain)
bool publish_tf = true;

// LiDAR odometry integration
bool enable_lidar_odom = false;
ros::Time last_lidar_odom_time(0.0);
double lidar_timeout = 3.0;
double gps_float_covariance = 50000.0;

bool isLidarActive() {
    return enable_lidar_odom && (ros::Time::now() - last_lidar_odom_time).toSec() < lidar_timeout;
}

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if (!has_gyro) {
        if (!skip_gyro_calibration) {
            if (gyro_offset_samples == 0) {
                ROS_INFO_STREAM("Started gyro calibration");
                gyro_calibration_start = msg->header.stamp;
                gyro_offset = 0;
            }
            gyro_offset += msg->angular_velocity.z;
            gyro_offset_samples++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (gyro_offset_samples > 0) {
                gyro_offset /= gyro_offset_samples;
            } else {
                gyro_offset = 0;
            }
            gyro_offset_samples = 0;
            ROS_INFO_STREAM("Calibrated gyro offset: " << gyro_offset);
        } else {
            ROS_WARN("Skipped gyro calibration");
            has_gyro = true;
            return;
        }
    }

    core.predict(vx, msg->angular_velocity.z - gyro_offset, (msg->header.stamp - last_imu.header.stamp).toSec());
    auto x = core.updateSpeed(vx, msg->angular_velocity.z - gyro_offset, 0.01);

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    // Fill odometry covariance from EKF (6x6: x, y, z, roll, pitch, yaw)
    {
        auto P = core.getCovariance();
        odometry.pose.covariance.fill(0);
        odometry.pose.covariance[0]  = P(0, 0); // x-x
        odometry.pose.covariance[1]  = P(0, 1); // x-y
        odometry.pose.covariance[5]  = P(0, 2); // x-yaw
        odometry.pose.covariance[6]  = P(1, 0); // y-x
        odometry.pose.covariance[7]  = P(1, 1); // y-y
        odometry.pose.covariance[11] = P(1, 2); // y-yaw
        odometry.pose.covariance[30] = P(2, 0); // yaw-x
        odometry.pose.covariance[31] = P(2, 1); // yaw-y
        odometry.pose.covariance[35] = P(2, 2); // yaw-yaw
    }

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odometry.header;
    odom_trans.child_frame_id = odometry.child_frame_id;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;

    if (publish_tf) {
        static tf2_ros::TransformBroadcaster transform_broadcaster;
        transform_broadcaster.sendTransform(odom_trans);
    }

    if (publish_debug) {
        auto state = core.getState();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.theta = state.theta();
        state_msg.vx = state.vx();
        state_msg.vr = state.vr();

        kalman_state.publish(state_msg);
    }

    odometry_pub.publish(odometry);

    xb_absolute_pose_msg.header = odometry.header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg.motion_vector_valid = false;

    // Position accuracy: pass through GPS receiver accuracy
    if (has_gps) {
        xb_absolute_pose_msg.position_accuracy = last_gps.position_accuracy;
    } else {
        xb_absolute_pose_msg.position_accuracy = 999;
    }

    // Flag: recent absolute pose if GPS or LiDAR is providing corrections
    if ((ros::Time::now() - last_gps_time).toSec() < 10.0) {
        xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else {
        xb_absolute_pose_msg.position_accuracy = 999;
    }

    // When LiDAR is actively providing corrections, derive accuracy from
    // EKF covariance ratio: how much has the fused uncertainty improved
    // relative to the prediction-only baseline.
    // This reflects real LiDAR quality: good SLAM match → low ratio → good accuracy,
    // poor match (rejected updates) → ratio stays near 1 → GPS accuracy unchanged.
    if (isLidarActive()) {
        xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;

        // EKF covariance reflects fusion quality in internal units.
        // Normalize: GPS-only steady-state variance is ~gps_cov (measurement noise
        // dominates). With LiDAR fused, variance drops proportionally.
        // Map the ratio to GPS receiver's accuracy scale.
        auto P = core.getCovariance();
        double var_xy = P(xbot::positioning::State<double>::X, xbot::positioning::State<double>::X)
                      + P(xbot::positioning::State<double>::Y, xbot::positioning::State<double>::Y);
        // Baseline: GPS-only variance at current quality level
        double baseline_var = has_gps ? gps_float_covariance : 50000.0;
        if (has_gps && (last_gps.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED)) {
            baseline_var = 500.0;
        }
        // Ratio < 1 means LiDAR is helping; ratio >= 1 means no improvement
        double ratio = std::min(1.0, var_xy / (2.0 * baseline_var));
        // Map to accuracy: GPS accuracy at ratio=1, best possible (0.02m) at ratio→0
        double gps_acc = has_gps ? (double)last_gps.position_accuracy : 0.5;
        double lidar_best = 0.02; // best achievable with good SLAM
        xb_absolute_pose_msg.position_accuracy = lidar_best + ratio * (gps_acc - lidar_best);
    }

    // Orientation accuracy from EKF covariance
    {
        auto P = core.getCovariance();
        double var_theta = P(xbot::positioning::State<double>::THETA, xbot::positioning::State<double>::THETA);
        xb_absolute_pose_msg.orientation_accuracy = std::sqrt(var_theta);
    }
    xb_absolute_pose_msg.pose = odometry.pose;
    xb_absolute_pose_msg.vehicle_heading = x.theta();
    xb_absolute_pose_msg.motion_heading = x.theta();

    // Only emit position when accuracy is within 10cm
    if (xb_absolute_pose_msg.position_accuracy <= 0.1) {
        xbot_absolute_pose_pub.publish(xb_absolute_pose_msg);
    }


    last_imu = *msg;
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if (!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (
                           1 / (double) msg->wheel_tick_factor);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (
                           1 / (double) msg->wheel_tick_factor);

    if (msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if (msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }


    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;

    last_ticks = *msg;
}

void onTwistIn(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    vx = msg->twist.linear.x;
}

void onLidarOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    last_lidar_odom_time = ros::Time::now();

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double cov_x = msg->pose.covariance[0];
    double cov_y = msg->pose.covariance[7];
    double cov_theta = msg->pose.covariance[35];

    if (cov_x <= 0) cov_x = 1.0;
    if (cov_y <= 0) cov_y = 1.0;
    if (cov_theta <= 0) cov_theta = 1.0;

    core.updateLidarOdometry(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw,
                             cov_x, cov_y, cov_theta);

    ROS_INFO_STREAM_THROTTLE(5, "LiDAR odometry update applied. Covariance: "
        << cov_x << ", " << cov_y << ", " << cov_theta);
}

bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    tf2::Quaternion q;
    tf2::fromMsg(req.robot_pose.orientation, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw, 0, 0);
    return true;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if (!gps_enabled) {
        ROS_INFO_STREAM_THROTTLE(gps_message_throttle, "dropping GPS update, since gps_enabled = false.");
        return;
    }
    bool is_rtk_fixed = (msg->flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
    if (!is_rtk_fixed && !isLidarActive()) {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not RTK Fixed and LiDAR is not active");
        return;
    }

    if (msg->position_accuracy > max_gps_accuracy && !isLidarActive()) {
        ROS_INFO_STREAM_THROTTLE(
            1, "Dropped GPS update, since it's not accurate enough. Accuracy was: " << msg->position_accuracy <<
            ", limit is:" << max_gps_accuracy);
        return;
    }

    double time_since_last_gps = (ros::Time::now() - last_gps_time).toSec();
    if (time_since_last_gps > 5.0) {
        ROS_WARN_STREAM("Last GPS was " << time_since_last_gps << " seconds ago.");
        has_gps = false;
        valid_gps_samples = 0;
        gps_outlier_count = 0;
        last_gps = *msg;
        // we have GPS for next time
        last_gps_time = ros::Time::now();
        return;
    }

    tf2::Vector3 gps_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x, last_gps.pose.pose.position.y,
                              last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        if (!has_gps && valid_gps_samples > 10) {
            ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM(
                "First GPS data, moving kalman filter to " << msg->pose.pose.position.x << ", " << msg->pose.pose.
                position.y);
            // we don't even have gps yet, set odometry to first estimate
            double init_cov = is_rtk_fixed ? 0.001 : gps_float_covariance;
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, init_cov);

            has_gps = true;
        } else if (has_gps) {
            // gps was valid before, we apply the filter
            double gps_cov;
            if (is_rtk_fixed) {
                gps_cov = 500.0;
            } else if (isLidarActive()) {
                // LiDAR is providing corrections — use moderate Float covariance
                // so EKF blends GPS Float + LiDAR instead of ignoring GPS entirely
                gps_cov = 5000.0;
            } else {
                gps_cov = gps_float_covariance;
            }
            core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, gps_cov);
            if (!is_rtk_fixed) {
                ROS_INFO_STREAM_THROTTLE(1, "GPS float update applied with covariance: " << gps_cov);
            }
            if (publish_debug) {
                auto m = core.om2.h(core.ekf.getState());
                geometry_msgs::Vector3 dbg;
                dbg.x = m.vx();
                dbg.y = m.vy();
                dbg_expected_motion_vector.publish(dbg);
            }
            if (std::sqrt(std::pow(msg->motion_vector.x, 2) + std::pow(msg->motion_vector.y, 2)) >= min_speed) {
                core.updateOrientation2(msg->motion_vector.x, msg->motion_vector.y, 10000.0);
            }
        }
    } else {
        ROS_WARN_STREAM("GPS outlier found. Distance was: " << distance_to_last_gps);
        gps_outlier_count++;
        // ~10 sec
        if (gps_outlier_count > 10) {
            ROS_ERROR_STREAM("too many outliers, assuming that the current gps value is valid.");
            // store the gps as last
            last_gps = *msg;
            last_gps_time = ros::Time::now();
            has_gps = false;

            valid_gps_samples = 0;
            gps_outlier_count = 0;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_positioning");

    has_gps = false;
    gps_enabled = true;
    vx = 0.0;
    has_gyro = false;
    has_ticks = false;
    gyro_offset = 0;
    gyro_offset_samples = 0;

    valid_gps_samples = 0;
    gps_outlier_count = 0;

    antenna_offset_x = antenna_offset_y = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::ServiceServer gps_service = n.advertiseService("xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("xbot_positioning/set_robot_pose", setPose);

    paramNh.param("skip_gyro_calibration", skip_gyro_calibration, false);
    paramNh.param("gyro_offset", gyro_offset, 0.0);
    paramNh.param("min_speed", min_speed, 0.01);
    paramNh.param("max_gps_accuracy", max_gps_accuracy, 0.1);
    paramNh.param("debug", publish_debug, false);
    paramNh.param("antenna_offset_x", antenna_offset_x, 0.0);
    paramNh.param("antenna_offset_y", antenna_offset_y, 0.0);
    paramNh.param("gps_message_throttle", gps_message_throttle, 1);
    paramNh.param("enable_lidar_odom", enable_lidar_odom, false);
    paramNh.param("publish_tf", publish_tf, true);
    paramNh.param("gps_float_covariance", gps_float_covariance, 50000.0);
    paramNh.param("lidar_timeout", lidar_timeout, 3.0);
    std::string lidar_odom_topic;
    paramNh.param<std::string>("lidar_odom_topic", lidar_odom_topic, "odom_lidar");

    if (enable_lidar_odom) {
        ROS_INFO_STREAM("LiDAR odometry fusion enabled on topic: " << lidar_odom_topic);
    }

    core.setAntennaOffset(antenna_offset_x, antenna_offset_y);
    ROS_INFO_STREAM("Antenna offset: " << antenna_offset_x << ", " << antenna_offset_y);

    double lidar_offset_x = 0.0, lidar_offset_y = 0.0;
    paramNh.param("lidar_offset_x", lidar_offset_x, 0.0);
    paramNh.param("lidar_offset_y", lidar_offset_y, 0.0);
    core.setLidarOffset(lidar_offset_x, lidar_offset_y);
    ROS_INFO_STREAM("LiDAR offset: " << lidar_offset_x << ", " << lidar_offset_y);

    if (gyro_offset != 0.0 && skip_gyro_calibration) {
        ROS_WARN_STREAM("Using gyro offset of: " << gyro_offset);
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom_out", 50);
    xbot_absolute_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
    if (publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);
    }

    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber twist_sub = paramNh.subscribe("twist_in", 10, onTwistIn);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose_in", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);
    ros::Subscriber lidar_odom_sub;
    if (enable_lidar_odom) {
        lidar_odom_sub = paramNh.subscribe(lidar_odom_topic, 10, onLidarOdom);
    }

    ros::spin();
    return 0;
}
