//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "SystemModel.hpp"

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "xbot_msgs/AbsolutePose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "xbot_positioning_core.h"
#include "xbot_msgs/WheelTick.h"
#include "xbot_positioning/KalmanState.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"

ros::Publisher odometry_pub;
ros::Publisher fix_pub;
ros::Publisher imu_pub;
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
sensor_msgs::NavSatFix last_fix;

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
int gps_message_throttle=1;

ros::Time last_gps_time(0.0);


void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if(!skip_gyro_calibration) {
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
    auto x = core.updateSpeed(vx, msg->angular_velocity.z - gyro_offset,0.01);

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    if(publish_debug) {
        auto state = core.getState();
        state_msg.x = state.x();
        state_msg.y = state.y();
        state_msg.theta = state.theta();
        state_msg.vx = state.vx();
        state_msg.vr = state.vr();

        kalman_state.publish(state_msg);
    }

    odometry_pub.publish(odometry);

    last_imu = *msg;
    // Publish calibrated imu to imu_out
    sensor_msgs::Imu imu_out = *msg;
    // TODO(damonkohler): This relies on the z-axis alignment of the
    // IMU with the Kobuki base.
    imu_out.angular_velocity.z = msg->angular_velocity.z - gyro_offset;
    imu_out.linear_acceleration.x = 0.;
    imu_out.linear_acceleration.y = 0.;
    imu_out.linear_acceleration.z = 9.8;
    imu_pub.publish(imu_out);
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1/(double)msg->wheel_tick_factor);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1/(double)msg->wheel_tick_factor);

    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }


    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;
    if(abs(vx) > 0.6) {
        ROS_WARN_STREAM("got vx > 0.6 (" << vx << ") - dropping measurement");
        vx = 0.0;
        return;
    }

    last_ticks = *msg;
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
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw,0,0);
    return true;
}

void onTrackedPose(const geometry_msgs::PoseStamped::ConstPtr &pose_stamped) {
    xb_absolute_pose_msg.header = pose_stamped->header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;

    xb_absolute_pose_msg.orientation_valid = true;
    // TODO: send motion vector
    xb_absolute_pose_msg.motion_vector_valid = false;
    // TODO: set real value from kalman filter, not the one from the GPS.
    if(has_gps) {
        xb_absolute_pose_msg.position_accuracy = last_gps.position_accuracy;
    } else {
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    if((ros::Time::now() - last_gps_time).toSec() < 10.0) {
        xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    } else {
        // on GPS timeout, we set accuracy to 0.
        xb_absolute_pose_msg.position_accuracy = 999;
    }
    // TODO: set real value
    xb_absolute_pose_msg.orientation_accuracy = 0.01;
    geometry_msgs::PoseWithCovariance pose_with_covariance;
    pose_with_covariance.pose = pose_stamped->pose;
    xb_absolute_pose_msg.pose = pose_with_covariance;
    xb_absolute_pose_msg.vehicle_heading = pose_stamped->pose.orientation.z;
    xb_absolute_pose_msg.motion_heading = pose_stamped->pose.orientation.z;

    core.updatePosition(pose_stamped->pose.position.x, pose_stamped->pose.position.y, 500.0);
    xbot_absolute_pose_pub.publish(xb_absolute_pose_msg);
}

// Constants for WGS84
const double a = 6378137.0; // semi-major axis in meters
const double b = 6356752.314245; // semi-minor axis in meters
const double e_sq = (a * a - b * b) / (a * a); // square of eccentricity
const double e_prime_sq = (a * a - b * b) / (b * b); // secondary eccentricity

void geodeticToECEF(double lat, double lon, double alt, double &X, double &Y, double &Z) {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / sqrt(1 - e_sq * pow(sin(lat_rad), 2));

    X = (N + alt) * cos(lat_rad) * cos(lon_rad);
    Y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    Z = (N * (1 - e_sq) + alt) * sin(lat_rad);
}

void ecefToGeodetic(double X, double Y, double Z, double &lat, double &lon) {
    double p = sqrt(X * X + Y * Y);
    double theta = atan2(Z * a, p * b);

    lon = atan2(Y, X);
    lat = atan2(Z + e_prime_sq * b * pow(sin(theta), 3), p - e_sq * a * pow(cos(theta), 3));

    lat = lat * 180.0 / M_PI;
    lon = lon * 180.0 / M_PI;
}

void onPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
// Datum point (latitude, longitude, altitude)
    double datum_lat = 48.8831951; // example latitude
    double datum_lon = 2.1661984; // example longitude
    double datum_alt = 0; // example altitude

    // Convert datum point to ECEF
    double datum_x, datum_y, datum_z;
    geodeticToECEF(datum_lat, datum_lon, datum_alt, datum_x, datum_y, datum_z);

    // Relative position in meters (x, y, z)
    double rel_x = msg->pose.pose.position.x;
    double rel_y = msg->pose.pose.position.y;
    double rel_z = msg->pose.pose.position.z;

    // New ECEF coordinates
    double new_x = datum_x + rel_x;
    double new_y = datum_y + rel_y;
    double new_z = datum_z + rel_z;

    // Convert new ECEF coordinates back to geodetic coordinates
    double new_lat, new_lon;
    ecefToGeodetic(new_x, new_y, new_z, new_lat, new_lon);
   /* if(!gps_enabled) {
        ROS_INFO_STREAM_THROTTLE(gps_message_throttle, "dropping GPS update, since gps_enabled = false.");
        return;
    }*/
    last_fix.header.stamp = ros::Time::now();
    last_fix.header.seq++;
    last_fix.header.frame_id = "base_link";
    last_fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    if((msg->flags & (xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED)) == 0) {
        last_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
    last_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GALILEO;

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

    tf2::Vector3 gps_pos(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    tf2::Vector3 last_gps_pos(last_gps.pose.pose.position.x,last_gps.pose.pose.position.y,last_gps.pose.pose.position.z);

    double distance_to_last_gps = (last_gps_pos - gps_pos).length();

    if (distance_to_last_gps < 5.0) {
        // inlier, we treat it normally
        // store the gps as last
        last_gps = *msg;
        last_gps_time = ros::Time::now();

        gps_outlier_count = 0;
        valid_gps_samples++;

        last_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        if (!has_gps && valid_gps_samples > 10) {
            ROS_INFO_STREAM("GPS data now valid");
            ROS_INFO_STREAM("First GPS data, moving kalman filter to " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y);
            last_fix.longitude = new_lon;
            last_fix.latitude = new_lat;
            last_fix.altitude = msg->pose.pose.position.z;
            last_fix.position_covariance[0] = msg->position_accuracy * msg->position_accuracy;
            last_fix.position_covariance[4] = msg->position_accuracy * msg->position_accuracy;
            last_fix.position_covariance[8] = msg->position_accuracy * msg->position_accuracy;
            has_gps = true;
        } else if (has_gps) {
            last_fix.longitude = new_lon;
            last_fix.latitude = new_lat;
            last_fix.altitude = msg->pose.pose.position.z;
            last_fix.position_covariance[0] = msg->position_accuracy * msg->position_accuracy;
            last_fix.position_covariance[4] = msg->position_accuracy * msg->position_accuracy;
            last_fix.position_covariance[8] = msg->position_accuracy * msg->position_accuracy;
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
    fix_pub.publish(last_fix);
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

    core.setAntennaOffset(antenna_offset_x, antenna_offset_y);

    ROS_INFO_STREAM("Antenna offset: " << antenna_offset_x << ", " << antenna_offset_y);

    if(gyro_offset != 0.0 && skip_gyro_calibration) {
        ROS_WARN_STREAM("Using gyro offset of: " << gyro_offset);
    }

    odometry_pub = paramNh.advertise<nav_msgs::Odometry>("odom_out", 50);
    imu_pub = paramNh.advertise<sensor_msgs::Imu>("imu_out", 50);
    fix_pub = paramNh.advertise<sensor_msgs::NavSatFix>("fix_out", 50);
    xbot_absolute_pose_pub = paramNh.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
    if(publish_debug) {
        dbg_expected_motion_vector = paramNh.advertise<geometry_msgs::Vector3>("debug_expected_motion_vector", 50);
        kalman_state = paramNh.advertise<xbot_positioning::KalmanState>("kalman_state", 50);
    }

    ros::Subscriber tracked_pose_sub = paramNh.subscribe("/tracked_pose", 10, onTrackedPose);
    ros::Subscriber imu_sub = paramNh.subscribe("imu_in", 10, onImu);
    ros::Subscriber pose_sub = paramNh.subscribe("xb_pose_in", 10, onPose);
    ros::Subscriber wheel_tick_sub = paramNh.subscribe("wheel_ticks_in", 10, onWheelTicks);

    ros::spin();
    return 0;
}
