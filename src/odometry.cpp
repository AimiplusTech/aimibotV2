#include "../include/aimibot/odometry.hpp"

namespace Aimi {

/*****************************************************************************
** Implementation
*****************************************************************************/

Odometry::Odometry():
  odom_frame("odom"),
  base_frame("base_footprint"),
  use_imu_heading(true),
  publish_tf(true)
{};

void Odometry::init(ros::NodeHandle& nh,ros::NodeHandle& nh_p, const std::string& name) {
  double timeout;
  nh_p.param("cmd_vel_timeout", timeout, 0.6);
  cmd_vel_timeout.fromSec(timeout);
  ROS_INFO_STREAM("Aimibot : Velocity commands timeout: " << cmd_vel_timeout << " seconds [" << name << "].");

  if (!nh_p.getParam("odom_frame", odom_frame)) {
    ROS_WARN_STREAM("Aimibot : no param server setting for odom_frame, using default [" << odom_frame << "][" << name << "].");
  } else {
    ROS_INFO_STREAM("Aimibot : using odom_frame [" << odom_frame << "][" << name << "].");
  }

  if (!nh_p.getParam("base_frame", base_frame)) {
    ROS_WARN_STREAM("Aimibot : no param server setting for base_frame, using default [" << base_frame << "][" << name << "].");
  } else {
    ROS_INFO_STREAM("Aimibot : using base_frame [" << base_frame << "][" << name << "].");
  }

  if (!nh_p.getParam("publish_tf", publish_tf)) {
    ROS_WARN_STREAM("Aimibot : no param server setting for publish_tf, using default [" << publish_tf << "][" << name << "].");
  } else {
    if ( publish_tf ) {
      ROS_INFO_STREAM("Aimibot : publishing transforms [" << name << "].");
    } else {
      ROS_INFO_STREAM("Aimibot : not publishing transforms (see robot_pose_ekf) [" << name << "].");
    }
  }

  if (!nh_p.getParam("use_imu_heading", use_imu_heading)) {
    ROS_WARN_STREAM("Aimibot : no param server setting for use_imu_heading, using default [" << use_imu_heading << "][" << name << "].");
  } else {
    if ( use_imu_heading ) {
      ROS_INFO_STREAM("Aimibot : using imu data for heading [" << name << "].");
    } else {
      ROS_INFO_STREAM("Aimibot : using encoders for heading (see robot_pose_ekf) [" << name << "].");
    }
  }

  odom_trans.header.frame_id = odom_frame;
  odom_trans.child_frame_id = base_frame;

  pose.setIdentity();

  odom_publisher = nh.advertise<nav_msgs::Odometry>("/odom", 50); // topic name and queue size
}

bool Odometry::commandTimeout() const {
  if ( (!last_cmd_time.isZero()) && ((ros::Time::now() - last_cmd_time) > cmd_vel_timeout) ) {
    return true;
  } else {
    return false;
  }
}

void Odometry::update(const ecl::LegacyPose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
                      double imu_heading, double imu_angular_velocity) {
  pose *= pose_update;

  if (use_imu_heading == true) {
    // Overwite with gyro heading data
    pose.heading(imu_heading);
    pose_update_rates[2] = imu_angular_velocity;
  }

  //since all ros tf odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.heading());

  if ( ros::ok() ) {
    publishTransform(odom_quat);
    publishOdometry(odom_quat, pose_update_rates);
  }
}

/*****************************************************************************
** Private Implementation
*****************************************************************************/

void Odometry::publishTransform(const geometry_msgs::Quaternion &odom_quat)
{
  if (publish_tf == false)
    return;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = pose.x();
  odom_trans.transform.translation.y = pose.y();
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);
}

void Odometry::publishOdometry(const geometry_msgs::Quaternion &odom_quat,
                               const ecl::linear_algebra::Vector3d &pose_update_rates)
{
  // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

  // Header
  odom->header.stamp = ros::Time::now();
  odom->header.frame_id = odom_frame;
  odom->child_frame_id = base_frame;

  // Position
  odom->pose.pose.position.x = pose.x();
  odom->pose.pose.position.y = pose.y();
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation = odom_quat;

  // Velocity
  odom->twist.twist.linear.x = pose_update_rates[0];
  odom->twist.twist.linear.y = pose_update_rates[1];
  odom->twist.twist.angular.z = pose_update_rates[2];

  // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
  // Odometry yaw covariance must be much bigger than the covariance provided
  // by the imu, as the later takes much better measures
  odom->pose.covariance[0]  = 0.1;
  odom->pose.covariance[7]  = 0.1;
  odom->pose.covariance[35] = use_imu_heading ? 0.05 : 0.2;

  odom->pose.covariance[14] = 1e10; // set a non-zero covariance on unused
  odom->pose.covariance[21] = 1e10; // dimensions (z, pitch and roll); this
  odom->pose.covariance[28] = 1e10; // is a requirement of robot_pose_ekf

  odom_publisher.publish(odom);
}

}
