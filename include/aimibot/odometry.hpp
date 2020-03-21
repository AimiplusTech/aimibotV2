#ifndef __ODOMETRY_HPP__
#define __ODOMETRY_HPP__

#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ecl/geometry/legacy_pose2d.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Aimi {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief  Odometry for the kobuki node.
 **/
class Odometry {
public:
  Odometry();
  void init(ros::NodeHandle& nh,ros::NodeHandle& nh_p, const std::string& name);
  bool commandTimeout() const;
  void update(const ecl::LegacyPose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
              double imu_heading, double imu_angular_velocity);
  void resetOdometry() { pose.setIdentity(); }
  const ros::Duration& timeout() const { return cmd_vel_timeout; }
  void resetTimeout() { last_cmd_time = ros::Time::now(); }

private:
  geometry_msgs::TransformStamped odom_trans;
  ecl::LegacyPose2D<double> pose;
  std::string odom_frame;
  std::string base_frame;
  ros::Duration cmd_vel_timeout;
  ros::Time last_cmd_time;
  bool publish_tf;
  bool use_imu_heading;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_publisher;

  void publishTransform(const geometry_msgs::Quaternion &odom_quat);
  void publishOdometry(const geometry_msgs::Quaternion &odom_quat, const ecl::linear_algebra::Vector3d &pose_update_rates);
};
} 
#endif
