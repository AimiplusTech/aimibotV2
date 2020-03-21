#ifndef __DIFF_DRIVER_HPP__
#define __DIFF_DRIVER_HPP__

#include <vector>
#include <climits>
#include <stdint.h>
#include <ecl/geometry/legacy_pose2d.hpp>
#include <ecl/mobile_robot.hpp>
#include <ecl/config/macros.hpp>
#include <ecl/config/ecl.hpp>
#include <ecl/threads/mutex.hpp>
#include <ros/ros.h>
namespace Aimi{
  
class  DiffDriver
{
public:
  DiffDriver();
  const ecl::DifferentialDrive::Kinematics& kinematics() { return diff_drive_kinematics; }
  void update(const uint16_t &time_stamp,
              const uint16_t &left_encoder,
              const uint16_t &right_encoder,
              ecl::LegacyPose2D<double> &pose_update,
              ecl::linear_algebra::Vector3d &pose_update_rates);
  void reset();
  void getWheelJointStates(double &wheel_left_angle, double &wheel_left_angle_rate,
                           double &wheel_right_angle, double &wheel_right_angle_rate);
  void setVelocityCommands(const double &vx, const double &wz);
  void velocityCommands(const double &vx, const double &wz);
  void velocityCommands(const short &cmd_speed, const short &cmd_radius);
  void velocityCommands(const std::vector<double> &cmd) { velocityCommands(cmd[0], cmd[1]); }
  void velocityCommands(const std::vector<short>  &cmd) { velocityCommands(cmd[0], cmd[1]); }

  /*********************
  ** Command Accessors
  **********************/
  std::vector<short> velocityCommands(); // (speed, radius), in [mm/s] and [mm]
  std::vector<double> pointVelocity() const; // (vx, wz), in [m/s] and [rad/s]

  /*********************
  ** Property Accessors
  **********************/
  double wheel_bias() const { return bias; }
private:
  unsigned short last_timestamp;
  double last_velocity_left, last_velocity_right;
  double last_diff_time;

  unsigned short last_tick_left, last_tick_right;
  double last_rad_left, last_rad_right;

  //double v, w; // in [m/s] and [rad/s]
  std::vector<double> point_velocity; // (vx, wz), in [m/s] and [rad/s]
  double radius; // in [mm]
  double speed;  // in [mm/s]
  double bias; //wheelbase, wheel_to_wheel, in [m]
  double wheel_radius; // in [m]
  int imu_heading_offset;
  const double tick_to_rad;

  ecl::DifferentialDrive::Kinematics diff_drive_kinematics;
  ecl::Mutex velocity_mutex, state_mutex;
  // Utility
  short bound(const double &value);
};
 
}
#endif