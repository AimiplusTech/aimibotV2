#ifndef __MESSAGE_CALLBACK__H
#define __MESSAGE_CALLBACK__H

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "data_struct.hpp"
#include "std_msgs/Empty.h"

struct command_id
{
  uint8_t base_control=0x01;uint8_t base_control_lenght=0x04;
  uint8_t sound_control=0x03;uint8_t sound_control_lenght=0x03;
  uint8_t power_control=0x08;uint8_t power_control_lenght=0x02;
  uint8_t requestex_control=0x09;uint8_t requestex_control_lenght=0x02;
  uint8_t gpout_control=0x0c;uint8_t gpout_control_lenght=0x02;
};
void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg);
void subscribeLed1Command(const aimibot::LedConstPtr msg);
void subscribeLed2Command(const aimibot::LedConstPtr msg);
void subscribeDigitalOutputCommand(const aimibot::DigitalOutputConstPtr msg);
void subscribeExternalPowerCommand(const aimibot::DigitalOutputConstPtr msg);
void subscribeResetOdometry(const std_msgs::EmptyConstPtr msg);
void subscribeSoundCommand(const aimibot::SoundConstPtr msg);
void subscribeMotorPower(const aimibot::MotorPowerConstPtr msg);
void subscribeControllerInfoCommand(const aimibot::ControllerInfoConstPtr msg);


#endif