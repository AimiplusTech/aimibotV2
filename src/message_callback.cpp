#include "iostream"
#include "ros/ros.h"
#include "../include/aimibot/message_callback.hpp"
#include "serial/serial.h"
#include "limits.h"
const double bias=0.365;
const double epsilon = 0.0001;
const int16_t speed_limited=2500;
const int16_t min_speed_limited=10;
uint8_t buffer_send[256];
uint8_t i;
uint8_t cs;
command_id command;
uint8_t header0 = 0xAA;
uint8_t header1 = 0x55;
extern serial::Serial sp;
short bound(const double &value) {
  if (value > static_cast<double>(SHRT_MAX)) return SHRT_MAX;
  if (value < static_cast<double>(SHRT_MIN)) return SHRT_MIN;
  return static_cast<short>(value);
}
void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  // vx: in m/s
  // wz: in rad/s
  int16_t send_speed,send_radius;
  double wz = msg->angular.z;
  double vx = msg->linear.x;
  double radius;
  double speed;
  cs=0;
  // Special Case #1 : Straight Run
  if( std::abs(wz) < epsilon ) {
    radius = 0.0f;
    speed  = 1000.0f * vx;
  }
  else{
  radius = vx * 1000.0f / wz;
  // Special Case #2 : Pure Rotation or Radius is less than or equal to 1.0 mm
  if( std::abs(vx) < epsilon || std::abs(radius) <= 1.0f ) {
    speed  = 1000.0f * bias * wz / 2.0f;
    radius = 1.0f;
  }
  else{
  // General Case :
  if( radius > 0.0f ) {
    speed  = (radius + 1000.0f * bias / 2.0f) * wz;
  } else {
    speed  = (radius - 1000.0f * bias / 2.0f) * wz;
  }
  }
  }
//   ROS_INFO("radius: %f speed: %f ",radius,speed);
  send_speed = (int16_t)bound(speed) ; 
  send_radius= (int16_t)bound(radius);

 if(abs(send_speed)>speed_limited)  
 {
  send_speed= send_speed/abs(send_speed)*speed_limited;
//   ROS_INFO("SEND_SPEED IS OUT OF RANGE!!");
 }
 if(send_radius==1)
 {
    if(abs(send_speed)<min_speed_limited)  
    {
      if((send_speed>-min_speed_limited)&&(send_speed<=0))  send_speed=-min_speed_limited;
      else if((send_speed<min_speed_limited)&&(send_speed>0))  send_speed=min_speed_limited;
    }
 }
  buffer_send[0]=header0; buffer_send[1]=header1;
  buffer_send[2]=0x06;    
  buffer_send[3]=command.base_control;
  buffer_send[4]=command.base_control_lenght;
  
  buffer_send[5]=(send_speed&0xff);
  buffer_send[6]=((send_speed>>8)&0xff);     //speed

  buffer_send[7]=(send_radius&0xff);
  buffer_send[8]=((send_radius>>8)&0xff);    //radius
  for(i=2;i<command.base_control_lenght+5;i++)
    cs^=buffer_send[i];
  buffer_send[9]=cs;
  sp.write(buffer_send,10);
  memset(buffer_send, 0, 256);
}

void subscribeLed1Command(const aimibot::LedConstPtr msg)
{
   cs=0;
  if((msg->value==0x01)||(msg->value==0x02))
  {
  buffer_send[0]=header0; buffer_send[1]=header1;
  buffer_send[2]=0x04;    
  buffer_send[3]=command.gpout_control;
  buffer_send[4]=command.gpout_control_lenght;	 
  buffer_send[5]=0x00; 
  buffer_send[6]=msg->value; 
  for(i=2;i<command.gpout_control_lenght+5;i++)
    cs^=buffer_send[i];
  buffer_send[7]=cs;
  sp.write(buffer_send,8);
  }
  memset(buffer_send, 0, 256);
}

void subscribeLed2Command(const aimibot::LedConstPtr msg)
{
   cs=0;
    if((msg->value==0x04)||(msg->value==0x08))
  {
  buffer_send[0]=header0; buffer_send[1]=header1;
  buffer_send[2]=0x04;    
  buffer_send[3]=command.gpout_control;
  buffer_send[4]=command.gpout_control_lenght;	 
  buffer_send[5]=0x00; 
  buffer_send[6]=msg->value; 
  for(i=0;i<7;i++)
    cs^=buffer_send[i];
  buffer_send[7]=cs;
  sp.write(buffer_send,8);
  }
  memset(buffer_send, 0, 256);
}

void subscribeDigitalOutputCommand(const aimibot::DigitalOutputConstPtr msg)
{
  
}

void subscribeExternalPowerCommand(const aimibot::DigitalOutputConstPtr msg)
{
  
}

void subscribeResetOdometry(const std_msgs::EmptyConstPtr msg)
{
  
}

void subscribeSoundCommand(const aimibot::SoundConstPtr msg)
{
  
}

void subscribeMotorPower(const aimibot::MotorPowerConstPtr msg)
{
  
}

void subscribeControllerInfoCommand(const aimibot::ControllerInfoConstPtr msg)
{
  
}
