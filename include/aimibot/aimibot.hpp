#ifndef __AIMIBOT__H
#define __AIMIBOT__H

#include <ros/ros.h> 
#include "rosconsole/macros_generated.h"
#include "serial/serial.h"
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include "message_callback.hpp"
#include <ecl/math.hpp>
#include <ecl/geometry/angle.hpp>
#include <sensor_msgs/JointState.h>
#include "diff_driver.hpp"
#include "odometry.hpp"
#include "math.h"
#define pi 3.1415926


Aimi::DiffDriver diff_drive;

uint8_t ready_get=1;   //defalut for read header[0] header[1]
bool findheader0=false;
bool findheader1=false;
static uint8_t hasread=0;
uint8_t payload_size=0;
double heading_offset=0.0;

ros::Publisher  version_info_publisher,
		joint_state_publisher,
		dock_ir_publisher,
		inertia_publisher,
		raw_imu_data_publisher,
		unique_id_publisher,
		current_publisher,
		gpinput_publisher,
		cliff_state_publisher,
		controller_info_publisher,
		ultrasonic_data_publisher;
		
ros::Subscriber
		velocity_command_subscriber,
		led1_command_subscriber,
		led2_command_subscriber,
		digital_output_command_subscriber,
		external_power_command_subscriber,
		sound_command_subscriber,
		reset_odometry_subscriber,
		motor_power_subscriber,
		controller_info_command_subscriber;


aimibot::Cliff cliff_pubdata;
aimibot::CoreSensors coresense_pubdata;
aimibot::hardware hardware_pubdata;
aimibot::ControllerInfo controlinfo_pubdata;
aimibot::DockInfraRed dockin_pubdata;
aimibot::Inertia iner_pubdata;
aimibot::ThreeAxisGyro threeag_pubdata;
aimibot::UniqueDeviceID uniid_pubdata;
aimibot::Current cur_pubdata;
aimibot::GpInput gpin_pubdata;
aimibot::Ultrasonic ultrasonic_pubdata;

CoreSensors_data coresensors_data;
ControllerInfo_data controlinfo_data;
UniqueDeviceID_data uniqid_data;
GpInput_data gpin_data;
ThreeAxisGyro_data threeAG_data;
Firmware_data firmware_Data;
Hardware_data hardware_data;
Current_data current_data;
Cliff_data cliff_Data;
Inertia_data iner_data;
DockInfraRed_data dockin_data;
Ultrasonic ultrasonic_data;
/**初始化 机器人头部朝向***/
double init_head()
{
  return  (static_cast<double>(iner_data.angle) / 100.0) * pi / 180.0;
}
/**返回 机器人偏航角***/
ecl::Angle<double> getHeading() 
{
  ecl::Angle<double> heading;
  // raw data angles are in hundredths of a degree, convert to radians.
  heading = (static_cast<double>(iner_data.angle) / 100.0) * pi / 180.0;
  return ecl::wrap_angle(heading - heading_offset);
}
void reset_odometry()
{
  diff_drive.reset();
  heading_offset = (static_cast<double>(iner_data.angle) / 100.0) * pi / 180.0; 
}
double getAngularVelocity() 
{
  // raw data angles are in hundredths of a degree, convert to radians.
  return (static_cast<double>(iner_data.angle_rate) / 100.0) * pi / 180.0;
}

void rostopic_pub(CoreSensors_data coresensors_data_,
		  Ultrasonic ultrasonic_data_,
		//  UniqueDeviceID_data uniqid_data_,
		  GpInput_data gpin_data_,
		  ThreeAxisGyro_data threeAG_data_,
		  Firmware_data firmware_Data_,
		  Hardware_data hardware_data_,
		  Current_data current_data_,
		  Cliff_data cliff_Data_,
		  Inertia_data iner_data_,
		  DockInfraRed_data dockin_data_)
{
	  int k;
	  short int angle_temp , anglerate_temp;
	  ultrasonic_pubdata.disl1=ultrasonic_data.DISL1;
	  ultrasonic_pubdata.disl2=ultrasonic_data.DISL2;
	  ultrasonic_pubdata.disl3=ultrasonic_data.DISL3;
	  ultrasonic_pubdata.disl4=ultrasonic_data.DISL4;
	  ultrasonic_pubdata.disl5=ultrasonic_data.DISL5;
	  cliff_pubdata.bottom[0] = cliff_Data_.bottom[0];
	  cliff_pubdata.bottom[1] =  cliff_Data_.bottom[1];
	  cliff_pubdata.bottom[2] =  cliff_Data_.bottom[2];

	  coresense_pubdata.time_stamp= coresensors_data_.time_stamp; 
	  coresense_pubdata.bumper =coresensors_data_.bumper ;
	  coresense_pubdata.wheel_drop=coresensors_data_.wheel_drop;
	  coresense_pubdata.cliff =coresensors_data_.cliff;
	  coresense_pubdata.left_encoder =coresensors_data_.left_encoder ; 
	  coresense_pubdata.right_encoder= coresensors_data_.right_encoder; 
	  coresense_pubdata.left_pwm =coresensors_data_.left_pwm;
	  coresense_pubdata.right_pwm =coresensors_data_.right_pwm;
	  coresense_pubdata.buttons =coresensors_data_.buttons;
	  coresense_pubdata.charger =coresensors_data_.charger;
	  coresense_pubdata.battery =0.12*coresensors_data_.battery;
	  coresense_pubdata.over_current =coresensors_data_.over_current;


	  hardware_pubdata.hw_major = firmware_Data_.version.major;
	  hardware_pubdata.hw_minor = firmware_Data_.version.minor;
	  hardware_pubdata.hw_patch = firmware_Data_.version.patch;
	  // controlinfo_pubdata.p_gain
	  // controlinfo_pubdata.i_gain
	  // controlinfo_pubdata.d_gain
	  // controlinfo_pubdata.type
	  dockin_pubdata.Docking[0]=dockin_data_.Docking[0];
	  dockin_pubdata.Docking[1]=dockin_data_.Docking[1];
	  dockin_pubdata.Docking[2]=dockin_data_.Docking[2];
	  iner_pubdata.angle= getHeading();
	  iner_pubdata.angle_rate= (iner_data_.angle_rate / 100.0)*pi / 180.0;
	  iner_pubdata.acc[0]== iner_data_.acc[0];
	  iner_pubdata.acc[1]== iner_data_.acc[1];
	  iner_pubdata.acc[2]== iner_data_.acc[2];
	  threeag_pubdata.angular_velocity_x=threeag_pubdata.angular_velocity_y=threeag_pubdata.angular_velocity_z=0;
	  for(k=0;k<threeAG_data_.followed_data_length;k++)
	    {
	      threeag_pubdata.angular_velocity_x +=      threeAG_data_.data[k].x_axis;
	      threeag_pubdata.angular_velocity_y +=      threeAG_data_.data[k].y_axis ;
	      threeag_pubdata.angular_velocity_z +=      threeAG_data_.data[k].z_axis;	      
	    }
	  threeag_pubdata.frame_id  =    threeAG_data_.data[k-1].frame_id;
	  threeag_pubdata.angular_velocity_x = threeag_pubdata.angular_velocity_x/k;
	  threeag_pubdata.angular_velocity_y = threeag_pubdata.angular_velocity_y/k;
	  threeag_pubdata.angular_velocity_z = threeag_pubdata.angular_velocity_z/k;
	  /*	  
	  uniid_pubdata.udid0 ;
	  uniid_pubdata.udid1 ;
	  uniid_pubdata.udid2 ;*/

	  cur_pubdata.left_motor =current_data_.left_motor;
	  cur_pubdata.right_motor =current_data_.right_motor;
	  gpin_pubdata.digital_input =gpin_data_.digital_input;
	  gpin_pubdata.analog_input[0] =gpin_data_.analog_input[0];
	  gpin_pubdata.analog_input[1] =gpin_data_.analog_input[1];
	  gpin_pubdata.analog_input[2] =gpin_data_.analog_input[2];
	  gpin_pubdata.analog_input[3] =gpin_data_.analog_input[3];

	  cliff_state_publisher.publish(cliff_pubdata);
	  joint_state_publisher.publish(coresense_pubdata);
	  version_info_publisher.publish(hardware_pubdata);
	  controller_info_publisher.publish(controlinfo_pubdata);
	  dock_ir_publisher.publish(dockin_pubdata); 
	  inertia_publisher.publish(iner_pubdata); 
	  raw_imu_data_publisher.publish(threeag_pubdata); 
	  unique_id_publisher.publish(uniid_pubdata); 
	  gpinput_publisher.publish(gpin_pubdata);
	  ultrasonic_data_publisher.publish(ultrasonic_pubdata);
}

void subscribeTopics(ros::NodeHandle& nh)
{
  velocity_command_subscriber = nh.subscribe<geometry_msgs::Twist>("/aimibot/commands/velocity", 10, &subscribeVelocityCommand);
  led1_command_subscriber =  nh.subscribe<aimibot::Led>("/aimibot/commands/led_1", 10, &subscribeLed1Command);
  led2_command_subscriber =  nh.subscribe<aimibot::Led>("/aimibot/commands/led_2", 10, &subscribeLed2Command);
  digital_output_command_subscriber =  nh.subscribe<aimibot::DigitalOutput>("/aimibot/commands/digi_output", 10, &subscribeDigitalOutputCommand);
  external_power_command_subscriber =  nh.subscribe<aimibot::DigitalOutput>("/aimibot/commands/exter_power", 10, &subscribeExternalPowerCommand);
  sound_command_subscriber =  nh.subscribe<aimibot::Sound>("/aimibot/commands/beep_sound", 10, &subscribeSoundCommand);
  reset_odometry_subscriber = nh.subscribe<std_msgs::Empty>("/aimibot/commands/reset_odometry", 10, &subscribeResetOdometry);
  motor_power_subscriber = nh.subscribe<aimibot::MotorPower>("/aimibot/commands/mot_power", 10, &subscribeMotorPower);
  controller_info_command_subscriber =  nh.subscribe<aimibot::ControllerInfo>("/aimibot/commands/control_info", 10, &subscribeControllerInfoCommand);
  
}
void advertiseTopics(ros::NodeHandle& nh)
{

 /*********************
  ** AIMIJIA Esoterics
  **********************/
  cliff_state_publisher = nh.advertise <aimibot::Cliff>("/aimibot/cliff",100);
  joint_state_publisher = nh.advertise <aimibot::CoreSensors>("/aimibot/CoreSensors",100);
  version_info_publisher = nh.advertise < aimibot::hardware > ("/aimibot/version_info",  100, true); // latched publisher
  controller_info_publisher = nh.advertise < aimibot::ControllerInfo > ("/aimibot/controller_info",  100, true); // latched publisher
  dock_ir_publisher = nh.advertise < aimibot::DockInfraRed > ("/aimibot/sensors/dock_ir", 100);
  inertia_publisher = nh.advertise < aimibot::Inertia > ("/aimibot/sensors/imu_data", 100);
  raw_imu_data_publisher = nh.advertise < aimibot::ThreeAxisGyro > ("/aimibot/sensors/imu_data_raw", 100);
  unique_id_publisher = nh.advertise< aimibot::UniqueDeviceID > ("/aimibot/unique_id", 100);
  current_publisher = nh.advertise< aimibot::Current > ("/aimibot/current", 100);
  gpinput_publisher = nh.advertise< aimibot::GpInput > ("/aimibot/GpInput", 100);
  ultrasonic_data_publisher = nh.advertise< aimibot::Ultrasonic > ("/aimibot/Ultrasonic", 100);
//   basecontrol_publisher = ;
//   sound_publisher = ;
//   setpower_publisher = ;
//   request_extra_publisher = ; 
//   general_purpose_output_publisher = ;
}

enum packetFinderState
  {
    waitingForHead,
    waitingForPayloadSize,
    waitingForPayload,
    waitingForCheckSum
  };
packetFinderState state=waitingForHead;

class IDHeader {
public:
  enum PayloadType {
  // Streamed payloads
  CoreSensors = 1, DockInfraRed = 3, Inertia = 4, Cliff = 5, Current = 6,

  // Service Payloads
  Hardware = 10, Firmware = 11, ThreeAxisGyro = 13, Eeprom = 15, GpInput = 16,

  UniqueDeviceID = 19, Reserved = 20, Ultrasonic = 21
  };
};
#endif