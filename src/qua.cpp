#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
std_msgs::Float64MultiArray  IMU_Euler ;
void callback(const nav_msgs::OdometryConstPtr msg)
{
      IMU_Euler.data.resize(3);
  IMU_Euler.data[0]= 180.0/3.1415926 * atan(2*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.x+msg->pose.pose.orientation.y*msg->pose.pose.orientation.z)
	  /(1-2*(msg->pose.pose.orientation.x*msg->pose.pose.orientation.x+msg->pose.pose.orientation.y*msg->pose.pose.orientation.y))) ;
 
  IMU_Euler.data[1]= 180.0/3.1415926 * asin(2*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.y-
	  msg->pose.pose.orientation.z*msg->pose.pose.orientation.x)) ;
  
  IMU_Euler.data[2]= 180.0/3.1415926 * atan(2*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.z+msg->pose.pose.orientation.x*msg->pose.pose.orientation.y)
	  /(1-2*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.y+msg->pose.pose.orientation.z*msg->pose.pose.orientation.z))) ;
	
  ROS_INFO_STREAM("Euler_r: " << IMU_Euler.data[0] << "   " <<
		  "Euler_p: " << IMU_Euler.data[1] << "   " <<
		  "Euler_y"  << IMU_Euler.data[2]);


}
int main(int argv , char **argc)
{
  ros::init(argv, argc, "qua");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odom",10,callback);
  ros::spin();
  return 0;
}