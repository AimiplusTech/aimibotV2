#ifndef _RUNNING_TEST_H
#define _RUNNING_TEST_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  
#include <geometry_msgs/TwistStamped.h>  

#include "../aimibot/data_struct.hpp"

#include <nav_msgs/Odometry.h>  
#include "tf/LinearMath/Matrix3x3.h"  
#include "geometry_msgs/Quaternion.h"

#define CV_PI   3.1415926535897932384626433832795 


  
#include <iostream>  
#include <stdlib.h>  

using namespace std;

typedef struct AimibotPos
{
    double x;
    double y;
    double theta;
    void init(double x1,double x2,double x3)
    {
        x = x1;
        y = x2;
        theta = x3;
    }
}AimibotPos_type;

typedef struct AimibotUltrasonicdis
{
    unsigned short disl1;
    unsigned short disl2;
    unsigned short disl3;
    unsigned short disl4;
    unsigned short disl5;
}AimibotUltrasonicdis_type;



class runningtest
{
 public:
    runningtest();
    AimibotPos_type             start_pos_;
    AimibotPos_type             curr_pos_;
    AimibotUltrasonicdis_type   curr_dis_;
    int state_; 

 protected: 
    void runningtestultrasonic_data_callback(const aimibot::UltrasonicConstPtr &sonic_pubdata);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
 private:

    double rect_width_;  
    double rect_height_;  
    double offset_xy_;  
    double offset_theta_;
    unsigned short ObstacDis_;  
  
    double vel_line_;  
    double vel_angle_;  
  
     
    bool Is_Fininsh_;  
  
    ros::NodeHandle handle_;  
    ros::Publisher  vel_pub_ ;  
    ros::Subscriber sub_odem_;  
    ros::Subscriber sub_sonic_;
                                                                                       
    geometry_msgs::Twist  controlVel_;  
    double angleWrap(double angle);  
    bool init();  
    bool velocityControl();  
  
    int vel_pub_speed_;  
};

#endif




