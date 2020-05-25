#include "../include/aimibot/UltrasnoicObstacle.h"

runningtest::runningtest()
{
    init();
    //publish the velocity  information
    vel_pub_ = handle_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    //subscribe the odem and sonic data
    sub_odem_ = handle_.subscribe("/odom",2, &runningtest::odomCallback,this);
    sub_sonic_ = handle_.subscribe("/aimibot/Ultrasonic",1,&runningtest::runningtestultrasonic_data_callback,this);
}

bool runningtest::init()
{
    start_pos_.init(0.0, 0.0, 0.0);
    curr_pos_.init(0.0, 0.0, 0.0);
    rect_width_ = 0.20;

    vel_line_ = 0.2;  //m
    vel_angle_ = 0.2;//m
    ObstacDis_ = 30; //cm

    vel_pub_speed_ = 9;

    runningtest::state_ = 7;
    Is_Fininsh_ = true;
}
void runningtest::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{   
    static int count = 0;
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
    //Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR.
    //Yaw around Z axis; Pitch around Y axis; Roll around X axis
    curr_pos_.x = msg->pose.pose.position.x;
    curr_pos_.y = msg->pose.pose.position.y;
    curr_pos_.theta = yaw;
    
    if(Is_Fininsh_)
    {
        start_pos_.x = curr_pos_.x;
        start_pos_.y = curr_pos_.y;
        start_pos_.theta = curr_pos_.theta;
    }
    
    count++;
    if( count %= vel_pub_speed_ )
    {
        velocityControl();
    }


}

void runningtest::runningtestultrasonic_data_callback(const aimibot::UltrasonicConstPtr& sonic_pubdata)
{  
    
    curr_dis_.disl1=sonic_pubdata->disl1;
    curr_dis_.disl2=sonic_pubdata->disl2;
    curr_dis_.disl3=sonic_pubdata->disl3;
    curr_dis_.disl4=sonic_pubdata->disl4;
    curr_dis_.disl5=sonic_pubdata->disl5;

    if((curr_dis_.disl4<=ObstacDis_)||
            (curr_dis_.disl5<=ObstacDis_)||
            (curr_dis_.disl4<=ObstacDis_&&curr_dis_.disl5<=ObstacDis_)||
            (curr_dis_.disl3<=ObstacDis_&&curr_dis_.disl4<=ObstacDis_&&curr_dis_.disl5<=ObstacDis_))
    {
        state_=1;//right
    }
    else if((curr_dis_.disl1<=ObstacDis_)||
            (curr_dis_.disl2<=ObstacDis_)||
            (curr_dis_.disl1<=ObstacDis_&&curr_dis_.disl2<=ObstacDis_))
    {
        state_=2;//left
    }
    else if(curr_dis_.disl3>ObstacDis_)
    {
        state_=0;
    }
    else if((curr_dis_.disl1<=ObstacDis_&&curr_dis_.disl2<=ObstacDis_&&curr_dis_.disl3<=ObstacDis_&&curr_dis_.disl4<=ObstacDis_&&curr_dis_.disl5<=ObstacDis_)||
            (curr_dis_.disl2<=ObstacDis_&&curr_dis_.disl3<=ObstacDis_&&curr_dis_.disl4<=ObstacDis_)||
            (curr_dis_.disl3<=ObstacDis_&&curr_dis_.disl2<=ObstacDis_&&curr_dis_.disl1<=ObstacDis_)||
            (curr_dis_.disl3<=ObstacDis_&&curr_dis_.disl4<=ObstacDis_)||
            (curr_dis_.disl3<=ObstacDis_&&curr_dis_.disl2<=ObstacDis_)||
            (curr_dis_.disl3<=ObstacDis_))
    {
        state_=2;//left
    }
}


double runningtest::angleWrap(double angle)
{
    //把角度规划到-pi至pi之间  101SW
    if (angle>=CV_PI)
        while (angle >= CV_PI)
        { angle = angle-2*CV_PI;}
    else if (angle < -1.0*CV_PI)
        while (angle < -1.0*CV_PI)
        { angle = angle+2*CV_PI;}
    return angle;

}
bool runningtest::velocityControl()
{
    geometry_msgs::Twist controlVel_;
    switch (state_)
    {
    case 0://前进
        controlVel_.angular.z = 0;
        controlVel_.linear.x  = vel_line_;
        controlVel_.linear.y  = 0;
        controlVel_.linear.z  = 0;
        break;
    case 1://右转
        controlVel_.angular.z = vel_angle_;
        controlVel_.linear.x  = 0;
        controlVel_.linear.y  = 0;
        controlVel_.linear.z  = 0;
        break;
    case 2://左转
        controlVel_.angular.z = -vel_angle_;
        controlVel_.linear.x  = 0;
        controlVel_.linear.y  = 0;
        controlVel_.linear.z  = 0;
        break;
    default:
        controlVel_.angular.z = 0.0;
        controlVel_.linear.x  = vel_line_;
        controlVel_.linear.y  = 0;
        controlVel_.linear.z  = 0;
        break;
    }
    vel_pub_.publish(controlVel_);
}


int main(int argc,char** argv)  
{  
    ros::init(argc,argv,"odompub");
    runningtest    odom_sonic_vel_;
    ros::spin();
    return 0;
}
