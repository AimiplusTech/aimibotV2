#include "../include/aimibot/movecircular.h"
cirvelControl::cirvelControl()  
{  
    init();  
    vel_pub_ = handle_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);  
    sub_ = handle_.subscribe("/odom", 1, &cirvelControl::odomCallback,this); 
    //订阅"/odom"话题，当一个新的消息到达时，ROS将会调用velControl::odomCallback()函数。第二个参数是对列的长度，如果我们处理消息的速度不够快，会将收到的消息缓冲下来，满了的话，后面的到达的消息将会覆盖前面的消息。
}
bool cirvelControl::init()  
{  
    start_pos_.init(0.0, 0.0, 0.0);  
    curr_pos_.init(0.0, 0.0, 0.0);  
    rect_width_ = 0.5;  
    
    vel_line_ = 0.2;  
    vel_angle_ = 0.5; 
    handle_.getParam("/vel_line_",vel_line_);
    handle_.getParam("/vel_line_",vel_angle_);

    //vel_line_ = 0.2;  
    //vel_angle_ = 0.2;  
  
    vel_pub_speed_ = 5; 
  
    state_ = 0;  
    Is_Fininsh_ = true;  
}  
void cirvelControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)  
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
    start_pos_.x = curr_pos_.x;  
    start_pos_.y = curr_pos_.y;  
    start_pos_.theta = curr_pos_.theta;  

    count++;  
    if( count %= vel_pub_speed_ )  
    {  
        velocityControl();  
    }  
  
}  
bool cirvelControl::velocityControl()  
{  
    geometry_msgs::Twist controlVel_;  
    controlVel_.angular.z = vel_angle_;  
    controlVel_.linear.x  = vel_line_;  
    controlVel_.linear.y  = 0;  
    controlVel_.linear.z  = 0;  

    vel_pub_.publish(controlVel_);  
}  
double cirvelControl::angleWrap(double angle)  
{  
    //把角度规划到-pi至pi之间  
    if (angle>=CV_PI)  
        while (angle >= CV_PI)  
        { angle = angle-2*CV_PI;}  
    else if (angle < -1.0*CV_PI)  
        while (angle < -1.0*CV_PI)  
        { angle = angle+2*CV_PI;}  
  return angle;  
}  
  
  
int main(int argc,char** argv)  
{  
    ros::init(argc,argv,"odompub");  
    cirvelControl cirodom_vel_;  
    ros::spin(); 
    return 0;  
}
