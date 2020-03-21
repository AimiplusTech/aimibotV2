/*
 * Author: Yu miao
 * Data:2018.9.17
*/
#include <serial/serial.h>
#include <iostream>
#include "../include/aimibot/aimibot.hpp"
#include <signal.h> 

serial::Serial sp;
bool first_read=false;
const double digit_to_dps = 0.00875;
bool isconnect(false);

bool findpack_update(const unsigned char * incoming, unsigned int numberOfIncoming)
{
  if (!(numberOfIncoming > 0))
  {
    findheader0=findheader1=false;
    state=waitingForHead;
    return false;
  }
  bool found_packet(false);
  static unsigned char cs(0);
  uint8_t subpay_size=0;
  uint8_t offset=0;
  uint8_t add=0;
  uint8_t k;
  uint8_t location;
  uint8_t checksum=0;
  if(state==waitingForPayload)
  {
     coresensors_data={0};
     controlinfo_data={0};
     uniqid_data={0};
     gpin_data={0};
     threeAG_data={0};
     firmware_Data={0};
     hardware_data={0};
     current_data={0};
     cliff_Data={0};
     iner_data={0};
     dockin_data={0};
  }
  switch(state)
  {
    case(waitingForHead): 
      if((incoming[0]==0xaa)&&findheader0==false)  {findheader0=true;ready_get=1;}
	else if((findheader0==true)&&(incoming[0]==0x55)&&(findheader1==false))  
	{ 
	 //std::cout << "found packet header:"<<incoming[0]<<"  "<<incoming[0]<<std::endl;
	  findheader1=true; state=waitingForPayloadSize;   
	}
      break;
    case(waitingForPayloadSize): 
      if(incoming[0]<3) { std::cout<< "payload_size is too short" << std::endl; break;}
     // ROS_INFO("PAYLOADSIZE:%d",incoming[0]);
      payload_size = incoming[0];
      ready_get=  incoming[0];
      state=waitingForPayload;
      cs^=incoming[0];
      break;
    case(waitingForPayload): 
      
      while (offset!=(payload_size))
      {
	add=offset+1;
      switch(incoming[offset])
       {
	case IDHeader::CoreSensors: 
	  subpay_size = incoming[add];
	      coresensors_data.time_stamp =(incoming[offset+3]<<8)|incoming[offset+2] ; 
	      coresensors_data.bumper = incoming[offset+4];
	      coresensors_data.wheel_drop= incoming[offset+5];
	      coresensors_data.cliff= incoming[offset+6];
	      coresensors_data.left_encoder= (incoming[offset+8]<<8)|incoming[offset+7] ; 
	      coresensors_data.right_encoder= (incoming[offset+10]<<8)|incoming[offset+9] ; 
	      coresensors_data.left_pwm= incoming[offset+11];
	      coresensors_data.right_pwm= incoming[offset+12]; 
	      coresensors_data.buttons= incoming[offset+13];
	      coresensors_data.charger= incoming[offset+14];
	      coresensors_data.battery= incoming[offset+15];
	      coresensors_data.over_current= incoming[offset+16];
	  break;
	case IDHeader::DockInfraRed:
	  subpay_size = incoming[add];
	    dockin_data.Docking[0]=incoming[offset+2];
	    dockin_data.Docking[1]=incoming[offset+3];
	    dockin_data.Docking[2]=incoming[offset+4];
	  break;
	case IDHeader::Inertia:
	  subpay_size = incoming[add];
	    iner_data.angle = (int16_t)(incoming[offset+3]<<8)|incoming[offset+2] ;
	    iner_data.angle_rate = (int16_t)(incoming[offset+5]<<8)|incoming[offset+4] ;
	    iner_data.acc[0] = incoming[offset+6];
	    iner_data.acc[1] = incoming[offset+7];
	    iner_data.acc[2] = incoming[offset+8];
	  break;
	case IDHeader::Cliff:
	  subpay_size = incoming[add]; 
	  cliff_Data.bottom[0]= (incoming[offset+3]<<8)|incoming[offset+2] ;
	  cliff_Data.bottom[1]= (incoming[offset+5]<<8)|incoming[offset+4] ;
	  cliff_Data.bottom[2]= (incoming[offset+7]<<8)|incoming[offset+6] ;
	  break;
	case IDHeader::Current:
	  subpay_size = incoming[add]; 
	  current_data.left_motor= (incoming[offset+3]<<8)|incoming[offset+2] ;
	  current_data.right_motor= (incoming[offset+5]<<8)|incoming[offset+4] ;
	  break;
	case IDHeader::ThreeAxisGyro:
	  subpay_size = incoming[add];
	    threeAG_data.frame_id = incoming[offset+2];
	    threeAG_data.followed_data_length = incoming[offset+3]/3 ;
	    for(k=0;k<threeAG_data.followed_data_length;k++)
	    {
	      threeAG_data.data[k].x_axis= -digit_to_dps * (short)( (incoming[offset+6*k+5]<<8)|incoming[offset+6*k+4]) ;
	      threeAG_data.data[k].y_axis= digit_to_dps * (short)((incoming[offset+6*k+7]<<8)|incoming[offset+6*k+6]) ;
	      threeAG_data.data[k].z_axis= digit_to_dps * (short)((incoming[offset+6*k+9]<<8)|incoming[offset+6*k+8]) ;
	      threeAG_data.data[k].frame_id=threeAG_data.frame_id;
	    }
	  break;
	case IDHeader::GpInput:
	  subpay_size = incoming[add];
	  gpin_data.digital_input=  (incoming[offset+3]<<8)|incoming[offset+2] ;
	  gpin_data.analog_input[0]=  (incoming[offset+5]<<8)|incoming[offset+4] ;
	  gpin_data.analog_input[1]=  (incoming[offset+7]<<8)|incoming[offset+6] ;
	  gpin_data.analog_input[2]=  (incoming[offset+9]<<8)|incoming[offset+8] ;
	  gpin_data.analog_input[3]=  (incoming[offset+11]<<8)|incoming[offset+10] ;
	  break;
	case IDHeader::Hardware:
	  subpay_size = incoming[add];
	  hardware_data.version.patch=  (incoming[offset+3]<<8)|incoming[offset+2] ;
	  hardware_data.version.minor=  (incoming[offset+5]<<8)|incoming[offset+4] ;
	  hardware_data.version.major=  (incoming[offset+7]<<8)|incoming[offset+6] ;
	  break;
	case IDHeader::Firmware: 
	  subpay_size = incoming[add];
	  firmware_Data.version.patch=  (incoming[offset+3]<<8)|incoming[offset+2] ;
	  firmware_Data.version.minor=  (incoming[offset+5]<<8)|incoming[offset+4] ;
	  firmware_Data.version.major=  (incoming[offset+7]<<8)|incoming[offset+6] ;
	  break;
	case IDHeader::UniqueDeviceID:
	  subpay_size = incoming[add];
	  break;
	case IDHeader::Ultrasonic: 
	  subpay_size = incoming[add];
	  ultrasonic_data.DISL1=  (incoming[offset+3]<<8)|incoming[offset+2] ;
	  ultrasonic_data.DISL2=  (incoming[offset+5]<<8)|incoming[offset+4] ;
	  ultrasonic_data.DISL3=  (incoming[offset+7]<<8)|incoming[offset+6] ;
	  ultrasonic_data.DISL4=  (incoming[offset+9]<<8)|incoming[offset+8] ;
	  ultrasonic_data.DISL5=  (incoming[offset+11]<<8)|incoming[offset+10] ;
	  break;
	default:
// 	  ROS_INFO("CAN NOT FOUND DATA_ID: %d",incoming[offset]);
	  subpay_size=payload_size-offset-2;
	  break;
       }
       offset+=(subpay_size+2);
      }
      for(k=0;k<payload_size;k++)
	cs^=incoming[k];
      state=waitingForCheckSum;
      ready_get=1;
      break;
    case(waitingForCheckSum):
      checksum=incoming[0];
//      ROS_INFO("checksum: %02x cal_cs: %02x",checksum,cs);
      if(checksum!=cs) {
        ROS_INFO("PACK FOUND ERR");
        found_packet=false;
	
      }
      else  { 
	found_packet=true; 
// 	if(!first_read){
// 	first_read = true ;
// 	ROS_INFO("A FULL PACK READ FINISHED!!");
// 	}
      }
      cs=0;
      findheader0=findheader1=false;
      state=waitingForHead;
      isconnect=true;
//       std::cout<< std::endl;
//       std::cout<< std::endl;
      break;
    default: 
      break;
  }

  return found_packet;
}
void shutdown(int sig)  
{  
       sp.close();
       ros::shutdown();  
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "serial_port"); 
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    Aimi::Odometry odometry;
    signal(SIGINT, shutdown);
    advertiseTopics(nh);
    subscribeTopics(nh);
    std::string port = "/dev/ttyUSB0";
    ros::param::get("~serial_port", port);
    try
    {
       //创建timeout
       serial::Timeout to = serial::Timeout::simpleTimeout(1000);
       //设置要打开的串口名称
       sp.setPort(port);
       //设置串口通信的波特率
       sp.setBaudrate(115200);
       //串口设置timeout
       sp.setTimeout(to);
       //打开串口
       sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("Device is opened.");
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(100);
    ros::Time last_getdata = ros::Time::now();
    uint8_t temp_buffer[256];
    uint8_t buffer[256];
    uint8_t i =0;
    odometry.init(nh,nh_p,"aimibot");
    reset_odometry();
    bool head_init = false;
    
    ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states",100);
    ecl::Angle<double> gethand;
    sensor_msgs::JointState joint_states;
    ecl::LegacyPose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    
    joint_states.name.push_back("leftwheel_joint");
    joint_states.name.push_back("rightwheel_joint");
    
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.read(temp_buffer, ready_get);  
        //读出数据
        if(n==0) 
         {
           std::cout<<"waiting for data 10s" << std::endl;
           if(ros::Time::now() - last_getdata > ros::Duration(10.0))
            {
              std::cout<< "TIME OUT!CLOSE SERIAL!"<<std::endl;
              sp.close();
              ros::shutdown();
            }continue;
          }
        if(findpack_update(temp_buffer,n)) 
	{
           if(!head_init) { heading_offset = init_head(); head_init = true;}
           last_getdata=ros::Time::now();
           rostopic_pub(coresensors_data,
                       ultrasonic_data,
                       gpin_data,
                       threeAG_data,
                       firmware_Data,
                       hardware_data,
                       current_data,
                       cliff_Data,
                       iner_data,
                       dockin_data);
	   
	 joint_states.position.resize(2,0.0);
         joint_states.velocity.resize(2,0.0);
         joint_states.effort.resize(2,0.0);

	 diff_drive.update(coresensors_data.time_stamp, coresensors_data.left_encoder, coresensors_data.right_encoder,
                                     pose_update, pose_update_rates);
         diff_drive.getWheelJointStates(joint_states.position[0], joint_states.velocity[0],
				    joint_states.position[1], joint_states.velocity[1]);
         odometry.update(pose_update,pose_update_rates,getHeading(),getAngularVelocity());
      if(ros::ok())
       {
        joint_states.header.stamp = ros::Time::now();
        joint_state_publisher.publish(joint_states);     
       }
	 // Odometry publish 
	}

        ros::spinOnce();}
    //关闭串口
    sp.close();
    return 0;
}
