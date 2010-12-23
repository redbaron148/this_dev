#include "ros/ros.h"
#include "crustcrawler_arm/SetServoPosition.h"
#include "SerialPort.cpp"
#include <iostream>


#define NO_SERIAL TRUE
#define DEBUG     TRUE

#if(NO_SERIAL)
SerialPort lsc;
#endif

bool ssp(crustcrawler_arm::SetServoPosition::Request  &req,
         crustcrawler_arm::SetServoPosition::Response &res )
{
  ROS_INFO("request: servo_index=%d, value_is_pwm=%d, value=%d", 
            req.servo_index, req.desired_position.value_is_pwm, 
            req.desired_position.value);
            
  std::ostringstream num_conv;
  
  std::string request = "Set ";
  
  num_conv << (int)req.servo_index;
  request += num_conv.str();
  num_conv.str("");
  
  if(req.desired_position.value_is_pwm)
    request += " PWM ";
  else
    request += " Degree ";
    
  num_conv << (int)req.desired_position.value;
  request += num_conv.str() + ";";
  num_conv.str("");
  
  #if(DEBUG)
  ROS_INFO(request.c_str());
  ROS_INFO("size of string is %d",sizeof(request));
  #endif
  
  res.success = true;
  #if(NO_SERIAL)
  if(lsc.isOpen())
    lsc.writeBlock(request.c_str(),request.length()+1);
  else
    res.success = false;
  
  #endif
  
  #if(DEBUG)
  ROS_INFO("sending back response: [%d]", (bool)res.success);
  #endif
  return true;
}

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "add_two_ints_server");
  ros::init(argc, argv, "crustcrawler_server");
  ros::NodeHandle n;
  
  #if(NO_SERIAL)
  lsc.setPort("/dev/ttyUSB0");
  lsc.setBaudRate(B115200);
  
  if(!lsc.openPort())
  {
    ROS_INFO("Unable to open serial port for crustcrawler_server");
    return 1;
  }
  ROS_INFO("Serial port open");
  #endif

  ros::ServiceServer service = n.advertiseService("set_servo_position", ssp);
  ROS_INFO("set_servo_position service started");
  ros::spin();

  #if(NO_SERIAL)
  lsc.closePort();
  ROS_INFO("Serial port closed");
  #endif
  return 0;
}
