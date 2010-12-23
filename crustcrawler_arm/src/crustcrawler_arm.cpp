#include "ros/ros.h"
#include "crustcrawler_arm/SetServoPosition.h"
#include "crustcrawler_arm/GetServoPosition.h"
#include "crustcrawler_arm/SetAllServoPositions.h"
#include "crustcrawler_arm/SetPosition.h"
#include "crustcrawler_arm/InitServo.h"
#include "crustcrawler_arm/InitAllServos.h"
#include "crustcrawler_arm/EndInit.h"
#include "crustcrawler_arm/GetLight.h"
#include "SerialPort.cpp"
#include <iostream>


#define SERIAL_ON   1
#define DEBUG_MODE  1

#if(SERIAL_ON)
SerialPort lsc;
#endif

bool is_initalizing = false;
bool is_initalizing_all = false;
int init_count = 0;
float positions[6] = {0,0,0,0,0,0};

char port[] = "/dev/ttyUSB1";

std::string int_to_string(float num)
{
  std::ostringstream num_conv;
  num_conv << num;
  return num_conv.str();
}

bool send_request(std::string request)
{
  #if(DEBUG_MODE)
  ROS_INFO("DEBUG MODE request to serial port:\n %s",request.c_str());
  #endif
  #if(SERIAL_ON)
  if(lsc.isOpen())
  {
    lsc.writeData(request.c_str(),request.length());
    usleep(15000);
  }
  else
    return false;
  #endif
  return true;
}

bool sap(crustcrawler_arm::SetAllServoPositions::Request &req,
         crustcrawler_arm::SetAllServoPositions::Response &res )
{
  if(!is_initalizing)
  {
	//removed pwm
    ROS_INFO("Set Servo Request: value_0=%d, value_1=%d, value_2=%d, value_3=%d, value_4=%d, value_5=%d", 
             req.value_0, req.value_1, req.value_2, req.value_3, req.value_4, req.value_5);
             
    std::string request = "Set All ";
    
    //if((bool)req.values_are_pwm)
      //request += "PWM ";
    //else
      request += "Degree ";
      
    request += int_to_string(req.value_0) + " " + 
               int_to_string(req.value_1) + " " + 
               int_to_string(req.value_2) + " " + 
               int_to_string(req.value_3) + " " + 
               int_to_string(req.value_4) + " " + 
               int_to_string(req.value_5) + ";";
               
    res.success = send_request(request);
              
    return res.success;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot send request, initalizing servos");
  #endif
  return false;
}

bool ssp(crustcrawler_arm::SetServoPosition::Request  &req,
         crustcrawler_arm::SetServoPosition::Response &res )
{
  if(!is_initalizing)
  {
	//removed pwm
    ROS_INFO("Set Servo Request: servo_index=%d, value=%d", 
              req.servo_index, 
              req.value);
  
    std::string request = "Set ";
    
    positions[req.servo_index] = req.value;
  
    request += int_to_string((int)req.servo_index);
  
    //if(req.value_is_pwm)
      //request += " PWM ";
    //else
      request += " Degree ";

    request += int_to_string((int)req.value) + ";";
  
    res.success = send_request(request);
  
    #if(DEBUG_MODE)
    ROS_INFO("sending back response: [%d]", (bool)res.success);
    #endif
    return true;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot send request, initalizing servos");
  #endif
  return false;
}

float read_servo_position(int servo_index)
{
  char* buff = new char[20];
  float value = 0.0;
  std::string request = "Read " + int_to_string(servo_index) + " Degree;";
  
  send_request(request);
  usleep(500);
  lsc.readData(buff,20);
  
  value = atof(buff);
  delete [] buff;
  return value;
}

bool light(crustcrawler_arm::GetLight::Request  &req,
           crustcrawler_arm::GetLight::Response &res)
{
  if(!is_initalizing)
  {
    char* buff = new char[20];
    float value = 0.0;
    std::string request = "Light;";
  
    send_request(request);
    usleep(100000);
    lsc.readData(buff,20);
  
    value = atof(buff);
    delete [] buff;
    res.value = value;
    return true;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot send request, initalizing servos");
  #endif
  return false;
}

bool gsp(crustcrawler_arm::GetServoPosition::Request  &req,
         crustcrawler_arm::GetServoPosition::Response &res )
{
  if(!is_initalizing)
  {
    read_servo_position(req.servo_index);
  
    ROS_INFO("Get Servo Request: servo_index=%d", 
              req.servo_index);
            
    //std::string request = "Read ";
  
    //request += int_to_string(req.servo_index);
  
    //if(req.value_is_pwm)
      //request += " PWM;";
    //else
      //request += " Degree;";
  
    //send_request(request);
    
    //sleep(1); 
    #if(SERIAL_ON)
    //lsc.readData(buff,10);
    #endif
    //sleep(1);
    #if(DEBUG_MODE)
    //ROS_INFO("buff is = %s",buff);
    #endif

    res.value = read_servo_position(req.servo_index);
    //delete [] buff;
    return true;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot send request, initalizing servos");
  #endif
  return false;
}

/*
 * Not quite sure how to interupt a running service... probably gonna have to 
 * make a service inside this service which can be subscribed to to stop init.
 */
bool init_servo(crustcrawler_arm::InitServo::Request  &req,
                crustcrawler_arm::InitServo::Response &res )
{
  if(!is_initalizing)
  {
    ROS_INFO("Initalize Servo Request: servo_index=%d, desired range=0-%d", 
              req.servo_index, req.desired_range);
            
    std::string request = "Init ";
    request +=  int_to_string(req.servo_index) + " " + 
                int_to_string(req.desired_range) + ";";
    res.success=send_request(request);
  
    if(res.success)
    {
      is_initalizing_all = false;
      is_initalizing = true;
    }
    return res.success;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot send request, initalizing servos");
  #endif
  return false;
}

/*bool init_all_servos(crustcrawler_arm::InitAllServos::Request  &req,
                     crustcrawler_arm::InitAllServos::Response &res )
{
  if(!is_initalizing)
  {
    ROS_INFO("Initalize All Servo Request: desired range_0=0-%d, range_1=0-%d, range_2=0-%d, range_3=0-%d, range_4=0-%d, range_5=0-%d", 
              req.range_0, req.range_1, req.range_2, req.range_3, req.range_4, 
              req.range_5);
            
    std::string request = "Init All ";
    request +=  int_to_string(req.range_0) + " " +
                int_to_string(req.range_1) + " " +
                int_to_string(req.range_2) + " " +
                int_to_string(req.range_3) + " " +
                int_to_string(req.range_4) + " " + 
                int_to_string(req.range_5) + ";";
                
    res.success=send_request(request);
  
    if(res.success)
    {
      is_initalizing_all = true;
      is_initalizing = true;
      init_count = 0;
    }
    return res.success;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot send request, initalizing servos");
  #endif
  return false;
}*/

bool send_init_message(crustcrawler_arm::EndInit::Request  &req,
              crustcrawler_arm::EndInit::Response &res )
{
  if(is_initalizing)
  {
    ROS_INFO("Request To End Initalization: message=%s",
             req.message.c_str());
  
    std::string request;
  
    if(req.message[0]=='o' || req.message[0]=='O')
      request = "Ok;";
    else if(req.message[0]=='x' || req.message[0]=='X')
      request = "X;";
    else if(req.message[0]=='c' || req.message[0]=='C')
      request = "Center;";
    else if(req.message[0]=='M' || req.message[0]=='m')
      request = "Max;";
    else return false;
  
    res.success = send_request(request);
    if(res.success)
      init_count++;
    if(!is_initalizing_all && init_count == 3)
    {
      is_initalizing = false;
      init_count = 0;
    }
    else if(is_initalizing_all && init_count == 19)
    {
      is_initalizing=false;
      is_initalizing_all=false;
      init_count = 0;
    }
    if(request == "X;" && init_count%3!=0)
    {
      #if(DEBUG_MODE)
      ROS_INFO("init mode exited");
      #endif
      is_initalizing=false;
      is_initalizing_all=false;
      init_count=0;
    }
    #if(DEBUG_MODE)
    ROS_INFO("init message request is %s, init_count=%d, init_count mod 3=%d",request.c_str(),init_count,init_count%3);
    #endif
    return res.success;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot end initalization, no servos initalizing");
  #endif
  return false;
}

bool set_position(crustcrawler_arm::SetPosition::Request &req,
         crustcrawler_arm::SetPosition::Response &res )
{
  if(!is_initalizing)
  {
    ROS_INFO("Position Request: x=%f, y=%f, z=%f, attack_angle=%f", 
             req.x, req.y, req.z, req.attack_angle);
             
    std::string request = "Position ";
      
    request += int_to_string(req.x) + " " + 
               int_to_string(req.y) + " " + 
               int_to_string(req.z) + " " + 
               int_to_string(req.attack_angle) + ";";
               
    res.success = send_request(request);
              
    return res.success;
  }
  #if(DEBUG_MODE)
  ROS_INFO("Cannot send request, initalizing servos");
  #endif
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crustcrawler_server");
  ros::NodeHandle n;
  
  #if(DEBUG_MODE)
  ROS_INFO("crustcrawler_arm is in debug mode");
  #endif
  
  #if(SERIAL_ON)
  lsc.setPort(port);
  lsc.setBaudRate(B115200);
  
  if(!lsc.openPort())
  {
    ROS_INFO("Unable to open serial port for crustcrawler_server");
    return 1;
  }
  ROS_INFO("Serial port open");
  #endif

  ros::ServiceServer ssp_service = n.advertiseService("set_servo_position", ssp);
  ROS_INFO("set_servo_position service started");
  ros::ServiceServer sap_service = n.advertiseService("set_all_servo_positions", sap);
  ROS_INFO("set_all_servo_positions service started");
  ros::ServiceServer set_position_service = n.advertiseService("set_position", set_position);
  ROS_INFO("set_position service started");
  ros::ServiceServer gsp_service = n.advertiseService("get_servo_position", gsp);
  ROS_INFO("get_servo_position service started");
  ros::ServiceServer init_servo_service = n.advertiseService("init_servo",init_servo);
  ros::ServiceServer get_light_service = n.advertiseService("get_light",light);
  ros::ServiceServer send_init_message_service = n.advertiseService("send_init_message",send_init_message);
  ROS_INFO("init_servo service started");
  ros::spin();

  #if(SERIAL_ON)
  lsc.closePort();
  ROS_INFO("Serial port closed");
  #endif
  return 0;
}
