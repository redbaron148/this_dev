#ifndef ARM_LIB_H
#define ARM_LIB_H

#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <crustcrawler_arm/SetPosition.h>
#include <crustcrawler_arm/SetServoPosition.h>
#include <crustcrawler_arm/GetServoPosition.h>
#include <crustcrawler_arm/SetAllServoPositions.h>
#include <crustcrawler_arm/GetLight.h>
#include "ThisDefines.h"

//#define DEBUG_MODE 1

ros::Time START_TIME;

ros::ServiceClient set_position_client;
ros::ServiceClient set_servo_position_client;
ros::ServiceClient get_servo_position_client;
ros::ServiceClient set_all_servo_positions_client;
ros::ServiceClient get_light_client;

//cmvision::Blob center_on_blob(std::string type);
bool set_position(float x, float y, float z, float attack_angle);
bool set_position(float x, float y, float z);
bool sap(float value_0, float value_1, float value_2, float value_3, float value_4, float value_5);
//bool grab_blob(const cmvision::Blob &b);
//bool pick_up_pom(const cmvision::Blob &b);
//bool has_something();
//bool has_something_special();
//void wait_till_arrived();

bool ssp(int s, float degrees);
float gsp(int s);
float get_light();

float get_light()
{
  crustcrawler_arm::GetLight srv;
  if(get_light_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("get light value");
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("get_light service did not work");
    #endif
  }
  return srv.response.value;
}

/*
 * same as above only centers on middle of image rather than arm.
 */
/*cmvision::Blob center_on_blob(cmvision::Blob blob)
{
  int start_position = gsp(BASE);
  int new_position = 0;
  int current_position = gsp(BASE);
  ros::Time start = ros::Time::now();
 
  ssp(CLAW,OPEN);
  sap(REST_POSITION);
  while(ros::ok() && (ros::Time::now().toSec()-start.toSec()) < 5.)
  {
    #if(DEBUG_MODE)
    ROS_INFO("blob x = %d  degrees from center = %f",b.x,degrees_from_middle(b)/CAMERA_P);
    #endif
    new_position = current_position+degrees_from_middle(b)/CAMERA_P;
    ssp(BASE,new_position);
    usleep(200000);
    current_position = new_position;
    ros::spinOnce();
  }
  
  if(!blob_is_valid(b))
  {
    ssp(BASE,start_position);

    b = getBlob(type);
    if(!blob_is_valid(b))
      return b;
  }
  
  return b;
}*/

/*
 * sends the command for the IK solver to move the arm to point x,y,z with 
 * attack angle of attack_angle
 */
bool set_position(float x, float y, float z, float attack_angle)
{
  crustcrawler_arm::SetPosition srv;
  srv.request.x = x;
  srv.request.y = y;
  srv.request.z = z;
  srv.request.attack_angle = attack_angle;
  if(set_position_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("set arm to position x=%f, y=%f, z=%f, a=%f",x,y,z,attack_angle);
    usleep(400000);
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_arm_position service did not work");
    #endif
  }
  return srv.response.success;
}

/*
 * sets the servo at index s to the degrees degrees
 */
bool ssp(int s, float degrees)
{
  crustcrawler_arm::SetServoPosition srv;
  srv.request.servo_index = s;
  srv.request.value = degrees;
  //float old_pos = gsp(s);
  if(set_servo_position_client.call(srv))
  {
  	ROS_INFO("called service");
    #if(DEBUG_MODE)
    //ROS_INFO("set %s servo to %f degrees, time slept = %f,old servo pos = %f",servo[s].c_str(),degrees,(SERVO_SPEED*abs(old_pos-degrees)/60.),old_pos);
    #endif
    //usleep(SERVO_SPEED*abs(old_pos-degrees)/7.*100000);
  }
  else
  {
    ROS_INFO("set_servo_position service did not work");
  }
  return srv.response.success;
}

/*
 * sends the signal for arm to go to point x,y,z where attack angle is computed 
 * as a function of the distance from the tip of the arm.
 */
bool set_position(float x, float y, float z)
{
  float angle = -90.;
  float distance = sqrt((x*x)+(y*y));
  float dst_ratio = (distance-MIN_ARM_REACH)/(MAX_ARM_REACH-MIN_ARM_REACH);
  angle *= dst_ratio;
  if(angle < -60.) angle = -60.;
  
  return set_position(x,y,z,(-90.)-angle);
}

/*
 * sets all positions of the arm.
 */
bool sap(float value_0, float value_1, float value_2, float value_3, float value_4, float value_5)
{
  crustcrawler_arm::SetAllServoPositions srv;
  srv.request.value_0 = value_0;
  srv.request.value_1 = value_1;
  srv.request.value_2 = value_2;
  srv.request.value_3 = value_3;
  srv.request.value_4 = value_4;
  srv.request.value_5 = value_5;
  if(set_all_servo_positions_client.call(srv))
  {
    #if(DEBUG_MODE)
    ROS_INFO("set all servo positions");
    usleep(600000);
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_all_servo_positions service did not work");
    #endif
  }
  return srv.response.success;
}

/*
 * returns the position of the servo at index s
 */
float gsp(int s)
{
  crustcrawler_arm::GetServoPosition srv;
  srv.request.servo_index = s;
  if(get_servo_position_client.call(srv))
  {
    #if(DEBUG_MODE)
    //ROS_INFO("get %s servo degrees",servo[s].c_str());
    #endif
  }
  else
  {
    #if(DEBUG_MODE)
    ROS_INFO("set_servo_position service did not work");
    #endif
  }
  return srv.response.value;
}

/*bool pick_up_pom(const cmvision::Blob &b)
{
  if(blob_is_valid(b) && blob_is_centered(b))
  {
    float dist = calculate_distance_from_robot(b);
    float angle = (gsp(BASE)+2.5)*(PIE/180.);
    float x = -dist*sin(angle);
    float y = dist*cos(angle);
    float z = 2.5;
    
    ssp(CLAW,50.);
    sleep(1);
    
    if(dist < 15.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is close and is a pom!");
      #endif
      set_position(x,y,z+4,-70.);
      usleep(200000);
      set_position(x,y,z,-70.);
    }
    else if(dist < 19.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is medium and is a pom!");
      #endif
      set_position(x,y,z+8);
      usleep(200000);
      set_position(x,y,z+1);
    }
    else if(dist < MAX_ARM_REACH)
    {
      dist = calculate_distance_from_robot(b)-5;
      angle = (gsp(BASE)+2.5)*(PIE/180.);
      x = -dist*sin(angle);
      y = dist*cos(angle);
      #if(DEBUG_MODE)
      ROS_INFO("The blob is far and is a pom!");
      #endif
      sap(gsp(0),0,60,0,20,-87);
      set_position(x,y,z);
      usleep(200000);
      dist = 22;
      angle = (gsp(BASE)+2.5)*(PIE/180.);
      x = -dist*sin(angle);
      y = dist*cos(angle);
      set_position(x,y,z+2);
    }
    else 
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is too far away!... and is a pom!");
      #endif
      return false;
    }
    
    usleep(50000);
    ssp(CLAW,0);
    usleep(50000);
    //sap(REST_POSITION);
    
    return has_something();
  }
  return false;
}*/

/*bool grab_blob_bucket_run(const cmvision::Blob &b)
{
  if(blob_is_valid(b) && blob_is_centered(b))
  {
    float base_pose = gsp(BASE);
    for(int i =0;i < 5 && base_pose == 0.0;i++)
      base_pose = gsp(BASE);
    float dist = calculate_distance_from_robot(b);
    float angle = (base_pose+2.5)*(PIE/180.);
    float x = -dist*sin(angle);
    float y = dist*cos(angle);
    float z = 8;
    
    ssp(CLAW,OPEN);
    //sap(REST_POSITION);
    
    #if(DEBUG_MODE)
    ROS_INFO("set position to x=%f, y=%f", x, y);
    #endif
    
    if(blob_is_pom(b))
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is a pom!");
      #endif
      dist = 16;
      angle = (base_pose+2.5)*(PIE/180.);
      x = -dist*sin(angle);
      y = dist*cos(angle);
      #if(DEBUG_MODE)
      ROS_INFO("The blob is far and is a pom!");
      #endif
      sap(gsp(0),0,60,0,20,-87);
      set_position(x,y,3,-30);
      usleep(2000000);
      dist = 22;
      angle = (base_pose+2.5)*(PIE/180.);
      x = -dist*sin(angle);
      y = dist*cos(angle);
      set_position(x,y,10,-10);
      usleep(2000000);
      ssp(CLAW,0);
      usleep(500000);
      ssp(SHOULDER,90);
      ssp(ELBOW,70);
      return true;
    }
    //ssp(CLAW,60);
    if(dist < 15.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is close and not a pom!");
      #endif
      set_position(x,y,z);
      usleep(200000);
      set_position(x,y,3.);
    }
    else if(dist < 19.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is medium and not a pom!");
      #endif
      set_position(x,y,z+2);
      usleep(200000);
      set_position(x,y,z-4);
    }
    else if(dist < MAX_ARM_REACH)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is far and not a pom!");
      #endif
      set_position(x,y,z+4);
      usleep(200000);
      set_position(x,y,z-2,-20);
    }
    else 
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is too far away!... and not a pom! i'm gonna try anyways");
      #endif
      
      dist = calculate_distance_from_robot(b);
      if(dist > 22) dist = 22;
      angle = (gsp(BASE)+2.6)*(PIE/180.);
      x = -dist*sin(angle);
      y = dist*cos(angle);
      z = 8;

      set_position(x,y,z+2);
      usleep(200000);
      set_position(x,y,z,-20);

    }
 
    usleep(1000000);
    ssp(CLAW,0);
    usleep(500000);
    
    return has_something_special();
  }
  return false;
}*/

/*bool grab_blob_special(const cmvision::Blob &b)
{
  if(blob_is_valid(b) && blob_is_centered(b))
  {
    float dist = calculate_distance_from_robot(b);
    float angle = (gsp(BASE)+2.5)*(PIE/180.);
    float x = -dist*sin(angle);
    float y = dist*cos(angle);
    float z = 8;
    
    ssp(CLAW,OPEN);
    //sap(REST_POSITION);
    
    #if(DEBUG_MODE)
    ROS_INFO("set position to x=%f, y=%f", x, y);
    #endif
    
    if(blob_is_pom(b))
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is a pom!");
      #endif
      return pick_up_pom(b);
    }
    //ssp(CLAW,60);
    if(dist < 15.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is close and not a pom!");
      #endif
      set_position(x,y,z);
      usleep(200000);
      set_position(x,y,3.);
    }
    else if(dist < 19.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is medium and not a pom!");
      #endif
      set_position(x,y,z+2);
      usleep(200000);
      set_position(x,y,z-4);
    }
    else if(dist < MAX_ARM_REACH)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is far and not a pom!");
      #endif
      set_position(x,y,z+4);
      usleep(200000);
      set_position(x,y,z-2,-20);
    }
    else 
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is too far away!... and not a pom! i'm gonna try anyways");
      #endif
      
      dist = calculate_distance_from_robot(b);
      if(dist > 22) dist = 22;
      angle = (gsp(BASE)+2.6)*(PIE/180.);
      x = -dist*sin(angle);
      y = dist*cos(angle);
      z = 8;

      set_position(x,y,z+2);
      usleep(200000);
      set_position(x,y,z,-20);

    }
 
    usleep(1000000);
    ssp(CLAW,0);
    usleep(50000);
    
    return has_something_special();
  }
  return false;
}*/

/*bool grab_blob(const cmvision::Blob &b)
{
  if(blob_is_valid(b) && blob_is_centered(b))
  {
    float base_pose = gsp(BASE);
    for(int i = 0;i < 5 && base_pose == 0.0;i++)
      base_pose = gsp(BASE);
    float dist = calculate_distance_from_robot(b)+1;
    float angle = (base_pose+3.7)*(PIE/180.);
    float x = -dist*sin(angle);
    float y = dist*cos(angle);
    float z = 10;
    
    ssp(CLAW,OPEN);
    //sap(REST_POSITION);
    
    #if(DEBUG_MODE)
    ROS_INFO("set position to x=%f, y=%f", x, y);
    #endif
    
    if(blob_is_pom(b))
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is a pom!");
      #endif
      return pick_up_pom(b);
    }
    //ssp(CLAW,60);
    if(dist < 15.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is close and not a pom!");
      #endif
      set_position(x,y,z);
      usleep(500000);
      set_position(x,y,3.);
    }
    else if(dist < 19.)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is medium and not a pom!");
      #endif
      set_position(x,y,z+2);
      usleep(500000);
      set_position(x,y,z-4);
    }
    else if(dist < MAX_ARM_REACH)
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is far and not a pom!");
      #endif
      set_position(x,y,z+4);
      usleep(500000);
      set_position(x,y,z-2,-20);
    }
    else 
    {
      #if(DEBUG_MODE)
      ROS_INFO("The blob is too far away!... and not a pom! i'm gonna try anyways");
      #endif
      
      dist = calculate_distance_from_robot(b);
      if(dist > 22) dist = 22;
      angle = (gsp(BASE)+2.6)*(PIE/180.);
      x = -dist*sin(angle);
      y = dist*cos(angle);
      z = 8;

      set_position(x,y,z+2);
      usleep(500000);
      set_position(x,y,z,-20);

    }
 
    usleep(1000000);
    ssp(CLAW,0);
    usleep(500000);
    
    return has_something();
  }
  return false;
}*/

/*bool has_something_special()
{
  return true;
}*/

/*bool has_something()
{
  if(gsp(CLAW) == OPEN) return false;
  return true;
  sap(-45.,130,CLOSE,HAS_SOMETHING_ANGLE,60,TWIST_HAS_SOMETHING_ANGLE);
  //ssp(SHOULDER,130);
  //ssp(ELBOW,60);
  //ssp(WRIST_FLEX,HAS_SOMETHING_ANGLE);
  //sleep(1);
  //cmvision::Blob c = getBlob("center");
  //ssp(WRIST_TWIST,TWIST_HAS_SOMETHING_ANGLE);
  sleep(2);
  cmvision::Blob b = getBlob("center");
  ssp(WRIST_TWIST,0);
  ssp(BASE,0);
  bool val = (blob_is_valid(b) && (b.channel == orange || b.channel == greenFoam || b.channel == greenPom));
  if(!val) sap(REST_POSITION);
  return (val);
}*/

/*bool check_for_botguy()
{
  cmvision::Blob b = get_biggest_blob(botguy);
  return blob_is_valid(b);
}*/

#endif
