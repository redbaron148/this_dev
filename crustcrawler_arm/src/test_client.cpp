#include "ros/ros.h"
#include "crustcrawler_arm/SetServoPosition.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_servo_position_client");
  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
         return 1;
  }*/

  ros::NodeHandle n;
  //ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  ros::ServiceClient client = n.serviceClient<crustcrawler_arm::SetServoPosition>("set_servo_position");
  //beginner_tutorials::AddTwoInts srv;
  crustcrawler_arm::SetServoPosition srv;
  
  srv.request.servo_index = 0;
  srv.request.value_is_pwm = false;
  srv.request.value = 180;
  
  if (client.call(srv))
  {
    ROS_INFO("Success: %d", (bool)srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service set_servo_position");
    return 1;
  }

  return 0;
}

