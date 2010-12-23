#include "ros/ros.h"
#include "crustcrawler_arm/SetServoPosition.h"

bool ssp(crustcrawler_arm::SetServoPosition::Request  &req,
         crustcrawler_arm::SetServoPosition::Response &res )
{
  res.success = true;
  ROS_INFO("request: servo_index=%d, value_is_pwm=%d, value=%d", 
            req.servo_index, req.value_is_pwm, 
            req.value);
  ROS_INFO("sending back response: [%d]", (bool)res.success);
  return true;
}

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "add_two_ints_server");
  ros::init(argc, argv, "set_servo_position_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("set_servo_position", ssp);
  ROS_INFO("BWAH");
  ros::spin();

  return 0;
}
