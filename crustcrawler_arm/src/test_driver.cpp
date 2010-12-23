#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 100);
  ros::Rate loop_rate(5);
  int count = 0;
  while (ros::ok())
  {
    std::stringstream ss;
    ss << "Hello there! This is message [" << count << "]";
    std_msgs::String msg;
    msg.data = ss.str();
    chatter_pub.publish(msg);
    ROS_INFO("I published [%s]", ss.str().c_str());
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}

