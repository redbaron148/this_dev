/*
 *  File Name:      ThisDriverNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-27-2010
 *  Description:    Node for controlling the robot this
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <irobot_create_2_1/SensorPacket.h>

using namespace std;

bool PACKET_FLAG = false;

void callback(irobot_create_2_1::SensorPacket msg)
{
	PACKET_FLAG = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "this_driver");
    ros::NodeHandle n("~");

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = n.subscribe("/sensorPacket", 1, callback);

    geometry_msgs::Twist cmd_vel;
    
    ROS_INFO("waiting for create driver");
    
    while(ros::ok() && !PACKET_FLAG) ros::spinOnce();
    
    ROS_INFO("starting test!");
    
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel);

	return 0;
}
