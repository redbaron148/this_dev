/*
 *  File Name:      ThisDriverNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-27-2010
 *  Description:    Node for controlling the robot this
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

void blobsCallback(cmvision::Blobs msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "this_driver");
    ros::NodeHandle n("~");

    ros::Publisher cmd_vel_pub = n.advertise<cmvision::Blobs>("/cmd_vel", 1);

    geometry_msgs::Twist cmd_vel;
    
    cmd_vel.linear[0] = 100;
    cmd_vel.angular[2] = 0;
    cmd_vel_pub.publish(cmd_vel);
    
    sleep(2);
    
    cmd_vel.linear[0] = 0;
    cmd_vel.angular[2] = 0.1;
    cmd_vel_pub.publish(cmd_vel);
    
    sleep(2);

	return 0;
}
