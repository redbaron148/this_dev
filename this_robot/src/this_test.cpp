#include <ros/ros.h>
#include <irobot_create_2_1/Tank.h>
#include <nav_msgs/Odometry.h>

void poseCallback(const nav_msgs::Odometry& msg)
{
    ROS_INFO("Recieved new Odometry state.\nX: %f \nY: %f \n",msg.pose.pose.position.x,msg.pose.pose.position.y);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "this_test");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/odom", 10, &poseCallback);
    
    ros::ServiceClient client = node.serviceClient<irobot_create_2_1::Tank>("/tank");
    irobot_create_2_1::Tank srv;
    srv.request.clear = 1;
    srv.request.left = 100;
    srv.request.right = 100;
    if (client.call(srv))
    {
        ROS_INFO("Successfully called the service");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    
    sleep(1);
    
    srv.request.clear = 1;
    srv.request.left = 0;
    srv.request.right = 0;
    if (client.call(srv))
    {
        ROS_INFO("Successfully called the service");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
};
