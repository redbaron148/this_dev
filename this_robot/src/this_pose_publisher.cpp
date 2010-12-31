#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher pose_pub;

void poseCallback(const irobot_create_2_1::SensorPacket& msg)
{
    ROS_INFO("this is a test message!");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "this_pose_publisher");

    ros::NodeHandle node("this_pose_publisher");
    ros::Subscriber sub = node.subscribe("sensorPacket", 10, &poseCallback);
    ros::Publisher pose_pub = node.advertise<geometry_msgs::Pose2D>("pose", 5);

    ros::spin();
    return 0;
};
