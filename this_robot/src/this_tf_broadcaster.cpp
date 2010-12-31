#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(3.14159,0,3.14159), tf::Vector3(-0.1651, 0.0, 0.03175)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
}
