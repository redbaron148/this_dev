#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>


class TeleopThis
{
public:
    TeleopThis();

private:
    void joyCallback(const joy::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};


TeleopThis::TeleopThis(): linear_(1), angular_(2)
{
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);


    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


    joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopThis::joyCallback, this);
}

void TeleopThis::joyCallback(const joy::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.linear.x = (a_scale_*joy->axes[angular_])*-1;
    vel.angular.z = l_scale_*joy->axes[linear_];
    vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_This");
    TeleopThis teleop_This;

    ros::spin();
}
