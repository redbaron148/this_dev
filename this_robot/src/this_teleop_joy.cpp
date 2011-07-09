#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
#include "ArmLib.h"


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
    set_servo_position_client = nh_.serviceClient<crustcrawler_arm::SetServoPosition>("/set_servo_position");
    get_servo_position_client = nh_.serviceClient<crustcrawler_arm::GetServoPosition>("/get_servo_position");
    set_position_client = nh_.serviceClient<crustcrawler_arm::SetPosition>("/set_position");
    set_all_servo_positions_client = nh_.serviceClient<crustcrawler_arm::SetAllServoPositions>("/set_all_servo_positions");

    joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopThis::joyCallback, this);
}

void TeleopThis::joyCallback(const joy::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.linear.x = (a_scale_*joy->axes[angular_]);
    vel.angular.z = l_scale_*joy->axes[linear_];
    if(joy->buttons[0])
    {
        if(gsp(CLAW) == OPEN)
        {
            ssp(CLAW,CLOSE);
        }
        else ssp(CLAW,OPEN);
        std::cout << "doing stuff!" <<  std::endl;
    }
    if(joy->buttons[1])
    {
        sap(0,0,gsp(CLAW),0,0,0);
    }
    else if(joy->buttons[2])
    {
        set_position(0,12.5,4,-60);
    }
    vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_This");
    TeleopThis teleop_This;

    ros::spin();
}
