#include <ros/ros.h>
#include <crustcrawler_arm/SetServoPosition.h>
#include <crustcrawler_arm/GetServoPosition.h>
#include <crustcrawler_arm/SetAllServoPositions.h>
#include <crustcrawler_arm/InitServo.h>
#include <crustcrawler_arm/EndInit.h>
#include <crustcrawler_arm/ArmPose.h>
#include <crustcrawler_arm/CrustcrawlerState.h>
#include "SerialPort.cpp"

using namespace std;

class CrustcrawlerArm
{
	public:
		//constructors
		CrustcrawlerArm();
		~CrustcrawlerArm();
		
	private:
		//variables
		SerialPort port;
		bool is_initalizing;
		bool is_initializing_all;
		string port_path;
		ros::NodeHandle nh;
		crustcrawler_arm::ArmPose arm_pose;
		crustcrawler_arm::ServoPositions raw_servo_positions;
		crustcrawler_arm::ServoPositions degree_servo_positions;
		
		ros::Publisher state_pub;
		ros::Subscriber arm_pose_sub;
		ros::ServiceServer ssp_service;
		ros::ServiceServer gsp_service;
		ros::ServiceServer sap_service;
		ros::ServiceServer init_servo_service;
		
		//utility functions
		string int_to_string(float num);
		bool send_request(string request);
		void poseCallback(crustcrawler_arm::ArmPose msg);
		bool get_servo_position(crustcrawler_arm::GetServoPosition::Request  &req, crustcrawler_arm::GetServoPosition::Response &res )
		bool set_servo_position(crustcrawler_arm::SetServoPosition::Request  &req, crustcrawler_arm::SetServoPosition::Response &res );
};

CrustcrawlerArm::CrustcrawlerArm() : port(), is_initalizing(false), is_initializing_all(false), nh("~"), arm_pose(), raw_servo_positions(), degree_servo_positions()
{
	nh.param<std::string>("port", port_path, "/dev/ttyUSB0");
	port.setPort(port_path);
	port.setBaudRate(B115200);
	if(!port.openPort())
	{
		ROS_ERROR("Could not open serial port %s at %d for communication.",port_path.c_str(),115200);
	}
	
	state_pub = nh.advertise<crustcrawler_arm::CrustcrawlerState>("/crustcrawler/state", 1);
	arm_pose_sub = nh.subscribe("/crustcrawler/arm_pose", &poseCallback);
	ssp_service = nh.advertiseService("/crustcrawler/set_servo_position", &set_servo_position);
	gsp_service = nh.advertiseService("/crustcrawler/get_servo_position", &get_servo_position);
	sap_service = nh.advertiseService("/crustcrawler/set_all_servo_positions", &set_all_servo_positions);
	init_servo_service = nh.advertiseService("/crustcrawler/init_servo", &init_servo);
}

void CrustcrawlerArm::poseCallback(crustcrawler_arm::ArmPose msg)
{
	
}

bool CrustcrawlerArm::get_servo_position(crustcrawler_arm::GetServoPosition::Request  &req,
         crustcrawler_arm::GetServoPosition::Response &res )
{
	if(!is_initalizing)
	{
		read_servo_position(req.servo_index);

		ROS_INFO("Get Servo Request: servo_index=%d", req.servo_index);

		res.value = read_servo_position(req.servo_index);
		return true;
	}
	return false;
}

CrustcrawlerArm::~CrustcrawlerArm()
{
	port.closePort();
}

std::string CrustcrawlerArm::int_to_string(float num)
{
	std::ostringstream num_conv;
	num_conv << num;
	return num_conv.str();
}

bool CrustcrawlerArm::send_request(std::string request)
{
	ROS_DEBUG("request to serial port:\n %s",request.c_str());

	if(!port.isOpen()) return false;

	port.writeData(request.c_str(),request.length());
	usleep(15000);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "crustcrawler_arm");
	//CrustcrawlerArm
}
