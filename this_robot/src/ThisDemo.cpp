/*
 *  File Name:      ThisDemo.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      02-07-2011
 *  Description:    Demo code for this robot
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmvision/Blobs.h>
#include <this_robot/LaserBlobs.h>
#include "ArmLib.h"

using namespace std;

boost::shared_ptr<sensor_msgs::LaserScan> scan;
boost::shared_ptr<cmvision::Blobs> blobs;
boost::shared_ptr<this_robot::LaserBlobs> lblobs(new this_robot::LaserBlobs);

float findBlobAngle(const int &num, const cmvision::Blobs &blobs);
int findBiggestBlob(const cmvision::Blobs &blobs);
void laserCallback(boost::shared_ptr<sensor_msgs::LaserScan> msg);
void blobCallback(boost::shared_ptr<cmvision::Blobs> msg);

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "this_demo");
    ros::NodeHandle n("~");

    //ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber blobs_sub = n.subscribe("/blobs", 1, blobCallback);
    ros::Subscriber laser_sub = n.subscribe("/scan_filtered", 1, laserCallback);
    
    set_position_client = n.serviceClient<crustcrawler_arm::SetPosition>("/set_position");
	set_servo_position_client = n.serviceClient<crustcrawler_arm::SetServoPosition>("/set_servo_position");
	get_servo_position_client = n.serviceClient<crustcrawler_arm::GetServoPosition>("/get_servo_position");
	set_all_servo_positions_client = n.serviceClient<crustcrawler_arm::SetAllServoPositions>("/set_all_servo_positions");
	get_light_client = n.serviceClient<crustcrawler_arm::GetLight>("/get_light");

	sleep(3);	
	
	ros::Rate loop_rate(10);
	
	ssp(CLAW,OPEN);

	while(ros::ok())
	{
		sap(REST_POSITION);
		sleep(2);
		
		//ROS_INFO("printing to the screen!");
		ros::spinOnce();
		while(ros::ok() && blobs != NULL && blobs->blob_count && !(blobs->blobs[findBiggestBlob(*blobs)].x>ARM_CENTER_X-25 && blobs->blobs[findBiggestBlob(*blobs)].x<ARM_CENTER_X+25))
		{
			if(blobs->blobs[findBiggestBlob(*blobs)].x>ARM_CENTER_X-25)
			{
				ssp(BASE,gsp(BASE)-1.6);
			}
			else
			{
				ssp(BASE,gsp(BASE)+1.6);
			}
			loop_rate.sleep();
			ros::spinOnce();
		}
		if(blobs != NULL && blobs->blob_count)
		{
			float angle = gsp(BASE);
			ROS_INFO("angle: %f", angle);
			for(int i = 0;i < lblobs->blob_count;i++)
			{
				ROS_INFO("angle of laser blob: %f",lblobs->blobs[i].angle);
				if(lblobs->blobs[i].angle-7<angle && lblobs->blobs[i].angle+7>angle)
				{
					ROS_INFO("gotcha! Dist is: %f",lblobs->blobs[i].distance);
					ROS_INFO("I think it's this one: %f",lblobs->blobs[i].angle);
					float x = 100*-sin(lblobs->blobs[i].angle*3.14159/180.)*lblobs->blobs[i].distance;
					float y = 100*cos(lblobs->blobs[i].angle*3.14159/180.)*lblobs->blobs[i].distance;
					ROS_INFO("x: %f",x);
					ROS_INFO("y: %f",y);
					
					set_position(x,y,4,-35);
					i=lblobs->blob_count;
				}
			}
			
			sleep(5);
		}
	}

	return 0;
}

float findBlobAngle(const int &num, const cmvision::Blobs &blobs)
{
	static float center_x = blobs.image_width/2.0;
    static float center_y = blobs.image_height/2.0;
    static float radians_per_pixel_vert = 64.*3.14159/180.0/(float)blobs.image_height;
    static float radians_per_pixel_horiz = 64.*3.14159/180.0/(float)blobs.image_width;
    
	return (blobs.blobs[num].x-center_x)*radians_per_pixel_horiz;
}

int findBiggestBlob(const cmvision::Blobs &blobs)
{
	int biggest = 0;
	for(int i = blobs.blob_count-1;i>=1;i--)
	{
		if(blobs.blobs[i].area > blobs.blobs[biggest].area)
			biggest = i;
	}
	return biggest;
}

void laserCallback(boost::shared_ptr<sensor_msgs::LaserScan> msg)
{
	lblobs->header = msg->header;
	lblobs->angle_min = msg->angle_min;
	lblobs->angle_max = msg->angle_max;
	lblobs->angle_increment = msg->angle_increment;
	lblobs->time_increment = msg->time_increment;
	lblobs->scan_time = msg->scan_time;
	lblobs->range_min = msg->range_min;
	lblobs->range_max = msg->range_max;
	lblobs->blobs.clear();
	lblobs->blob_count = 0;
	
	scan = msg;
	
	this_robot::LaserBlob b;
	b.end = msg->ranges.size()-1;
	
	for(int i = msg->ranges.size()-2;i>=0;i--)
	{
		if(!(msg->ranges[i] >= msg->ranges[i-1]-0.005 && msg->ranges[i] <= msg->ranges[i-1]+0.005))
		{
			if(b.end-i-1 >= 20 && msg->ranges[(b.end+i-1)/2] != 0)
			{
				b.start = i-1;
				b.distance = msg->ranges[(b.end+b.start)/2];
				b.width = msg->angle_increment*(b.end-b.start)*180.0/3.14159;
				b.angle = -((msg->angle_increment*((b.end+b.start)/2)+msg->angle_min)*180./3.14159);
				lblobs->blob_count++;
				
				lblobs->blobs.push_back(b);
			}
			b.end = i;
		}
	}
	//cout << (*lblobs) << endl;
}

void blobCallback(boost::shared_ptr<cmvision::Blobs> msg)
{
	blobs = msg;
}
