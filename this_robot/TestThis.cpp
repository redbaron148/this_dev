/*
 *  File Name:      ThisDriverNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-27-2010
 *  Description:    Node for controlling the robot this
 */

#include <ros/ros.h>
#include <irobot_create_2_1/SensorPacket.h>

using namespace std;

ros::Publisher filtered_blob_pub;

void blobsCallback(cmvision::Blobs msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_filter");
    ros::NodeHandle n("blob_filter");
    
    getParams(n);

    ros::Subscriber blobs_sub = n.subscribe("/blobs", BLOBS_MSG_BUFFER, &blobsCallback);
    filtered_blob_pub = n.advertise<cmvision::Blobs>("/blob_filter/blobs", MSG_QUEUE);

    ros::Rate loop_rate(PUBLISH_FREQ);
    
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
