/**
 * @author Nate Roney
 * copyright 2010
 *
     This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *
 * usb_camera_node
 * 
 * A ROS node whose function is to capture an image from a webcam, convert that Image
 *  from an OpenCV IplImage to a ROS Image Message, and to broadcast the
 *  converted image over the ROS network on the topic "image_<V4LID>", where V4LID matches
 *   the camera ID.
 *
 * Default behavior is to use the second camera attached to the computer. This is done
 *  because most of our laptops have an integrated webcam, and we typically prefer an external.
 *
 *  @params
 *
 *  V4L_ID : the V4L index of the camera to be used. These correspond to /dev/video*
 *
 *  FPS : framerate at which to capture. Default is 20 
 */

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "sensor_msgs/fill_image.h"
#include <string>
#include <boost/lexical_cast.hpp>

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "usb_camera_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	string topic = "image_";
	int camID = 1;
	int FPS = 10;
	
	nh.param( "V4L_ID", camID, int(camID) );
	nh.param( "FPS", FPS, int(FPS));
	
	ROS_INFO( "Using USB camera at V4L index: %d at %d FPS.", camID, FPS );
	
	//delete params to support multiple instances of this node with different
	// config options
	nh.deleteParam( "V4L_ID" );
	nh.deleteParam( "FPS" );	
	
	//append the V4L id to this topic for multiple camera support
	topic += boost::lexical_cast<std::string>(camID);
	
	//setup the publisher
	ros::Publisher cameraPub = n.advertise<sensor_msgs::Image>(topic, 1);
	
	ros::Rate loop_rate(FPS);
	sensor_msgs::CvBridge bridge_;
	CvCapture *camera;
	IplImage *rawImg = NULL;
	camera = cvCaptureFromCAM(camID);
	
	
	//These don't seem to do anything with the current version of OpenCV
	cvSetCaptureProperty( camera, CV_CAP_PROP_FRAME_WIDTH, 320 );
	cvSetCaptureProperty( camera, CV_CAP_PROP_FRAME_HEIGHT, 240 );
	

	while (ros::ok())
	{
		rawImg = cvQueryFrame(camera);
		cvFlip(rawImg,NULL,-1);

		try
		{
			cameraPub.publish(bridge_.cvToImgMsg(rawImg));
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error converting IplImage with CvBridge");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

