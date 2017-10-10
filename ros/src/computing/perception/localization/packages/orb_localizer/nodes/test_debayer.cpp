/*
 * test_debayer.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sujiwo
 */

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libraw/libraw.h>

using namespace std;


void imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
	static LibRaw ImageProcessor;

	// simple
	cv::Mat image_simple;
	image_simple = cv_bridge::toCvCopy(msg, "bgr8")->image;
	cv::imshow ("raw image", image_simple);
	cv::waitKey(1);



}


int main (int argc, char *argv[])
{
	ros::init(argc, argv, "orb_matching", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler;

	image_transport::TransportHints th ("raw");
	image_transport::ImageTransport imageBuf (nodeHandler);
	image_transport::Subscriber imageSub = imageBuf.subscribe ("/camera1/image_raw", 1, imageCallback, th);

	cv::namedWindow("raw image");

	ros::spin();

	ros::shutdown();
}
