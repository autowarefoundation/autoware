/*
 * orb_evaluator.cpp
 *
 *  Created on: Nov 17, 2016
 *      Author: sujiwo
 */

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "System.h"
#include "ORBextractor.h"

using namespace std;
using namespace ORB_SLAM2;
using namespace boost::posix_time;
namespace enc = sensor_msgs::image_encodings;



void imageProcess (cv::Mat &image)
{
	static ORBextractor *orbExt = NULL;
	static cv::Mat imDescriptor;
	vector<cv::KeyPoint> orbKeys;

	if (!orbExt) {
		orbExt = new ORBextractor (3000, 1.2, 8, 20, 7);
	}

	cv::cvtColor (image, image, CV_RGB2GRAY);

	(*orbExt)(image, cv::Mat(), orbKeys, imDescriptor);

	cv::cvtColor (image, image, CV_GRAY2RGB);
	cout << "Size: " << orbKeys.size() << endl;

	for (auto &kp: orbKeys) {
		cv::circle(image, kp.pt, 2, cv::Scalar(0,255,0), -1);
	}
}


void orbEvaluatorCallback (const sensor_msgs::ImageConstPtr &imageMsg)
{
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(imageMsg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat image;
	// Check if we need debayering
	if (enc::isBayer(imageMsg->encoding)) {
		int code=-1;
		if (imageMsg->encoding == enc::BAYER_RGGB8 ||
				imageMsg->encoding == enc::BAYER_RGGB16) {
			code = cv::COLOR_BayerBG2BGR;
		}
		else if (imageMsg->encoding == enc::BAYER_BGGR8 ||
				imageMsg->encoding == enc::BAYER_BGGR16) {
			code = cv::COLOR_BayerRG2BGR;
		}
		else if (imageMsg->encoding == enc::BAYER_GBRG8 ||
				imageMsg->encoding == enc::BAYER_GBRG16) {
			code = cv::COLOR_BayerGR2BGR;
		}
		else if (imageMsg->encoding == enc::BAYER_GRBG8 ||
				imageMsg->encoding == enc::BAYER_GRBG16) {
			code = cv::COLOR_BayerGB2BGR;
		}
		cv::cvtColor(cv_ptr->image, image, code);
	}
	else
		image = cv_ptr->image;
	cv::resize (image, image, cv::Size(800,600));
	imageProcess (image);

	cv::imshow ("XXX", image);
	cv::waitKey(10);
}



int main (int argc, char *argv[])
{
	ros::init(argc, argv, "orb_evaluator");
	ros::start();
	ros::NodeHandle nodeHandler;

	image_transport::TransportHints th ("raw");
	image_transport::ImageTransport imageBuf (nodeHandler);
	image_transport::Subscriber imageSub = imageBuf.subscribe ("/camera/image_raw", 1, orbEvaluatorCallback, th);
	cv::namedWindow("XXX", cv::WINDOW_AUTOSIZE);

	cout << "Ready" << endl;

	ros::spin();

	ros::shutdown();
	return 0;
}
