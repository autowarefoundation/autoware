#include <vector>
#include <thread>
#include <string>
#include <iostream>

#include "Map.h"
#include "TrackingThread.h"
#include "SystemMT.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>



using namespace std;
namespace enc = sensor_msgs::image_encodings;



SystemMT *localizer;





void imageProcess (const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat image;

	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (enc::isBayer(msg->encoding)) {
		int code=-1;
		if (msg->encoding == enc::BAYER_RGGB8 ||
			msg->encoding == enc::BAYER_RGGB16) {
			code = cv::COLOR_BayerBG2BGR;
		}
		else if (msg->encoding == enc::BAYER_BGGR8 ||
				 msg->encoding == enc::BAYER_BGGR16) {
			code = cv::COLOR_BayerRG2BGR;
		}
		else if (msg->encoding == enc::BAYER_GBRG8 ||
				 msg->encoding == enc::BAYER_GBRG16) {
			code = cv::COLOR_BayerGR2BGR;
		}
		else if (msg->encoding == enc::BAYER_GRBG8 ||
				 msg->encoding == enc::BAYER_GRBG16) {
			code = cv::COLOR_BayerGB2BGR;
		}
		cv::cvtColor(cv_ptr->image, image, code);
	}
	else
		image = cv_ptr->image;

	localizer->Track(image, msg->header.stamp.toSec());
}



int main (int argc, char *argv[])
{
	if (argc < 3) {
		cerr << "Usage: orb_matching_mt setting_file map1 <map2> ... <map_n>" << endl;
		exit (-1);
	}

	const string settingPath = argv[1];
	vector<string> mapPaths;
	for (int i=2; i<argc; i++) {
		string mp = argv[i];
		mapPaths.push_back(mp);
	}

    ros::init(argc, argv, "orb_matching_mt");
    ros::start();
    ros::NodeHandle nodeHandler;

    // This macro should be set by Cmake
	string orbVocabFile (ORB_SLAM_VOCABULARY);

	localizer = new SystemMT (nodeHandler, mapPaths, orbVocabFile, settingPath);

	image_transport::TransportHints th;
	const cv::FileStorage &fsetting = *(localizer->getSettings());
	if ((int)fsetting["Camera.compressed"]==0) {
		th = image_transport::TransportHints ("raw");
	}
	else if ((int)fsetting["Camera.compressed"]==1) {
		th = image_transport::TransportHints ("compressed");
	}
	image_transport::ImageTransport it (nodeHandler);
	image_transport::Subscriber sub = it.subscribe ((string)fsetting["Camera.topic"], 1, imageProcess, th);
	cout << endl << "Mono Camera topic: " << (string)fsetting["Camera.topic"] << endl;
	cout << "Compressed images? " << ((int)fsetting["Camera.compressed"]==1 ? "True" : "False") << endl;

	ros::spin();
	cout << "... Done" << endl;

	delete (localizer);
	return 0;
}
