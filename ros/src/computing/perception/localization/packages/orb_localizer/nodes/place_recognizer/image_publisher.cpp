/*
 * image_publisher.cpp
 *
 *  Created on: May 21, 2017
 *      Author: sujiwo
 */

/*
 * The purpose of this node is to publish a single image. It is used to test
 * the performance of place recognizer.
 */


#include <iostream>
#include <string>
#include <ros/ros.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


using namespace std;


int main (int argc, char *argv[])
{
	ros::init(argc, argv, "image_testpub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	std::string imgTopic (argv[2]);
	image_transport::Publisher pub = it.advertise(imgTopic, 1);

	cv::Mat cImage = cv::imread (argv[1], CV_LOAD_IMAGE_ANYCOLOR);
	if (cImage.empty()) {
		cerr << "Unable to open image\n";
		exit (1);
	}

	cout << "Sending single image in " << imgTopic << endl;
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImage).toImageMsg();

	ros::Rate lp (5);
	while (nh.ok()) {
		pub.publish (msg);
		ros::spinOnce();
		lp.sleep();
//		break;
	}
//	pub.publish(msg);
//	ros::spinOnce();
//	ros::shutdown();

	return 0;
}
