#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

static void image_viewer_callback(const sensor_msgs::Image& image_source)
{
	const auto& encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
							     encoding);
	IplImage frame = cv_image->image;
	cvShowImage("Image Viewer", &frame);
	cvWaitKey(2);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_viewer");
	ros::NodeHandle n;
	ros::Subscriber scriber = n.subscribe("/image_raw", 1,
					      image_viewer_callback);

	ros::spin();
	return 0;
}
