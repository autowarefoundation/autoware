#include <boost/foreach.hpp>
#include <rosbag/view.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <exception>
#include "common.h"


#define foreach BOOST_FOREACH


using std::cout;
using std::endl;
using std::exception;


void tfToCV(const tf::Transform &src, cv::Mat &position, cv::Mat &orientation)
{
	position = cv::Mat (3,1,CV_64F);
	tf::Vector3 p = src.getOrigin();
	position.at<double>(0) = p.x(),
		position.at<double>(1) = p.y(),
		position.at<double>(2) = p.z();

	orientation = cv::Mat (4,1,CV_64F);
	tf::Quaternion otn = src.getRotation();
	orientation.at<double>(0) = otn.x(),
		orientation.at<double>(1) = otn.y(),
		orientation.at<double>(2) = otn.z(),
		orientation.at<double>(3) = otn.w();
}


void recomputeNewCameraParameter (
	// Original
	double fx1, double fy1, double cx1, double cy1,
	// New
	double &fx2, double &fy2, double &cx2, double &cy2,
	int width1, int height1,
	int width2, int height2
)
{
	double ratio = (double)width1 / (double)width2;
	fx2 = fx1 / ratio;
	fy2 = fy1 / ratio;
	cx2 = cx1 / ratio;
	cy2 = cy1 / ratio;
}
