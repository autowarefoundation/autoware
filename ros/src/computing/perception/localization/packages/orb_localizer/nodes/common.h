
#ifndef _ORB_UTILS_H
#define _ORB_UTILS_H 1

#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <rosbag/bag.h>
#include <string>
#include <vector>
#include <exception>


using std::string;
using std::vector;
using std::exception;


void tfToCV(const tf::Transform &src, cv::Mat &position, cv::Mat &orientation);
void recomputeNewCameraParameter (
	// Original
	double fx1, double fy1, double cx1, double cy1,
	// New
	double &fx2, double &fy2, double &cx2, double &cy2,
	int width1, int height1,
	int width2, int height2
);

void tf2positiondirection (const tf::Transform &pose, float positiondirection[6]);


#endif /* _ORB_UTILS_H */
