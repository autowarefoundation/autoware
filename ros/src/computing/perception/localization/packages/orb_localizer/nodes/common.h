
#ifndef _ORB_UTILS_H
#define _ORB_UTILS_H 1

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <rosbag/bag.h>
#include <string>
#include <vector>
#include <exception>
#include "Frame.h"
#include "KeyFrame.h"


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

tf::Transform FramePose (ORB_SLAM2::Frame *cframe);

void equalizeImageHistogramFromMask (cv::Mat &input, cv::Mat &output, cv::Mat &mask);

float detectCcdSmear (cv::Mat &colorInput);

// Keyframe positions
tf::Transform KeyFramePoseToTf (ORB_SLAM2::KeyFrame *kf);
tf::Transform getKeyFrameExtPose (const ORB_SLAM2::KeyFrame *kf);

cv::Vec3d tfToCv (const tf::Vector3 &pos);



class OrbException: public exception
{
public:
	inline OrbException (const string &s):
		msg(s)
	{}

	inline const char *what() const noexcept
	{ return msg.c_str(); }

protected:
	string msg;
};





#endif /* _ORB_UTILS_H */
