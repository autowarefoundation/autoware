/*
 * ImagePreprocessor.h
 *
 *  Created on: Jan 20, 2017
 *      Author: sujiwo
 */

#ifndef _IMAGEPREPROCESSOR_H_
#define _IMAGEPREPROCESSOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <vector>
#include <string>


using std::string;


class ImagePreprocessor
{
public:

	enum ProcessMode {
		AS_IS = 0,
		AGC = 1,
		ILLUMINATI = 2
	};

	ImagePreprocessor (ProcessMode m,
		const string &subscribedImageTopic,
		const string &publishedImageTopic,
		ros::NodeHandle &nh);

	void messageCallback (const sensor_msgs::ImageConstPtr &msg);

	virtual ~ImagePreprocessor();

	static cv::Mat cdf (cv::Mat &grayImage, cv::Mat mask=cv::Mat());

	static float detectSmear (cv::Mat &rgbImage, const float tolerance);

	static cv::Mat setGamma (cv::Mat &grayImage, const float gamma, bool LUT_only=false);

	static cv::Mat autoAdjustGammaRGB (cv::Mat &rgbImage, cv::Mat mask=cv::Mat());
	static cv::Mat autoAdjustGammaRGB (cv::Mat &rgbImage, std::vector<cv::Mat> &rgbBuf, cv::Mat mask=cv::Mat());
	static cv::Mat autoAdjustGammaMono (cv::Mat &grayImage, float *gamma=NULL, cv::Mat mask=cv::Mat());
	static cv::Mat toIlluminatiInvariant (const cv::Mat &rgbImage, const float alpha);

	void setMask (cv::Mat &maskSrc);
	void setIAlpha (const float &a)
	{ iAlpha = a; }

protected:
	ros::NodeHandle &node;
	ros::Subscriber imgsub;
	ros::Publisher imgpub;
	ProcessMode pMode;

	// buffers
	std::vector<cv::Mat> rgbImageBuf;
	cv::Mat mask;

	float iAlpha;
};

#endif /* _IMAGEPREPROCESSOR_H_ */
