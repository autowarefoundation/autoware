/*
 * hist_match.cpp
 *
 *  Created on: Jan 4, 2017
 *      Author: sujiwo
 */

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "common.h"
#include <vector>

using namespace std;
namespace enc = sensor_msgs::image_encodings;


image_transport::ImageTransport *mTrans;
image_transport::Subscriber mImageHandler;
image_transport::Publisher mImageResult;
cv::Mat cmask;

class pFrame
{
public:
	pFrame (cv::Mat &inImage) :
		imageBgr(inImage)
	{
		cv::Mat imageHsv;
		cv::cvtColor (imageBgr, imageHsv, CV_BGR2HSV);
		cv::split(imageHsv, HSV);
	}

	cv::Mat outputRgbFromHsv ()
	{
		cv::Mat bgr, hsv;
		cv::merge(HSV, hsv);
		cv::cvtColor (hsv, bgr, CV_HSV2BGR);
		return bgr;
	}


#define CDF_MEDIUM_TOLERANCE 0.1
	bool needEqualize ()
	{
		cv::Mat ccdf = cdf (HSV[2]);
		cout << ccdf.at<float>(127) << endl;
		return (ccdf.at<float>(127) < 0.5-CDF_MEDIUM_TOLERANCE
			or ccdf.at<float>(127) > 0.5+CDF_MEDIUM_TOLERANCE);
	}

	void equalizeByMask (const cv::Mat &mask)
	{
		cv::Mat LUT = cv::Mat::zeros(1, 256, CV_8U);
		cv::MatND hist;
		int histSize = 256;
		float range[] = {0,255};
		const float *ranges[] = {range};
	//	uint8_t
		cv::calcHist (&HSV[2], 1, 0, mask, hist, 1, &histSize, ranges, true, false);

		int i=0;
		while (!hist.at<uint8_t>(i))
			i++;
		int total = (int)HSV[2].total();

		int miz=(int)hist.at<float>(i);
		float scale = 255.0f / (total-miz);
		int sum = 0;
		for (LUT.at<uchar>(i++)=0; i<256; ++i) {
			sum += (int)hist.at<float>(i);
			LUT.at<uchar>(i) = cv::saturate_cast<uchar>(sum * scale);
		}

		cv::LUT(HSV[2], LUT, HSV[2]);
	//	cv::equalizeHist(output, output);

	}


	static cv::Mat cdf (const cv::Mat &scImage)
	{
		cv::Mat rcdf = cv::Mat::zeros(1,256,CV_32F);
		cv::MatND hist;
		int histSize = 256;
		float range[] = {0,255};
		const float *ranges[] = {range};
	//	uint8_t
		cv::calcHist (&scImage, 1, 0, cv::Mat(), hist, 1, &histSize, ranges, true, false);
		// cumulative sum
		rcdf.at<float>(0) = hist.at<float>(0);
		for (int i=1; i<histSize; i++) {
			rcdf.at<float>(i) = rcdf.at<float>(i-1) + hist.at<float>(i);
		}

		float s = cv::sum(hist)[0];
		rcdf = rcdf * (1.0/s);
		return rcdf;
	}


	float detectSmear ()
	{

	}

private:
	cv::Mat &imageBgr;
	vector<cv::Mat> HSV;
};


void imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat image;
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvShare(msg);

	// Check if we need debayering
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

	pFrame pcImage (image);
	pcImage.needEqualize();
//	pcImage.equalizeByMask(cmask);
//	sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pcImage.outputRgbFromHsv()).toImageMsg();
//	mImageResult.publish(imageMsg);
//	if (detectCcdSmear(image)>0)
//		cout << "Smear detected\n";
//	else
//		cout << "Clear\n";
//
//	cv::cvtColor (image, image, cv::COLOR_BGR2GRAY);
//	cv::Mat pcImage;
//
//	equalizeImageHistogramFromMask (image, pcImage, cmask);
//	sensor_msgs::ImagePtr imagePc = cv_bridge::CvImage (std_msgs::Header(), "mono8", pcImage).toImageMsg();
//	mImageResult.publish(imagePc);
//	cv::cvtColor(cv_ptr->image, image, code);
}


int main (int argc, char *argv[])
{

	ros::init(argc, argv, "hist_match", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	mTrans = new image_transport::ImageTransport (nodeHandler);
	mImageHandler = mTrans->subscribe ("/camera/image_raw", 1, imageCallback, image_transport::TransportHints("raw"));
	mImageResult = mTrans->advertise("/camera/image_hs", 1);

	cmask = cv::imread("/tmp/mask.png", CV_LOAD_IMAGE_GRAYSCALE);
	if (cmask.empty())
		cerr << "Unable to open mask\n";

	ros::spin();

	delete (mTrans);
	ros::shutdown();
	return 0;
}
