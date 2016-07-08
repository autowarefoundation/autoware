/*
 * ImageGrabber.cc
 *
 *  Created on: May 31, 2016
 *      Author: sujiwo
 */

#include "ImageGrabber.h"
#include "boost/date_time/posix_time/posix_time.hpp"


using namespace std;
using namespace boost::posix_time;

using ORB_SLAM2::Frame;
namespace enc = sensor_msgs::image_encodings;



ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM, bool runOffline) :
	mpSLAM(pSLAM),
	doStop (false),
	doDebayer (false),
	offlineMode (runOffline),
	mTfBr (NULL),
	extListener (NULL)
{
	// External localization
	extFrame1 = (string)mpSLAM->fsSettings["ExternalLocalization.frame1"];
	extFrame2 = (string)mpSLAM->fsSettings["ExternalLocalization.frame2"];
	cout << "External Reference: from " << extFrame1 << " to " << extFrame2 << endl;

	offsetKeyframe = (int)mpSLAM->fsSettings["ExternalLocalization.OffsetKeyframes"];

	// Initialize TF
	if (offlineMode==false) {
		tf::Transform tfT;
		tfT.setIdentity();
		mTfBr = new tf::TransformBroadcaster();

		mTfBr->sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));
	}

	logFd.open("/tmp/orb_slam.log", ios_base::out);
	logFd << std::fixed << setprecision(7);
}


ImageGrabber::~ImageGrabber()
{
	if (mTfBr != NULL)
		delete (mTfBr);
	if (extListener != NULL)
		delete (extListener);
}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
	// Activate this timer if you need time logging
//	ros::Time rT1, rT2;
	ptime rT1, rT2;
	double rtd;
	rT1 = microsec_clock::local_time();

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

	cv::Mat image;
	// Check if we need debayering
	if (enc::isBayer(msg->encoding)) {
		int code=-1;
		if (msg->encoding == enc::BAYER_RGGB8 ||
			msg->encoding == enc::BAYER_RGGB16) {
//			cout << "BGR2BGR" << endl;
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

	const double imageTime = msg->header.stamp.toSec();

	// Do Resizing and cropping here
	cv::resize(image, image,
		cv::Size(
			(int)mpSLAM->fsSettings["Camera.WorkingResolution.Width"],
			(int)mpSLAM->fsSettings["Camera.WorkingResolution.Height"]
		));
	image = image(
		cv::Rect(
			(int)mpSLAM->fsSettings["Camera.ROI.x0"],
			(int)mpSLAM->fsSettings["Camera.ROI.y0"],
			(int)mpSLAM->fsSettings["Camera.ROI.width"],
			(int)mpSLAM->fsSettings["Camera.ROI.height"]
		)).clone();

	mpSLAM->TrackMonocular(image, imageTime);

	// Reinsert TF publisher, but only for localization. Original ORB-SLAM2 removes it.
	bool tfOk = false;
	tf::Transform locRef;
	Frame &cframe = mpSLAM->getTracker()->mCurrentFrame;
	if (mpSLAM->opMode==ORB_SLAM2::System::LOCALIZATION and
		!cframe.mTcw.empty() and
		offlineMode == false
	) {

		tf::Transform tfTcw = FramePose(&cframe);
		mTfBr->sendTransform(tf::StampedTransform(tfTcw, ros::Time(imageTime), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));

	} else { }

	rT2 = microsec_clock::local_time();
	rtd = (rT2-rT1).total_microseconds() * 1e-6;

	// Log to file
	// Image timestamp
	logFd << imageTime << " ";
	// General status; 1: OK, 0: Lost
	logFd << (int)tfOk << " ";
	// Tracking mode
	logFd << (int)mpSLAM->getTracker()->lastTrackingMode << " ";
	// Frame processing time
	logFd << rtd;

	if (mpSLAM->opMode==ORB_SLAM2::System::LOCALIZATION) {
		if (!cframe.mTcw.empty()) {
			tf::Vector3 &p = locRef.getOrigin();
			logFd << " " << p.x() << " " << p.y() << " " << p.z();
		}
		else {
			logFd << " NaN NaN NaN";
		}
	}

	logFd << endl;
	logFd.flush();
}


void ImageGrabber::externalLocalizerGrab()
{
	if (extListener==NULL)
		extListener = new tf::TransformListener ();

	ros::Rate fps((int)mpSLAM->fsSettings["Camera.fps"] * 2);

	while (ros::ok()) {

		if (doStop == true)
			break;

		try {

			extListener->lookupTransform (extFrame1, extFrame2, ros::Time(0), extPose);
			unique_lock<mutex> lock(ORB_SLAM2::KeyFrame::extPoseMutex);
			tfToCV (extPose, ORB_SLAM2::KeyFrame::extEgoPosition, ORB_SLAM2::KeyFrame::extEgoOrientation);

		} catch (tf::TransformException &e) {

			unique_lock<mutex> lock(ORB_SLAM2::KeyFrame::extPoseMutex);
			ORB_SLAM2::KeyFrame::extEgoPosition.release();
			ORB_SLAM2::KeyFrame::extEgoOrientation.release();

		}

		fps.sleep();
	}
}



tf::Transform ImageGrabber::localizeByReference(Frame *sframe)
{
//	const tf::Transform
}
