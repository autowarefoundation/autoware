/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  XXX: Licensing has not been cleared yet.
*/

#include <iostream>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "System.h"
#include "../common.h"


using namespace std;
using namespace ORB_SLAM2;
using namespace boost::posix_time;
namespace enc = sensor_msgs::image_encodings;


class ORB_Mapper
{
public:
	ORB_Mapper (ORB_SLAM2::System& pSL, ros::NodeHandle &nh, int fps_=10) :
		SLAMSystem (pSL),
		rosnode (nh),
		extListener (NULL),
		lastImageTimestamp (0.0),
		doStop (false),
		fps (fps_),
		gotFirstFrame (false)
	{
		// Image Subscription
	    if ((int)SLAMSystem.fsSettings["Camera.compressed"]==0) {
	    	th = image_transport::TransportHints ("raw");
	    }
	    else if ((int)SLAMSystem.fsSettings["Camera.compressed"]==1) {
	    	th = image_transport::TransportHints ("compressed");
	    }
		imageBuf = new image_transport::ImageTransport(rosnode);
//		imageSub = imageBuf->subscribe((string)SLAMSystem.fsSettings["Camera.topic"], 1, &ORB_Mapper::imageCallback, this, th);

		string groundTruthPose;
		rosnode.getParam("external_localization_topic", groundTruthPose);
		poseSub = rosnode.subscribe (groundTruthPose, 2, &ORB_Mapper::poseCallback, this);
	}


	~ORB_Mapper ()
	{
		delete (imageBuf);
		if (extListener != NULL)
			delete (extListener);
	}


	void slamFeeder ()
	{
		ros::Rate hzs (fps);

		while (doStop==false) {
			hzs.sleep();
		}
	}


	void poseCallback (const geometry_msgs::PoseStampedConstPtr &cpose)
	{
//		printf ("%f %f %f\n", cpose->pose.position.x, cpose->pose.position.y, cpose->pose.position.z);
		unique_lock <mutex> lock (ORB_SLAM2::KeyFrame::extPoseMutex);
		ORB_SLAM2::KeyFrame::extEgoPosition = cv::Mat (3,1,CV_64F);
		ORB_SLAM2::KeyFrame::extEgoPosition.at<double>(0) = cpose->pose.position.x,
		ORB_SLAM2::KeyFrame::extEgoPosition.at<double>(1) = cpose->pose.position.y,
		ORB_SLAM2::KeyFrame::extEgoPosition.at<double>(2) = cpose->pose.position.z;

		ORB_SLAM2::KeyFrame::extEgoOrientation = cv::Mat (4,1,CV_64F);
		ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(0) = cpose->pose.orientation.x,
		ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(1) = cpose->pose.orientation.y,
		ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(2) = cpose->pose.orientation.z,
		ORB_SLAM2::KeyFrame::extEgoOrientation.at<double>(3) = cpose->pose.orientation.w;
	}


	void imageCallback (const sensor_msgs::ImageConstPtr& msg)
	{
		// Activate this timer if you need time logging
		ptime rT1, rT2;
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
		lastImageTimestamp = imageTime;

		if (gotFirstFrame==false) {
			double fx2, fy2, cx2, cy2;
			recomputeNewCameraParameter (
				(double)SLAMSystem.fsSettings["Camera.fx"],
				(double)SLAMSystem.fsSettings["Camera.fy"],
				(double)SLAMSystem.fsSettings["Camera.cx"],
				(double)SLAMSystem.fsSettings["Camera.cy"],
				fx2, fy2, cx2, cy2,
				msg->width, msg->height,
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Width"],
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Height"]);
			// send camera parameters to tracker
			SLAMSystem.getTracker()->ChangeCalibration (fx2, fy2, cx2, cy2);
			gotFirstFrame = true;
		}

		// Processing before sending image to tracker
		// Do Resizing and cropping here
		cv::resize(image, image,
			cv::Size(
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Width"],
				(int)SLAMSystem.fsSettings["Camera.WorkingResolution.Height"]
			));
		image = image(
			cv::Rect(
				(int)SLAMSystem.fsSettings["Camera.ROI.x0"],
				(int)SLAMSystem.fsSettings["Camera.ROI.y0"],
				(int)SLAMSystem.fsSettings["Camera.ROI.width"],
				(int)SLAMSystem.fsSettings["Camera.ROI.height"]
			)).clone();

		SLAMSystem.TrackMonocular(image, imageTime);

	}


private:
	// External localization
	tf::TransformListener *extListener;
	tf::StampedTransform extPose;

	ros::NodeHandle &rosnode;
	ORB_SLAM2::System &SLAMSystem;
	ros::Subscriber poseSub;

//	string externalFrameFixed;
//	string externalFrameMoving;

	double lastImageTimestamp;
	bool gotFirstFrame;

	int fps;

	mutex mtxCImage;
	cv::Mat currentImage;

public:
	volatile bool doStop;

	image_transport::TransportHints th;
	image_transport::ImageTransport *imageBuf;
	image_transport::Subscriber imageSub;

	thread slamFeederT;

};






int main (int argc, char *argv[])
{
	const string orbVocabFile (ORB_SLAM_VOCABULARY);

	ros::init(argc, argv, "orb_mapping", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler("~");

	string mapPath;
	nodeHandler.getParam("map_file", mapPath);

	string configFile;
	nodeHandler.getParam("configuration_file", configFile);

	ORB_SLAM2::System SLAM(orbVocabFile,
		configFile,
		ORB_SLAM2::System::MONOCULAR,
		true,
		mapPath,
		System::MAPPING);

	ORB_Mapper Mapper (SLAM, nodeHandler);

	string imageTopic;
	nodeHandler.getParam("image_topic", imageTopic);
	Mapper.imageSub = Mapper.imageBuf->subscribe (imageTopic, 100,  &ORB_Mapper::imageCallback, &Mapper, Mapper.th);

	ros::spin();
    Mapper.doStop = true;
//    Mapper.externalLocalizerThread->join();

    ros::shutdown();
    SLAM.Shutdown();

    return 0;
}
