/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <signal.h>

#include "System.h"
#include "Map.h"
#include "Frame.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include "KeyFrame.h"
#include "Converter.h"
#include "utils.h"



using namespace std;
using ORB_SLAM2::Frame;
namespace enc = sensor_msgs::image_encodings;



class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM2::System* pSLAM) :
		mpSLAM(pSLAM),
		doStop (false),
		doDebayer (false)
	{
		// External localization
		extFrame1 = (string)mpSLAM->fsSettings["ExternalLocalization.frame1"];
		extFrame2 = (string)mpSLAM->fsSettings["ExternalLocalization.frame2"];
		cout << "External Reference: from " << extFrame1 << " to " << extFrame2 << endl;

		// Initialize TF
		tf::Transform tfT;
		tfT.setIdentity();
		mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));
	}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    // This function runs in separate thread
    void externalLocalizerGrab ();

    ORB_SLAM2::System* mpSLAM;

    // External localization
    tf::TransformBroadcaster mTfBr;
    tf::TransformListener extListener;
    tf::StampedTransform extPose;
    string extFrame1, extFrame2;
    bool doStop;
    bool doDebayer;

    tf::Transform localizeByReference (const tf::Transform &tfOrb, ORB_SLAM2::KeyFrame *kf);
    tf::Transform localizeByReference (const tf::Transform &tfOrb);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam_mapper");
    ros::start();

    if(argc < 2)
    {
        cerr << endl << "Usage: orb_slam_mapper path_to_settings [path_to_map]\n" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    string mapPath = (argc==3) ? argv[2] : string();

    // This macro should be set by Cmake
    string orbVocabFile (ORB_SLAM_VOCABULARY);
    ORB_SLAM2::System SLAM(orbVocabFile,argv[1],ORB_SLAM2::System::MONOCULAR,true, mapPath);

    ImageGrabber igb(&SLAM);
    std::thread* extLocalizerThd = new std::thread (&ImageGrabber::externalLocalizerGrab, &igb);

    ros::NodeHandle nodeHandler;
    image_transport::TransportHints th;
    if ((int)SLAM.fsSettings["Camera.compressed"]==0) {
    	th = image_transport::TransportHints ("raw");
    }
    else if ((int)SLAM.fsSettings["Camera.compressed"]==1) {
    	th = image_transport::TransportHints ("compressed");
    }
    image_transport::ImageTransport it (nodeHandler);
    image_transport::Subscriber sub = it.subscribe ((string)SLAM.fsSettings["Camera.topic"], 1, &ImageGrabber::GrabImage, &igb, th);

    cout << endl << "Mono Camera topic: " << (string)SLAM.fsSettings["Camera.topic"] << endl;
    cout << "Compressed images? " << ((int)SLAM.fsSettings["Camera.compressed"]==1 ? "True" : "False") << endl;

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    igb.doStop = true;
    extLocalizerThd->join();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
//	cout << "IMG" << endl;
	// Time record
	ros::Time rT1, rT2;
	double rtd;

	rT1 = ros::Time::now();

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

	mpSLAM->TrackMonocular(image,cv_ptr->header.stamp.toSec());

	// Reinsert TF publisher. Original ORB-SLAM2 removes it.
	Frame &cframe = mpSLAM->getTracker()->mCurrentFrame;
	if (!cframe.mTcw.empty()) {
		cv::Mat Rwc = cframe.mTcw.rowRange(0,3).colRange(0,3).t();
		cv::Mat twc = -Rwc*cframe.mTcw.rowRange(0,3).col(3);
		tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
						Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
						Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
		tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));
		tf::Transform tfTcw(M,V);
		mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));

		// Here, we use offset of external localization from the keyframe
		if (mpSLAM->getTracker()->mbOnlyTracking==true) {
			ORB_SLAM2::KeyFrame *ckf = cframe.mpReferenceKF;
//			cout << "Keyframe localized: " << ckf->mnId << endl;
			try {
				tf::Transform locRef = localizeByReference(tfTcw);
				mTfBr.sendTransform(tf::StampedTransform(locRef, ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/ExtCamera"));
			} catch (...) {}
		}

	} else { }

	rT2 = ros::Time::now();
	rtd = (rT2-rT1).toSec();
	cout << "Timed: " << rtd << endl;
}


void ImageGrabber::externalLocalizerGrab()
{
	ros::Rate fps((int)mpSLAM->fsSettings["Camera.fps"] * 2);

	while (ros::ok()) {

		if (doStop == true)
			break;

		try {
			extListener.lookupTransform (extFrame1, extFrame2, ros::Time(0), extPose);
			unique_lock<mutex> lock(ORB_SLAM2::KeyFrame::extPoseMutex);
			tfToCV (extPose, ORB_SLAM2::KeyFrame::extEgoPosition, ORB_SLAM2::KeyFrame::extEgoOrientation);

		} catch (tf::TransformException &e) {

			unique_lock<mutex> lock(ORB_SLAM2::KeyFrame::extPoseMutex);
			ORB_SLAM2::KeyFrame::extEgoPosition.release();
			ORB_SLAM2::KeyFrame::extEgoOrientation.release();

		}

		fps.sleep();
	}

	// XXX: Make loop here
}


#define DEFAULT_KEYFRAME_OFFSET 10
#define DEFAULT_OFFSET_TIME 2.5


tf::Transform ImageGrabber::localizeByReference (const tf::Transform &tfOrb, ORB_SLAM2::KeyFrame *kf)
{
	ORB_SLAM2::KeyFrame *kOffset = mpSLAM->getMap()->offsetKeyframe(kf, DEFAULT_KEYFRAME_OFFSET);
	if (kOffset==NULL)
		throw std::out_of_range("No offset keyframe found");

	cv::Mat kfPos = kf->GetCameraCenter();
	double offDistO = cv::norm(kfPos - kOffset->GetCameraCenter());
	if (kf->extPosition.empty() or kOffset->extPosition.empty())
		throw std::out_of_range("External reference of keyframe not found");
	double offDistE = cv::norm(kf->extPosition - kOffset->extPosition);
	double scale = offDistE / offDistO;

	tf::Transform flipAxes;
	flipAxes.setOrigin(tf::Vector3(0, 0, 0));
	flipAxes.setRotation (tf::Quaternion(M_PI/2, 0, -M_PI/2).normalize());

	tf::Transform kfTr;
	{
		cv::Mat t = kf->GetCameraCenter();
		cv::Mat orient = kf->GetRotation().t();
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(orient);
		kfTr.setOrigin(tf::Vector3(t.at<float>(0), t.at<float>(1), t.at<float>(2)));
		kfTr.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));
	}
	tf::Transform extRef;
	{
		extRef.setOrigin(tf::Vector3(
				kf->extPosition.at<double>(0),
				kf->extPosition.at<double>(1),
				kf->extPosition.at<double>(2)
			));
		extRef.setRotation(tf::Quaternion(
				kf->extOrientation.at<double>(0),
				kf->extOrientation.at<double>(1),
				kf->extOrientation.at<double>(2),
				kf->extOrientation.at<double>(3)
			));
	}

	tf::Transform orbRel = kfTr.inverse() * tfOrb;
	tf::Transform scaledRel = orbRel;
	scaledRel.setOrigin(orbRel.getOrigin() * scale);
	scaledRel = flipAxes * scaledRel;

	return extRef * scaledRel;
}


tf::Transform ImageGrabber::localizeByReference(const tf::Transform &tfOrb)
{
	ORB_SLAM2::KeyFrame *kfNear = mpSLAM->getMap()->getNearestKeyFrame(
		tfOrb.getOrigin().x(),
		tfOrb.getOrigin().y(),
		tfOrb.getOrigin().z());
	if (kfNear==NULL)
		throw std::out_of_range("No keyframe found");
	return localizeByReference (tfOrb, kfNear);
}
