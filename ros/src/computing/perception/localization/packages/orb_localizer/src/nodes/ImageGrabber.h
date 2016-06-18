/*
 * ImageGrabber.h
 *
 *  Created on: May 31, 2016
 *      Author: sujiwo
 */

#ifndef _IMAGEGRABBER_H_
#define _IMAGEGRABBER_H_


#include <string>


#include "System.h"
#include "Map.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "utils.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>


using namespace std;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Frame;


class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM2::System* pSLAM, bool runOffline=false);
	~ImageGrabber ();

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    // This function runs in separate thread
    void externalLocalizerGrab ();

    ORB_SLAM2::System* mpSLAM;

    // External localization
    tf::TransformBroadcaster *mTfBr;
    tf::TransformListener *extListener;
    tf::StampedTransform extPose;
    string extFrame1, extFrame2;
    bool doStop;
    bool doDebayer;
    bool offlineMode;
    int offsetKeyframe;


    tf::Transform localizeByReference (const tf::Transform &tfOrb, KeyFrame *kf);
    tf::Transform localizeByReference (const tf::Transform &tfOrb);
    tf::Transform localizeByReference (Frame *sframe);

    static tf::Transform localizeByReference (
    	const tf::Transform &tfOrb,
		const tf::Transform &tfOrbMap, const tf::Transform &tfOrbMapOffset,
    	const tf::Transform &realMapPose, const tf::Transform &realMapOffset);



	static inline tf::Transform getKeyFrameExtPose (const KeyFrame *kf)
	{
		tf::Transform Ext;
		Ext.setOrigin (tf::Vector3(
			kf->extPosition.at<double>(0),
			kf->extPosition.at<double>(1),
			kf->extPosition.at<double>(2) ));
		Ext.setRotation(tf::Quaternion(
			kf->extOrientation.at<double>(0),
			kf->extOrientation.at<double>(1),
			kf->extOrientation.at<double>(2),
			kf->extOrientation.at<double>(3) ));
		return Ext;
	}


	static inline tf::Transform KeyFramePoseToTf (KeyFrame *kf)
	{
		tf::Transform kfpose;

		cv::Mat t = kf->GetCameraCenter();
		cv::Mat orient = kf->GetRotation().t();
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(orient);

		kfpose.setOrigin(tf::Vector3(t.at<float>(0), t.at<float>(1), t.at<float>(2)));
		kfpose.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));

		return kfpose;
	}


	static inline tf::Transform FramePose (Frame *cframe)
	{
		cv::Mat Rwc = cframe->mTcw.rowRange(0,3).colRange(0,3).t();
		cv::Mat twc = -Rwc * cframe->mTcw.rowRange(0,3).col(3);
		tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
						Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
						Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
		tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

		return tf::Transform(M, V);
	}


	static inline cv::Vec3d tfToCv (const tf::Vector3 &pos)
	{
		cv::Vec3d cvVec;
		cvVec[0] = pos.x();
		cvVec[1] = pos.y();
		cvVec[2] = pos.z();
		return cvVec;
	}
};


#endif /* _IMAGEGRABBER_H_ */
