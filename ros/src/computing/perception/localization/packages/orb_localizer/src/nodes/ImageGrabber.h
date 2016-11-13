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

// I Know this is unstable
#include <tf2_msgs/TFMessage.h>
#include <orb_localizer/debug.h>

using namespace std;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Frame;


class ImageGrabber
{
public:
	ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle *nh, bool runOffline=false);
	~ImageGrabber ();

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    // This function runs in separate thread
    void externalLocalizerGrab ();

    ORB_SLAM2::System* mpSLAM;
    ros::NodeHandle *rosNode;

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

    static tf::Transform localizeByReference (const tf::Transform &tfOrb, ORB_SLAM2::Map *mapsrc, const int offsetNum);

	static tf::Transform getKeyFrameExtPose (const KeyFrame *kf);

	static tf::Transform KeyFramePoseToTf (KeyFrame *kf);

	static tf::Transform FramePose (Frame *cframe);

	static cv::Vec3d tfToCv (const tf::Vector3 &pos);

	static cv::Mat tfToCv (const tf::Transform &tfsrc);

	// I Know this is unstable
	static tf2_msgs::TFMessage createTfMessage (const tf::Transform &srcTransform, const string &frameSrc, const string &frameTarget, double timestamp);


	// Logging
	image_transport::ImageTransport *imageTransport;
	image_transport::Publisher visualDebugView;
	ros::Publisher debugMsgPublisher;

	cv::Mat framebufferDebug;
	uint32_t lastKeyframeId;
	double cputimeDebug;
	double lastImageTimestamp;

private:
	void publishDebug ();
};


#endif /* _IMAGEGRABBER_H_ */
