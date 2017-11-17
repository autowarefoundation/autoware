/*
 * Matcher.h
 *
 *  Created on: Feb 15, 2017
 *      Author: sujiwo
 */

#ifndef _MATCHER_H_
#define _MATCHER_H_


#include <iostream>
#include <string>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "System.h"
#include "Converter.h"
#include "../common.h"
#include <orb_localizer/debug.h>

using namespace std;
using namespace ORB_SLAM2;
using namespace boost::posix_time;
namespace enc = sensor_msgs::image_encodings;


class Matcher
{

public:

	Matcher (ros::NodeHandle &nh, bool _doPublish=true);
	~Matcher();

	void imageCallback (const sensor_msgs::ImageConstPtr& msg);

	tf::Transform getCurrentRealPose ()
	{ return currentRealPose; }

	double getLastLocalizationTimestamp ()
	{ return lastGoodLocalizationTimestamp; }

	/*
	 * Main routine for localization by reference
	 */
	static tf::Transform localizeByReference (
		const tf::Transform &orbOriginalResult,
		const tf::Transform &orbMap, const tf::Transform &orbMapOffset,
		const tf::Transform &realMapPose, const tf::Transform &realMapOffset);

//	enum PublishMode {
//		SEND_REAL_COORD,
//		SEND_ORB_COORD,
//		SEND_BOTH_COORD
//	} rosPublishMode;

private:
	tf::Transform localizeByReference(const tf::Transform &tfOrb);
	tf::Transform localizeByReference (const tf::Transform &tfOrb, ORB_SLAM2::KeyFrame *kf);

	void publishPose (const tf::Transform *tf);

protected:
	bool doPublish;
	tf::StampedTransform extPose;

	ros::NodeHandle &rosnode;
	ORB_SLAM2::System *SLAMSystem;
	ros::Publisher posePublisher;
	ros::Publisher poseCovPublisher;
	tf::TransformBroadcaster *mTfBr;

	// Debug
	image_transport::Publisher visualDebugView;
	ros::Publisher debugMsgPublisher;
	float cpuTimeDebug;

	string externalFrameFixed;
	string externalFrameMoving;

	double lastImageTimestamp;
	double lastGoodLocalizationTimestamp;
	bool gotFirstFrame;

	// Logging
	uint32_t lastKeyframeId;
	int offsetKeyframe;

	// Current pose and mutex
	tf::Transform
		currentRealPose,		// In metric
		currentOrbPose;			// In ORB
	std::mutex cPoseX;

	image_transport::TransportHints th;
	image_transport::ImageTransport *imageBuf;
	image_transport::Subscriber imageSub;

};

#endif /* _MATCHER_H_ */
