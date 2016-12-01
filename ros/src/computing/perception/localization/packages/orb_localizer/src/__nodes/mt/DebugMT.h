/*
 * DebugMT.h
 *
 *  Created on: Jul 28, 2016
 *      Author: sujiwo
 */

#ifndef _DEBUGMT_H_
#define _DEBUGMT_H_


#include <string>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <orb_localizer/debug.h>


using namespace std;


class TrackingThread;


class DebugMT {
public:
	DebugMT (TrackingThread *th, ros::NodeHandle &nh, const string &sid);
	virtual ~DebugMT();

	void notify ();

	void publishParticles ();

private:
	TrackingThread *proc;
	ros::NodeHandle &node;
	const string &identifier;
	tf::TransformBroadcaster mTfBr;

	image_transport::ImageTransport *imageTransport;
	image_transport::Publisher visualDebugView;
	ros::Publisher debugMsgPublisher;

	cv::Mat framebufferDebug;
	uint32_t lastKeyframeId;
	double cputimeDebug;
	double lastImageTimestamp;

};

#endif /* _DEBUGMT_H_ */
