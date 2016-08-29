/*
 * DebugMT.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: sujiwo
 */

#include "DebugMT.h"
#include "TrackingThread.h"



extern const char
	*framebufferDebugTopic,
	*internalTopic;



DebugMT::DebugMT (TrackingThread *t, ros::NodeHandle &nh, const string &sid) :

	node (nh),
	identifier (sid),
	proc (t)

{
	imageTransport = new image_transport::ImageTransport (node);
	visualDebugView = imageTransport->advertise(string(framebufferDebugTopic) + "/map" + identifier, 1);
	debugMsgPublisher = node.advertise<orb_localizer::debug> (string(internalTopic) + "/map" + identifier, 1);
}


DebugMT::~DebugMT()
{}


void DebugMT::notify()
{
	mTfBr.sendTransform(tf::StampedTransform(
		proc->getCurrent(),
		ros::Time (proc->getLastTime()),
		proc->parentFrame,
		proc->targetFrame
	));

	proc->framedraw->DrawFrame();
	framebufferDebug = proc->framedraw->getLastFrame();

	cv_bridge::CvImage bagImage;
	bagImage.image = framebufferDebug;
	bagImage.header.stamp = ros::Time(lastImageTimestamp);
	bagImage.header.frame_id = identifier;
	bagImage.encoding = "bgr8";
	visualDebugView.publish(bagImage.toImageMsg());

//	orb_localizer::debug internalDebugMsg;
//	internalDebugMsg.header.stamp = ros::Time (lastImageTimestamp);
//	internalDebugMsg.header.frame_id = identifier;
//	internalDebugMsg.keyframe_id = lastKeyframeId;
//	internalDebugMsg.cputime = cputimeDebug;
//	debugMsgPublisher.publish (internalDebugMsg);

}
