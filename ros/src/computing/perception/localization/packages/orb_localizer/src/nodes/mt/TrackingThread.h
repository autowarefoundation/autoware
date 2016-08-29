#ifndef _TRACKING_THREAD_H
#define _TRACKING_THREAD_H

#include <vector>
#include <string>
#include <thread>
#include <condition_variable>


#include "Tracking.h"
#include "FrameDrawer.h"
#include "DebugMT.h"


using namespace std;


class SystemMT;


class TrackingThread
{
public:

	TrackingThread (const string &mapFilename, SystemMT *_sysmt, const string &ids);
	~TrackingThread ();

	void trackImage ();

	void run ();
	void stop ();

	friend class SystemMT;
	friend class DebugMT;

private:
	ORB_SLAM2::Map *tMap;
	ORB_SLAM2::KeyFrameDatabase *kfdb;
	ORB_SLAM2::Tracking *tTrack;
	ORB_SLAM2::FrameDrawer *framedraw;

	DebugMT *debugger;

	const string mapFilename;
	string identity;
	const int offsetKeyframeNum;

	const string parentFrame;
	const string targetFrame;

	SystemMT *sysParent;
	volatile bool doStop;
	thread *proc;

	void output (cv::Mat &trackOutput);

	union {
		struct {
			float x, y, z, qx, qy, qz, qw;
		};
		float data[8];
	} currentPose;

	inline bool poseIsValid()
	{
		return !isnanf(currentPose.x)
			and !isnanf(currentPose.y)
			and !isnanf(currentPose.z)
			and !isnanf(currentPose.qx)
			and !isnanf(currentPose.qy)
			and !isnanf(currentPose.qz)
			and !isnanf(currentPose.qw);
	}

	tf::Transform getCurrent ();
	double getLastTime ();
};


#define invalidPose \
	for (auto __i=0; __i<8; __i++) currentPose.data[__i] = NAN;


#endif
