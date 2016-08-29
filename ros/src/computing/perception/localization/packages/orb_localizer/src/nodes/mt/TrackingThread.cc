#include "TrackingThread.h"
#include "SystemMT.h"
#include "System.h"
// XXX: Refactor ImageGrabber.{cc,h}
#include "../ImageGrabber.h"
#include "DebugMT.h"

#include <tf/tf.h>


using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrameDatabase;
using ORB_SLAM2::Tracking;
using ORB_SLAM2::System;
using ORB_SLAM2::FrameDrawer;



TrackingThread::TrackingThread (const string &mappath, SystemMT *sysmt, const string &ids) :
	sysParent (sysmt),
	doStop (false),
	mapFilename (mappath),
	identity (ids),

	parentFrame ((string)(sysmt->fSetting["ExternalLocalization.frame1"])),
	targetFrame ((string)(sysmt->fSetting["ExternalLocalization.frame2"]) + "/map"+ids),

	offsetKeyframeNum ( (int)sysmt->fSetting["ExternalLocalization.OffsetKeyframes"] )
{
	tMap = new Map ();
	kfdb = new KeyFrameDatabase (*sysParent->sVocab);
	framedraw = new FrameDrawer (tMap);

	debugger = new DebugMT (this, sysParent->rosnode, identity);

	// start map loading from here
	proc = new thread (&TrackingThread::run, this);
}


TrackingThread::~TrackingThread()
{
	doStop = true;
	proc->join ();
	cout << "Localizer " << identity << " ended\n";
}


void TrackingThread::run ()
{
	tMap->loadFromDisk(mapFilename, kfdb);

	tTrack = new Tracking (NULL, sysParent->sVocab, framedraw, NULL, tMap, kfdb, sysParent->settingPath, System::MONOCULAR);
	tTrack->setMapLoaded();
	tTrack->InformOnlyTracking(true);

	cout << "Worker " << identity << " ready" << endl;
	sysParent->readyCheck.wait();

	while (true) {

		{
			boost::unique_lock<boost::mutex> lock (sysParent->imgLock);
			while (sysParent->imgIsNew==false and doStop==false)
				sysParent->imgMon.wait(lock);
			if (doStop==true) {
				cout << "Worker " << identity << " quit" << endl;
				break;
			}
		}

		tTrack->GrabImageMonocular(sysParent->currentImage, sysParent->currentTimestamp);
		Frame &cframe = tTrack->mCurrentFrame;
		if (tTrack->mLastProcessedState==Tracking::OK) {

			tf::Transform orbCPose = ImageGrabber::FramePose(&cframe);
			try {
				tf::Transform cp = ImageGrabber::localizeByReference(orbCPose, tMap, offsetKeyframeNum);
				currentPose.x = cp.getOrigin().x();
				currentPose.y = cp.getOrigin().y();
				currentPose.z = cp.getOrigin().z();
				currentPose.qx = cp.getRotation().x();
				currentPose.qy = cp.getRotation().y();
				currentPose.qz = cp.getRotation().z();
				currentPose.qw = cp.getRotation().w();
			} catch (...) {
				invalidPose;
			}
		}
		else {
			invalidPose;
		}

		// XXX: Do something useful here
		debugger->notify();
//		usleep(50000);

		sysParent->readyCheck.wait();
	}
}


void TrackingThread::stop()
{
	doStop = true;
}


void TrackingThread::output(cv::Mat &out)
{

}


double TrackingThread::getLastTime()
{ return sysParent->currentTimestamp; }


tf::Transform TrackingThread::getCurrent ()
{
	tf::Transform p;
	p.setOrigin (tf::Vector3(currentPose.x, currentPose.y, currentPose.z));
	p.setRotation(tf::Quaternion(currentPose.qx, currentPose.qy, currentPose.qz, currentPose.qw));
	return p;
}
