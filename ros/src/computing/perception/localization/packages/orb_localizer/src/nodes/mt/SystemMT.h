/*
 * SystemMT.h
 *
 *  Created on: Jul 22, 2016
 *      Author: sujiwo
 */

#ifndef _SYSTEMMT_H_
#define _SYSTEMMT_H_


#include <string>
#include <vector>
#include <mutex>
#include <boost/thread.hpp>

#include <tf/tf.h>

//#include "System.h"
#include "TrackingThread.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "ParticleFilter.h"

//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>


using namespace std;

#define frameTimeout 0.1		// seconds
#define NUMBER_OF_PARTICLE 500


struct CamState : public tf::Transform
{
	CamState (const tf::Transform &tfsc)
	{
		setOrigin(tfsc.getOrigin());
		setRotation(tfsc.getRotation());
		velocity.setX(0.0); velocity.setY(0.0); velocity.setZ(0.0);
	}

	CamState () {}

	CamState (float dt[8])
	{
		setOrigin(tf::Vector3(dt[0], dt[1], dt[2]));
		setRotation(tf::Quaternion(dt[3], dt[4], dt[5], dt[6]));
		velocity.setX(0.0); velocity.setY(0.0); velocity.setZ(0.0);
	}

//	tfScalar& x() { return getOrigin().m_floats[0]; }
//	tfScalar& y() { return getOrigin().m_floats[1]; }
//	tfScalar& z() { return getOrigin().m_floats[2]; }
//	tfScalar& qx() { return getRotation().m_floats[0]; }
//	tfScalar& qx() { return getRotation().m_floats[0]; }
//	tfScalar& qx() { return getRotation().m_floats[0]; }
//	tfScalar& qx() { return getRotation().m_floats[0]; }

	// Velocity vector
	tf::Vector3 velocity;
};


struct Motion: public tf::Vector3
{
Motion()
{}

public:
	double timestamp;
};


class OrbMapFusion :
	public PF::VehicleBase<CamState, tf::Transform, double>
{
public:
	OrbMapFusion ();
	void preinitialize (const tf::Transform &initialPose, const double t);

	CamState initializeParticleState () const;

	CamState motionModel (const CamState &vstate, const double &v) const;

	double measurementModel (const CamState &state, const vector<tf::Transform> &observations) const;

	bool isInitialized () const { return initialized; }

private:
	tf::Transform initPose;
	bool initialized;
	double prevTimestamp;
};



class SystemMT
{
public:

	SystemMT (ros::NodeHandle &nh, const vector<string> &mapPaths, const string &vocabPath, const string &settingsPath);
	~SystemMT ();

	void Track (const cv::Mat &srcImage, const double timestamp);

	const cv::FileStorage *getSettings () { return &fSetting; }

	friend class TrackingThread;

private:
	ros::NodeHandle &rosnode;
	vector<TrackingThread*> children;
	ORB_SLAM2::ORBVocabulary *sVocab;
	const string &settingPath;

	cv::FileStorage fSetting;

	boost::barrier readyCheck;

	// Monitor for localizer threads
	boost::condition_variable imgMon;
	boost::mutex imgLock;
	volatile bool imgIsNew;

	cv::Mat currentImage;
	double currentTimestamp;

	void filter ();

	PF::ParticleFilter<CamState, tf::Transform, double> *pfilter;
	OrbMapFusion vehicleModel;
};

#endif /* _SYSTEMMT_H_ */
