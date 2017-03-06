/*
 * FusionOdometry.h
 *
 *  Created on: Jan 25, 2017
 *      Author: sujiwo
 */

#ifndef _FUSIONODOMETRY_H_
#define _FUSIONODOMETRY_H_


#include <Eigen/Core>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "ParticleFilter.h"
#include <vector>
#include <functional>


typedef vector<geometry_msgs::PoseWithCovarianceStamped> ObservationList;


inline bool isInvalidPose (const geometry_msgs::PoseWithCovarianceStamped &p)
{
	return (p.pose.pose.position.x>=1e6 and p.pose.pose.position.y>=1e6 and p.pose.pose.position.z>=1e6);
}


struct VehicleState {
	Eigen::Vector3d position;
	double roll, pitch, yaw;

	VehicleState () :
		position (Eigen::Vector3d::Zero()),
		yaw (0.0),
		pitch (0.0),
		roll (0.0)
	{}

	tf::Transform toTf () const
	{
		tf::Transform ctrans;
		ctrans.setOrigin (tf::Vector3(position.x(), position.y(), position.z()));
		ctrans.getRotation().setEuler(yaw, pitch, roll);
		return ctrans;
	}
};


/*
 * This motionControl represents motion in 2D
 */
struct MotionControl {
	double
		velocity,
		yawrate,
		timestamp,
		timediff;
	MotionControl() :
		velocity(0.0),
		yawrate(0.0),
		timestamp(-1),
		timediff(0)
	{}
	static MotionControl fromOdometryMsg (const nav_msgs::OdometryConstPtr &msg);
};


class FusionOdometry :
	public PF::VehicleBase<VehicleState, geometry_msgs::PoseWithCovarianceStamped, MotionControl>
{
public:

	FusionOdometry (ros::NodeHandle &nh, const string &odomTopic);

//	template<class T>
//	void setPoseFunction (ObservationList (T::*fp)(void), T* obj)
//	{ poseFunc = boost::bind (fp, obj); }
//
//	void setPoseFunction (ObservationList(*fp)())
//	{ poseFunc = boost::function<ObservationList()> (fp); }
	void setPoseFunction (std::function<ObservationList()> posefunc)
	{ poseFunc = posefunc; }

	VehicleState initializeParticleState () const;
	VehicleState motionModel (const VehicleState &s, const MotionControl &ct) const;
	double measurementModel (const VehicleState &vs, const vector<geometry_msgs::PoseWithCovarianceStamped> &observations) const;

	void callback (const nav_msgs::OdometryConstPtr &msg);


protected:
	ros::NodeHandle rosnode;
	ros::Subscriber odomSub;
	tf::TransformBroadcaster *tfPub;

	PF::ParticleFilter<VehicleState, geometry_msgs::PoseWithCovarianceStamped, MotionControl> filter;
//	geometry_msgs::PoseWithCovarianceStamped (*getPoseObservation)

	double prevTime;
	unsigned long int prevCount;
	bool positionFound;
	geometry_msgs::PoseWithCovarianceStamped currentPose, lastGoodPose;

//	boost::function<ObservationList(void)> poseFunc;
	std::function<ObservationList()> poseFunc;

	std::vector<VehicleState*> pfStates;

	ros::Publisher pfDebugPub;
	void publish ();

	static VehicleState getAverage (const vector<VehicleState*> &pfStates);
};

#endif /* _FUSIONODOMETRY_H_ */
