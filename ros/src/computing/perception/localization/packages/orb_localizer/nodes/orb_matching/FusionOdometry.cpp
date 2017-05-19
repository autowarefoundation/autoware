/*
 * FusionOdometry.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: sujiwo
 */

#include <numeric>

#include "FusionOdometry.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define NumberOfParticles 500
#define InitialPositionRandomizer 0.5

void QuaternionToRPY (const double &qx, const double &qy, const double &qz, const double &qw,
		double &roll, double &pitch, double &yaw)
{
	tf::Matrix3x3 mq(tf::Quaternion(qx, qy, qz, qw));
	mq.getRPY(roll, pitch, yaw);
}

MotionControl MotionControl::fromOdometryMsg (const nav_msgs::OdometryConstPtr &msg)
{
	MotionControl motion;
	motion.velocity = msg->twist.twist.linear.x;
	motion.timestamp = msg->header.stamp.toSec();
	motion.yawrate = msg->twist.twist.angular.x;
	return motion;
}


FusionOdometry::FusionOdometry(ros::NodeHandle &nh, const string &odomTopic) :
	rosnode (nh),
	filter (NumberOfParticles, *this),
	prevCount (0),
	prevTime (0),
	positionFound (false),
	tfPub (new tf::TransformBroadcaster)
{
	odomSub = rosnode.subscribe(odomTopic, 100, &FusionOdometry::callback, this);
	pfDebugPub = rosnode.advertise<visualization_msgs::Marker> ("fusion_odometry", 1);
	pfStates.resize(filter.getNumberOfParticles());
}


VehicleState
FusionOdometry::initializeParticleState () const
{
	VehicleState cst;
	cst.position.x() = currentPose.pose.pose.position.x;
	cst.position.y() = currentPose.pose.pose.position.y;
	cst.position.z() = currentPose.pose.pose.position.z;

	QuaternionToRPY(
		currentPose.pose.pose.orientation.x,
		currentPose.pose.pose.orientation.y,
		currentPose.pose.pose.orientation.z,
		currentPose.pose.pose.orientation.w,
		cst.roll, cst.pitch, cst.yaw);

	// Randomize
	cst.position.x() += PF::nrand(InitialPositionRandomizer);
	cst.position.y() += PF::nrand(InitialPositionRandomizer);

	return cst;
}


VehicleState
FusionOdometry::motionModel (const VehicleState &vstate, const MotionControl &ctrl) const
{
	VehicleState nstate;
	double dt = ctrl.timediff;
	double roll, pitch, yaw;
	double vx, vy;

	// disturb velocity & yawrate with some random noise
#define VELOCITY_STD 1.0
#define YAWRATE_STD 0.9
#define VELOCITY_THRESHOLD 0.08

	double velocity, yawrate;
	if (ctrl.velocity <= VELOCITY_THRESHOLD)
		velocity = 0;
	else
		velocity = ctrl.velocity + PF::nrand(VELOCITY_STD);
	yawrate = ctrl.yawrate + PF::nrand(YAWRATE_STD);

	vx = velocity * sin(vstate.yaw);
	vy = velocity * cos(vstate.yaw);

	nstate.position.x() = vstate.position.x() - vx*dt;
	nstate.position.y() = vstate.position.y() + vy*dt;
	if (isInvalidPose(currentPose)==false)
		nstate.position.z() = currentPose.pose.pose.position.z;
	else
		nstate.position.z() = vstate.position.z();

	QuaternionToRPY (
		lastGoodPose.pose.pose.orientation.x,
		lastGoodPose.pose.pose.orientation.y,
		lastGoodPose.pose.pose.orientation.z,
		lastGoodPose.pose.pose.orientation.w,
		roll, pitch, yaw);
	nstate.roll = roll;
	nstate.pitch = pitch;
	nstate.yaw = vstate.yaw + yawrate*dt;

	return nstate;

}


double
FusionOdometry::measurementModel
(const VehicleState &vs, const vector<geometry_msgs::PoseWithCovarianceStamped> &observations)
const
{
#define DISTANCE_THRESHOLD 2.0

	double orbError = 0.5;
//		return 1;
	vector<double> obsWeights;
	double w = 0;

	for (const auto &pose: observations) {

		double wo,
			xt = pose.pose.pose.orientation.x,
			yt = pose.pose.pose.orientation.y,
			xs = vs.position.x(),
			ys = vs.position.y();

			double distX = xt-xs,
				distY = yt - ys,
				dist = sqrt(distX*distX + distY*distY);
	//				if (dist >= DISTANCE_THRESHOLD)
	//					wo = 0;
	//
	//				else
			wo = exp (-(pow(distX,2) / (2*orbError*orbError) +
						pow(distY,2) / (2*orbError*orbError)
				));

		obsWeights.push_back(wo);
		w += wo;
	}

	w = *std::max_element(obsWeights.begin(), obsWeights.end());
	//		w = w / (double)observations.size();
	return max (w, 1e-5);

}


void
FusionOdometry::callback (const nav_msgs::OdometryConstPtr &msg)
{
	ObservationList currentObservations = poseFunc();
	MotionControl curMotion = MotionControl::fromOdometryMsg(msg);

	// Initialize PF when ready
	if (positionFound==false) {
		if (isInvalidPose(currentObservations[0]) == false) {
			currentPose = currentObservations[0];
			lastGoodPose = currentPose;
			filter.initializeParticles();
			positionFound = true;
			prevTime = curMotion.timestamp;
			cout << "PF initialized" << endl;
		}

		else {
			cout << "PF Uninitialized" << endl;
		}

		return;
	}

	curMotion.timediff = curMotion.timestamp - prevTime;
	filter.update (curMotion, currentObservations);
	prevTime = curMotion.timestamp;

	if (isInvalidPose(currentPose)==false)
		lastGoodPose = currentPose;

	// XXX: Make average pose
//	filter.getStates (pfStates);
	publish();
}


void
FusionOdometry::publish ()
{
	filter.getStates (pfStates);
	VehicleState pfAvg = FusionOdometry::getAverage (pfStates);
	tf::Transform cPose = pfAvg.toTf();
	tfPub->sendTransform(tf::StampedTransform(cPose, currentPose.header.stamp, "world", "vision"));

	visualization_msgs::Marker pfPoints;
	pfPoints.header.frame_id = "world";
	pfPoints.header.stamp = ros::Time::now();
	pfPoints.ns = "particle_filter";
	pfPoints.type = visualization_msgs::Marker::POINTS;
	pfPoints.action = visualization_msgs::Marker::ADD;
	pfPoints.lifetime = ros::Duration (0.5);
	pfPoints.color.r = 1.0;
	pfPoints.color.g = 1.0;
	pfPoints.color.b = 0.0;
	pfPoints.color.a = 0.5;
	pfPoints.scale.x = 0.1;
	pfPoints.scale.y = 0.1;
	pfPoints.scale.z = 0.1;

	for (auto state: pfStates) {
		geometry_msgs::Point ppf;
		ppf.x = state->position.x();
		ppf.y = state->position.y();
		ppf.z = state->position.z();
		pfPoints.points.push_back(ppf);
	}

	pfDebugPub.publish(pfPoints);
}


VehicleState
FusionOdometry::getAverage (const vector<VehicleState*> &pfStates)
{
	VehicleState avg, pInit;

	avg = std::accumulate(pfStates.begin(), pfStates.end(), pInit,
			[](VehicleState accum, VehicleState *c)
			{
				VehicleState stateT;
				stateT.position.x() = accum.position.x() + c->position.x();
				stateT.position.y() = accum.position.y() + c->position.y();
				stateT.position.z() = accum.position.z() + c->position.z();
				stateT.roll = accum.roll + c->roll;
				stateT.pitch = accum.pitch + c->pitch;
				stateT.yaw = accum.yaw + c->yaw;
				return stateT;
			}
		);

	avg.position.x() /= pfStates.size();
	avg.position.y() /= pfStates.size();
	avg.position.z() /= pfStates.size();
	avg.roll /= pfStates.size();
	avg.pitch /= pfStates.size();
	avg.yaw /= pfStates.size();
	return avg;
}
