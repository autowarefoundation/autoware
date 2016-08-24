/*
 * SimpleTracker.h
 *
 *  Created on: Aug 11, 2016
 *      Author: hatem
 */

#ifndef SimpleTracker_H_
#define SimpleTracker_H_

#include "RoadNetwork.h"
#include "opencv2/video/tracking.hpp"
#include <vector>
#include "UtilityH.h"
#include "math.h"
#include <iostream>

namespace SimulationNS
{

#define MAX_ASSOCIATION_DISTANCE 2.0

class KFTrack
{
private:
	cv::KalmanFilter m_filter;
	double prev_x, prev_y, prev_v;
	timespec m_Timer;
	long m_id;

public:
	long GetTrackID()
	{
		return m_id;
	}

	KFTrack(double x, double y, double a, long id)
	{
		prev_x = x;
		prev_y = y;
		prev_v = 0;
		UtilityHNS::UtilityH::GetTickCount(m_Timer);

		m_id = id;
		m_filter = cv::KalmanFilter(4,2,0);
		m_filter.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1.0,0.0,1.0,0.0,   0.0,1.0,0.0,1.0,  0.0,0.0,1.0,0.0,  0.0,0.0,0.0,1.0);
		m_filter.statePre.at<float>(0) = x;
		m_filter.statePre.at<float>(1) = y;
		m_filter.statePre.at<float>(2) = 0;
		m_filter.statePre.at<float>(3) = 0;
		setIdentity(m_filter.measurementMatrix);
		cv::setIdentity(m_filter.processNoiseCov, cv::Scalar::all(1e-11));
		cv::setIdentity(m_filter.measurementNoiseCov, cv::Scalar::all(1e-6));
		cv::setIdentity(m_filter.errorCovPost, cv::Scalar::all(.075));
		m_filter.predict();
	}

	void UpdateTracking(const double& x, const double& y, const double& a, double& x_new, double & y_new , double& a_new, double& v)
	{
		cv::Mat_<float> measurement(2,1);
		cv::Mat_<float> prediction(4,1);
		measurement(0) = x;
		measurement(1) = y;

		prediction = m_filter.correct(measurement);
		x_new = prediction.at<float>(0);
		y_new = prediction.at<float>(1);
		v =  fabs(prediction.at<float>(2)*20.0);

		m_filter.predict();
	    /*This prevents the filter from getting stuck in a state when no corrections are executed.*/
		m_filter.statePre.copyTo(m_filter.statePost);
		m_filter.errorCovPre.copyTo(m_filter.errorCovPost);

//		x_new = x;
//		y_new = y;
		double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer);
		UtilityHNS::UtilityH::GetTickCount(m_Timer);
		double d = hypot(y_new - prev_y, x_new - prev_x);
		if(d == 0)
		{
			v = 0;
			//a_new = a;
		}
		else
		{
			v = d/dt;
			//double angle = atan2(y - prev_y, x - prev_x);

//			double diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(
//					UtilityHNS::UtilityH::FixNegativeAngle(angle),
//					UtilityHNS::UtilityH::FixNegativeAngle(prev_a));

			//a_new = angle;
			//std::cout << " >>> Calculated Angle : " << a_new << ", "<< a_new <<", "<< prev_a << ", "<< diff << std::endl;
		}

		if(v < 0.01)
		{
			v = 0;
			//a_new = a;
		}


		prev_x = x;
		prev_y = y;
//		prev_v = v;

	}

	virtual ~KFTrack(){}
};

class SimpleTracker
{
public:
	PlannerHNS::DetectedObject m_Car;
	KFTrack* m_pCarTracker;
	std::vector<KFTrack*> m_Tracks;
	long iTracksNumber;
	PlannerHNS::WayPoint m_PrevState;
	std::vector<PlannerHNS::DetectedObject> m_PrevDetectedObjects;
	std::vector<PlannerHNS::DetectedObject> m_DetectedObjects;

	void CreateTrack(PlannerHNS::DetectedObject& o);
	KFTrack* FindTrack(long index);
	void Track(std::vector<PlannerHNS::DetectedObject>& objects_list);
	void CoordinateTransform(const PlannerHNS::WayPoint& refCoordinate, PlannerHNS::DetectedObject& obj);
	void CoordinateTransformPoint(const PlannerHNS::WayPoint& refCoordinate, PlannerHNS::GPSPoint& obj);
	void Initialize(const PlannerHNS::WayPoint& currPose);
	void AssociateObjects();
	void DoOneStep(const PlannerHNS::WayPoint& currPose, const std::vector<PlannerHNS::DetectedObject>& obj_list);

	SimpleTracker();
	virtual ~SimpleTracker();
};

} /* namespace BehaviorsNS */

#endif /* SimpleTracker_H_ */
