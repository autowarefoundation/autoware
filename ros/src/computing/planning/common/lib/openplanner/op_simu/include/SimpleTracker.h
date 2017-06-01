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
#include <math.h>
#include <iostream>

namespace SimulationNS
{

#define DEBUG_TRACKER 0
#define NEVER_GORGET_TIME -1000
#define MIN_EVIDENCE_NUMBER 3

struct Kalman1dState
{
    double MovCov; //double q; //moving noise covariance
    double MeasureCov; //double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain
};

class  kalmanFilter1D
{
public:

	Kalman1dState result;

    kalmanFilter1D()
	{

	}
    kalmanFilter1D(double MovCov, double MeasureCov, double p, double intial_value)
    {
        result.MovCov = MovCov;
        result.MeasureCov = MeasureCov;
        result.p = p;
        result.x = intial_value;
    }

    Kalman1dState Update(double measurement)
    {
    	//prediction update
		//omit x = x
		result.p = result.p + result.MovCov;

		//measurement update
		result.k = result.p / (result.p + result.MeasureCov);
		result.x = result.x + result.k * (measurement - result.x);
		result.p = (1 - result.k) * result.p;

		return result;
    }
};

class KFTrackV
{
private:
	cv::KalmanFilter m_filter;
	double prev_x, prev_y, prev_v, prev_a;
	long m_id;
	double dt;
	int nStates;
	int nMeasure;
	double circ_angle;

public:
	int region_id;
	double forget_time;
	int m_iLife;
	PlannerHNS::DetectedObject obj;
	kalmanFilter1D errorSmoother;

	long GetTrackID()
	{
		return m_id;
	}

	KFTrackV(double x, double y, double a, long id, double _dt)
	{
		circ_angle = 0;
		errorSmoother.result.MovCov = 0.125;
		errorSmoother.result.MeasureCov = 0.1;
		errorSmoother.result.p = 1;
		errorSmoother.result.x = 0;
		region_id = -1;
		forget_time = NEVER_GORGET_TIME; // this is very bad , dangerous
		m_iLife = 0;
		dt = _dt;
		prev_x = x;
		prev_y = y;
		prev_v = 0;
		prev_a = a;
		nStates = 4;
		nMeasure = 2;

		m_id = id;

		m_filter = cv::KalmanFilter(nStates,nMeasure);
#if (CV_MAJOR_VERSION == 2)
		m_filter.transitionMatrix = *(cv::Mat_<float>(nStates, nStates) << 1	,0	,dt	,0  ,
				0	,1	,0	,dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#elif (CV_MAJOR_VERSION == 3)
		m_filter.transitionMatrix = (cv::Mat_<float>(nStates, nStates) << 1	,0	,dt	,0  ,
				0	,1	,0	,dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#endif		

		m_filter.statePre.at<float>(0) = x;
		m_filter.statePre.at<float>(1) = y;
		m_filter.statePre.at<float>(2) = 0;
		m_filter.statePre.at<float>(3) = 0;

		m_filter.statePost = m_filter.statePre;

		setIdentity(m_filter.measurementMatrix);

		cv::setIdentity(m_filter.measurementNoiseCov, cv::Scalar::all(0.0001));
		cv::setIdentity(m_filter.processNoiseCov, cv::Scalar::all(0.0001));
		cv::setIdentity(m_filter.errorCovPost, cv::Scalar::all(0.075));

		m_filter.predict();

		errorSmoother.Update(a);
	}

	void UpdateTracking(double _dt, const double& x, const double& y, const double& a, double& x_new, double & y_new , double& a_new, double& v)
	{
		dt = _dt;
#if (CV_MAJOR_VERSION == 2)
		m_filter.transitionMatrix = *(cv::Mat_<float>(nStates, nStates) << 1	,0	,dt	,0  ,
				0	,1	,0	,dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#elif (CV_MAJOR_VERSION == 3)
		m_filter.transitionMatrix = (cv::Mat_<float>(nStates, nStates) << 1	,0	,dt	,0  ,
				0	,1	,0	,dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#endif		
		double a_old = a;

		cv::Mat_<float> measurement(nMeasure,1);
		cv::Mat_<float> prediction(nStates,1);

		measurement(0) = x;
		measurement(1) = y;

		prediction = m_filter.correct(measurement);

		x_new = prediction.at<float>(0);
		y_new = prediction.at<float>(1);
		double vx  = prediction.at<float>(2);
		double vy  = prediction.at<float>(3);

		if(m_iLife > 2)
		{
			v = sqrt(vx*vx+vy*vy);
			double diff_y = y_new - prev_y;
			double diff_x = x_new - prev_x;
			if(hypot(diff_y, diff_x) > 0.5)
			{
				prev_y = y;
				prev_x = x;
				a_new = atan2(diff_y, diff_x);
			}
			else
				a_new = a;

		}
		else
		{
			v = 0;
			a_new = a;
		}

		circ_angle = UtilityHNS::UtilityH::GetCircularAngle(circ_angle, UtilityHNS::UtilityH::FixNegativeAngle(a_old), UtilityHNS::UtilityH::FixNegativeAngle(a_new));

		circ_angle =  errorSmoother.Update(circ_angle).x;

		a_new = UtilityHNS::UtilityH::SplitPositiveAngle(circ_angle);

		if(v < 0.1)
			v = 0;

		//std::cout << "Track: Old (" << x << ", " << y << "), New (" << x_new << ", " << y_new << ")" << std::endl;
		//std::cout << "Track: " << m_id << ", A: " << a << ", A_new:(" << circ_angle << "," <<  a_new << ") , V" << v << ", dt: " << dt << ", forget_time: " << forget_time << std::endl;

		m_filter.predict();
		m_filter.statePre.copyTo(m_filter.statePost);
		m_filter.errorCovPre.copyTo(m_filter.errorCovPost);

		forget_time -= dt;
		m_iLife++;
	}
	virtual ~KFTrackV(){}
};

class InterestCircle
{
public:
	int id;
	double radius;
	double forget_time;
	std::vector<KFTrackV*> pTrackers;
	InterestCircle* pPrevCircle;
	InterestCircle* pNextCircle;

	InterestCircle(int _id)
	{
		id = _id;
		radius = 0;
		forget_time = NEVER_GORGET_TIME; // never forget
		pPrevCircle = 0;
		pNextCircle = 0;
	}
};

class CostRecordSet
{
public:
	int currobj;
	int prevObj;
	double cost;
	CostRecordSet(int curr_id, int prev_id, double _cost)
	{
		currobj = curr_id;
		prevObj = prev_id;
		cost = _cost;
	}
};

class SimpleTracker
{
public:
	std::vector<InterestCircle*> m_InterestRegions;
	std::vector<KFTrackV*> m_Tracks;
	timespec m_TrackTimer;
	long iTracksNumber;
	PlannerHNS::WayPoint m_PrevState;
	std::vector<PlannerHNS::DetectedObject> m_PrevDetectedObjects;
	std::vector<PlannerHNS::DetectedObject> m_DetectedObjects;

	void CreateTrack(PlannerHNS::DetectedObject& o);
	void CreateTrackV2(PlannerHNS::DetectedObject& o);
	KFTrackV* FindTrack(long index);
	void Track(std::vector<PlannerHNS::DetectedObject>& objects_list);
	void TrackV2();
	void CoordinateTransform(const PlannerHNS::WayPoint& refCoordinate, PlannerHNS::DetectedObject& obj);
	void CoordinateTransformPoint(const PlannerHNS::WayPoint& refCoordinate, PlannerHNS::GPSPoint& obj);
	void AssociateObjects();
	void InitializeInterestRegions(double horizon, double init_raduis, double init_time, std::vector<InterestCircle*>& regions);
	void AssociateAndTrack();
	void AssociateToRegions(KFTrackV& detectedObject);
	void CleanOldTracks();

	void DoOneStep(const PlannerHNS::WayPoint& currPose, const std::vector<PlannerHNS::DetectedObject>& obj_list);

	SimpleTracker(double horizon = 100);
	virtual ~SimpleTracker();

public:
	double m_DT;
	double m_MAX_ASSOCIATION_DISTANCE;
	int m_MAX_TRACKS_AFTER_LOSING;
	bool m_bUseCenterOnly;
	double m_MaxKeepTime;
	bool m_bFirstCall;
};

} /* namespace BehaviorsNS */

#endif /* SimpleTracker_H_ */
