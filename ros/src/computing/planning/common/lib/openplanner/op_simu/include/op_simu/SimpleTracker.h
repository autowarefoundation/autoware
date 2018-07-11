
/// \file SimpleTracker.h
/// \brief Kalman Filter based object tracker
/// \author Hatem Darweesh
/// \date Aug 11, 2016

#ifndef SimpleTracker_H_
#define SimpleTracker_H_

#include "op_planner/RoadNetwork.h"
#include "op_planner/PlanningHelpers.h"
#include "opencv2/video/tracking.hpp"
#include <vector>
#include "op_utility/UtilityH.h"
#include <math.h>
#include <iostream>

namespace SimulationNS
{

#define DEBUG_TRACKER 0
#define NEVER_GORGET_TIME -1000

enum TRACKING_TYPE {ASSOCIATE_ONLY = 0, SIMPLE_TRACKER = 1, CONTOUR_TRACKER = 2};

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
	int nStates;
	int nMeasure;
	int MinAppearanceCount;

public:
	int m_bUpdated;
	int region_id;
	double forget_time;
	int m_iLife;
	PlannerHNS::DetectedObject obj; // Used for associate only , don't remove
	//kalmanFilter1D errorSmoother;

	long GetTrackID()
	{
		return m_id;
	}

	KFTrackV(double x, double y, double a, long id, double _dt, int _MinAppearanceCount = 1)
	{
//		errorSmoother.result.MovCov = 0.125;
//		errorSmoother.result.MeasureCov = 0.1;
//		errorSmoother.result.p = 1;
//		errorSmoother.result.x = 0;
		region_id = -1;
		forget_time = NEVER_GORGET_TIME; // this is very bad , dangerous
		m_iLife = 0;
		prev_x = x;
		prev_y = y;
		prev_v = 0;
		prev_a = a;
		nStates = 4;
		nMeasure = 2;

		MinAppearanceCount = _MinAppearanceCount;

		m_id = id;
		m_bUpdated = true;

		m_filter = cv::KalmanFilter(nStates,nMeasure);
#if (CV_MAJOR_VERSION == 2)
		m_filter.transitionMatrix = *(cv::Mat_<float>(nStates, nStates) << 1	,0	,_dt	,0  ,
				0	,1	,0	,_dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#elif (CV_MAJOR_VERSION == 3)
		m_filter.transitionMatrix = (cv::Mat_<float>(nStates, nStates) << 1	,0	,_dt	,0  ,
				0	,1	,0	,_dt	,
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

		//errorSmoother.Update(a);
	}

	void UpdateTracking(double _dt, const PlannerHNS::DetectedObject& oldObj, PlannerHNS::DetectedObject& predObj)
	{

#if (CV_MAJOR_VERSION == 2)
		m_filter.transitionMatrix = *(cv::Mat_<float>(nStates, nStates) << 1	,0	,_dt	,0  ,
				0	,1	,0	,_dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#elif (CV_MAJOR_VERSION == 3)
		m_filter.transitionMatrix = (cv::Mat_<float>(nStates, nStates) << 1	,0	,_dt	,0  ,
				0	,1	,0	,_dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#endif		

		cv::Mat_<float> measurement(nMeasure,1);
		cv::Mat_<float> prediction(nStates,1);

		measurement(0) = oldObj.center.pos.x;
		measurement(1) = oldObj.center.pos.y;

		prediction = m_filter.correct(measurement);

		predObj.center.pos.x = prediction.at<float>(0);
		predObj.center.pos.y = prediction.at<float>(1);
		double vx  = prediction.at<float>(2);
		double vy  = prediction.at<float>(3);

		double currA = 0;
		double currV = 0;

		if(m_iLife > 1)
		{
			currV = sqrt(vx*vx+vy*vy);

			double diff_y = predObj.center.pos.y - prev_y;
			double diff_x = predObj.center.pos.x - prev_x;
			if(hypot(diff_y, diff_x) > 0.2)
			{
				currA = atan2(diff_y, diff_x);
			}
			else
			{
				currA = prev_a;
			}
		}


		if(m_iLife > MinAppearanceCount)
		{
			predObj.center.pos.a = currA;
			predObj.center.v = currV;
			predObj.bVelocity = true;
			predObj.acceleration = UtilityHNS::UtilityH::GetSign(predObj.center.v - prev_v);
		}
		else
		{
			predObj.bDirection = false;
			predObj.bVelocity = false;
		}

		if(predObj.centers_list.size() > 30)
					predObj.centers_list.erase(predObj.centers_list.begin()+0);

		if(predObj.centers_list.size() > 1)
		{
			double diff_y = predObj.center.pos.y - predObj.centers_list.at(predObj.centers_list.size()-1).pos.y;
			double diff_x = predObj.center.pos.x - predObj.centers_list.at(predObj.centers_list.size()-1).pos.x;
			if(hypot(diff_y, diff_x) > 0.1)
			{
				predObj.centers_list.push_back(predObj.center);
				PlannerHNS::PlanningHelpers::SmoothPath(predObj.centers_list, 0.3, 0.4, 0.1);
				PlannerHNS::PlanningHelpers::CalcAngleAndCost(predObj.centers_list);
			}
		}
		else
			predObj.centers_list.push_back(predObj.center);

		if(predObj.centers_list.size()>3)
		{
			predObj.bDirection = true;
			predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size()-1).pos.a + predObj.centers_list.at(predObj.centers_list.size()-2).pos.a + predObj.centers_list.at(predObj.centers_list.size()-3).pos.a)/3.0;
		}
		else if(predObj.centers_list.size()>2)
		{
			predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size()-1).pos.a + predObj.centers_list.at(predObj.centers_list.size()-2).pos.a)/2.0;
		}
		else if(predObj.centers_list.size()>1)
		{
			predObj.center.pos.a = predObj.centers_list.at(predObj.centers_list.size()-1).pos.a;
		}


		m_filter.predict();
		m_filter.statePre.copyTo(m_filter.statePost);
		m_filter.errorCovPre.copyTo(m_filter.errorCovPost);

		prev_a = currA;
		prev_y = predObj.center.pos.y;
		prev_x = predObj.center.pos.x;
		prev_v = currV;
		forget_time -= _dt;
		m_iLife++;
	}

	void PredictTracking(double _dt, const PlannerHNS::DetectedObject& oldObj, PlannerHNS::DetectedObject& predObj)
	{
		if(m_iLife < MinAppearanceCount)
		{
			forget_time -= _dt;
			return;
		}

#if (CV_MAJOR_VERSION == 2)
		m_filter.transitionMatrix = *(cv::Mat_<float>(nStates, nStates) << 1	,0	,_dt	,0  ,
				0	,1	,0	,_dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#elif (CV_MAJOR_VERSION == 3)
		m_filter.transitionMatrix = (cv::Mat_<float>(nStates, nStates) << 1	,0	,_dt	,0  ,
				0	,1	,0	,_dt	,
				0	,0	,1	,0	,
				0	,0	,0	,1	);
#endif


		cv::Mat_<float> prediction(nStates,1);

		prediction = m_filter.predict();

		predObj.center.pos.x = prediction.at<float>(0);
		predObj.center.pos.y = prediction.at<float>(1);
		double vx  = prediction.at<float>(2);
		double vy  = prediction.at<float>(3);

		double currA = 0;
		double currV = 0;

		if(m_iLife > 1)
		{
			currV = sqrt(vx*vx+vy*vy);

			double diff_y = predObj.center.pos.y - prev_y;
			double diff_x = predObj.center.pos.x - prev_x;
			if(hypot(diff_y, diff_x) > 0.2)
			{
				prev_y = oldObj.center.pos.y;
				prev_x = oldObj.center.pos.x;
				currA = atan2(diff_y, diff_x);
				prev_a = currA;
			}
			else
			{
				currA = prev_a;
			}

			for(unsigned int k=0; k < obj.contour.size(); k++)
			{
				obj.contour.at(k).x += diff_x;
				obj.contour.at(k).y += diff_y;
			}
		}


		if(m_iLife > MinAppearanceCount)
		{
			predObj.center.pos.a = currA;
			predObj.center.v = currV;

			predObj.bVelocity = true;
			predObj.acceleration = UtilityHNS::UtilityH::GetSign(predObj.center.v - oldObj.center.v);
		}
		else
		{
			predObj.bDirection = false;
			predObj.bVelocity = false;
		}

		if(predObj.centers_list.size() > 30)
					predObj.centers_list.erase(predObj.centers_list.begin()+0);

		if(predObj.centers_list.size() > 1)
		{
			double diff_y = predObj.center.pos.y - predObj.centers_list.at(predObj.centers_list.size()-1).pos.y;
			double diff_x = predObj.center.pos.x - predObj.centers_list.at(predObj.centers_list.size()-1).pos.x;
			if(hypot(diff_y, diff_x) > 0.1)
			{
				predObj.centers_list.push_back(predObj.center);
				PlannerHNS::PlanningHelpers::SmoothPath(predObj.centers_list, 0.3, 0.4, 0.1);
				PlannerHNS::PlanningHelpers::CalcAngleAndCost(predObj.centers_list);
			}
		}
		else
			predObj.centers_list.push_back(predObj.center);

		if(predObj.centers_list.size()>3)
		{
			predObj.bDirection = true;
			predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size()-1).pos.a + predObj.centers_list.at(predObj.centers_list.size()-2).pos.a + predObj.centers_list.at(predObj.centers_list.size()-3).pos.a)/3.0;
		}
		else if(predObj.centers_list.size()>2)
		{
			predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size()-1).pos.a + predObj.centers_list.at(predObj.centers_list.size()-2).pos.a)/2.0;
		}
		else if(predObj.centers_list.size()>1)
		{
			predObj.center.pos.a = predObj.centers_list.at(predObj.centers_list.size()-1).pos.a;
		}


		//m_filter.predict();
		m_filter.statePre.copyTo(m_filter.statePost);
		m_filter.errorCovPre.copyTo(m_filter.errorCovPost);

		prev_v = currV;

		forget_time -= _dt;
		m_iLife++;
	}

	void UpdateAssociateOnly(double _dt, const PlannerHNS::DetectedObject& oldObj, PlannerHNS::DetectedObject& predObj)
	{
		if(predObj.centers_list.size() > 30)
			predObj.centers_list.erase(predObj.centers_list.begin()+0);

		if(predObj.centers_list.size() > 1)
		{
			double diff_y = predObj.center.pos.y - predObj.centers_list.at(predObj.centers_list.size()-1).pos.y;
			double diff_x = predObj.center.pos.x - predObj.centers_list.at(predObj.centers_list.size()-1).pos.x;
			if(hypot(diff_y, diff_x) > 0.1)
			{
				predObj.centers_list.push_back(predObj.center);
				PlannerHNS::PlanningHelpers::SmoothPath(predObj.centers_list, 0.3, 0.4, 0.1);
				PlannerHNS::PlanningHelpers::CalcAngleAndCost(predObj.centers_list);
			}
		}
		else
			predObj.centers_list.push_back(predObj.center);

		if(predObj.centers_list.size()>3)
		{
			predObj.bDirection = true;
			predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size()-1).pos.a + predObj.centers_list.at(predObj.centers_list.size()-2).pos.a + predObj.centers_list.at(predObj.centers_list.size()-3).pos.a)/3.0;
		}
		else if(predObj.centers_list.size()>2)
		{
			predObj.bDirection = true;
			predObj.center.pos.a = (predObj.centers_list.at(predObj.centers_list.size()-1).pos.a + predObj.centers_list.at(predObj.centers_list.size()-2).pos.a)/2.0;
		}
		else if(predObj.centers_list.size()>1)
		{
			predObj.bDirection = false;
			predObj.center.pos.a = predObj.centers_list.at(predObj.centers_list.size()-1).pos.a;
		}
		else
			predObj.bDirection = false;

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

	InterestCircle(int _id)
	{
		id = _id;
		radius = 0;
		forget_time = NEVER_GORGET_TIME; // never forget
	}
};

class CostRecordSet
{
public:
	int i_obj;
	int i_track;
	double cost;
	 double size_diff;
	 double width_diff;
	 double length_diff;
	 double height_diff;
	 double angle_diff;
	 double distance_diff;
	CostRecordSet(int obj_index, int track_index, double _distance_diff, double _size_diff, double _width_diff, double _length_diff, double _height_diff, double _angle_diff)
	{
		i_obj = obj_index;
		i_track = track_index;
		size_diff = _size_diff;
		angle_diff = _angle_diff;
		distance_diff = _distance_diff;
		width_diff = _width_diff;
		length_diff = _length_diff;
		height_diff = _height_diff;
		cost = 0;
	}

	CostRecordSet()
	{
		i_obj = -1;
		i_track = -1;
		size_diff = 0;
		angle_diff = 0;
		distance_diff = 0;
		width_diff = 0;
		length_diff = 0;
		height_diff = 0;
		cost = 0;
	}
};

class SimpleTracker
{
public:
	std::vector<CostRecordSet> m_CostsLists;
	std::vector<InterestCircle*> m_InterestRegions;
	std::vector<KFTrackV*> m_Tracks;
	std::vector<KFTrackV> m_TrackSimply;
	timespec m_TrackTimer;
	long iTracksNumber;
	PlannerHNS::WayPoint m_PrevState;
	PlannerHNS::WayPoint m_StateDiff;
	std::vector<PlannerHNS::DetectedObject> m_PrevDetectedObjects;
	std::vector<PlannerHNS::DetectedObject> m_DetectedObjects;

	void DoOneStep(const PlannerHNS::WayPoint& currPose, const std::vector<PlannerHNS::DetectedObject>& obj_list, const TRACKING_TYPE& type = SIMPLE_TRACKER);

	SimpleTracker();
	virtual ~SimpleTracker();
	void InitSimpleTracker();
	void InitializeInterestRegions(std::vector<InterestCircle*>& regions);

public:
	double m_dt;
	double m_MAX_ASSOCIATION_DISTANCE;
	bool m_bUseCenterOnly;
	double m_MaxKeepTime;
	bool m_bFirstCall;
	int m_nMinTrustAppearances;
	double m_Horizon;
	double m_CirclesResolution;
	double m_MAX_ASSOCIATION_SIZE_DIFF;
	double m_MAX_ASSOCIATION_ANGLE_DIFF;

private:
	std::vector<KFTrackV> newObjects;
	void AssociateAndTrack();
	void AssociateSimply();
	void AssociateToRegions(KFTrackV& detectedObject);
	void CleanOldTracks();
	void AssociateOnly();
	void MergeObjectAndTrack(KFTrackV& track, PlannerHNS::DetectedObject& obj);
	int InsidePolygon(const std::vector<PlannerHNS::GPSPoint>& polygon,const PlannerHNS::GPSPoint& p);

	void MatchClosest();
	void MatchClosestCost();

};

}

#endif /* SimpleTracker_H_ */
