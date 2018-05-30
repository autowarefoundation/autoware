/*
 * TrajectoryCosts.h
 *
 *  Created on: Dec 14, 2016
 *      Author: user
 */

#ifndef TRAJECTORYCOSTS_H_
#define TRAJECTORYCOSTS_H_

#include "RoadNetwork.h"
#include "PlannerCommonDef.h"
#include "PlanningHelpers.h"

using namespace std;

namespace PlannerHNS
{

class TrajectoryCosts
{
public:
	TrajectoryCosts();
	virtual ~TrajectoryCosts();

	TrajectoryCost DoOneStep(const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths,
			const WayPoint& currState, const int& currTrajectoryIndex, const int& currLaneIndex, const PlanningParams& params,
			const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState, const std::vector<PlannerHNS::DetectedObject>& obj_list);

public:
	int m_PrevCostIndex;
	vector<TrajectoryCost> m_TrajectoryCosts;
	PlanningParams m_Params;
	PolygonShape m_SafetyBorder;
	vector<WayPoint> m_AllContourPoints;
	vector<WayPoint> m_CollisionPoints;
	double m_WeightPriority;
	double m_WeightTransition;
	double m_WeightLong;
	double m_WeightLat;
	double m_WeightLaneChange;
	double m_LateralSkipDistance;



private:
	bool ValidateRollOutsInput(const vector<vector<vector<WayPoint> > >& rollOuts);
	vector<TrajectoryCost> CalculatePriorityAndLaneChangeCosts(const vector<vector<WayPoint> >& laneRollOuts, const int& lane_index, const PlanningParams& params);
	void NormalizeCosts(vector<TrajectoryCost>& trajectoryCosts);
	void CalculateLateralAndLongitudinalCosts(vector<TrajectoryCost>& trajectoryCosts, const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths, const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState);
	void CalculateTransitionCosts(vector<TrajectoryCost>& trajectoryCosts, const int& currTrajectoryIndex, const PlanningParams& params);
	bool CalculateIntersectionVelocities(const std::vector<WayPoint>& path, const DetectedObject& obj, const WayPoint& currState,const CAR_BASIC_INFO& carInfo, WayPoint& collisionPoint);

};

}

#endif /* TRAJECTORYCOSTS_H_ */
