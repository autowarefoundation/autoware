/*
 * TrajectoryPrediction.h
 *
 *  Created on: Aug 8, 2017
 *      Author: user
 */

#ifndef TRAJECTORYPREDICTION_H_
#define TRAJECTORYPREDICTION_H_

#include <RoadNetwork.h>
#include "PlannerCommonDef.h"
#include "PlanningHelpers.h"

namespace PlannerHNS
{

class TrajectoryPrediction
{
public:
	TrajectoryPrediction();
	virtual ~TrajectoryPrediction();

	void DoOneStep(PlannerHNS::RoadNetwork& map, const PlannerHNS::VehicleState& vstatus, const PlannerHNS::WayPoint& currState, std::vector<PlannerHNS::WayPoint>& currPath, std::vector<PlannerHNS::DetectedObject>& obj_list, const double& minfDist);
	double PredictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::VehicleState& vstatus, const PlannerHNS::WayPoint& currState,const double& minfDist);
	void PredictObstacleTrajectory(PlannerHNS::RoadNetwork& map, const DetectedObject& obj, const double& predTime, std::vector<std::vector<PlannerHNS::WayPoint> >& paths,const double& minfDist);
public:
	double m_MaxCollisionPredictionTime;

};

} /* namespace PlannerHNS */

#endif /* TRAJECTORYPREDICTION_H_ */
