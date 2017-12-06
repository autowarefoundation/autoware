/*
 * TrajectoryPrediction.cpp
 *
 *  Created on: Aug 8, 2017
 *      Author: user
 */

#include "TrajectoryPrediction.h"
#include "PlannerH.h"
#include "MappingHelpers.h"

namespace PlannerHNS
{

TrajectoryPrediction::TrajectoryPrediction()
{
	m_MaxCollisionPredictionTime = 6;
}

TrajectoryPrediction::~TrajectoryPrediction()
{
}

void TrajectoryPrediction::DoOneStep(PlannerHNS::RoadNetwork& map, const PlannerHNS::VehicleState& vstatus, const PlannerHNS::WayPoint& currState, std::vector<PlannerHNS::WayPoint>& currPath, std::vector<PlannerHNS::DetectedObject>& obj_list, const double& minfDist)
{
	double predTime = PredictTimeCostForTrajectory(currPath, vstatus, currState, minfDist);
	if(predTime > m_MaxCollisionPredictionTime)
		predTime = m_MaxCollisionPredictionTime;

	for(unsigned int i = 0; i < obj_list.size(); i++)
	{
		if(obj_list.at(i).bVelocity && obj_list.at(i).center.v > 0.1)
		{
			obj_list.at(i).predTrajectories.clear();
			PredictObstacleTrajectory(map, obj_list.at(i), predTime, obj_list.at(i).predTrajectories, minfDist);
		}
		else
		{
			obj_list.at(i).bVelocity = false;
		}
	}
}

double TrajectoryPrediction::PredictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::VehicleState& vstatus, const PlannerHNS::WayPoint& currState, const double& minfDist)
{

	//1- Calculate time prediction for each trajectory
	if(path.size() == 0) return 0;
	double accum_time = 0;
	double endDistance = minfDist;

	for(unsigned int i = 0 ; i < path.size(); i++)
	{
		path.at(i).collisionCost = 0;
		path.at(i).timeCost = -1;
	}

	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, currState, info);
	double total_distance = 0;
	path.at(info.iFront).timeCost = 0;
	if(info.iFront == 0 ) info.iFront++;
	for(unsigned int i=info.iFront; i<path.size(); i++)
	{
		total_distance += hypot(path.at(i).pos.x- path.at(i-1).pos.x,path.at(i).pos.y- path.at(i-1).pos.y);
		if(vstatus.speed > 0.1 && total_distance > 0.1)
			accum_time = total_distance/vstatus.speed;

		path.at(i).timeCost = accum_time;
		if(total_distance > endDistance)
			break;
	}

	return accum_time;
}

void TrajectoryPrediction::PredictObstacleTrajectory(PlannerHNS::RoadNetwork& map, const DetectedObject& obj, const double& predTime, std::vector<std::vector<PlannerHNS::WayPoint> >& paths,const double& minfDist)
{
	PlannerHNS::PlanningParams planningDefaultParams;
	planningDefaultParams.rollOutNumber = 0;
	planningDefaultParams.microPlanDistance = predTime*obj.center.v;
	if(planningDefaultParams.microPlanDistance > minfDist)
		planningDefaultParams.microPlanDistance = minfDist;

	planningDefaultParams.pathDensity = 1;

	//cout << "Pred Time: " << predTime << ", Object Speed: " <<obj.center.v << endl;
	WayPoint* pClosestWP =  MappingHelpers::GetClosestWaypointFromMap(obj.center, map, obj.bDirection);
	PlannerHNS::PlannerH planner;

	if(planningDefaultParams.microPlanDistance > 0)
	{
		WayPoint obj_center = obj.center;
		obj_center.pos.a = pClosestWP->pos.a;
		planner.PredictPlanUsingDP(obj_center, pClosestWP, planningDefaultParams.microPlanDistance, paths, false);
		for(unsigned int j=0; j < paths.size(); j++)
		{
			PlanningHelpers::FixPathDensity(paths.at(j), planningDefaultParams.pathDensity);
			PlanningHelpers::CalcAngleAndCost(paths.at(j));
		}

		for(unsigned int j=0; j < paths.size(); j++)
		{
			if(paths.at(j).size() > 0)
			{
				double timeDelay = 0;
				double total_distance = 0;
				paths.at(j).at(0).timeCost = 0;
				paths.at(j).at(0).v = obj.center.v;
				for(unsigned int i=1; i<paths.at(j).size(); i++)
				{
					paths.at(j).at(i).v = obj.center.v;
					total_distance += hypot(paths.at(j).at(i).pos.y- paths.at(j).at(i-1).pos.y, paths.at(j).at(i).pos.x- paths.at(j).at(i-1).pos.x);
					if(obj.center.v > 0.1 && total_distance > 0.1)
						timeDelay = total_distance/obj.center.v;
					paths.at(j).at(i).timeCost = timeDelay;
					if(total_distance > planningDefaultParams.microPlanDistance)
						break;
				}
			}
		}
	}
}

} /* namespace PlannerHNS */
