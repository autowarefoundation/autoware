/*
 * BehaviorPrediction.cpp
 *
 *  Created on: Jul 6, 2017
 *      Author: user
 */

#include "BehaviorPrediction.h"
#include "MappingHelpers.h"
#include "MatrixOperations.h"


namespace PlannerHNS
{

BehaviorPrediction::BehaviorPrediction()
{
	m_MaxLaneDetectionDistance = 0.5;
	m_PredictionDistance = 20.0;
	m_bGenerateBranches = false;
	m_bUseFixedPrediction = true;
}

BehaviorPrediction::~BehaviorPrediction()
{
}

void BehaviorPrediction::FilterObservations(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& filtered_list)
{
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		if(obj_list.at(i).t == SIDEWALK || obj_list.at(i).center.v < 1.0)
			continue;

		bool bFound = false;
		int found_index = 0;
		for(unsigned int ip=0; ip < filtered_list.size(); ip++)
		{
			if(filtered_list.at(ip).id == obj_list.at(i).id)
			{
				found_index = ip;
				bFound = true;
				break;
			}
		}

		if(bFound)
			filtered_list.at(found_index) = obj_list.at(i);
		else
			filtered_list.push_back(obj_list.at(i));
	}

	for(int ip=0; ip < filtered_list.size(); ip++)
	{
		//check for cleaning
		bool bRevFound = false;
		for(unsigned int ic=0; ic < obj_list.size(); ic++)
		{
			if(filtered_list.at(ip).id == obj_list.at(ic).id)
			{
				bRevFound = true;
				break;
			}
		}

		if(!bRevFound)
		{
			filtered_list.erase(filtered_list.begin()+ip);
			ip--;
		}
	}
}

void BehaviorPrediction::DoOneStep(const std::vector<DetectedObject>& obj_list, const WayPoint& currPose, const double& minSpeed, const double& maxDeceleration, RoadNetwork& map)
{
	if(!m_bUseFixedPrediction)
		m_PredictionDistance = -pow(currPose.v, 2)/(maxDeceleration);

	ExtractTrajectoriesFromMap(obj_list, map, m_PredictedObjects);

	for(unsigned int i=0; i < m_PredictedObjects.size(); i++)
	{
		for(unsigned int j=0; j < m_PredictedObjects.at(i).predTrajectories.size(); j++)
		{
			PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_PredictedObjects.at(i).predTrajectories.at(j), m_PredictedObjects.at(i).center, minSpeed, m_PredictionDistance);
			PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_PredictedObjects.at(i).predTrajectories.at(j));
		}
	}
}

void BehaviorPrediction::ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& obj_list,RoadNetwork& map, std::vector<DetectedObject>& old_list)
{
	PlannerH planner;
	m_temp_list.clear();
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		bool bFound = false;
		int found_index = 0;
		for(unsigned int ip=0; ip < old_list.size(); ip++)
		{
			if(old_list.at(ip).id == obj_list.at(i).id)
			{
				found_index = ip;
				bFound = true;
				break;
			}
		}

		if(!bFound)
		{
			old_list.push_back(DetectedObject());
			found_index = old_list.size()-1;
		}

		unsigned int prev_trajectories_num = old_list.at(found_index).predTrajectories.size();

		old_list.at(found_index) = obj_list.at(i);
		old_list.at(found_index).predTrajectories.clear();

		if(old_list.at(found_index).bDirection && old_list.at(found_index).bVelocity)
		{
			old_list.at(found_index).pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(old_list.at(found_index).center, map, m_MaxLaneDetectionDistance, old_list.at(found_index).bDirection);
			planner.PredictTrajectoriesUsingDP(old_list.at(found_index).center, old_list.at(found_index).pClosestWaypoints, m_PredictionDistance, old_list.at(found_index).predTrajectories, m_bGenerateBranches, old_list.at(found_index).bDirection);
		}
		m_temp_list.push_back(old_list.at(found_index));
	}

	old_list = m_temp_list;
}

} /* namespace PlannerHNS */
