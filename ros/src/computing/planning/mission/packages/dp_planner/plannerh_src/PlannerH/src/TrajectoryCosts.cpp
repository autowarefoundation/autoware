/*
 * TrajectoryCosts.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: user
 */

#include "TrajectoryCosts.h"
#include "MatrixOperations.h"

namespace PlannerHNS
{


TrajectoryCosts::TrajectoryCosts()
{
	m_CurrentTrajectoryIndex = -1;
}

TrajectoryCosts::~TrajectoryCosts()
{
}

TrajectoryCost TrajectoryCosts::DoOneStep(const vector<vector<vector<WayPoint> > >& rollOuts,
		const vector<vector<WayPoint> >& totalPaths, const WayPoint& currState, const int& currTrajectoryIndex,
		const PlanningParams& params, const CAR_BASIC_INFO& carInfo,
		const std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	TrajectoryCost bestTrajectory;
	bestTrajectory.bBlocked = true;
	bestTrajectory.closest_obj_distance = params.horizonDistance;
	bestTrajectory.closest_obj_velocity = 0;
	bestTrajectory.index = -1;

	if(!ValidateRollOutsInput(rollOuts) || rollOuts.size() != totalPaths.size()) return bestTrajectory;

	if(m_TrajectoryCosts.size() != rollOuts.size())
		m_CurrentTrajectoryIndex = -1;
	else
		m_CurrentTrajectoryIndex = currTrajectoryIndex;

	m_TrajectoryCosts.clear();


	for(unsigned int il = 0; il < rollOuts.size(); il++)
	{
		vector<TrajectoryCost> costs = CalculatePriorityAndLaneChangeCosts(rollOuts.at(il), il, params);
		m_TrajectoryCosts.insert(m_TrajectoryCosts.end(), costs.begin(), costs.end());
	}

	vector<WayPoint> contourPoints;
	WayPoint p;
	for(unsigned int io=0; io<obj_list.size(); io++)
	{
		for(unsigned int icon=0; icon < obj_list.at(io).contour.size(); icon++)
		{
			p.pos = obj_list.at(io).contour.at(icon);
			p.v = obj_list.at(io).center.v;
			contourPoints.push_back(p);
		}
	}

	CalculateLateralAndLongitudinalCosts(m_TrajectoryCosts, rollOuts, totalPaths, currState, contourPoints, params, carInfo);

	NormalizeCosts(m_TrajectoryCosts);

	int smallestIndex = -1;
	double smallestCost = 9999999999;
	for(unsigned int ic = 0; ic < m_TrajectoryCosts.size(); ic++)
	{
		if(!m_TrajectoryCosts.at(ic).bBlocked && m_TrajectoryCosts.at(ic).cost < smallestCost)
		{
			smallestCost = m_TrajectoryCosts.at(ic).cost;
			smallestIndex = ic;
		}
	}

	//All is blocked !
	if(smallestIndex == -1 && currTrajectoryIndex < (int)m_TrajectoryCosts.size())
	{
		bestTrajectory =  m_TrajectoryCosts.at(currTrajectoryIndex);
		bestTrajectory.index = smallestIndex;
	}
	else if(smallestIndex >= 0)
	{
		bestTrajectory = m_TrajectoryCosts.at(smallestIndex);
		bestTrajectory.index = smallestIndex;
	}

//	cout << "Blockage Test: " << smallestIndex << ", currentTrajIndex: " << currTrajectoryIndex << ", Trajectory Cost Size: " << m_TrajectoryCosts.size() << ", Best Block: " << bestTrajectory.bBlocked <<", Best Index: " << bestTrajectory.index<< ", D: " << bestTrajectory.closest_obj_distance << endl;

	return bestTrajectory;
}

void TrajectoryCosts::CalculateLateralAndLongitudinalCosts(vector<TrajectoryCost>& trajectoryCosts,
		const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths,
		const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params,
		const CAR_BASIC_INFO& carInfo)
{
	double critical_lateral_distance =  carInfo.width/2.0 + params.horizontalSafetyDistancel;
	double critical_long_front_distance =  carInfo.wheel_base/2.0 + carInfo.length/2.0 + params.verticalSafetyDistance;
	double critical_long_back_distance =  carInfo.length/2.0 + params.verticalSafetyDistance - carInfo.wheel_base/2.0;
	int iCostIndex = 0;
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);
	PlannerHNS::Mat3 invRotationMat(currState.pos.a-M_PI_2);
	PlannerHNS::Mat3 invTranslationMat(currState.pos.x, currState.pos.y);

	GPSPoint bottom_left(-critical_lateral_distance ,-critical_long_back_distance,  currState.pos.z, 0);
	GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance,  currState.pos.z, 0);

	GPSPoint top_right(critical_lateral_distance , critical_long_front_distance,  currState.pos.z, 0);
	GPSPoint top_left(-critical_lateral_distance , critical_long_front_distance, currState.pos.z, 0);

	bottom_left = invRotationMat*bottom_left;
	bottom_left = invTranslationMat*bottom_left;

	top_right = invRotationMat*top_right;
	top_right = invTranslationMat*top_right;

	bottom_right = invRotationMat*bottom_right;
	bottom_right = invTranslationMat*bottom_right;

	top_left = invRotationMat*top_left;
	top_left = invTranslationMat*top_left;

	m_SafetyBox.clear();
	m_SafetyBox.push_back(bottom_left);
	m_SafetyBox.push_back(bottom_right);
	m_SafetyBox.push_back(top_right);
	m_SafetyBox.push_back(top_left);

	m_SafetyBorder.points.clear();
	m_SafetyBorder.points.push_back(bottom_left) ;
	m_SafetyBorder.points.push_back(bottom_right) ;
	m_SafetyBorder.points.push_back(top_right) ;
	m_SafetyBorder.points.push_back(top_left) ;




	for(unsigned int il=0; il < rollOuts.size(); il++)
	{
		RelativeInfo car_info;
		PlanningHelpers::GetRelativeInfo(totalPaths.at(il), currState, car_info);

		for(unsigned int it=0; it< rollOuts.at(il).size(); it++)
		{
//			int iCurrIndex = PlanningHelpers::GetClosestNextPointIndex(totalPaths.at(il), currState);
//			double long_diff_distance = 0;
//			WayPoint perpWP = PlanningHelpers::GetPerpendicularOnTrajectory(totalPaths.at(il), currState, long_diff_distance);
//			double longitudinalDist = hypot(totalPaths.at(il).at(iCurrIndex).pos.y - perpWP.pos.y, totalPaths.at(il).at(iCurrIndex).pos.x - perpWP.pos.x);

			for(unsigned int icon = 0; icon < contourPoints.size(); icon++)
			{
				RelativeInfo obj_info;
				PlanningHelpers::GetRelativeInfo(totalPaths.at(il), contourPoints.at(icon), obj_info);
				double longitudinalDist = PlanningHelpers::GetExactDistanceOnTrajectory(totalPaths.at(il), car_info, obj_info);
				//longitudinalDist += PlanningHelpers::GetDistanceOnTrajectory(totalPaths.at(il), iCurrIndex, contourPoints.at(icon));

				if(longitudinalDist< -carInfo.length)
				{
					//cout << "Skip , long Dist: " << longitudinalDist << endl;
					continue;
				}

				double close_in_percentage = 1;
//				if(longitudinalDist > (params.carTipMargin + params.rollInMargin - distanceOnLocal))
//					close_in_percentage = 1;
//				else if(longitudinalDist <= (params.carTipMargin-distanceOnLocal))
//					close_in_percentage = 0;
//				else
//					close_in_percentage = (longitudinalDist+carInfo.length/2.0)/params.rollInMargin;
//
//				if(close_in_percentage>1) close_in_percentage = 1;
//				if(close_in_percentage<0) close_in_percentage = 0;

//				PlannerHNS::GPSPoint relative_point;
//				relative_point = translationMat*contourPoints.at(icon).pos;
//				relative_point = rotationMat*relative_point;


				//double lateralDist =  fabs(PlanningHelpers::GetPerpDistanceToTrajectorySimple(totalPaths.at(il), contourPoints.at(icon), iCurrIndex) - (trajectoryCosts.at(iCostIndex).distance_from_center*close_in_percentage));
				double lateralDist = fabs(obj_info.perp_distance - (trajectoryCosts.at(iCostIndex).distance_from_center*close_in_percentage));


				longitudinalDist = longitudinalDist - critical_long_front_distance;

				if((lateralDist <= critical_lateral_distance && longitudinalDist >= 0 &&  longitudinalDist < params.minFollowingDistance) || (m_SafetyBorder.PointInsidePolygon(m_SafetyBorder, contourPoints.at(icon).pos) == true))// || (longitudinalDist < params.maxDistanceToAvoid && fabs(relative_point.y) <= critical_lateral_distance ))
					trajectoryCosts.at(iCostIndex).bBlocked = true;

				//cout << ", Lat D: " << lateralDist << ", Lat C: " << critical_lateral_distance <<", Lon D: " << longitudinalDist << ", Lon C: "<< critical_long_back_distance << ", Safety Box: " << m_SafetyBorder.PointInsidePolygon(m_SafetyBorder, contourPoints.at(icon).pos)<< endl;


//				if(lateralDist==0)
//					trajectoryCosts.at(iCostIndex).lateral_cost += 999999;
//				else
					trajectoryCosts.at(iCostIndex).lateral_cost += 1.0/lateralDist;

//				if(longitudinalDist==0)
//					trajectoryCosts.at(iCostIndex).longitudinal_cost += 999999;
//				else
					trajectoryCosts.at(iCostIndex).longitudinal_cost += 1.0/longitudinalDist;

				if(longitudinalDist >= -critical_long_front_distance && longitudinalDist < trajectoryCosts.at(iCostIndex).closest_obj_distance)
				{
					trajectoryCosts.at(iCostIndex).closest_obj_distance = longitudinalDist;
					trajectoryCosts.at(iCostIndex).closest_obj_velocity = contourPoints.at(icon).v;
				}
			}

			iCostIndex++;
		}
	}
}

void TrajectoryCosts::NormalizeCosts(vector<TrajectoryCost>& trajectoryCosts)
{
	//Normalize costs
	double totalPriorities = 0;
	double totalChange = 0;
	double totalLateralCosts = 0;
	double totalLongitudinalCosts = 0;
	double transitionCosts = 0;

	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		totalPriorities += trajectoryCosts.at(ic).priority_cost;
		totalChange += trajectoryCosts.at(ic).lane_change_cost;
		totalLateralCosts += trajectoryCosts.at(ic).lateral_cost;
		totalLongitudinalCosts += trajectoryCosts.at(ic).longitudinal_cost;
		transitionCosts += trajectoryCosts.at(ic).transition_cost;
	}

	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		if(totalPriorities != 0)
			trajectoryCosts.at(ic).priority_cost = trajectoryCosts.at(ic).priority_cost / totalPriorities;
		else
			trajectoryCosts.at(ic).priority_cost = 0;

		if(totalChange != 0)
			trajectoryCosts.at(ic).lane_change_cost = trajectoryCosts.at(ic).lane_change_cost / totalChange;
		else
			trajectoryCosts.at(ic).lane_change_cost = 0;

		if(totalLateralCosts != 0)
			trajectoryCosts.at(ic).lateral_cost = trajectoryCosts.at(ic).lateral_cost / totalLateralCosts;
		else
			trajectoryCosts.at(ic).lateral_cost = 0;

		if(totalLongitudinalCosts != 0)
			trajectoryCosts.at(ic).longitudinal_cost = trajectoryCosts.at(ic).longitudinal_cost / totalLongitudinalCosts;
		else
			trajectoryCosts.at(ic).longitudinal_cost = 0;

		if(transitionCosts != 0)
			trajectoryCosts.at(ic).transition_cost = trajectoryCosts.at(ic).transition_cost / transitionCosts;
		else
			trajectoryCosts.at(ic).transition_cost = 0;

		trajectoryCosts.at(ic).cost = (
				0.25*trajectoryCosts.at(ic).priority_cost +
				trajectoryCosts.at(ic).lane_change_cost +
				trajectoryCosts.at(ic).lateral_cost +
				trajectoryCosts.at(ic).longitudinal_cost +
				trajectoryCosts.at(ic).transition_cost) / 5.0;
	}
}

vector<TrajectoryCost> TrajectoryCosts::CalculatePriorityAndLaneChangeCosts(const vector<vector<WayPoint> >& laneRollOuts,
		const int& lane_index, const PlanningParams& params)
{
	vector<TrajectoryCost> costs;
	TrajectoryCost tc;
	int centralIndex = params.rollOutNumber/2;

	tc.lane_index = lane_index;
	for(unsigned int it=0; it< laneRollOuts.size(); it++)
	{
		tc.index = it;
		tc.relative_index = it - centralIndex;
		tc.distance_from_center = params.rollOutDensity*tc.relative_index;
		tc.priority_cost = fabs(tc.distance_from_center);
		tc.closest_obj_distance = params.horizonDistance;

		if(laneRollOuts.at(it).at(0).bDir == FORWARD_LEFT_DIR || laneRollOuts.at(it).at(0).bDir == FORWARD_RIGHT_DIR)
			tc.lane_change_cost = 1;
		else if(laneRollOuts.at(it).at(0).bDir == BACKWARD_DIR || laneRollOuts.at(it).at(0).bDir == BACKWARD_RIGHT_DIR || laneRollOuts.at(it).at(0).bDir == BACKWARD_LEFT_DIR)
			tc.lane_change_cost = 2;
		else
			tc.lane_change_cost = 0;

		costs.push_back(tc);
	}

	return costs;
}

void TrajectoryCosts::CalculateTransitionCosts(vector<TrajectoryCost>& trajectoryCosts, const int& currTrajectoryIndex, const PlanningParams& params)
{
	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		trajectoryCosts.at(ic).transition_cost = fabs(params.rollOutDensity * (ic - currTrajectoryIndex));
	}
}

/**
 * @brief Validate input, each trajectory must have at least 1 way point
 * @param rollOuts
 * @return true if data isvalid for cost calculation
 */
bool TrajectoryCosts::ValidateRollOutsInput(const vector<vector<vector<WayPoint> > >& rollOuts)
{
	if(rollOuts.size()>0)
	{
			for(unsigned int il = 0; il < rollOuts.size(); il++)
			{
				if(rollOuts.at(il).size() > 0)
				{
					for(unsigned int it = 0; it < rollOuts.at(il).size(); it++)
					{
						if(rollOuts.at(il).at(it).size() < 2)
							return false;
					}
				}
				else
					return false;
			}
	}
	else
		return false;

	return true;
}



















































}
