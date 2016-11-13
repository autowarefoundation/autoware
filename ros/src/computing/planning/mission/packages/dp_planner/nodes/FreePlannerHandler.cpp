/*
 * FreePlannerHandler.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: ai-driver
 */

#include "FreePlannerHandler.h"

namespace PlannerXNS {

FreePlannerHandler::FreePlannerHandler() : PlannerH_Handler()
{
}

FreePlannerHandler::~FreePlannerHandler()
{
}



bool FreePlannerHandler::GeneratePlan(const geometry_msgs::Pose& currentPose, const cv_tracker::obj_label& detectedObstacles,
		const lidar_tracker::CloudClusterArray& clusters,
		const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
		AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize,
		waypoint_follower::LaneArray& pathToFollow, jsk_recognition_msgs::BoundingBoxArray& trackedObjects,
		visualization_msgs::MarkerArray& detectedPolygons,
		const bool& bEmergencyStop, const bool& bGreenLight, const bool& bOutsideControl)
{

	bool bNewPath = false;
//	m_State.state = PlannerHNS::WayPoint(currentPose.position.x+m_OriginPoint.pos.x,
//			currentPose.position.y + m_OriginPoint.pos.y, currentPose.position.z + m_OriginPoint.pos.z, tf::getYaw(currentPose.orientation));
//	std::vector<PlannerHNS::DetectedObject> obst;
//	ConvertFromAutowareObstaclesToPlannerH(detectedObstacles, obst);
//	PlannerHNS::VehicleState vehState;
//	vehState.speed = carState.speed;
//	vehState.steer = carState.steer;
//	vehState.shift = ConvertShiftFromAutowareToPlannerH(carState.shift);
//	PlannerHNS::BehaviorState behavior;
//
//	if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
//		std::cout << "Shift DDDDD" << std::endl;
//	else 	if(vehState.shift == PlannerHNS::SHIFT_POS_NN)
//		std::cout << "Shift NNNNN" << std::endl;
//
//	if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
//		m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 1;
//	else
//		m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 0;
//
//	m_State.CalculateImportantParameterForDecisionMaking(obst, vehState, m_Goal.pos, m_Map);
//
//
//	m_State.m_pCurrentBehaviorState = m_State.m_pCurrentBehaviorState->GetNextState();
//
//	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_State.m_pCurrentBehaviorState->GetCalcParams();
//	behavior.state = m_State.m_pCurrentBehaviorState->m_Behavior;
//	if(behavior.state == PlannerHNS::FOLLOW_STATE)
//		behavior.followDistance = preCalcPrams->distanceToNext;
//	else
//		behavior.followDistance = 0;
//
//	if(preCalcPrams->bUpcomingRight)
//		behavior.indicator = PlannerHNS::INDICATOR_RIGHT;
//	else if(preCalcPrams->bUpcomingLeft)
//		behavior.indicator = PlannerHNS::INDICATOR_LEFT;
//	else
//		behavior.indicator = PlannerHNS::INDICATOR_NONE;
//
//	//TODO fix this , make get lookahead velocity work
//	double max_velocity = 2; //BehaviorsNS::MappingHelpers::GetLookAheadVelocity(m_CarState.m_Path, GetCarPos(), 25);
//
//	behavior.maxVelocity   = max_velocity;
//	behavior.minVelocity	= 0;
//	behavior.stopDistance 	= preCalcPrams->distanceToStop();
//	behavior.followVelocity = preCalcPrams->velocityOfNext;
//
//	std::cout <<  preCalcPrams->ToString(behavior.state) << std::endl;
//
//	if(behavior.state == PlannerHNS::INITIAL_STATE && m_State.m_Path.size() == 0)
//	{
//		std::vector<PlannerHNS::WayPoint> generatedPath;
//		//planner.PlanUsingReedShepp(m_State.state, m_goal, generatedPath);
//		//ROS_INFO(m_OriginPoint.pos.ToString().c_str());
//		//ROS_INFO(m_State.state.pos.ToString().c_str());
//		//ROS_INFO(m_Goal.pos.ToString().c_str());
//		//m_pPlannerH->PlanUsingDP(m_State.pLane, m_State.state, m_Goal, m_State.state, 2550, m_PredefinedPath, generatedPath);
//
//		m_pPlannerH->PlanUsingReedShepp(m_State.state, m_Goal, generatedPath);
//
//		m_State.m_TotalPath = generatedPath;
//	}
//
//	if(m_State.m_TotalPath.size()>0)
//	{
//
//		int currIndex = 1;
//
//		if(m_State.m_Path.size()>0)
//		{
//			currIndex = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_State.m_Path, m_State.state);
////			std::cout << "before Roll Outs .. " << currIndex << std::endl;
////			std::cout << m_State.state.pos.ToString() << std::endl;
////			std::cout << m_State.m_Path.at(0).pos.ToString() << std::endl;
//		}
//
//		if(m_State.m_RollOuts.size() == 0 || currIndex*2.0 > m_State.m_Path.size())
//		{
//			m_pPlannerH->GenerateRunoffTrajectory(m_State.m_TotalPath, m_State.state, false,  5, 35, 5, 0,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.carTipMargin,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollInMargin,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollInSpeedFactor,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.pathDensity,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollOutDensity,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingDataWeight,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingSmoothWeight,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingToleranceError,
//					m_State.m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor,
//					false, m_State.m_RollOuts);
//
//			//std::cout << "Safe Trajectoy : " << preCalcPrams->iCurrSafeTrajectory << std::endl;
//
//
//			m_State.m_Path = m_State.m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
//
//			if(m_State.m_Path.size() >  0 )
//				bNewPath = true;
//
//			if(m_State.m_Path.size()<5)
//				preCalcPrams->bGoalReached = true;
//
//			//std::cout << "after get next .. " << std::endl;
//
//		}
//
//	}
//
//
////	ROS_INFO(preCalcPrams->ToString(behavior.state).c_str());
//
//
//	//behaviorState = behavior;
//	if(bNewPath)
//	{
//		//ROS_INFO("Convert New Path To Visualization.");
//		ConvertFromPlannerHToAutowarePathFormat(m_State.m_Path, pathToFollow);
//		ConvertFromPlannerHToAutowareVisualizePathFormat(m_State.m_TotalPath, m_State.m_Path, m_State.m_RollOuts, pathToVisualize);
//	}

	return bNewPath;
}

} /* namespace PlannerXNS */
