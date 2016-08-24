/*
 * CarState.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: hatem
 */

#include "CarState.h"
#include "UtilityH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include "MatrixOperations.h"
#include "PlannerH.h"
#include <cmath>

using namespace PlannerHNS;
using namespace UtilityHNS;

namespace SimulationNS
{

CarState::CarState()
{
	pLane = 0;
	w = 0;
	l = 0;
	m_CurrentVelocity =  m_CurrentVelocityD =0;
	m_CurrentSteering = m_CurrentSteeringD =0;
	m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;
	m_pCurrentBehaviorState = 0;
	m_pGoToGoalState = 0;
	m_pStopState= 0;
	m_pWaitState= 0;
	m_pMissionCompleteState= 0;
	m_pAvoidObstacleState = 0;
	m_pFollowState = 0;

	InitBehaviorStates();
}

CarState::~CarState()
{

}

void CarState::Init(const double& width, const double& length, const CAR_BASIC_INFO& carInfo)
 	{
 		m_CarInfo = carInfo;
 		w = width;
 		l = length;
 		m_CurrentVelocity =  m_CurrentVelocityD =0;
 		m_CurrentSteering = m_CurrentSteeringD =0;
 		m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
 		m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;
 	}

void CarState::InitBehaviorStates()
{

	m_pStopState 				= new StopState(0);
	m_pMissionCompleteState 	= new MissionAccomplishedState(0);
	m_pGoToGoalState 			= new ForwardState(m_pMissionCompleteState);
	m_pWaitState 				= new WaitState(m_pGoToGoalState);
	m_pInitState 				= new InitState(m_pGoToGoalState);
	m_pFollowState				= new FollowState(m_pGoToGoalState);
	m_pAvoidObstacleState		= new SwerveState(m_pGoToGoalState);

	m_pGoToGoalState->InsertNextState(m_pStopState);
	m_pGoToGoalState->InsertNextState(m_pWaitState);
	m_pGoToGoalState->InsertNextState(m_pFollowState);
	m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);

	m_pAvoidObstacleState->InsertNextState(m_pStopState);
	m_pAvoidObstacleState->InsertNextState(m_pWaitState);
	m_pAvoidObstacleState->InsertNextState(m_pFollowState);

	m_pFollowState->InsertNextState(m_pStopState);
	m_pFollowState->InsertNextState(m_pWaitState);
	m_pFollowState->InsertNextState(m_pAvoidObstacleState);

	m_pCurrentBehaviorState = m_pInitState;

}

void CarState::InitPolygons()
{
	double l2 = l/2.0;
	double w2 = w/2.0;

	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 0,0));

//	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 1,0));
//	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 1,0));
//	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 1,0));
//	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 1,0));
}

 void CarState::FirstLocalizeMe(const WayPoint& initCarPos)
 {
	pLane = initCarPos.pLane;
	state = initCarPos;
	m_OdometryState.pos.a = initCarPos.pos.a;
	m_OdometryState.pos.x = initCarPos.pos.x + (m_CarInfo.wheel_base/2.0 * cos(initCarPos.pos.a));
	m_OdometryState.pos.y = initCarPos.pos.y + (m_CarInfo.wheel_base/2.0 * sin(initCarPos.pos.a));
 }

 void CarState::LocalizeMe(const double& dt)
{
	//calculate the new x, y ,
	 WayPoint currPose = state;

	if(m_CurrentShift == SHIFT_POS_DD)
	{
		m_OdometryState.pos.x	 +=  m_CurrentVelocity * dt * cos(currPose.pos.a);
		m_OdometryState.pos.y	 +=  m_CurrentVelocity * dt * sin(currPose.pos.a);
		m_OdometryState.pos.a	 +=  m_CurrentVelocity * dt * tan(m_CurrentSteering)  / m_CarInfo.wheel_base;

	}
	else if(m_CurrentShift == SHIFT_POS_RR )
	{
		m_OdometryState.pos.x	 +=  -m_CurrentVelocity * dt * cos(currPose.pos.a);
		m_OdometryState.pos.y	 +=  -m_CurrentVelocity * dt * sin(currPose.pos.a);
		m_OdometryState.pos.a	 +=  -m_CurrentVelocity * dt * tan(m_CurrentSteering);
	}

	m_OdometryState.pos.a = atan2(sin(m_OdometryState.pos.a), cos(m_OdometryState.pos.a));
	m_OdometryState.pos.a = UtilityH::FixNegativeAngle(m_OdometryState.pos.a);

	state.pos.a = m_OdometryState.pos.a;
	state.pos.x = m_OdometryState.pos.x	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base) * cos (m_OdometryState.pos.a));
	state.pos.y = m_OdometryState.pos.y	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base/2.0) * sin (m_OdometryState.pos.a));
}

 double CarState::GetMomentumScaleFactor(const double& v)
 {
 	if(v < 0.3)
 		return 0.6;
 	else if(v <6.4)
 		return 0.3;
 	else if(v < 20)
 	{
 		double m = 0.7/3.6;
 		return m*(v - 6.4) + 0.3;
 	}
 	else
 		return 1.0;
 }

 void CarState::UpdateState(const bool& bUseDelay)
  {
	 m_CurrentSteering 	= m_CurrentSteeringD;
	 m_CurrentShift 	= m_CurrentShiftD;
	 m_CurrentVelocity = m_CurrentVelocityD;
  }

 void CarState::AddAndTransformContourPoints(const PlannerHNS::DetectedObject& obj, std::vector<PlannerHNS::WayPoint>& contourPoints)
 {
	 contourPoints.clear();
	 WayPoint  p;
	 for(unsigned int i=0; i< obj.contour.size(); i++)
	 {
		 p.pos = obj.contour.at(i);
		 TransformPoint(obj.center, p.pos);
		 contourPoints.push_back(p);
	 }

	 contourPoints.push_back(obj.center);
 }

 void CarState::TransformPoint(const PlannerHNS::WayPoint& refPose, PlannerHNS::GPSPoint& p)
 {
 	PlannerHNS::Mat3 rotationMat(refPose.pos.a);
 	PlannerHNS::Mat3 translationMat(refPose.pos.x, refPose.pos.y);
	p = rotationMat*p;
	p = translationMat*p;
 }

 void CarState::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
		 const PlannerHNS::GPSPoint& goal)
 {
 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	//Mission Complete
 	pValues->bGoalReached = IsGoalAchieved(goal);
 	pValues->minStoppingDistance	= car_state.speed * 3.6 * 1.5;
 	if(pValues->distanceToNext > 0 || pValues->distanceToStop()>0)
 		pValues->minStoppingDistance += 1.0;
 	pValues->iCentralTrajectory		= m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber/2;
 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= car_state.speed;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
 	pValues->bRePlan 				= false;
 	FindSafeTrajectory(pValues->iCurrSafeTrajectory, pValues->distanceToNext, pValues->velocityOfNext);
 	if(pValues->iCurrSafeTrajectory == -1)
 		pValues->bFullyBlock = true;
 	else
 		pValues->bFullyBlock = false;

 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

void CarState::InitializeTrajectoryCosts()
{
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;
	int centralIndex = pParams->rollOutNumber/2;
	std::vector<double> end_distance_list;

	m_TrajectoryCosts.clear();

	//double totalCost = 1.0 / (double)pParams->rollOutNumber;
	double totalDistance = 0;
	for(int i=0; i< pParams->rollOutNumber+1; i++)
	{
		PlannerHNS::TrajectoryCost tc;
		tc.index = i;
		tc.relative_index = i - centralIndex;
		tc.distance_from_center = pParams->rollOutDensity*tc.relative_index;
		tc.priority_cost = fabs(tc.distance_from_center);
		totalDistance += tc.priority_cost;
		m_TrajectoryCosts.push_back(tc);
	}

	if(totalDistance==0) return ;

	//Normalize cost
	for(unsigned int i = 0; i< m_TrajectoryCosts.size(); i++)
	{
		m_TrajectoryCosts.at(i).priority_cost = m_TrajectoryCosts.at(i).priority_cost/totalDistance;
	}
}

void CarState::CalculateTransitionCosts()
{
	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;

	double totalDistance = 0;
	//pValues->iCurrSafeTrajectory = 4;
	if(pValues->iCentralTrajectory < 0)
		pValues->iCentralTrajectory = pParams->rollOutNumber / 2;

	if(pValues->iCurrSafeTrajectory < 0)
		pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;

	if(pValues->iPrevSafeTrajectory < 0)
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;

	for(unsigned int i = 0; i< m_TrajectoryCosts.size(); i++)
	{
		m_TrajectoryCosts.at(i).transition_cost = fabs(pParams->rollOutDensity* (m_TrajectoryCosts.at(i).index - pValues->iCurrSafeTrajectory));
		totalDistance += m_TrajectoryCosts.at(i).transition_cost;
	}

	if(totalDistance==0) return ;

	//Normalize cost
	for(unsigned int i = 0; i< m_TrajectoryCosts.size(); i++)
	{
		m_TrajectoryCosts.at(i).transition_cost = m_TrajectoryCosts.at(i).transition_cost/totalDistance;
	}
}

void CarState::CalculateDistanceCosts(const PlannerHNS::VehicleState& vstatus, const std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;
	double critical_lateral_distance = pParams->rollOutDensity/2.0 + m_CarInfo.width/2.0;

	if(m_TotalPath.size()==0) return;

	//First Filtering
	int iEgoIndex = PlanningHelpers::GetClosestPointIndex(m_TotalPath, state);
	for(unsigned int i = 0 ; i < obj_list.size(); i++)
	{
		std::vector<WayPoint> contourPoints;
		AddAndTransformContourPoints(obj_list.at(i), contourPoints);

		double distance_direct_smallest = 9999999;
		double distance_on_trajectory_smallest = 9999999;
		for(unsigned int j = 0; j < contourPoints.size(); j++)
		{
			double distance_direct = distance2points(state.pos, contourPoints.at(j).pos);
			if(distance_direct < distance_direct_smallest)
				distance_direct_smallest = distance_direct;

			double distance_on_trajectory  = PlanningHelpers::GetDistanceOnTrajectory(m_TotalPath, iEgoIndex, contourPoints.at(j)) - m_CarInfo.length/2.0;
			if(distance_on_trajectory > 0 && distance_on_trajectory < distance_on_trajectory_smallest)
				distance_on_trajectory_smallest = distance_on_trajectory;
		}

		for(unsigned int j = 0; j < contourPoints.size(); j++)
		{
			PlannerHNS::WayPoint wp;
			wp = contourPoints.at(j);

			double distance_lateral = PlanningHelpers::GetPerpDistanceToTrajectorySimple(m_TotalPath, wp);

//			if(distance_direct > pParams->horizonDistance)
//				continue;

			for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
			{
				double normalized_cost = 1.0 - (distance_on_trajectory_smallest / pParams->horizonDistance);
				double d_diff = fabs(distance_lateral - m_TrajectoryCosts.at(c).distance_from_center);
				m_TrajectoryCosts.at(c).lateral_costs.push_back(std::make_pair(j,d_diff));

				if(d_diff < critical_lateral_distance && (distance_on_trajectory_smallest < m_TrajectoryCosts.at(c).closest_obj_distance || m_TrajectoryCosts.at(c).closest_obj_distance <= 0))
				{
					//if(normalized_cost > m_TrajectoryCosts.at(c).closest_obj_cost)
					{
						m_TrajectoryCosts.at(c).closest_obj_cost = normalized_cost;
						m_TrajectoryCosts.at(c).closest_obj_distance = distance_on_trajectory_smallest;
						m_TrajectoryCosts.at(c).closest_obj_velocity = obj_list.at(i).center.v;

					}
				}
			}
		}
	}
}

void  CarState::FindSafeTrajectory(int& safe_index, double& closest_distance, double& closest_velocity)
{
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;

	//if the  closest_obj_cost is less than 0.9 (12 meter) consider this trajectory blocked
	closest_distance = pParams->horizonDistance;
//	std::cout << "----------------------------------------------------" << std::endl;
//	std::cout << ">> Costs: " << std::endl;
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
//		std::cout << m_TrajectoryCosts.at(c).ToString() << std::endl;

		if(m_TrajectoryCosts.at(c).closest_obj_cost >= 0.8)
			m_TrajectoryCosts.at(c).cost = 1;
		else
			m_TrajectoryCosts.at(c).cost =
					(m_TrajectoryCosts.at(c).closest_obj_cost +
					m_TrajectoryCosts.at(c).priority_cost +
					m_TrajectoryCosts.at(c).transition_cost)/3.0;

		if(m_TrajectoryCosts.at(c).closest_obj_distance > 0 && m_TrajectoryCosts.at(c).closest_obj_distance < closest_distance)
		{
			closest_distance = m_TrajectoryCosts.at(c).closest_obj_distance;
			closest_velocity = m_TrajectoryCosts.at(c).closest_obj_velocity;
		}
	}

	int smallestIndex = -1;
	double smallestCost = 1;
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
		if(m_TrajectoryCosts.at(c).cost < smallestCost)
		{
			smallestCost = m_TrajectoryCosts.at(c).cost;
			smallestIndex = c;
		}
	}

	safe_index = smallestIndex;

//	std::cout << "Selected Trajectory: " << safe_index << ", Closest Distance: " << closest_distance << std::endl;
//	std::cout << "----------------------------------------------------" << std::endl;
}

void CarState::FindNextBestSafeTrajectory(int& safe_index)
{
	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
		m_TrajectoryCosts.at(c).cost =
				(m_TrajectoryCosts.at(c).priority_cost +
				m_TrajectoryCosts.at(c).transition_cost)/2.0;
	}

	int smallestIndex = pValues->iCentralTrajectory;
	double smallestCost = 1;
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
		if(m_TrajectoryCosts.at(c).cost < smallestCost)
		{
			smallestCost = m_TrajectoryCosts.at(c).cost;
			smallestIndex = c;
		}
	}

	safe_index = smallestIndex;
}

 void CarState::UpdateCurrentLane(PlannerHNS::RoadNetwork& map, const double& search_distance)
 {
	PlannerHNS::Lane* pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
	PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, search_distance);

	if(pPathLane)
		pLane = pPathLane;
	else if(pMapLane)
		pLane = pMapLane;
	else
		pLane = 0;
 }

 void CarState::SimulateOdoPosition(const double& dt, const PlannerHNS::VehicleState& vehicleState)
 {
	SetSimulatedTargetOdometryReadings(vehicleState.speed, vehicleState.steer, vehicleState.shift);
	UpdateState(false);
	LocalizeMe(dt);
 }

 bool CarState::IsGoalAchieved(const PlannerHNS::GPSPoint& goal)
 {
	double distance_to_goal = distance2points(state.pos , goal);
	if(distance_to_goal < 1.5)
		return true;
	else
		return false;
 }

 bool CarState::SelectSafeTrajectoryAndSpeedProfile(const PlannerHNS::VehicleState& vehicleState)
 {
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	bool bNewTrajectory = false;
	if(m_TotalPath.size()>0)
	{
		int currIndex = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_Path, state);
		int index_limit = m_Path.size() - 20;
		if(index_limit<=0)
			index_limit =  m_Path.size()/2.0;
		if(m_RollOuts.size() == 0
				|| currIndex > index_limit
				|| m_pCurrentBehaviorState->GetCalcParams()->bRePlan
				|| m_pCurrentBehaviorState->m_Behavior == OBSTACLE_AVOIDANCE_STATE)
		{
			PlannerHNS::PlanningInternalParams params;
			PlannerHNS::PlannerH planner(params);
			planner.GenerateRunoffTrajectory(m_TotalPath, state, false,
					vehicleState.speed,
					m_pCurrentBehaviorState->m_PlanningParams.microPlanDistance,
					m_pCurrentBehaviorState->m_PlanningParams.maxSpeed,
					m_pCurrentBehaviorState->m_PlanningParams.minSpeed,
					m_pCurrentBehaviorState->m_PlanningParams.carTipMargin,
					m_pCurrentBehaviorState->m_PlanningParams.rollInMargin,
					m_pCurrentBehaviorState->m_PlanningParams.rollInSpeedFactor,
					m_pCurrentBehaviorState->m_PlanningParams.pathDensity,
					m_pCurrentBehaviorState->m_PlanningParams.rollOutDensity,
					m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber,
					m_pCurrentBehaviorState->m_PlanningParams.smoothingDataWeight,
					m_pCurrentBehaviorState->m_PlanningParams.smoothingSmoothWeight,
					m_pCurrentBehaviorState->m_PlanningParams.smoothingToleranceError,
					m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor,
					false, m_RollOuts);

			m_pCurrentBehaviorState->GetCalcParams()->bRePlan = false;

			//FindNextBestSafeTrajectory(pValues->iCurrSafeTrajectory);
			if(preCalcPrams->iCurrSafeTrajectory >= 0
					&& preCalcPrams->iCurrSafeTrajectory < m_RollOuts.size()
					&& m_pCurrentBehaviorState->m_Behavior == OBSTACLE_AVOIDANCE_STATE)
			{
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCurrSafeTrajectory;
				m_Path = m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
				bNewTrajectory = true;
			}
			else
			{
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCentralTrajectory;
				m_Path = m_RollOuts.at(preCalcPrams->iCentralTrajectory);
				bNewTrajectory = true;
			}

			PlanningHelpers::GenerateRecommendedSpeed(m_Path,
					m_pCurrentBehaviorState->m_PlanningParams.maxSpeed,
					m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor);
			PlanningHelpers::SmoothSpeedProfiles(m_Path, 0.15,0.35, 0.1);
			std::ostringstream str_out;
			str_out << DataRW::LoggingFolderPath;
			str_out << DataRW::PathLogFolderName;
			str_out << "_";
			PlanningHelpers::WritePathToFile(str_out.str(), m_Path);
//			}
//			else if(m_RollOuts.size() > 0)
//				std::cout << "Error .. Error .. Slected Trajectory is out of range !! ( " << preCalcPrams->iCurrSafeTrajectory << ")" << std::endl;

		}
	}

	return bNewTrajectory;
 }

 PlannerHNS::BehaviorState CarState::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
 {
	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	if(currentBehavior.state == PlannerHNS::FOLLOW_STATE)
		currentBehavior.followDistance = preCalcPrams->distanceToNext;
	else
		currentBehavior.followDistance = 0;

	if(preCalcPrams->bUpcomingRight)
		currentBehavior.indicator = PlannerHNS::INDICATOR_RIGHT;
	else if(preCalcPrams->bUpcomingLeft)
		currentBehavior.indicator = PlannerHNS::INDICATOR_LEFT;
	else
		currentBehavior.indicator = PlannerHNS::INDICATOR_NONE;

	currentBehavior.maxVelocity 	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, state, 1.5*vehicleState.speed*3.6);
	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;

	return currentBehavior;
 }

 PlannerHNS::BehaviorState CarState::DoOneStep(const double& dt,
		 const PlannerHNS::VehicleState& vehicleState,
		 const std::vector<PlannerHNS::DetectedObject>& obj_list,
		 const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map	,const bool& bLive)
{
	 if(!bLive)
		 SimulateOdoPosition(dt, vehicleState);

	UpdateCurrentLane(map, 3.0);

	InitializeTrajectoryCosts();

	CalculateTransitionCosts();

	CalculateDistanceCosts(vehicleState, obj_list);

	CalculateImportantParameterForDecisionMaking(vehicleState, goal);

	PlannerHNS::BehaviorState beh = GenerateBehaviorState(vehicleState);

	beh.bNewPlan = SelectSafeTrajectoryAndSpeedProfile(vehicleState);

	return beh;
 }


 SimulatedCarState::SimulatedCarState()
 {
 	pLane = 0;
 	w = 0;
 	l = 0;
 	maxSpeed = 0;
 	m_CurrentVelocity =  m_CurrentVelocityD =0;
 	m_CurrentSteering = m_CurrentSteeringD =0;
 	m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
 	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;

 }

 SimulatedCarState::~SimulatedCarState()
 {

 }

 void SimulatedCarState::Init(const double& width, const double& length, const CAR_BASIC_INFO& carInfo)
  	{
  		m_CarInfo = carInfo;
  		w = width;
  		l = length;
  		m_CurrentVelocity =  m_CurrentVelocityD =0;
  		m_CurrentSteering = m_CurrentSteeringD =0;
  		m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
  		m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;
  	}


 void SimulatedCarState::InitPolygons()
 {
 	double l2 = l/2.0;
 	double w2 = w/2.0;

 	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 0,0));
 	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 0,0));
 	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 0,0));
 	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 0,0));

// 	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 1,0));
// 	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 1,0));
// 	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 1,0));
// 	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 1,0));
 }

  void SimulatedCarState::FirstLocalizeMe(const WayPoint& initCarPos)
  {
 	pLane = initCarPos.pLane;
 	state = initCarPos;
 	m_OdometryState.pos.a = initCarPos.pos.a;
 	m_OdometryState.pos.x = initCarPos.pos.x + (m_CarInfo.wheel_base/2.0 * cos(initCarPos.pos.a));
 	m_OdometryState.pos.y = initCarPos.pos.y + (m_CarInfo.wheel_base/2.0 * sin(initCarPos.pos.a));
  }

  void SimulatedCarState::LocalizeMe(const double& dt)
 {
 	//calculate the new x, y ,
 	 WayPoint currPose = state;

 	if(m_CurrentShift == SHIFT_POS_DD)
 	{
 		m_OdometryState.pos.x	 +=  m_CurrentVelocity * dt * cos(currPose.pos.a);
 		m_OdometryState.pos.y	 +=  m_CurrentVelocity * dt * sin(currPose.pos.a);
 		m_OdometryState.pos.a	 +=  m_CurrentVelocity * dt * tan(m_CurrentSteering)  / m_CarInfo.wheel_base;

 	}
 	else if(m_CurrentShift == SHIFT_POS_RR )
 	{
 		m_OdometryState.pos.x	 +=  -m_CurrentVelocity * dt * cos(currPose.pos.a);
 		m_OdometryState.pos.y	 +=  -m_CurrentVelocity * dt * sin(currPose.pos.a);
 		m_OdometryState.pos.a	 +=  -m_CurrentVelocity * dt * tan(m_CurrentSteering);
 	}

 	m_OdometryState.pos.a = atan2(sin(m_OdometryState.pos.a), cos(m_OdometryState.pos.a));
 	m_OdometryState.pos.a = UtilityH::FixNegativeAngle(m_OdometryState.pos.a);

 	state.pos.a = m_OdometryState.pos.a;
 	state.pos.x = m_OdometryState.pos.x	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base) * cos (m_OdometryState.pos.a));
 	state.pos.y = m_OdometryState.pos.y	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base/2.0) * sin (m_OdometryState.pos.a));
 }

  double SimulatedCarState::GetMomentumScaleFactor(const double& v)
  {
  	if(v < 0.3)
  		return 0.6;
  	else if(v <6.4)
  		return 0.3;
  	else if(v < 20)
  	{
  		double m = 0.7/3.6;
  		return m*(v - 6.4) + 0.3;
  	}
  	else
  		return 1.0;
  }

  void SimulatedCarState::UpdateState(const bool& bUseDelay)
   {
 	 m_CurrentSteering 	= m_CurrentSteeringD;
 	 m_CurrentShift 	= m_CurrentShiftD;
 	 m_CurrentVelocity = m_CurrentVelocityD;
   }

  void SimulatedCarState::CalculateImportantParameterForDecisionMaking(const std::vector<PlannerHNS::DetectedObject>& obj_list,
 		 const PlannerHNS::VehicleState& car_state, const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map)
  {

  	PlannerHNS::Lane* pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
  	PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, 3.0);

  	if(pPathLane)
  		pLane = pPathLane;
  	else if(pMapLane)
  		pLane = pMapLane;
  	else
  		pLane = 0;

  	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;

  }

  PlannerHNS::BehaviorState SimulatedCarState::DoOneStep(const double& dt, const PlannerHNS::VehicleState& vehicleState, const PlannerHNS::WayPoint& currPose,
  			const std::vector<PlannerHNS::DetectedObject>& obj_list, const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map, const bool& bLive)
{
	if(!bLive)
	{
		SetSimulatedTargetOdometryReadings(vehicleState.speed, vehicleState.steer, vehicleState.shift);
		UpdateState(false);
		LocalizeMe(dt);
	}

 	CalculateImportantParameterForDecisionMaking(obj_list, vehicleState, goal, map);

 	PlannerHNS::BehaviorState currentBehavior;

 	currentBehavior.state = PlannerHNS::FORWARD_STATE;
 	currentBehavior.followDistance = 0;
 	currentBehavior.maxVelocity 	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, currPose, 1.5*vehicleState.speed*3.6);
 	currentBehavior.minVelocity		= 0;
 	currentBehavior.stopDistance 	= 0;
 	currentBehavior.followVelocity 	= 0;

 	return currentBehavior;
}

} /* namespace SimulationNS */
