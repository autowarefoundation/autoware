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

	m_pGoToGoalState->InsertNextState(m_pStopState);
	m_pGoToGoalState->InsertNextState(m_pWaitState);

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

	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 1,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 1,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 1,0));
	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 1,0));
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
 	pValues->distanceToNext  		= 0;
 	pValues->velocityOfNext			= 0;
 	pValues->bFullyBlock			= false;
 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= car_state.speed;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
 	pValues->iCurrSafeTrajectory    = pValues->iCentralTrajectory;


 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

void CarState::InitializeTrajectoryCosts()
{


}

void CarState::CalculateTransitionCosts()
{

}

void CarState::CalculateDistanceCosts(const PlannerHNS::VehicleState& state, const std::vector<PlannerHNS::DetectedObject>& obj_list)
{

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

 void CarState::SelectSafeTrajectoryAndSpeedProfile()
 {
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	if(preCalcPrams->iCurrSafeTrajectory >= 0 && preCalcPrams->iCurrSafeTrajectory < m_RollOuts.size())
	{
		m_Path = m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
		PlanningHelpers::GenerateRecommendedSpeed(m_Path,
				m_pCurrentBehaviorState->m_PlanningParams.maxSpeed,
				m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor);
		PlanningHelpers::SmoothSpeedProfiles(m_Path, 0.15,0.35, 0.1);
		std::ostringstream str_out;
		str_out << DataRW::LoggingFolderPath;
		str_out << DataRW::PathLogFolderName;
		str_out << "_";
		PlanningHelpers::WritePathToFile(str_out.str(), m_Path);
	}
	else if(m_RollOuts.size() > 0)
		std::cout << "Error .. Error .. Slected Trajectory is out of range !! ( " << preCalcPrams->iCurrSafeTrajectory << ")" << std::endl;

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

	SelectSafeTrajectoryAndSpeedProfile();

	return GenerateBehaviorState(vehicleState);
 }

 SimulatedCarState::SimulatedCarState()
 {
 	pLane = 0;
 	w = 0;
 	l = 0;
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

 	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 1,0));
 	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 1,0));
 	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 1,0));
 	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 1,0));
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
