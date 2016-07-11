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
	state.pos.x = m_OdometryState.pos.x;//	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base) * cos (m_OdometryState.pos.a));
	state.pos.y = m_OdometryState.pos.y;//	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base/2.0) * sin (m_OdometryState.pos.a));
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

 void CarState::CalculateImportantParameterForDecisionMaking(const std::vector<PlannerHNS::Obstacle>& obj_list,
		 const PlannerHNS::VehicleState& car_state, const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map)
 {
 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	PlannerHNS::Lane* pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
 	PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, 3.0);

 	if(pPathLane)
 		pLane = pPathLane;
 	else if(pMapLane)
 		pLane = pMapLane;
 	else
 		pLane = 0;

 	pValues->minStoppingDistance	= car_state.speed * 3.6 * 1.5;

 	//pValues->minStoppingDistance	= 12;
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


 	//Mission Complete
 	double distance_to_goal = distance2points(state.pos , goal);
 	if(distance_to_goal < 1.5)
 		pValues->bGoalReached = true;
 	else
 		pValues->bGoalReached = false;

 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;

 }


} /* namespace SimulationNS */
