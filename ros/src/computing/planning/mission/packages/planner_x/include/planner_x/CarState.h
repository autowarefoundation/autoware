/*
 * CarState.h
 *
 *  Created on: Jun 20, 2016
 *      Author: hatem
 */

#ifndef CARSTATE_H_
#define CARSTATE_H_

#include "BehaviorStateMachine.h"
#include "CommonSimuDefinitions.h"
#include "RoadNetwork.h"



namespace SimulationNS
{

class CarState
{
public:
	PlannerHNS::WayPoint state;
	CAR_BASIC_INFO m_CarInfo;
	double w,l;
	std::vector<PlannerHNS::GPSPoint> m_CarShapePolygon;
	std::vector<PlannerHNS::WayPoint> m_Path;
	std::vector<PlannerHNS::WayPoint> m_TotalPath;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_RollOuts;
	std::string carId;
	PlannerHNS::Lane* pLane;

	PlannerHNS::BehaviorStateMachine* 		m_pCurrentBehaviorState;
	PlannerHNS::ForwardState * 				m_pGoToGoalState;
	PlannerHNS::StopState* 					m_pStopState;
	PlannerHNS::WaitState* 					m_pWaitState;
	PlannerHNS::InitState* 					m_pInitState;
	PlannerHNS::MissionAccomplishedState*	m_pMissionCompleteState;

	void InitBehaviorStates();

	void SetSimulatedTargetOdometryReadings(const double& velocity_d, const double& steering_d, const PlannerHNS::SHIFT_POS& shift_d)
	{
		m_CurrentVelocityD = velocity_d;
		m_CurrentSteeringD = steering_d;
		m_CurrentShiftD = shift_d;
	}

	double GetSimulatedVelocity()
	{
		return m_CurrentVelocity;
	}

	double GetSimulatedSteering()
	{
		return m_CurrentSteering;
	}

	double GetSimulatedShift()
	{
		return m_CurrentShift;
	}


	//For Simulation
	PlannerHNS::WayPoint m_OdometryState;
	double m_CurrentVelocity, m_CurrentVelocityD; //meter/second
	double m_CurrentSteering, m_CurrentSteeringD; //radians
	PlannerHNS::SHIFT_POS m_CurrentShift , m_CurrentShiftD;

	double m_CurrentAccSteerAngle; //degrees steer wheel range
	double m_CurrentAccVelocity; // kilometer/hour

public:

	CarState();
	virtual ~CarState();
	void Init(const double& width, const double& length, const CAR_BASIC_INFO& carInfo);
	void InitPolygons();
	void FirstLocalizeMe(const PlannerHNS::WayPoint& initCarPos);
	void LocalizeMe(const double& dt); // in seconds
	double GetMomentumScaleFactor(const double& v); //v is in meter/second
	void UpdateState(const bool& bUseDelay = false);
	void CalculateImportantParameterForDecisionMaking(const std::vector<PlannerHNS::Obstacle>& obj_list,
			const PlannerHNS::VehicleState& car_state, const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map);

};

} /* namespace SimulationNS */

#endif /* CARSTATE_H_ */
