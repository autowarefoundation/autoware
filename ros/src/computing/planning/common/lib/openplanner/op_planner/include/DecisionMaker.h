/*
 * CarState.h
 *
 *  Created on: Dec 14, 2016
 *      Author: hatem
 */

#ifndef BEHAVIOR_DECISION_MAKER
#define BEHAVIOR_DECISION_MAKER

#include "BehaviorStateMachine.h"
#include "PlannerCommonDef.h"
#include "RoadNetwork.h"

namespace PlannerHNS
{

class DecisionMaker
{
public:
	WayPoint state;
	CAR_BASIC_INFO m_CarInfo;
	ControllerParams m_ControlParams;
	std::vector<WayPoint> m_Path;
	std::vector<std::vector<WayPoint> > m_TotalPath;
	std::vector<std::vector<WayPoint> > m_TotalOriginalPath;
	PlannerHNS::RoadNetwork m_Map;

	double m_MaxLaneSearchDistance;
	int m_iCurrentTotalPathId;
	int m_iSafeTrajectory;
	std::vector<std::vector<WayPoint> > m_RollOuts;
	Lane* pLane;
	int m_PrevBrakingWayPoint;

	BehaviorStateMachine* 		m_pCurrentBehaviorState;
	ForwardState * 				m_pGoToGoalState;
	StopState* 					m_pStopState;
	WaitState* 					m_pWaitState;
	InitState* 					m_pInitState;
	MissionAccomplishedState*	m_pMissionCompleteState;
	GoalState*					m_pGoalState;
	FollowState*				m_pFollowState;
	SwerveState*				m_pAvoidObstacleState;
	TrafficLightStopState*		m_pTrafficLightStopState;
	TrafficLightWaitState*		m_pTrafficLightWaitState;
	StopSignStopState* 			m_pStopSignStopState;
	StopSignWaitState* 			m_pStopSignWaitState;

	void InitBehaviorStates();

	//For Simulation
	UtilityHNS::PIDController 	m_pidVelocity;
	UtilityHNS::PIDController 	m_pidStopping;

public:

	DecisionMaker();
	virtual ~DecisionMaker();
	void Init(const ControllerParams& ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo);
	void CalculateImportantParameterForDecisionMaking(const VehicleState& car_state,
			const int& goalID, const bool& bEmergencyStop, const std::vector<TrafficLight>& detectedLights,
			const TrajectoryCost& bestTrajectory);

	BehaviorState DoOneStep(
			const double& dt,
			const PlannerHNS::WayPoint currPose,
			const PlannerHNS::VehicleState& vehicleState,
			const int& goalID,
			const std::vector<TrafficLight>& trafficLight,
			const TrajectoryCost& tc,
			const bool& bEmergencyStop);

private:
	bool GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<TrafficLight>& trafficLights, TrafficLight& trafficL);
	void UpdateCurrentLane(const double& search_distance);
	bool SelectSafeTrajectory();
	BehaviorState GenerateBehaviorState(const VehicleState& vehicleState);
	double UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt);

	bool NoWayTest(const double& min_distance, const int& iGlobalPathIndex);

	PlannerHNS::PlanningParams m_params;
};

} /* namespace PlannerHNS */

#endif /* BEHAVIOR_DECISION_MAKER */
