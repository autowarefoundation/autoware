
/// \file DecisionMaker.h
/// \brief Initialize behaviors state machine, and calculate required parameters for the state machine transition conditions
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#ifndef BEHAVIOR_DECISION_MAKER
#define BEHAVIOR_DECISION_MAKER

#include "op_planner/BehaviorStateMachine.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/RoadNetwork.h"

namespace PlannerHNS
{

class DecisionMaker
{
public:
	WayPoint state;
	CAR_BASIC_INFO m_CarInfo;
	ControllerParams m_ControlParams;
	std::vector<WayPoint> m_Path;
	PlannerHNS::RoadNetwork m_Map;

	double m_MaxLaneSearchDistance;
	int m_iCurrentTotalPathId;
	std::vector<std::vector<WayPoint> > m_RollOuts;
	Lane* pLane;

	BehaviorStateMachine* 		m_pCurrentBehaviorState;
	StopState* 					m_pStopState;
	WaitState* 					m_pWaitState;
	SwerveStateII*				m_pAvoidObstacleState;
	TrafficLightStopStateII*	m_pTrafficLightStopState;
	TrafficLightWaitStateII*	m_pTrafficLightWaitState;

	ForwardStateII * 			m_pGoToGoalState;;
	InitStateII* 				m_pInitState;
	MissionAccomplishedStateII*	m_pMissionCompleteState;
	GoalStateII*				m_pGoalState;
	FollowStateII*				m_pFollowState;
	StopSignStopStateII* 			m_pStopSignStopState;
	StopSignWaitStateII* 			m_pStopSignWaitState;

	void InitBehaviorStates();

	//For Simulation
	UtilityHNS::PIDController 	m_pidVelocity;
	UtilityHNS::PIDController 	m_pidStopping;
	UtilityHNS::PIDController 	m_pidFollowing;

public:

	DecisionMaker();
	virtual ~DecisionMaker();
	void Init(const ControllerParams& ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo);
	void CalculateImportantParameterForDecisionMaking(const VehicleState& car_state,
			const int& goalID, const bool& bEmergencyStop, const std::vector<TrafficLight>& detectedLights,
			const TrajectoryCost& bestTrajectory);
	void SetNewGlobalPath(const std::vector<std::vector<WayPoint> >& globalPath);

	BehaviorState DoOneStep(
			const double& dt,
			const PlannerHNS::WayPoint currPose,
			const PlannerHNS::VehicleState& vehicleState,
			const int& goalID,
			const std::vector<TrafficLight>& trafficLight,
			const TrajectoryCost& tc,
			const bool& bEmergencyStop);

protected:
	bool GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<TrafficLight>& trafficLights, TrafficLight& trafficL);
	void UpdateCurrentLane(const double& search_distance);
	bool SelectSafeTrajectory();
	BehaviorState GenerateBehaviorState(const VehicleState& vehicleState);
	double UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt);
	bool ReachEndOfGlobalPath(const double& min_distance, const int& iGlobalPathIndex);



	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	std::vector<std::vector<WayPoint> > m_TotalOriginalPath;
	std::vector<std::vector<WayPoint> > m_TotalPath;
	PlannerHNS::PlanningParams m_params;

};

} /* namespace PlannerHNS */

#endif /* BEHAVIOR_DECISION_MAKER */
