
/// \file LocalPlannerH.h
/// \brief OpenPlanner's local planing functions combines in one process, used in simulation vehicle and OpenPlanner old implementation like dp_planner node.
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#ifndef LOCALPLANNERH_H_
#define LOCALPLANNERH_H_

#include "BehaviorStateMachine.h"
#include "PlannerCommonDef.h"
#include "RoadNetwork.h"
#include "TrajectoryCosts.h"

#define AVOIDANCE_SPEED_FACTOR 0.75
namespace PlannerHNS
{

class LocalPlannerH
{
public:
	WayPoint state;
	CAR_BASIC_INFO m_CarInfo;
	ControllerParams m_ControlParams;
	std::vector<GPSPoint> m_CarShapePolygon;
	std::vector<WayPoint> m_Path;
	std::vector<WayPoint> m_OriginalLocalPath;
	std::vector<std::vector<WayPoint> > m_TotalPath;
	std::vector<std::vector<WayPoint> > m_TotalOriginalPath;
	std::vector<DetectedObject> m_PredictedTrajectoryObstacles;
	int m_iCurrentTotalPathId;
	int m_iSafeTrajectory;
	double m_InitialFollowingDistance;
//	int m_iGlobalPathPrevID;
//	std::vector<std::vector<WayPoint> > m_PredictedPath;
	std::vector<std::vector<std::vector<WayPoint> > > m_RollOuts;
	std::string carId;
	Lane* pLane;
	double m_SimulationSteeringDelayFactor; //second , time that every degree change in the steering wheel takes
	timespec m_SteerDelayTimer;
	double m_PredictionTime;
	double m_CostCalculationTime;
	double m_BehaviorGenTime;
	double m_RollOutsGenerationTime;
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

	TrajectoryCosts m_TrajectoryCostsCalculatotor;

	//for debugging

	std::vector<WayPoint> m_SampledPoints;

	void InitBehaviorStates();

	void ReInitializePlanner(const WayPoint& start_pose);

	void SetSimulatedTargetOdometryReadings(const double& velocity_d, const double& steering_d, const SHIFT_POS& shift_d)
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
	WayPoint m_OdometryState;
	double m_CurrentVelocity, m_CurrentVelocityD; //meter/second
	double m_CurrentSteering, m_CurrentSteeringD; //radians
	SHIFT_POS m_CurrentShift , m_CurrentShiftD;

	double m_CurrentAccSteerAngle; //degrees steer wheel range
	double m_CurrentAccVelocity; // kilometer/hour
	//std::vector<TrafficLight> m_TrafficLights;

	UtilityHNS::PIDController 	m_pidVelocity;
	UtilityHNS::PIDController 	m_pidStopping;

public:

	LocalPlannerH();
	virtual ~LocalPlannerH();
	void Init(const ControllerParams& ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo);
	void InitPolygons();
	void FirstLocalizeMe(const WayPoint& initCarPos);
	void LocalizeMe(const double& dt); // in seconds
	void UpdateState(const VehicleState& state, const bool& bUseDelay = false);
	void CalculateImportantParameterForDecisionMaking(const VehicleState& car_state,
			const int& goalID, const bool& bEmergencyStop, const vector<TrafficLight>& detectedLights,
			const TrajectoryCost& bestTrajectory);

	BehaviorState DoOneStep(
			const double& dt,
			const VehicleState& state,
			const std::vector<DetectedObject>& obj_list,
			const int& goalID,
			RoadNetwork& map,
			const bool& bEmergencyStop,
			const std::vector<TrafficLight>& trafficLight,
			const bool& bLive = false);

	void SimulateOdoPosition(const double& dt, const VehicleState& vehicleState);

private:

	//Obstacle avoidance functionalities
//	bool CalculateObstacleCosts(RoadNetwork& map, const VehicleState& vstatus, const std::vector<DetectedObject>& obj_list);
//
//	double PredictTimeCostForTrajectory(std::vector<WayPoint>& path,
//			const VehicleState& vstatus,
//			const WayPoint& currState);
//
//	void PredictObstacleTrajectory(RoadNetwork& map,
//			const WayPoint& pos,
//			const double& predTime,
//			std::vector<std::vector<WayPoint> >& paths);
//
//	bool CalculateIntersectionVelocities(std::vector<WayPoint>& path,
//			std::vector<std::vector<WayPoint> >& predctedPath,
//			const DetectedObject& obj);

	bool GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<TrafficLight>& trafficLights, TrafficLight& trafficL);
	void UpdateCurrentLane(RoadNetwork& map, const double& search_distance);
	bool SelectSafeTrajectoryAndSpeedProfile(const VehicleState& vehicleState);
	BehaviorState GenerateBehaviorState(const VehicleState& vehicleState);
	void TransformPoint(const WayPoint& refPose, GPSPoint& p);
	void AddAndTransformContourPoints(const DetectedObject& obj, std::vector<WayPoint>& contourPoints);
	double UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt);

	void ExtractHorizonAndCalculateRecommendedSpeed();

	bool NoWayTest(const double& min_distance, const int& iGlobalPathIndex);

	PlannerHNS::PlanningParams m_params;
};

} /* namespace PlannerHNS */

#endif /* LOCALPLANNERH_H_ */
