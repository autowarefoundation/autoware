/*
 * CarState.h
 *
 *  Created on: Dec 14, 2016
 *      Author: hatem
 */

#ifndef LOCALPLANNERH_H_
#define LOCALPLANNERH_H_

#include "BehaviorStateMachine.h"
#include "PlannerCommonDef.h"
#include "RoadNetwork.h"

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
	std::vector<std::vector<WayPoint> > m_TotalPath;
	std::vector<std::vector<WayPoint> > m_PredictedPath;
	std::vector<std::vector<WayPoint> > m_RollOuts;
	std::string carId;
	Lane* pLane;
	double m_SimulationSteeringDelayFactor; //second , time that every degree change in the steering wheel takes
	timespec m_SteerDelayTimer;
	double m_PredictionTime;

	BehaviorStateMachine* 		m_pCurrentBehaviorState;
	ForwardState * 				m_pGoToGoalState;
	StopState* 					m_pStopState;
	WaitState* 					m_pWaitState;
	InitState* 					m_pInitState;
	MissionAccomplishedState*	m_pMissionCompleteState;
	FollowState*				m_pFollowState;
	SwerveState*				m_pAvoidObstacleState;
	TrafficLightStopState*		m_pTrafficLightStopState;
	TrafficLightWaitState*		m_pTrafficLightWaitState;


	std::vector<TrajectoryCost> m_TrajectoryCosts;

	void InitBehaviorStates();

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
	std::vector<TrafficLight> m_TrafficLights;

public:

	LocalPlannerH();
	virtual ~LocalPlannerH();
	void Init(const ControllerParams ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo);
	void InitPolygons();
	void FirstLocalizeMe(const WayPoint& initCarPos);
	void LocalizeMe(const double& dt); // in seconds
	void UpdateState(const VehicleState& state, const bool& bUseDelay = false);
	void CalculateImportantParameterForDecisionMaking(const VehicleState& car_state,
			const GPSPoint& goal,
			const bool& bEmergencyStop,
			const bool& bGreenTrafficLight);

	BehaviorState DoOneStep(
			const double& dt,
			const VehicleState& state,
			const std::vector<DetectedObject>& obj_list,
			const GPSPoint& goal,
			RoadNetwork& map,
			const bool& bEmergencyStop,
			const bool& bGreenTrafficLight,
			const bool& bLive = false);

	void SimulateOdoPosition(const double& dt, const VehicleState& vehicleState);

private:

	//Obstacle avoidance functionalities
	void InitializeTrajectoryCosts();
	void CalculateTransitionCosts();
	void CalculateDistanceCosts(const VehicleState& vstatus, const std::vector<DetectedObject>& obj_list);
	bool CalculateObstacleCosts(RoadNetwork& map, const VehicleState& vstatus, const std::vector<DetectedObject>& obj_list);

	double PredictTimeCostForTrajectory(std::vector<WayPoint>& path,
			const VehicleState& vstatus,
			const WayPoint& currState);

	void PredictObstacleTrajectory(RoadNetwork& map,
			const WayPoint& pos,
			const double& predTime,
			std::vector<std::vector<WayPoint> >& paths);

	bool CalculateIntersectionVelocities(std::vector<WayPoint>& path,
			std::vector<std::vector<WayPoint> >& predctedPath,
			const DetectedObject& obj);
	void FindSafeTrajectory(int& safe_index, double& closest_distance, double& closest_velocity);
	void FindNextBestSafeTrajectory(int& safe_index);
	bool GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<TrafficLight>& trafficLights, TrafficLight& trafficL);
	bool IsGoalAchieved(const GPSPoint& goal);
	void UpdateCurrentLane(RoadNetwork& map, const double& search_distance);
	bool SelectSafeTrajectoryAndSpeedProfile(const VehicleState& vehicleState);
	BehaviorState GenerateBehaviorState(const VehicleState& vehicleState);
	void TransformPoint(const WayPoint& refPose, GPSPoint& p);
	void AddAndTransformContourPoints(const DetectedObject& obj, std::vector<WayPoint>& contourPoints);
};

} /* namespace PlannerHNS */

#endif /* LOCALPLANNERH_H_ */
