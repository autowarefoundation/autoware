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
	SimulationNS::ControllerParams m_ControlParams;
	std::vector<PlannerHNS::GPSPoint> m_CarShapePolygon;
	std::vector<PlannerHNS::WayPoint> m_Path;
	std::vector<PlannerHNS::WayPoint> m_TotalPath;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_PredictedPath;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_RollOuts;
	std::string carId;
	PlannerHNS::Lane* pLane;
	double m_delayFactor; //second , time that every degree change in the steering wheel takes
	timespec m_SteerDelayTimer;
	double m_PredictionTime;

	PlannerHNS::BehaviorStateMachine* 		m_pCurrentBehaviorState;
	PlannerHNS::ForwardState * 				m_pGoToGoalState;
	PlannerHNS::StopState* 					m_pStopState;
	PlannerHNS::WaitState* 					m_pWaitState;
	PlannerHNS::InitState* 					m_pInitState;
	PlannerHNS::MissionAccomplishedState*	m_pMissionCompleteState;
	PlannerHNS::FollowState*				m_pFollowState;
	PlannerHNS::SwerveState*				m_pAvoidObstacleState;

	std::vector<PlannerHNS::TrajectoryCost> m_TrajectoryCosts;

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
	void Init(const SimulationNS::ControllerParams ctrlParams, const PlannerHNS::PlanningParams& params, const CAR_BASIC_INFO& carInfo);
	void InitPolygons();
	void FirstLocalizeMe(const PlannerHNS::WayPoint& initCarPos);
	void LocalizeMe(const double& dt); // in seconds
	void UpdateState(const PlannerHNS::VehicleState& state, const bool& bUseDelay = false);
	void CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
			const PlannerHNS::GPSPoint& goal);

	PlannerHNS::BehaviorState DoOneStep(
			const double& dt,
			const PlannerHNS::VehicleState& state,
			const std::vector<PlannerHNS::DetectedObject>& obj_list,
			const PlannerHNS::GPSPoint& goal,
			PlannerHNS::RoadNetwork& map,
			const bool& bLive = false);

	void SimulateOdoPosition(const double& dt, const PlannerHNS::VehicleState& vehicleState);

private:

	//Obstacle avoidance functionalities
	void InitializeTrajectoryCosts();
	void CalculateTransitionCosts();
	void CalculateDistanceCosts(const PlannerHNS::VehicleState& vstatus, const std::vector<PlannerHNS::DetectedObject>& obj_list);
	bool CalculateObstacleCosts(PlannerHNS::RoadNetwork& map, const PlannerHNS::VehicleState& vstatus, const std::vector<PlannerHNS::DetectedObject>& obj_list);

	double PredictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint>& path,
			const PlannerHNS::VehicleState& vstatus,
			const PlannerHNS::WayPoint& currState);

	void PredictObstacleTrajectory(PlannerHNS::RoadNetwork& map,
			const PlannerHNS::WayPoint& pos,
			const double& predTime,
			std::vector<std::vector<PlannerHNS::WayPoint> >& paths);

	bool CalculateIntersectionVelocities(std::vector<PlannerHNS::WayPoint>& path,
			std::vector<std::vector<PlannerHNS::WayPoint> >& predctedPath,
			const PlannerHNS::DetectedObject& obj);
	void FindSafeTrajectory(int& safe_index, double& closest_distance, double& closest_velocity);
	void FindNextBestSafeTrajectory(int& safe_index);
	bool IsGoalAchieved(const PlannerHNS::GPSPoint& goal);
	void UpdateCurrentLane(PlannerHNS::RoadNetwork& map, const double& search_distance);
	bool SelectSafeTrajectoryAndSpeedProfile(const PlannerHNS::VehicleState& vehicleState);
	PlannerHNS::BehaviorState GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState);
	void TransformPoint(const PlannerHNS::WayPoint& refPose, PlannerHNS::GPSPoint& p);
	void AddAndTransformContourPoints(const PlannerHNS::DetectedObject& obj, std::vector<PlannerHNS::WayPoint>& contourPoints);
};

class SimulatedCarState
{
public:
	PlannerHNS::WayPoint state;
	CAR_BASIC_INFO m_CarInfo;
	std::vector<PlannerHNS::GPSPoint> m_CarShapePolygon;
	std::vector<PlannerHNS::WayPoint> m_Path;
	std::vector<PlannerHNS::WayPoint> m_TotalPath;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_RollOuts;
	std::string carId;
	PlannerHNS::Lane* pLane;
	bool bDetected;

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

	//double m_CurrentAccSteerAngle; //degrees steer wheel range
	//double m_CurrentAccVelocity; // kilometer/hour

public:

	SimulatedCarState();
	virtual ~SimulatedCarState();
	void Init(const CAR_BASIC_INFO& carInfo);
	void InitPolygons();
	void FirstLocalizeMe(const PlannerHNS::WayPoint& initCarPos);
	void LocalizeMe(const double& dt); // in seconds

	void UpdateState(const bool& bUseDelay = false);
	void CalculateImportantParameterForDecisionMaking(const std::vector<PlannerHNS::DetectedObject>& obj_list,
			const PlannerHNS::VehicleState& car_state, const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map);

	PlannerHNS::BehaviorState DoOneStep(
			const double& dt,
			const PlannerHNS::VehicleState& state,
			const PlannerHNS::WayPoint& currPose,
			const PlannerHNS::GPSPoint& goal,
			PlannerHNS::RoadNetwork& map);

private:
	void SimulateOdoPosition(const double& dt, const PlannerHNS::VehicleState& vehicleState);
	void UpdateCurrentLane(PlannerHNS::RoadNetwork& map, const double& search_distance);
	PlannerHNS::BehaviorState GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState);
	bool SelectSafeTrajectoryAndSpeedProfile(const PlannerHNS::VehicleState& vehicleState);


};

} /* namespace SimulationNS */

#endif /* CARSTATE_H_ */
