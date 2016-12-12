/*
 * TrajectoryFollower.h
 *
 *  Created on: Jun 18, 2016
 *      Author: hatem
 */

#ifndef TRAJECTORYFOLLOWER_H_
#define TRAJECTORYFOLLOWER_H_
#include "RoadNetwork.h"
#include "UtilityH.h"
#include "CommonSimuDefinitions.h"

namespace SimulationNS
{

class TrajectoryFollower
{
public:
	TrajectoryFollower();
	virtual ~TrajectoryFollower();

	void PrepareNextWaypoint(const PlannerHNS::WayPoint& CurPos, const double& currVelocity, const double& currSteering);

	void UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path);

	int SteerControllerUpdate(const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredSteerAngle);
	int VeclocityControllerUpdate(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity, PlannerHNS::SHIFT_POS& desiredShift);

	void Init(const ControllerParams& params, const CAR_BASIC_INFO& vehicleInfo);

	PlannerHNS::VehicleState DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
				const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
				const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory);

	//Testing Points
	PlannerHNS::WayPoint 	m_ForwardSimulation;
	PlannerHNS::WayPoint 	m_PerpendicularPoint;
	PlannerHNS::WayPoint 	m_FollowMePoint;
	double 					m_LateralError;
	double 					m_FollowingDistance;
	PlannerHNS::WayPoint 	m_CurrPos;
	int 					m_iCalculatedIndex;


private:
	double 						m_StartFollowDistance;
	double 						m_FollowAcc;
	ControllerParams 			m_Params;
	CAR_BASIC_INFO 				m_VehicleInfo;
	std::vector<PlannerHNS::WayPoint> 	m_Path;
	PlannerHNS::WayPoint 		m_DesPos;
	double						m_PrevDesiredSteer; // control output
	double 						m_FollowAcceleration;
	int 						m_iPrevWayPoint;
	UtilityHNS::PIDController 	m_pidSteer;
	UtilityHNS::LowpassFilter 	m_lowpassSteer;

	UtilityHNS::PIDController 	m_pidVelocity;
	UtilityHNS::LowpassFilter 	m_lowpassVelocity;

	std::vector<std::string>    m_LogData;
	std::vector<std::string>    m_LogSteerPIDData;
	std::vector<std::string>    m_LogVelocityPIDData;

	bool FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
			const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
			double& lateral_err, double& follow_distance);

	int SteerControllerPart(const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
			const double& lateral_error, double& steerd);

	void PredictMotion(double& x, double &y, double& heading, double steering, double velocity,
			double wheelbase, double time_elapsed);

//	PlannerHNS::WayPoint SimulatePathFollow(const double& sampling_rate, const double& sim_distance,
//			const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
//			const double& velocity, const ControllerParams& params, const CAR_BASIC_INFO& vehicleInfo);

	double GetPID_LinearChange(double minVal, double maxVal, double speedMax, double currSpeed);

	void AdjustPID(const double& v, const double& maxV,  PID_CONST& steerPID);

	int CalculateVelocityDesired(const double& dt, const double& currVel,const PlannerHNS::STATE_TYPE& CurrBehavior,
			double& desiredVel);



};

} /* namespace SimulationNS */

#endif /* TRAJECTORYFOLLOWER_H_ */
