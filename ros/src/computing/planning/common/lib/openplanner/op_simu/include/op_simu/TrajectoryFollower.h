
/// \file TrajectoryFollower.h
/// \brief PID based trajectory follower
/// \author Hatem Darweesh
/// \date Jun 18, 2016


#ifndef TRAJECTORYFOLLOWER_H_
#define TRAJECTORYFOLLOWER_H_
#include "op_planner/RoadNetwork.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlannerCommonDef.h"


#define MAX_ACCELERATION_2G 5 // meter /sec/sec
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

	void Init(const PlannerHNS::ControllerParams& params, const PlannerHNS::CAR_BASIC_INFO& vehicleInfo, bool bEnableLogs = false, bool bCalibration = false);

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
	bool					m_bEndPath;
	double 					m_WayPointsDensity;


private:
	double 						m_StartFollowDistance;
	double 						m_FollowAcc;
	PlannerHNS::ControllerParams 			m_Params;
	PlannerHNS::CAR_BASIC_INFO 				m_VehicleInfo;
	std::vector<PlannerHNS::WayPoint> 	m_Path;
	PlannerHNS::WayPoint 		m_DesPos;
	double						m_PrevDesiredSteer; // control output
	double 						m_FollowAcceleration;
	int 						m_iPrevWayPoint;
	UtilityHNS::PIDController 	m_pidSteer;
	UtilityHNS::LowpassFilter 	m_lowpassSteer;

	UtilityHNS::PIDController 	m_pidVelocity;
	UtilityHNS::LowpassFilter 	m_lowpassVelocity;

	bool						m_bEnableLog;
	std::vector<std::string>    m_LogData;
	std::vector<std::string>    m_LogSteerPIDData;
	std::vector<std::string>    m_LogVelocityPIDData;

	//Steering and Velocity Calibration Global Variables
	bool						m_bCalibrationMode;
	int							m_iNextTest;
	std::vector<std::string>    m_SteerCalibrationData;
	std::vector<std::string>    m_VelocityCalibrationData;
	PlannerHNS::VehicleState 	m_prevCurrState_steer;
	PlannerHNS::VehicleState 	m_prevDesiredState_steer;
	PlannerHNS::VehicleState 	m_prevCurrState_vel;
	PlannerHNS::VehicleState 	m_prevDesiredState_vel;
	struct timespec 			m_SteerDelayTimer;
	struct timespec 			m_VelocityDelayTimer;
	std::vector<std::pair<double, double> > m_CalibrationRunList;


	bool FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
			const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
			double& lateral_err, double& follow_distance);

	int SteerControllerPart(const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
			const double& lateral_error, double& steerd);

	void PredictMotion(double& x, double &y, double& heading, double steering, double velocity,
			double wheelbase, double time_elapsed);

	double GetPID_LinearChange(double minVal, double maxVal, double speedMax, double currSpeed);

	void AdjustPID(const double& v, const double& maxV,  PlannerHNS::PID_CONST& steerPID);

	int CalculateVelocityDesired(const double& dt, const double& currVel,const PlannerHNS::STATE_TYPE& CurrBehavior,
			double& desiredVel);

	void LogCalibrationData(const PlannerHNS::VehicleState& currState,const PlannerHNS::VehicleState& desiredState);
	void InitCalibration();
	void CalibrationStep(const double& dt, const PlannerHNS::VehicleState& CurrStatus, double& desiredSteer, double& desiredVelocity);
};

} /* namespace SimulationNS */

#endif /* TRAJECTORYFOLLOWER_H_ */
