/*
 * TrajectoryFollower.cpp
 *
 *  Created on: Jun 18, 2016
 *      Author: hatem
 */

#include "TrajectoryFollower.h"
#include "PlanningHelpers.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>

using namespace PlannerHNS;
using namespace UtilityHNS;
using namespace std;


namespace SimulationNS
{

TrajectoryFollower::TrajectoryFollower()
{
	m_WayPointsDensity = 1;
	m_bEndPath = false;
	m_FollowingDistance = 0;
	m_LateralError 		= 0;
	m_PrevDesiredSteer	= 0;
	m_FollowAcceleration= 0;
	m_iPrevWayPoint 	= -1;
	m_StartFollowDistance = 0;
	m_FollowAcc = 0.5;
	m_iCalculatedIndex = 0;
}

void TrajectoryFollower::Init(const ControllerParams& params, const CAR_BASIC_INFO& vehicleInfo)
{
	m_Params = params;
	m_VehicleInfo = vehicleInfo;

	//m_pidSteer.Init(0.1, 0.005, 0.001); // for 5 m/s
	//m_pidSteer.Init(0.07, 0.02, 0.01); // for 3 m/s
	//m_pidSteer.Init(0.9, 0.1, 0.2); //for lateral error
	//m_pidVelocity.Init(0.1, 0.005, 0.1);
	m_lowpassSteer.Init(2, 100, 4);

	m_pidSteer.Init(params.Steering_Gain.kP, params.Steering_Gain.kI, params.Steering_Gain.kD); // for 3 m/s
	m_pidSteer.Setlimit(m_VehicleInfo.max_steer_angle, -m_VehicleInfo.max_steer_angle);
	m_pidVelocity.Init(params.Velocity_Gain.kP, params.Velocity_Gain.kI, params.Velocity_Gain.kD);
}

TrajectoryFollower::~TrajectoryFollower()
{
//	DataRW::WriteLogData(UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName+DataRW::ControlLogFolderName, "ControlLog",
//			"time,X,Y,heading, Target, error,LateralError,SteerBeforLowPass,Steer,iIndex, pathSize",
//			m_LogData);
//
//	DataRW::WriteLogData(UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName+DataRW::ControlLogFolderName, "SteeringPIDLog",m_pidSteer.ToStringHeader(), m_LogSteerPIDData );
//	DataRW::WriteLogData(UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName+DataRW::ControlLogFolderName, "VelocityPIDLog",m_pidVelocity.ToStringHeader(), m_LogVelocityPIDData );
}

void TrajectoryFollower::PrepareNextWaypoint(const PlannerHNS::WayPoint& CurPos, const double& currVelocity, const double& currSteering)
{
	WayPoint pred_point = CurPos;

	//m_ForwardSimulation = SimulatePathFollow(0.01, m_Params.SteeringDelay*currVelocity, m_Path, pred_point, currVelocity, m_Params.Wheelbase);

	m_ForwardSimulation = pred_point;
	double nIterations = m_Params.SteeringDelay/0.01; //angle error
	//double nIterations = 0.5/0.01; //lateral  error
	for(unsigned int i=0; i< nIterations; i++)
	{
		PredictMotion(m_ForwardSimulation.pos.x, m_ForwardSimulation.pos.y, m_ForwardSimulation.pos.a, currSteering,currVelocity, m_VehicleInfo.wheel_base, 0.01);
	}

	m_CurrPos = m_ForwardSimulation;

	bool ret = FindNextWayPoint(m_Path, pred_point, currVelocity, m_FollowMePoint, m_PerpendicularPoint, m_LateralError, m_FollowingDistance);
	if(ret)
	{
		m_DesPos = m_FollowMePoint;
	//	m_DesPos.a = m_pDesAngCir->CalcAngle(m_DesPos.a);
	}
}

void TrajectoryFollower::UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path)
{
	//BehaviorsNS::MappingHelpers::ConvertFromWaypointsToVectorPath(path, m_Path);
	if(path.size()>0)
		m_WayPointsDensity = hypot(path.at(1).pos.y - path.at(0).pos.y, path.at(1).pos.x - path.at(0).pos.x);

	m_Path = path;
}

bool TrajectoryFollower::FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
		const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
		double& lateral_err, double& follow_distance)
{
	if(path.size()==0) return false;

	follow_distance = fabs(velocity) * 1.5;
	if(follow_distance < m_Params.minPursuiteDistance)
		follow_distance = m_Params.minPursuiteDistance;

	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, state, info);
	pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(path, info, follow_distance);
	prep = info.perp_point;
	lateral_err = info.perp_distance;
	m_iPrevWayPoint = info.iFront;

	double d_critical = (-velocity*velocity)/2.0*m_VehicleInfo.max_deceleration;
	int nPointToEnd = path.size() - m_iPrevWayPoint;
	double totalD = m_WayPointsDensity*nPointToEnd;

	if(totalD <= 1 || totalD <= d_critical)
	{
		m_bEndPath = true;
		cout << "Critical Distance: " << d_critical << endl;
	}
	else
		m_bEndPath = false;

	return true;
}

int TrajectoryFollower::SteerControllerUpdate(const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredSteerAngle)
{
	if(m_Path.size()==0) return -1;
	int ret = -1;
	//AdjustPID(CurrStatus.velocity, 18.0, m_Params.Gain);
	if(CurrBehavior.state == FORWARD_STATE || CurrBehavior.state == TRAFFIC_LIGHT_STOP_STATE || CurrBehavior.state == STOP_SIGN_STOP_STATE || CurrBehavior.state  == FOLLOW_STATE)
		ret = SteerControllerPart(m_CurrPos, m_DesPos, m_LateralError, desiredSteerAngle);

	if(ret < 0)
		desiredSteerAngle = m_PrevDesiredSteer;
	else
		m_PrevDesiredSteer = desiredSteerAngle;

	return ret;
}

int TrajectoryFollower::SteerControllerPart(const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
		const double& lateral_error, double& steerd)
{
	double current_a = UtilityH::SplitPositiveAngle(state.pos.a);
	double target_a = atan2(way_point.pos.y - state.pos.y, way_point.pos.x - state.pos.x);

	double e =  UtilityH::SplitPositiveAngle(target_a - current_a);

//	if(e > M_PI_2 || e < -M_PI_2)
//		return -1;


	double before_lowpass = m_pidSteer.getPID(e);
	//cout << m_pidSteer.ToString() << endl;

//	m_LogSteerPIDData.push_back(m_pidSteer.ToString());

	//TODO use lateral error instead of angle error
	//double future_lateral_error = PlanningHelpers::GetPerpDistanceToTrajectorySimple(m_Path, m_ForwardSimulation,0);


	//steerd = m_pidSteer.getPID( future_lateral_error*-1, 0);

//	if(m_LateralError < 0)
//		steerd = m_pidSteer.getPID(current_a+sqrt(abs(m_LateralError)), target_a);
//	else
//		steerd = m_pidSteer.getPID(current_a-sqrt(m_LateralError), target_a);


	//cout << "Error : " << e << ", Current A: " << current_a << ", Target A: " << target_a <<  " Steeting Angle = " << steerd*RAD2DEG << endl;
//	if(abs(before_lowpass) < m_Params.MaxSteerAngle*0.5)
//		steerd = m_lowpassSteer.getFilter(before_lowpass);
//	else
		steerd = before_lowpass;

//	timespec t;
//	UtilityH::GetTickCount(t);
//	std::ostringstream dataLine;
//	dataLine << UtilityH::GetLongTime(t) << "," << state.pos.x << "," << state.pos.y << "," <<  current_a << "," <<
//			target_a << "," <<  e << "," <<m_LateralError << "," <<  before_lowpass << "," <<  steerd <<  "," <<
//			m_iPrevWayPoint << "," << m_Path.size() << ",";
//	m_LogData.push_back(dataLine.str());

	return 1;
}

void TrajectoryFollower::PredictMotion(double& x, double &y, double& heading, double steering, double velocity, double wheelbase, double time_elapsed)
{
	x += velocity * time_elapsed *  cos(heading);
	y += velocity * time_elapsed *  sin(heading);
	heading = heading + ((velocity*time_elapsed*tan(steering))  / (wheelbase) );
}

int TrajectoryFollower::VeclocityControllerUpdate(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity, PlannerHNS::SHIFT_POS& desiredShift)
{
	double critical_long_front_distance =  2.0;

	if(CurrBehavior.state == TRAFFIC_LIGHT_STOP_STATE || CurrBehavior.state == STOP_SIGN_STOP_STATE || m_bEndPath)
	{
		double deceleration_critical = m_VehicleInfo.max_deceleration;
		if(CurrBehavior.stopDistance != 0)
			deceleration_critical = (-CurrStatus.speed*CurrStatus.speed)/(2.0*CurrBehavior.stopDistance);

		desiredVelocity = (deceleration_critical * dt) + CurrStatus.speed;
		//desiredVelocity = (m_VehicleInfo.max_deceleration * dt) + CurrStatus.speed;
		//desiredVelocity = 0;
	}
	else if(CurrBehavior.state == FORWARD_STATE || CurrBehavior.state == OBSTACLE_AVOIDANCE_STATE )
	{

		double e = CurrBehavior.maxVelocity - CurrStatus.speed;

		//Using PID for velocity
		//m_pidVelocity.Setlimit(CurrBehavior.maxVelocity, 0);
		//desiredVelocity = m_pidVelocity.getPID(e);

		//Using constant acceleration for velocity
		if(e >= 0)
			desiredVelocity = (m_VehicleInfo.max_acceleration * dt) + CurrStatus.speed;
		else
			desiredVelocity = (m_VehicleInfo.max_deceleration * dt) + CurrStatus.speed;

		if(desiredVelocity > CurrBehavior.maxVelocity)
			desiredVelocity = CurrBehavior.maxVelocity;
		else if(desiredVelocity<m_VehicleInfo.min_speed_forward)
			desiredVelocity = m_VehicleInfo.min_speed_forward;

		//std::cout << "Velocity from follower : dt=" << dt << ", e= " << e << ", acc_const=" << acc_const << ", desiredVelocity = "<<desiredVelocity<<  std::endl;

		m_StartFollowDistance = 0;
	}
	else if(CurrBehavior.state == STOPPING_STATE || CurrBehavior.state == FINISH_STATE)
	{
		desiredVelocity = 0;
	}
	else if(CurrBehavior.state == FOLLOW_STATE)
	{
		double deceleration_critical = 0;
		double inv_time = 2.0*((CurrBehavior.followDistance-critical_long_front_distance)-CurrStatus.speed);
		if(inv_time == 0)
			deceleration_critical = MAX_ACCELERATION_2G;
		else
			deceleration_critical = CurrStatus.speed*CurrStatus.speed/inv_time;

		if(deceleration_critical > 0) deceleration_critical = -deceleration_critical;
		if(deceleration_critical < -MAX_ACCELERATION_2G) deceleration_critical = -MAX_ACCELERATION_2G;

		desiredVelocity = (deceleration_critical * dt) + CurrStatus.speed;

		if(desiredVelocity > CurrBehavior.maxVelocity)
			desiredVelocity = CurrBehavior.maxVelocity;

		if((desiredVelocity < 0.1 && desiredVelocity > -0.1) || CurrBehavior.followDistance <= 0) //use only effective velocities
			desiredVelocity = 0;

		cout << "Follow State:  acceleration = " << deceleration_critical << ", speed = " << desiredVelocity <<  ", Distance = " << CurrBehavior.followDistance<< endl;
	}
	else
	{
		desiredVelocity = 0;
	}

	if(desiredVelocity > m_VehicleInfo.max_speed_forward)
		desiredVelocity = m_VehicleInfo.max_speed_forward;
	else if (desiredVelocity < 0)
		desiredVelocity = 0;
	//desiredVelocity = 2.0;



	desiredShift = PlannerHNS::SHIFT_POS_DD;
	//m_LogVelocityPIDData.push_back(m_pidVelocity.ToString());
	return 1;
}

PlannerHNS::VehicleState TrajectoryFollower::DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
		const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
		const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory)
{
	if(bNewTrajectory && path.size() > 0)
	{
		UpdateCurrentPath(path);
		m_iPrevWayPoint = -1;
	}

	PlannerHNS::VehicleState currState;

	if(m_Path.size()>0 && behavior.state != INITIAL_STATE )
	{
		PrepareNextWaypoint(currPose, vehicleState.speed, vehicleState.steer);
		VeclocityControllerUpdate(dt, vehicleState, behavior, currState.speed, currState.shift);
		SteerControllerUpdate(vehicleState, behavior, currState.steer);
	}
	else
	{
		currState.steer = 0;
		currState.speed = 0;
		currState.shift = PlannerHNS::SHIFT_POS_DD;
		//cout << ">>> Error, Very Dangerous, Following No Path !!." << endl;
	}

	return currState;
}

//PlannerHNS::WayPoint TrajectoryFollower::SimulatePathFollow(const double& sampling_rate, const double& sim_distance,
//			const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
//			const double& velocity, const ControllerParams& params, const CAR_BASIC_INFO& vehicleInfo)
//{
//	UtilityHNS::PIDController pidSteer, pidVelocity;
//	pidSteer.Init(params.Steering_Gain.kP, params.Steering_Gain.kI, params.Steering_Gain.kD); // for 3 m/s
//	pidSteer.Setlimit(m_VehicleInfo.max_steer_angle, -m_VehicleInfo.max_steer_angle);
//	pidVelocity.Init(params.Velocity_Gain.kP, params.Velocity_Gain.kI, params.Velocity_Gain.kD);
//}

} /* namespace SimulationNS */
