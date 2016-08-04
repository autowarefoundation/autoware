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
	m_FollowingDistance = 0;
	m_LateralError 		= 0;
	m_PrevDesiredSteer	= 0;
	m_FollowAcceleration= 0;
	m_iPrevWayPoint 	= -1;

	//m_pidSteer.Init(0.1, 0.005, 0.001); // for 5 m/s
	m_pidSteer.Init(0.07, 0.02, 0.01); // for 3 m/s
	//m_pidSteer.Init(0.9, 0.1, 0.2); //for lateral error
	m_pidSteer.Setlimit(m_Params.MaxSteerAngle, -m_Params.MaxSteerAngle);
	m_pidVelocity.Setlimit(3.0, 0);
	m_pidVelocity.Init(0.1, 0.01, 0);

	m_lowpassSteer.Init(2, 100, 5);
}

TrajectoryFollower::~TrajectoryFollower()
{
	DataRW::WriteLogData(DataRW::LoggingFolderPath+DataRW::ControlLogFolderName, "ControlLog",
			"time,X,Y,heading, Target, error,LateralError,SteerBeforLowPass,Steer,iIndex, pathSize",
			m_LogData);

	DataRW::WriteLogData(DataRW::LoggingFolderPath+DataRW::ControlLogFolderName, "SteeringPIDLog",m_pidSteer.ToStringHeader(), m_LogSteerPIDData );
	DataRW::WriteLogData(DataRW::LoggingFolderPath+DataRW::ControlLogFolderName, "VelocityPIDLog",m_pidVelocity.ToStringHeader(), m_LogVelocityPIDData );
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
		PredictMotion(m_ForwardSimulation.pos.x, m_ForwardSimulation.pos.y, m_ForwardSimulation.pos.a, currSteering,currVelocity, m_Params.Wheelbase, 0.01);
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
	m_Path = path;
}

bool TrajectoryFollower::FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
		const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
		double& lateral_err, double& follow_distance)
{
	if(path.size()==0) return false;

	if(velocity > 3.0)
		follow_distance = m_Params.PursuiteDistance + abs(velocity) * 0.5;
	else
		follow_distance = m_Params.PursuiteDistance + fabs(velocity) * 0.25;
	if(follow_distance < m_Params.PursuiteDistance)
		follow_distance = m_Params.PursuiteDistance;

	//follow_distance = 4.5;

	int iWayPoint =  PlanningHelpers::GetClosestNextPointIndex(path, state);
	if(m_iPrevWayPoint >=0  && m_iPrevWayPoint < path.size() && iWayPoint < m_iPrevWayPoint)
		iWayPoint = m_iPrevWayPoint;

	m_iPrevWayPoint = iWayPoint;

	double distance_to_perp = 0;
	prep = PlanningHelpers::GetPerpendicularOnTrajectory(path, state, distance_to_perp, iWayPoint);
	//m_LateralError = MathUtil::Distance(m_PerpendicularPoint.p, state.p);
	lateral_err = PlanningHelpers::GetPerpDistanceToTrajectorySimple(path, state, iWayPoint );
	//m_LateralError = CalculateLateralDistanceToCurve(m_Path, state, m_iNextWayPoint);
	pursuite_point = PlanningHelpers::GetNextPointOnTrajectory(path, follow_distance - distance_to_perp, iWayPoint);

	//lateral_err = distance_to_perp;

	return true;
}

int TrajectoryFollower::SteerControllerUpdate(const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredSteerAngle)
{
	if(m_Path.size()==0) return -1;

	//AdjustPID(CurrStatus.velocity, 18.0, m_Params.Gain);
	int ret = SteerControllerPart(m_CurrPos, m_DesPos, m_LateralError, desiredSteerAngle);
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
	m_LogSteerPIDData.push_back(m_pidSteer.ToString());

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

	timespec t;
	UtilityH::GetTickCount(t);
	std::ostringstream dataLine;
	dataLine << t.tv_nsec << "," << state.pos.x << "," << state.pos.y << "," <<  current_a << "," <<
			target_a << "," <<  e << "," <<m_LateralError << "," <<  before_lowpass << "," <<  steerd <<  "," <<
			m_iPrevWayPoint << "," << m_Path.size() << ",";
	m_LogData.push_back(dataLine.str());

	return 1;
}

void TrajectoryFollower::PredictMotion(double& x, double &y, double& heading, double steering, double velocity, double wheelbase, double time_elapsed)
{
	x += velocity * time_elapsed *  cos(heading);
	y += velocity * time_elapsed *  sin(heading);
	heading = heading + ((velocity*time_elapsed*tan(steering))  / (wheelbase) );
}

void TrajectoryFollower::UpdateParams(const ControllerParams& params)
{
	m_Params = params;
}

int TrajectoryFollower::VeclocityControllerUpdate(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity)
{

	desiredVelocity = CurrBehavior.maxVelocity;
	//desiredVelocity = m_pidVelocity.getPID(CurrStatus.speed, CurrBehavior.maxVelocity);
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

	if(behavior.state == PlannerHNS::FORWARD_STATE)
	{
		if(m_Path.size()>0)
		{
			PrepareNextWaypoint(currPose, vehicleState.speed, vehicleState.steer);
			VeclocityControllerUpdate(dt, currState,behavior, currState.speed);
			SteerControllerUpdate(currState, behavior, currState.steer);

			//currState.speed = 5;
			//cout << currState.speed << endl;
			currState.shift = PlannerHNS::SHIFT_POS_DD;
		}
	}
	else if(behavior.state == PlannerHNS::STOPPING_STATE)
	{
		currState.speed = 0;
		currState.shift = PlannerHNS::SHIFT_POS_DD;
	}
	else
	{
		currState.speed = 0;
		currState.shift = PlannerHNS::SHIFT_POS_NN;
	}

	return currState;
}

} /* namespace SimulationNS */
