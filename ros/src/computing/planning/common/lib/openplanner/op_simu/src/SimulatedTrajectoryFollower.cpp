/*
 * SimulatedTrajectoryFollower.cpp
 *
 *  Created on: Jun 18, 2016
 *      Author: hatem
 */

#include "op_simu/SimulatedTrajectoryFollower.h"
#include "op_planner/PlanningHelpers.h"
#include <cmath>
#include <stdlib.h>
#include <iostream>

using namespace PlannerHNS;
using namespace UtilityHNS;
using namespace std;


namespace SimulationNS
{

SimulatedTrajectoryFollower::SimulatedTrajectoryFollower()
{
	m_FollowingDistance = 0;
	m_LateralError 		= 0;
	m_PrevDesiredSteer	= 0;
	m_FollowAcceleration= 0;
	m_iPrevWayPoint 	= -1;
	m_iCalculatedIndex = 0;


	//m_pidSteer.Init(0.35, 0.01, 0.01); // for 5 m/s
	//m_pidSteer.Init(1.5, 0.00, 0.00); // for 3 m/s
	//m_pidSteer.Init(0.9, 0.1, 0.2); //for lateral error

}

void SimulatedTrajectoryFollower::Init(const ControllerParams& params, const CAR_BASIC_INFO& vehicleInfo)
{
	m_Params = params;
	m_VehicleInfo = vehicleInfo;
	m_pidSteer.Init(params.Steering_Gain.kP, params.Steering_Gain.kI, params.Steering_Gain.kD); // for 3 m/s
	m_pidSteer.Setlimit(vehicleInfo.max_steer_angle, -vehicleInfo.max_steer_angle);
	m_pidVelocity.Init(params.Velocity_Gain.kP, params.Velocity_Gain.kI, params.Velocity_Gain.kD);
}

SimulatedTrajectoryFollower::~SimulatedTrajectoryFollower()
{
}


void SimulatedTrajectoryFollower::PrepareNextWaypoint(const PlannerHNS::WayPoint& CurPos, const double& currVelocity, const double& currSteering)
{
	m_CurrPos = CurPos;
	FindNextWayPoint(m_Path, m_CurrPos, currVelocity, m_FollowMePoint, m_PerpendicularPoint, m_LateralError, m_FollowingDistance);
}

void SimulatedTrajectoryFollower::UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path)
{
	m_Path = path;
}

bool SimulatedTrajectoryFollower::FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
		const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
		double& lateral_err, double& follow_distance)
{
	if(path.size()==0) return false;

	follow_distance = m_Params.minPursuiteDistance;

//	int iWayPoint =  PlanningHelpers::GetClosestNextPointIndex(path, state);
//	m_iCalculatedIndex = iWayPoint;
////	if(m_iPrevWayPoint >=0  && m_iPrevWayPoint < path.size() && iWayPoint < m_iPrevWayPoint)
////		iWayPoint = m_iPrevWayPoint;
//
//
//	m_iPrevWayPoint = iWayPoint;

	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, state, info);
	unsigned int point_index = 0;
	pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(path, info, follow_distance, point_index);
	prep = info.perp_point;
	lateral_err = info.perp_distance;
	m_iPrevWayPoint = info.iFront;

	return true;
}

int SimulatedTrajectoryFollower::SteerControllerUpdate(const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredSteerAngle)
{
	if(m_Path.size()==0) return -1;

	//AdjustPID(CurrStatus.velocity, 18.0, m_Params.Gain);
	int ret = SteerControllerPart(m_CurrPos, m_FollowMePoint, m_LateralError, desiredSteerAngle);
	if(ret < 0)
		desiredSteerAngle = m_PrevDesiredSteer;
	else
		m_PrevDesiredSteer = desiredSteerAngle;

	return ret;
}

int SimulatedTrajectoryFollower::SteerControllerPart(const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
		const double& lateral_error, double& steerd)
{
	double current_a = UtilityH::SplitPositiveAngle(state.pos.a);
	double target_a = atan2(way_point.pos.y - state.pos.y, way_point.pos.x - state.pos.x);

	double e =  UtilityH::SplitPositiveAngle(target_a - current_a);

	if(e > M_PI_2 || e < -M_PI_2)
		return -1;

	steerd = m_pidSteer.getPID(e);

	return 1;
}

int SimulatedTrajectoryFollower::VeclocityControllerUpdate(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredVelocity)
{
	//desiredVelocity = CurrBehavior.maxVelocity;
	m_pidVelocity.Setlimit(CurrBehavior.maxVelocity, 0);
	double e = CurrBehavior.maxVelocity - CurrStatus.speed;

	desiredVelocity = m_pidVelocity.getPID(e);

	if(desiredVelocity > CurrBehavior.maxVelocity)
		desiredVelocity = CurrBehavior.maxVelocity;
	else if (desiredVelocity < 0)
		desiredVelocity = 0;
	return 1;
}


PlannerHNS::VehicleState SimulatedTrajectoryFollower::DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
		const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
		const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory)
{
	if(bNewTrajectory && path.size() > 0)
	{
		m_iPrevWayPoint = -1;
		UpdateCurrentPath(path);
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
