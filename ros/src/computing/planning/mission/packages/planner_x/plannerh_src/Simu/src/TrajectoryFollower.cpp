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
	m_iPrevWayPoint 	= 0;

	m_PrevContinousHeading = INFINITY;
	m_PrevContinousTargetHeading = INFINITY;

	m_pidSteer.Init(m_Params.steerPGain, m_Params.steerIGain, m_Params.steerDGain);
	m_pidSteer.Setlimit(m_Params.MaxSteerAngle, -m_Params.MaxSteerAngle);
	m_lowpassSteer.Init(2, 100, 5);
}

TrajectoryFollower::~TrajectoryFollower()
{
}


void TrajectoryFollower::PrepareNextWaypoint(const PlannerHNS::WayPoint& CurPos, const double& currVelocity, const double& currSteering)
{
	WayPoint pred_point = CurPos;

	//m_ForwardSimulation = SimulatePathFollow(0.01, m_Params.SteeringDelay*currVelocity, m_Path, pred_point, currVelocity, m_Params.Wheelbase);

	m_ForwardSimulation = pred_point;
//	PredictMotion(m_ForwardSimulation.pos.x, m_ForwardSimulation.pos.y, m_ForwardSimulation.pos.a, currSteering,currVelocity, m_Params.Wheelbase, m_Params.SteeringDelay*2.0);
	double nIterations = m_Params.SteeringDelay/0.01;
	for(unsigned int i=0; i< nIterations; i++)
	{
		//double prev_a = m_ForwardSimulation.a;
		PredictMotion(m_ForwardSimulation.pos.x, m_ForwardSimulation.pos.y, m_ForwardSimulation.pos.a, currSteering,currVelocity, m_Params.Wheelbase, 0.01);
//		if(abs(m_ForwardSimulation.a - prev_a) > 0.01)
//			prev_a = 0;
	}


//	pred_point.a = m_ForwardSimulation.a;
//	pred_point.cost = m_ForwardSimulation.cost;
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

	follow_distance = m_Params.PursuiteDistance + abs(velocity) * 0.4;
	//follow_distance = m_Params.PursuiteDistance + fabs(velocity) * 0.8;
	if(follow_distance < m_Params.PursuiteDistance)
		follow_distance = m_Params.PursuiteDistance;

	int iWayPoint =  PlanningHelpers::GetClosestNextPointIndex(path, state);
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
	m_PrevDesiredSteer = desiredSteerAngle;

	return ret;
}

int TrajectoryFollower::SteerControllerPart(const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
		const double& lateral_error, double& steerd)
{
	double current_a = state.pos.a;
	double target_a = atan2(way_point.pos.y - state.pos.y, way_point.pos.x - state.pos.x);

	if(!isinf(m_PrevContinousHeading) && !isinf(m_PrevContinousTargetHeading))
	{
		current_a = UtilityH::GetCircularAngle(m_PrevContinousHeading, current_a);
		target_a  = UtilityH::GetCircularAngle(m_PrevContinousTargetHeading, target_a);
	}

	m_PrevContinousHeading = current_a;
	m_PrevContinousTargetHeading = target_a;


	double e =  target_a - current_a;

	if(UtilityH::SplitPositiveAngle(e) > M_PI_2 || UtilityH::SplitPositiveAngle(e) < -M_PI_2)
		return -1;

	//TODO use lateral error instead of angle error
	//double future_lateral_error = PlanningHelpers::GetPerpDistanceToTrajectorySimple(m_Path, m_ForwardSimulation,0)*10.0;

	m_LateralError = 0;

	if(m_LateralError < 0)
		steerd = m_pidSteer.getPID(current_a+sqrt(abs(m_LateralError)), target_a);
	else
		steerd = m_pidSteer.getPID(current_a-sqrt(m_LateralError), target_a);

	//cout << "Error : " << e << ", Current A: " << current_a << ", Target A: " << target_a <<  " Steeting Angle = " << steerd*RAD2DEG << endl;
	steerd = m_lowpassSteer.getFilter(steerd);

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


} /* namespace SimulationNS */
