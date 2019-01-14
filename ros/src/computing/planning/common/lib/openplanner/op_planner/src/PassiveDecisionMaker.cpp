
/// \file  PassiveDecisionMaker.cpp
/// \brief Decision Maker for surrounding vehicle, to be integrated with particle filter for intention prediction
/// \author Hatem Darweesh
/// \date Jan 10, 2018

#include "op_planner/PassiveDecisionMaker.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"


namespace PlannerHNS
{

PassiveDecisionMaker::PassiveDecisionMaker()
{
}

PassiveDecisionMaker& PassiveDecisionMaker::operator=(const PassiveDecisionMaker& obj)
{
	return *this;
}

PassiveDecisionMaker::PassiveDecisionMaker(const PassiveDecisionMaker& obj)
{
}

PassiveDecisionMaker::~PassiveDecisionMaker()
{
}

 double PassiveDecisionMaker::GetVelocity(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo, const RelativeInfo& info)
 {
	double average_braking_distance = -pow(currPose.v, 2)/(carInfo.max_deceleration) + 1.0;
	int prev_index = 0;
	double velocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(path, info, prev_index, average_braking_distance);
	if(velocity > carInfo.max_speed_forward)
		velocity = carInfo.max_speed_forward;
	else if(velocity < carInfo.min_speed_forward)
		velocity = carInfo.min_speed_forward;

	return velocity;
 }

 double PassiveDecisionMaker::GetSteerAngle(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const RelativeInfo& info)
  {
     unsigned int point_index = 0;
     PlannerHNS::WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(path, info, 2, point_index);

     double current_a = UtilityHNS::UtilityH::SplitPositiveAngle(currPose.pos.a);
     double target_a = atan2(pursuite_point.pos.y - currPose.pos.y, pursuite_point.pos.x - currPose.pos.x);
     double e =  UtilityHNS::UtilityH::SplitPositiveAngle(target_a - current_a);
     double before_lowpass = e;//m_pidSteer.getPID(e);
     //std::cout << "CurrA: " << current_a << ", targetA: " << target_a << ", e: " << e << std::endl;
     return before_lowpass;

  }

 bool PassiveDecisionMaker::CheckForStopLine(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
 {
	 double minStoppingDistance = -pow(currPose.v, 2)/(carInfo.max_deceleration);
	 double critical_long_front_distance =  carInfo.wheel_base/2.0 + carInfo.length/2.0;

	int stopLineID = -1;
	int stopSignID = -1;
	int trafficLightID = -1;
	double distanceToClosestStopLine = 0;

	distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(path, currPose, 0, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

	if(distanceToClosestStopLine > -2 && distanceToClosestStopLine < minStoppingDistance)
	{
		return true;
	}

	return false;
 }

 PlannerHNS::BehaviorState PassiveDecisionMaker::MoveStep(const double& dt, PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
 {
	 PlannerHNS::BehaviorState beh;
	 if(path.size() == 0) return beh;

	 RelativeInfo info;
	 PlanningHelpers::GetRelativeInfo(path, currPose, info);

	 bool bStopLine = CheckForStopLine(currPose, path, carInfo);
	 if(bStopLine)
		 beh.state = PlannerHNS::STOPPING_STATE;
	 else
		 beh.state = PlannerHNS::FORWARD_STATE;

	 double average_braking_distance = -pow(currPose.v, 2)/(carInfo.max_deceleration) + 1.0;

	if(average_braking_distance  < 10)
		average_braking_distance = 10;

	beh.indicator = PlanningHelpers::GetIndicatorsFromPath(path, currPose, average_braking_distance);

	currPose.v = beh.maxVelocity = GetVelocity(currPose, path, carInfo, info);

	double steer = GetSteerAngle(currPose, path, info);

	currPose.pos.x += currPose.v * dt * cos(currPose.pos.a);
	currPose.pos.y += currPose.v * dt * sin(currPose.pos.a);
	currPose.pos.a += currPose.v * dt * tan(steer)  / carInfo.wheel_base;

	return beh;

 }

 PlannerHNS::ParticleInfo PassiveDecisionMaker::MoveStepSimple(const double& dt, PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
  {
 	 PlannerHNS::ParticleInfo beh;
 	 if(path.size() == 0) return beh;

 	 RelativeInfo info;
 	 PlanningHelpers::GetRelativeInfo(path, currPose, info);

 	 bool bStopLine = CheckForStopLine(currPose, path, carInfo);
 	 if(bStopLine)
 		 beh.state = PlannerHNS::STOPPING_STATE;
 	 else
 		 beh.state = PlannerHNS::FORWARD_STATE;

 	 double average_braking_distance = -pow(currPose.v, 2)/(carInfo.max_deceleration) + 15.0;


 	PlannerHNS::WayPoint startPose = path.at(0);
 	beh.indicator = PlanningHelpers::GetIndicatorsFromPath(path, startPose, average_braking_distance);


 	double speed = 0;
 	if(info.iFront < path.size())
 	{
 		beh.vel = path.at(info.iFront).v;
 	}
 	else
 		beh.vel = 0;

 	double steer = GetSteerAngle(currPose, path, info);

 	currPose.pos.x += currPose.v * dt * cos(currPose.pos.a);
 	currPose.pos.y += currPose.v * dt * sin(currPose.pos.a);
 	currPose.pos.a += currPose.v * dt * tan(steer)  / carInfo.wheel_base;

 	return beh;

  }

} /* namespace PlannerHNS */
