/*
 * CarState.h
 *
 *  Created on: Dec 14, 2016
 *      Author: hatem
 */

#ifndef PASSIVE_BEHAVIOR_DECISION_MAKER
#define PASSIVE_BEHAVIOR_DECISION_MAKER

#include "BehaviorStateMachine.h"
#include "PlannerCommonDef.h"
#include "RoadNetwork.h"

namespace PlannerHNS
{

class PassiveDecisionMaker
{
public:
	PassiveDecisionMaker();
	PassiveDecisionMaker(const PassiveDecisionMaker& obj);
	PassiveDecisionMaker& operator=(const PassiveDecisionMaker& obj);
	virtual ~PassiveDecisionMaker();
	PlannerHNS::BehaviorState MoveStep(const double& dt, PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo);
	PlannerHNS::ParticleInfo MoveStepSimple(const double& dt, PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo);

private:
	double GetVelocity(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo, const RelativeInfo& info);
	double GetSteerAngle(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const RelativeInfo& info);
	bool CheckForStopLine(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo);

};

} /* namespace PlannerHNS */

#endif /* PASSIVE_BEHAVIOR_DECISION_MAKER */
