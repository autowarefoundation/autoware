/*
 * BehaviorStateMachine.h
 *
 *  Created on: Jun 19, 2016
 *      Author: hatem
 */

#ifndef BEHAVIORSTATEMACHINE_H_
#define BEHAVIORSTATEMACHINE_H_

#include "RoadNetwork.h"
#include <sstream>

namespace PlannerHNS
{

#define ZERO_VELOCITY 0.1


class BehaviorStateMachine
{
public:
	virtual BehaviorStateMachine* GetNextState() = 0;
	virtual void Init();
	virtual void ResetTimer();
	virtual void InsertNextState(BehaviorStateMachine* nextState);
	BehaviorStateMachine(BehaviorStateMachine* nextState);
	virtual ~BehaviorStateMachine() ;

	STATE_TYPE m_Behavior;
	int m_currentStopSignID	;
	int m_currentTrafficLightID ;
	double decisionMakingTime;


	PreCalculatedConditions* GetCalcParams()
	{
		if(!m_pCalculatedValues)
				m_pCalculatedValues = new PreCalculatedConditions();

		return m_pCalculatedValues;
	}

	void SetBehaviorsParams(const PlanningParams& params)
	{
		m_PlanningParams = params;
	}


	static PreCalculatedConditions* m_pCalculatedValues;
	timespec m_StateTimer;
	std::vector<BehaviorStateMachine*> pNextStates;
	static PlanningParams m_PlanningParams;

	BehaviorStateMachine* FindBehaviorState(const STATE_TYPE& behavior);
};

class ForwardState : public BehaviorStateMachine
{
public:
	ForwardState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = FORWARD_STATE;}
	virtual ~ForwardState(){}
	virtual BehaviorStateMachine* GetNextState();
};

class MissionAccomplishedState : public BehaviorStateMachine
{
public:
	MissionAccomplishedState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = FINISH_STATE;}
	virtual ~MissionAccomplishedState(){}
	virtual BehaviorStateMachine* GetNextState();
};

class FollowState : public BehaviorStateMachine
{
public:
	FollowState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = FOLLOW_STATE;}
	virtual ~FollowState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class SwerveState : public BehaviorStateMachine
{
public:
	SwerveState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = OBSTACLE_AVOIDANCE_STATE;}
	virtual ~SwerveState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopState : public BehaviorStateMachine
{
public:
	StopState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = STOPPING_STATE;}
	virtual ~StopState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class TrafficLightStopState : public BehaviorStateMachine
{
public:
	TrafficLightStopState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = TRAFFIC_LIGHT_STOP_STATE;}
	virtual ~TrafficLightStopState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class TrafficLightWaitState : public BehaviorStateMachine
{
public:
	TrafficLightWaitState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = TRAFFIC_LIGHT_WAIT_STATE;}
	virtual ~TrafficLightWaitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopSignStopState : public BehaviorStateMachine
{
public:
	StopSignStopState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = STOP_SIGN_STOP_STATE;}
	virtual ~StopSignStopState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopSignWaitState : public BehaviorStateMachine
{
public:
	StopSignWaitState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = STOP_SIGN_WAIT_STATE;}
	virtual ~StopSignWaitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class WaitState : public BehaviorStateMachine
{
public:
	WaitState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = WAITING_STATE;}
	virtual ~WaitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class InitState : public BehaviorStateMachine
{
public:
	InitState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = INITIAL_STATE;}
	virtual ~InitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class GoalState : public BehaviorStateMachine
{
public:
	GoalState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = GOAL_STATE;}
	virtual ~GoalState(){}
	virtual BehaviorStateMachine* GetNextState();

};

} /* namespace PlannerHNS */

#endif /* BEHAVIORSTATEMACHINE_H_ */
