
/// \file BehaviorStateMachine.h
/// \author Hatem Darweesh
/// \brief OpenPlanner's state machine implementation for different driving behaviors
/// \date Jun 19, 2016


#ifndef BEHAVIORSTATEMACHINE_H_
#define BEHAVIORSTATEMACHINE_H_

#include "RoadNetwork.h"
#include <sstream>

namespace PlannerHNS
{
class BehaviorStateMachine
{
public:
	virtual BehaviorStateMachine* GetNextState() = 0;
	virtual void Init();
	virtual void ResetTimer();
	virtual void InsertNextState(BehaviorStateMachine* nextState);
	BehaviorStateMachine(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* nextState);
	virtual ~BehaviorStateMachine() ;

	STATE_TYPE m_Behavior;
	int m_currentStopSignID	;
	int m_currentTrafficLightID ;
	double decisionMakingTime;
	int decisionMakingCount;
	double m_zero_velocity;

	PreCalculatedConditions* GetCalcParams()
	{
		if(!m_pCalculatedValues)
				m_pCalculatedValues = new PreCalculatedConditions();

		return m_pCalculatedValues;
	}

	void SetBehaviorsParams(PlanningParams* pParams)
	{
		if(!pParams)
			m_pParams = new PlanningParams;
		else
			m_pParams = pParams;
	}


	PreCalculatedConditions* m_pCalculatedValues;
	PlanningParams* m_pParams;
	timespec m_StateTimer;
	std::vector<BehaviorStateMachine*> pNextStates;

	BehaviorStateMachine* FindBehaviorState(const STATE_TYPE& behavior);
	void UpdateLogCount(BehaviorStateMachine* pState);
	BehaviorStateMachine* FindBestState(int nMinCount);

private:
	std::vector<std::pair<BehaviorStateMachine*, int> > m_BehaviorsLog;
};

class ForwardState : public BehaviorStateMachine
{
public:
	ForwardState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = FORWARD_STATE;}
	virtual ~ForwardState(){}
	virtual BehaviorStateMachine* GetNextState();
};

class ForwardStateII : public BehaviorStateMachine
{
public:
	ForwardStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = FORWARD_STATE;}
	virtual ~ForwardStateII(){}
	virtual BehaviorStateMachine* GetNextState();
};

class MissionAccomplishedState : public BehaviorStateMachine
{
public:
	MissionAccomplishedState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = FINISH_STATE;}
	virtual ~MissionAccomplishedState(){}
	virtual BehaviorStateMachine* GetNextState();
};

class MissionAccomplishedStateII : public BehaviorStateMachine
{
public:
	MissionAccomplishedStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = FINISH_STATE;}
	virtual ~MissionAccomplishedStateII(){}
	virtual BehaviorStateMachine* GetNextState();
};

class FollowState : public BehaviorStateMachine
{
public:
	FollowState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = FOLLOW_STATE;}
	virtual ~FollowState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class FollowStateII : public BehaviorStateMachine
{
public:
	FollowStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = FOLLOW_STATE;}
	virtual ~FollowStateII(){}
	virtual BehaviorStateMachine* GetNextState();

};

class SwerveState : public BehaviorStateMachine
{
public:
	SwerveState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = OBSTACLE_AVOIDANCE_STATE;}
	virtual ~SwerveState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class SwerveStateII : public BehaviorStateMachine
{
public:
	SwerveStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = OBSTACLE_AVOIDANCE_STATE;}
	virtual ~SwerveStateII(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopState : public BehaviorStateMachine
{
public:
	StopState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = STOPPING_STATE;}
	virtual BehaviorStateMachine* GetNextState();

};

class TrafficLightStopState : public BehaviorStateMachine
{
public:
	TrafficLightStopState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = TRAFFIC_LIGHT_STOP_STATE;}
	virtual ~TrafficLightStopState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class TrafficLightWaitState : public BehaviorStateMachine
{
public:
	TrafficLightWaitState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = TRAFFIC_LIGHT_WAIT_STATE;}
	virtual ~TrafficLightWaitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopSignStopState : public BehaviorStateMachine
{
public:
	StopSignStopState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = STOP_SIGN_STOP_STATE;}
	virtual ~StopSignStopState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopSignStopStateII : public BehaviorStateMachine
{
public:
	StopSignStopStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = STOP_SIGN_STOP_STATE;}
	virtual ~StopSignStopStateII(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopSignWaitState : public BehaviorStateMachine
{
public:
	StopSignWaitState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = STOP_SIGN_WAIT_STATE;}
	virtual ~StopSignWaitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class StopSignWaitStateII : public BehaviorStateMachine
{
public:
	StopSignWaitStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = STOP_SIGN_WAIT_STATE;}
	virtual ~StopSignWaitStateII(){}
	virtual BehaviorStateMachine* GetNextState();

};

class WaitState : public BehaviorStateMachine
{
public:
	WaitState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = WAITING_STATE;}
	virtual ~WaitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class InitState : public BehaviorStateMachine
{
public:
	InitState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = INITIAL_STATE;}
	virtual ~InitState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class InitStateII : public BehaviorStateMachine
{
public:
	InitStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = INITIAL_STATE;}
	virtual ~InitStateII(){}
	virtual BehaviorStateMachine* GetNextState();
};

class GoalState : public BehaviorStateMachine
{
public:
	GoalState(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = GOAL_STATE;}
	virtual ~GoalState(){}
	virtual BehaviorStateMachine* GetNextState();

};

class GoalStateII : public BehaviorStateMachine
{
public:
	GoalStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = GOAL_STATE;}
	virtual ~GoalStateII(){}
	virtual BehaviorStateMachine* GetNextState();

};

class TrafficLightStopStateII : public BehaviorStateMachine
{
public:
	TrafficLightStopStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = TRAFFIC_LIGHT_STOP_STATE;}
	virtual ~TrafficLightStopStateII(){}
	virtual BehaviorStateMachine* GetNextState();
};

class TrafficLightWaitStateII : public BehaviorStateMachine
{
public:
	TrafficLightWaitStateII(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pParams, pPreCalcVal, pNextState){m_Behavior = TRAFFIC_LIGHT_WAIT_STATE;}
	virtual ~TrafficLightWaitStateII(){}
	virtual BehaviorStateMachine* GetNextState();
};

} /* namespace PlannerHNS */

#endif /* BEHAVIORSTATEMACHINE_H_ */
