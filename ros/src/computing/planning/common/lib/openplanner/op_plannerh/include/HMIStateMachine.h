/*
 * HMIStateMachine.h
 *
 *  Created on: February 14, 2017
 *      Author: Hatem Darweesh
 */

#ifndef HMIStateMachine_H_
#define HMIStateMachine_H_

#include "RoadNetwork.h"
#include <sstream>

namespace PlannerHNS
{

class HMIStateMachine
{
public:
	virtual HMIStateMachine* GetNextState() = 0;
	virtual void Init();
	virtual void ResetTimer();
	virtual void InsertNextState(HMIStateMachine* nextState);
	HMIStateMachine(HMIStateMachine* nextState);
	virtual ~HMIStateMachine() ;

	GLOBAL_STATE_TYPE m_Behavior;
	double decisionMakingTime;

	HMIPreCalculatedConditions* GetCalcParams()
	{
		if(!m_pCalculatedValues)
				m_pCalculatedValues = new HMIPreCalculatedConditions();

		return m_pCalculatedValues;
	}

	void SetBehaviorsParams(const PlanningParams& params)
	{
		m_PlanningParams = params;
	}

	static HMIPreCalculatedConditions* m_pCalculatedValues;
	timespec m_StateTimer;
	std::vector<HMIStateMachine*> pNextStates;
	static PlanningParams m_PlanningParams;

	HMIStateMachine* FindBehaviorState(const GLOBAL_STATE_TYPE& behavior);
};

class GWaitingState : public HMIStateMachine
{
public:
	GWaitingState(GWaitingState* pNextState)
	: HMIStateMachine(pNextState){m_Behavior = G_WAITING_STATE;}
	virtual ~GWaitingState(){}
	virtual HMIStateMachine* GetNextState();
};

class GPlanningState : public HMIStateMachine
{
public:
	GPlanningState(HMIStateMachine* pNextState)
	: HMIStateMachine(pNextState){m_Behavior = G_PLANING_STATE;}
	virtual ~GPlanningState(){}
	virtual HMIStateMachine* GetNextState();
};

class GForwardState : public HMIStateMachine
{
public:
	GForwardState(HMIStateMachine* pNextState)
	: HMIStateMachine(pNextState){m_Behavior = G_FORWARD_STATE;}
	virtual ~GForwardState(){}
	virtual HMIStateMachine* GetNextState();
};

class GBranchingState : public HMIStateMachine
{
public:
	GBranchingState(HMIStateMachine* pNextState)
	: HMIStateMachine(pNextState){m_Behavior = G_BRANCHING_STATE;}
	virtual ~GBranchingState(){}
	virtual HMIStateMachine* GetNextState();
};

class GEndState : public HMIStateMachine
{
public:
	GEndState(HMIStateMachine* pNextState)
	: HMIStateMachine(pNextState){m_Behavior = G_FINISH_STATE;}
	virtual ~GEndState(){}
	virtual HMIStateMachine* GetNextState();
};


} /* namespace PlannerHNS */

#endif /* HMIStateMachine_H_ */
