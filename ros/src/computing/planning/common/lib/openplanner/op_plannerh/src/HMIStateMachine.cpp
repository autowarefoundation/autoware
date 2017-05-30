/*
 * BehaviorStateMachine.cpp
 *
 *  Created on: Jun 19, 2016
 *      Author: hatem
 */

#include "HMIStateMachine.h"
#include "UtilityH.h"

using namespace UtilityHNS;

namespace PlannerHNS {

HMIPreCalculatedConditions* HMIStateMachine::m_pCalculatedValues = 0;
PlanningParams HMIStateMachine::m_PlanningParams;

HMIStateMachine::HMIStateMachine(HMIStateMachine* nextState)
{
	m_Behavior = G_WAITING_STATE;

	decisionMakingTime		= 0.0;

	if(nextState)
		pNextStates.push_back(nextState);

	pNextStates.push_back(this);

	Init();
}

void HMIStateMachine::InsertNextState(HMIStateMachine* nextState)
{
	if(nextState)
		pNextStates.push_back(nextState);
}

HMIStateMachine* HMIStateMachine::FindBehaviorState(const GLOBAL_STATE_TYPE& behavior)
{
	for(unsigned int i = 0 ; i < pNextStates.size(); i++)
	{
		HMIStateMachine* pState = pNextStates.at(i);
		if(pState && behavior == pState->m_Behavior )
		{
			pState->ResetTimer();
			return pState;
		}
	}

	return 0;
}

void HMIStateMachine::Init()
{
	UtilityH::GetTickCount(m_StateTimer);
}

void HMIStateMachine::ResetTimer()
{
	UtilityH::GetTickCount(m_StateTimer);
}

HMIStateMachine::~HMIStateMachine()
{
}

HMIStateMachine* GPlanningState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	HMIPreCalculatedConditions* pCParams = GetCalcParams();

	return FindBehaviorState(this->m_Behavior); // return and reset
}

HMIStateMachine* GWaitingState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	return FindBehaviorState(this->m_Behavior); // return and reset
}

HMIStateMachine* GForwardState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	return FindBehaviorState(this->m_Behavior); // return and reset
}

HMIStateMachine* GBranchingState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	return FindBehaviorState(this->m_Behavior); // return and reset
}

HMIStateMachine* GEndState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	return FindBehaviorState(this->m_Behavior); // return and reset
}

} /* namespace PlannerHNS */
