/*
 * BehaviorStateMachine.cpp
 *
 *  Created on: Jun 19, 2016
 *      Author: hatem
 */

#include "BehaviorStateMachine.h"
#include "UtilityH.h"

using namespace UtilityHNS;

namespace PlannerHNS {

PreCalculatedConditions* BehaviorStateMachine::m_pCalculatedValues = 0;
PlanningParams BehaviorStateMachine::m_PlanningParams;

BehaviorStateMachine::BehaviorStateMachine(BehaviorStateMachine* nextState)
{
	m_Behavior = INITIAL_STATE;

	m_currentStopSignID		= -1;
	m_currentTrafficLightID	= -1;
	decisionMakingTime		= 0.01;

	if(nextState)
		pNextStates.push_back(nextState);

	pNextStates.push_back(this);
}

void BehaviorStateMachine::InsertNextState(BehaviorStateMachine* nextState)
{
	if(nextState)
		pNextStates.push_back(nextState);
}

BehaviorStateMachine* BehaviorStateMachine::FindBehaviorState(const STATE_TYPE& behavior)
{
	for(unsigned int i = 0 ; i < pNextStates.size(); i++)
	{
		BehaviorStateMachine* pState = pNextStates.at(i);
		if(pState && behavior == pState->m_Behavior)
		{
			pState->Reset();
			return pState;
		}
	}

	return 0;
}

void BehaviorStateMachine::Init()
{
	UtilityH::GetTickCount(m_StateTimer);
}

void BehaviorStateMachine::Reset()
{
	UtilityH::GetTickCount(m_StateTimer);
}

BehaviorStateMachine::~BehaviorStateMachine()
{
}

BehaviorStateMachine* ForwardState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	if(GetCalcParams()->bOutsideControl == 0)
		return FindBehaviorState(WAITING_STATE);

	else if(GetCalcParams()->distanceToStop() > 0 && GetCalcParams()->distanceToStop() < GetCalcParams()->minStoppingDistance && GetCalcParams()->currentStopSignID > -1)
		return FindBehaviorState(STOP_SIGN_STOP_STATE);

	else if(GetCalcParams()->bTrafficIsRed && GetCalcParams()->distanceToStop() > 0 && GetCalcParams()->distanceToStop() < GetCalcParams()->minStoppingDistance && GetCalcParams()->currentTrafficLightID > -1)
		return FindBehaviorState(TRAFFIC_LIGHT_STOP_STATE);

	else if(GetCalcParams()->bFullyBlock && !GetCalcParams()->bCanChangeLane && GetCalcParams()->distanceToNext < m_PlanningParams.minFollowingDistance)
		return FindBehaviorState(FOLLOW_STATE);

	else if(GetCalcParams()->bGoalReached)
		return FindBehaviorState(FINISH_STATE);

	else if(GetCalcParams()->distanceToNext > 0 && GetCalcParams()->distanceToNext < m_PlanningParams.minDistanceToAvoid && !GetCalcParams()->bFullyBlock)
	{
		GetCalcParams()->bRePlan = true;
		return FindBehaviorState(OBSTACLE_AVOIDANCE_STATE);
	}
	else
		return FindBehaviorState(this->m_Behavior); // return and reset
}


BehaviorStateMachine* MissionAccomplishedState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* StopState::GetNextState()
{
	return this;
}

BehaviorStateMachine* WaitState::GetNextState()
{
	if(GetCalcParams()->bOutsideControl  == 1)
		return FindBehaviorState(FORWARD_STATE);
	else
		return this;
}

BehaviorStateMachine* InitState::GetNextState()
{
	if(GetCalcParams()->bOutsideControl == 1)
		return FindBehaviorState(FORWARD_STATE);
	else
		return this;
}

} /* namespace PlannerHNS */
