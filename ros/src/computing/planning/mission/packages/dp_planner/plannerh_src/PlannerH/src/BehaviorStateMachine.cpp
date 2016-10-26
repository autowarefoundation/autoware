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
	decisionMakingTime		= 0.0;

	if(nextState)
		pNextStates.push_back(nextState);

	pNextStates.push_back(this);

	Init();
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
		if(pState && behavior == pState->m_Behavior )
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
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(GetCalcParams()->bOutsideControl == 0)
		return FindBehaviorState(WAITING_STATE);

	else if(GetCalcParams()->bGoalReached)
		return FindBehaviorState(STOPPING_STATE);

	else if(GetCalcParams()->bFullyBlock
			&& !GetCalcParams()->bCanChangeLane
			&& GetCalcParams()->distanceToNext < m_PlanningParams.minFollowingDistance)
		return FindBehaviorState(FOLLOW_STATE);

	else if(GetCalcParams()->distanceToNext > 0
			&& GetCalcParams()->distanceToNext < m_PlanningParams.minDistanceToAvoid
			&& !GetCalcParams()->bFullyBlock
			&& GetCalcParams()->iCurrSafeTrajectory != GetCalcParams()->iPrevSafeTrajectory)
		return FindBehaviorState(OBSTACLE_AVOIDANCE_STATE);

	else
	{
		if(GetCalcParams()->iCurrSafeTrajectory == GetCalcParams()->iCentralTrajectory
				&& GetCalcParams()->iPrevSafeTrajectory != GetCalcParams()->iCurrSafeTrajectory)
			GetCalcParams()->bRePlan = true;

		return FindBehaviorState(this->m_Behavior); // return and reset
	}
}

BehaviorStateMachine* MissionAccomplishedState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* StopState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	return this;
}

BehaviorStateMachine* WaitState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(GetCalcParams()->bOutsideControl  == 1)
		return FindBehaviorState(FORWARD_STATE);
	else
		return this;
}

BehaviorStateMachine* InitState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(GetCalcParams()->bOutsideControl == 1)
		return FindBehaviorState(FORWARD_STATE);
	else
		return this;
}

BehaviorStateMachine* FollowState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//			return this; //return this behavior only , without reset

	if(GetCalcParams()->bOutsideControl == 0)
		return FindBehaviorState(WAITING_STATE);

	else if(GetCalcParams()->bGoalReached)
		return FindBehaviorState(STOPPING_STATE);

	else if(GetCalcParams()->bFullyBlock
			&& !GetCalcParams()->bCanChangeLane
			&& GetCalcParams()->distanceToNext < m_PlanningParams.maxFollowingDistance)
		return FindBehaviorState(this->m_Behavior);

	else if(GetCalcParams()->distanceToNext > 0
			&& GetCalcParams()->distanceToNext < m_PlanningParams.minDistanceToAvoid
			&& !GetCalcParams()->bFullyBlock
			&& GetCalcParams()->iCurrSafeTrajectory != GetCalcParams()->iPrevSafeTrajectory)
		return FindBehaviorState(OBSTACLE_AVOIDANCE_STATE);

	else
		return FindBehaviorState(FORWARD_STATE); // return and reset
}

BehaviorStateMachine* SwerveState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(GetCalcParams()->bOutsideControl == 0)
		return FindBehaviorState(WAITING_STATE);

	else if(GetCalcParams()->bGoalReached)
		return FindBehaviorState(STOPPING_STATE);

	else if(GetCalcParams()->bFullyBlock
				&& !GetCalcParams()->bCanChangeLane
				&& GetCalcParams()->distanceToNext < m_PlanningParams.minFollowingDistance)
		return FindBehaviorState(FOLLOW_STATE);

	else if(GetCalcParams()->distanceToNext > 0
				&& GetCalcParams()->distanceToNext < m_PlanningParams.minDistanceToAvoid
				&& !GetCalcParams()->bFullyBlock
				&& GetCalcParams()->iCurrSafeTrajectory != GetCalcParams()->iPrevSafeTrajectory)
		return FindBehaviorState(this->m_Behavior);

	else
		return FindBehaviorState(FORWARD_STATE);
}

} /* namespace PlannerHNS */
