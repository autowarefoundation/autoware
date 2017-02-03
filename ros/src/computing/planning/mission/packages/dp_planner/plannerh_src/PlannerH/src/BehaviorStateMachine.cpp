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
			pState->ResetTimer();
			return pState;
		}
	}

	return 0;
}

void BehaviorStateMachine::Init()
{
	UtilityH::GetTickCount(m_StateTimer);
}

void BehaviorStateMachine::ResetTimer()
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

	PreCalculatedConditions* pCParams = GetCalcParams();

	if(pCParams->bGoalReached)
		return FindBehaviorState(GOAL_STATE);

	if(pCParams->distanceToNext <= m_PlanningParams.maxDistanceToAvoid)
		return FindBehaviorState(STOPPING_STATE);

	else if(pCParams->bFullyBlock)
		return FindBehaviorState(FOLLOW_STATE);

	else if(pCParams->distanceToNext > m_PlanningParams.maxDistanceToAvoid
			&& pCParams->distanceToNext <= m_PlanningParams.minDistanceToAvoid
			&& !pCParams->bFullyBlock
			&& pCParams->iCurrSafeTrajectory != pCParams->iPrevSafeTrajectory)
		return FindBehaviorState(OBSTACLE_AVOIDANCE_STATE);

	else if(pCParams->currentTrafficLightID > 0
			&& pCParams->bTrafficIsRed
			&& pCParams->currentTrafficLightID != pCParams->prevTrafficLightID)
		return FindBehaviorState(TRAFFIC_LIGHT_STOP_STATE);

	else if(pCParams->currentStopSignID > 0
			&& pCParams->currentStopSignID != pCParams->prevStopSignID)
			return FindBehaviorState(STOP_SIGN_STOP_STATE);

	else
	{
		if(pCParams->iCurrSafeTrajectory == pCParams->iCentralTrajectory
				&& pCParams->iPrevSafeTrajectory != pCParams->iCurrSafeTrajectory)
			pCParams->bRePlan = true;

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

	if(pCParams->bOutsideControl  == 1)
	{
		pCParams->bOutsideControl = 0;
		pCParams->bRePlan = true;
		return FindBehaviorState(FORWARD_STATE);
	}
	else
		return this;
}

BehaviorStateMachine* TrafficLightStopState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset


	if(!pCParams->bTrafficIsRed)
	{
		pCParams->prevTrafficLightID = pCParams->currentTrafficLightID;
		return FindBehaviorState(FORWARD_STATE);
	}

	else if(pCParams->bTrafficIsRed && pCParams->currentVelocity < ZERO_VELOCITY)
			return FindBehaviorState(TRAFFIC_LIGHT_WAIT_STATE);
	else
		return this;
}

BehaviorStateMachine* TrafficLightWaitState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(!pCParams->bTrafficIsRed)
	{
		pCParams->prevTrafficLightID = pCParams->currentTrafficLightID;
		return FindBehaviorState(FORWARD_STATE);
	}

	else if(pCParams->currentVelocity > ZERO_VELOCITY)
		return FindBehaviorState(TRAFFIC_LIGHT_STOP_STATE);

	else
		return this;

}

BehaviorStateMachine* StopSignStopState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(pCParams->bFullyBlock
			&& m_PlanningParams.enableFollowing)
		return FindBehaviorState(FOLLOW_STATE);

	else if(pCParams->currentVelocity < ZERO_VELOCITY)
		return FindBehaviorState(STOP_SIGN_WAIT_STATE);

	else
		return this;
}

BehaviorStateMachine* StopSignWaitState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) > decisionMakingTime)
	{
		pCParams->prevStopSignID = pCParams->currentStopSignID;
		return FindBehaviorState(FORWARD_STATE);
	}

	else if(pCParams->currentVelocity > ZERO_VELOCITY)
		return FindBehaviorState(STOP_SIGN_STOP_STATE);

	else
		return this;

}

BehaviorStateMachine* WaitState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(pCParams->bOutsideControl  == 1)
	{
		pCParams->bOutsideControl = 0;
		return FindBehaviorState(FORWARD_STATE);
	}
	else
		return this;
}

BehaviorStateMachine* InitState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

	if(pCParams->bOutsideControl == 1)
	{
		pCParams->bOutsideControl = 0;
		return FindBehaviorState(FORWARD_STATE);
	}
	else
		return this;
}

BehaviorStateMachine* FollowState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//			return this; //return this behavior only , without reset

//	if(pCParams->bOutsideControl == 0)
//		return FindBehaviorState(WAITING_STATE);

	if(pCParams->bGoalReached)
		return FindBehaviorState(STOPPING_STATE);

	else if(pCParams->bFullyBlock)
		return FindBehaviorState(this->m_Behavior);

	else if(pCParams->distanceToNext > 0
			&& pCParams->distanceToNext < m_PlanningParams.minDistanceToAvoid
			&& !pCParams->bFullyBlock
			&& pCParams->iCurrSafeTrajectory != pCParams->iPrevSafeTrajectory
			&& m_PlanningParams.enableSwerving)
		return FindBehaviorState(OBSTACLE_AVOIDANCE_STATE);

	else if(pCParams->currentTrafficLightID > 0 && pCParams->bTrafficIsRed && pCParams->currentTrafficLightID != pCParams->prevTrafficLightID)
			return FindBehaviorState(TRAFFIC_LIGHT_STOP_STATE);

	else if(pCParams->currentStopSignID > 0 && pCParams->currentStopSignID != pCParams->prevStopSignID)
			return FindBehaviorState(STOP_SIGN_STOP_STATE);

	else
		return FindBehaviorState(FORWARD_STATE); // return and reset
}

BehaviorStateMachine* SwerveState::GetNextState()
{
//	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
//		return this; //return this behavior only , without reset

//	if(pCParams->bOutsideControl == 0)
//		return FindBehaviorState(WAITING_STATE);

	if(pCParams->bGoalReached)
		return FindBehaviorState(STOPPING_STATE);

	else if(pCParams->bFullyBlock && m_PlanningParams.enableFollowing)
		return FindBehaviorState(FOLLOW_STATE);

	else if(pCParams->distanceToNext > 0
				&& pCParams->distanceToNext < m_PlanningParams.minDistanceToAvoid
				&& !pCParams->bFullyBlock
				&& pCParams->iCurrSafeTrajectory != pCParams->iPrevSafeTrajectory)
		return FindBehaviorState(this->m_Behavior);

	else if(pCParams->currentTrafficLightID > 0 && pCParams->bTrafficIsRed && pCParams->currentTrafficLightID != pCParams->prevTrafficLightID)
			return FindBehaviorState(TRAFFIC_LIGHT_STOP_STATE);

	else
		return FindBehaviorState(FORWARD_STATE);
}


BehaviorStateMachine* GoalState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this; //return this behavior only , without reset

	if(pCParams->bOutsideControl == 1)
	{
		pCParams->bOutsideControl = 0;
		return FindBehaviorState(FORWARD_STATE);
	}
	else
		return this;
}

} /* namespace PlannerHNS */
