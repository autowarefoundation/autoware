/*
 * BehaviorStateMachine.cpp
 *
 *  Created on: Jun 19, 2016
 *      Author: hatem
 */

#include "BehaviorStateMachine.h"
#include "UtilityH.h"
#include <iostream>

using namespace UtilityHNS;


namespace PlannerHNS {

//PreCalculatedConditions* BehaviorStateMachine::m_pCalculatedValues = 0;
//PlanningParams BehaviorStateMachine::m_PlanningParams;

BehaviorStateMachine::BehaviorStateMachine(PlanningParams* pParams, PreCalculatedConditions* pPreCalcVal, BehaviorStateMachine* nextState)
{
	m_Behavior = INITIAL_STATE;

	m_currentStopSignID		= -1;
	m_currentTrafficLightID	= -1;
	decisionMakingTime		= 0.0;
	m_zero_velocity 		= 0.2;

	if(!pPreCalcVal)
		m_pCalculatedValues = new PreCalculatedConditions();
	else
		m_pCalculatedValues = pPreCalcVal;

	if(!pParams)
		m_pParams = new PlanningParams;
	else
		m_pParams = pParams;

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

//	if(pCParams->currentGoalID != pCParams->prevGoalID)
//		return FindBehaviorState(GOAL_STATE);

	if(m_pParams->enableSwerving
			&& pCParams->distanceToNext <= m_pParams->minDistanceToAvoid
			&& !pCParams->bFullyBlock
			&& (pCParams->iCurrSafeTrajectory != pCParams->iPrevSafeTrajectory || pCParams->iCurrSafeLane != pCParams->iPrevSafeLane)
			)
		return FindBehaviorState(OBSTACLE_AVOIDANCE_STATE);

	else if(m_pParams->enableTrafficLightBehavior
			&& pCParams->currentTrafficLightID > 0
			&& pCParams->bTrafficIsRed
			&& pCParams->currentTrafficLightID != pCParams->prevTrafficLightID)
		return FindBehaviorState(TRAFFIC_LIGHT_STOP_STATE);

	else if(m_pParams->enableStopSignBehavior
			&& pCParams->currentStopSignID > 0
			&& pCParams->currentStopSignID != pCParams->prevStopSignID)
			return FindBehaviorState(STOP_SIGN_STOP_STATE);

	else if(m_pParams->enableFollowing
			&& pCParams->bFullyBlock)
			return FindBehaviorState(FOLLOW_STATE);

//	else if(pCParams->distanceToNext <= m_pParams->maxDistanceToAvoid)
//		return FindBehaviorState(STOPPING_STATE);

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
	return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* StopState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	if(pCParams->distanceToNext > m_pParams->maxDistanceToAvoid)
		return FindBehaviorState(FORWARD_STATE);

	else
		return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* TrafficLightStopState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	if(!pCParams->bTrafficIsRed)
	{
		pCParams->prevTrafficLightID = pCParams->currentTrafficLightID;
		return FindBehaviorState(FORWARD_STATE);
	}

	else if(pCParams->bTrafficIsRed && pCParams->currentVelocity <= m_zero_velocity)
			return FindBehaviorState(TRAFFIC_LIGHT_WAIT_STATE);
	else
		return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* TrafficLightWaitState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	if(!pCParams->bTrafficIsRed)
	{
		pCParams->prevTrafficLightID = pCParams->currentTrafficLightID;
		return FindBehaviorState(FORWARD_STATE);
	}

	else if(pCParams->currentVelocity > m_zero_velocity)
		return FindBehaviorState(TRAFFIC_LIGHT_STOP_STATE);

	else
		return FindBehaviorState(this->m_Behavior); // return and reset

}

BehaviorStateMachine* StopSignStopState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	//std::cout << "From Stop Beh D: " << pCParams->distanceToStop() << ", Prev LineID: " << pCParams->prevStopSignID << ", Curr SignID: " << pCParams->currentStopSignID << std::endl;

	if(pCParams->currentVelocity < m_zero_velocity)
		return FindBehaviorState(STOP_SIGN_WAIT_STATE);

	else
		return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* StopSignWaitState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	//std::cout << "From Wait Beh D: " << pCParams->distanceToStop() << ", Prev LineID: " << pCParams->prevStopSignID << ", Curr SignID: " << pCParams->currentStopSignID << std::endl;

	pCParams->prevStopSignID = pCParams->currentStopSignID;

	return FindBehaviorState(FORWARD_STATE);
}

BehaviorStateMachine* WaitState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	return FindBehaviorState(FORWARD_STATE);
}

BehaviorStateMachine* InitState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	if(pCParams->bOutsideControl == 1)
	{
		pCParams->prevGoalID = pCParams->currentGoalID;
		return FindBehaviorState(FORWARD_STATE);
	}

	else
		return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* FollowState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
			return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	//std::cout << "Following State >> followDistance: " << pCParams->distanceToNext << ", followSpeed: " << pCParams->velocityOfNext << std::endl;

	if(pCParams->currentGoalID != pCParams->prevGoalID
			|| !pCParams->bFullyBlock)
		return FindBehaviorState(FORWARD_STATE);

	else
		return FindBehaviorState(this->m_Behavior); // return and reset
}

BehaviorStateMachine* SwerveState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	if(pCParams->distanceToNext > 0
				&& pCParams->distanceToNext < m_pParams->minDistanceToAvoid
				&& !pCParams->bFullyBlock
				&& pCParams->iCurrSafeTrajectory != pCParams->iPrevSafeTrajectory)
		return FindBehaviorState(this->m_Behavior);

	else
		return FindBehaviorState(FORWARD_STATE);
}

BehaviorStateMachine* GoalState::GetNextState()
{
	if(UtilityH::GetTimeDiffNow(m_StateTimer) < decisionMakingTime)
		return this;

	PreCalculatedConditions* pCParams = GetCalcParams();

	if(pCParams->currentGoalID == -1)
		return FindBehaviorState(FINISH_STATE);

	else if(pCParams->currentGoalID != pCParams->prevGoalID)
	{
		pCParams->prevGoalID = pCParams->currentGoalID;
		return FindBehaviorState(FORWARD_STATE);
	}

	else
		return FindBehaviorState(this->m_Behavior); // return and reset
}

} /* namespace PlannerHNS */
