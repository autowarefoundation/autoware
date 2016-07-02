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

class PlanningParams
{
public:
	double 	maxSpeed;
	double 	minSpeed;
	double 	planningDistance;
	double 	microPlanDistance;
	double 	carTipMargin;
	double 	rollInMargin;
	double 	rollInSpeedFactor;
	double 	pathDensity;
	double 	rollOutDensity;
	int 	rollOutNumber;
	double 	marginDistanceFromTrajectory;
	double 	horizonDistance;
	double 	minFollowingDistance;
	double 	maxFollowingDistance;
	double 	minDistanceToAvoid;
	double 	speedProfileFactor;
	int 	curvatureCalculationPoints;
	double 	smoothingDataWeight;
	double 	smoothingSmoothWeight;
	double 	smoothingToleranceError;

	bool 	enableSwerving;
	bool 	enableFollowing;
	bool 	enablePlanningAnywhere;
	bool 	enableHeadingSmoothing;
	bool 	enableWaitingBehavior;
	bool 	enableTrafficLightBehavior;

	PlanningParams()
	{
		maxSpeed 						= 15;
		minSpeed 						= 1;
		planningDistance 				= 10000;
		microPlanDistance 				= 250;
		carTipMargin					= 8.0;
		rollInMargin					= 20.0;
		rollInSpeedFactor				= 0.25;
		pathDensity						= 0.5;
		rollOutDensity					= 0.7;
		rollOutNumber					= 6;
		marginDistanceFromTrajectory	= 2.0;
		horizonDistance					= 500;
		minFollowingDistance			= 35;
		maxFollowingDistance			= 40;
		minDistanceToAvoid				= 10;
		speedProfileFactor				= 25.0;
		curvatureCalculationPoints		= 1;
		smoothingDataWeight				= 0.35;
		smoothingSmoothWeight			= 0.3;
		smoothingToleranceError			= 0.1;

		enableSwerving 					= false;
		enableFollowing					= true;
		enablePlanningAnywhere			= false;
		enableHeadingSmoothing			= false;
		enableWaitingBehavior			= false;
		enableTrafficLightBehavior		= false;
	}
};


class PreCalculatedConditions
{
public:
	//-------------------------------------------//
	//Following
	double 				distanceToNext;
	double				velocityOfNext;
	//-------------------------------------------//
	//For Lane Change
	double				distanceToGoBack;
	double 				timeToGoBack;
	double 				distanceToChangeLane;
	double				timeToChangeLane;
	int 				currentLaneID;
	int 				originalLaneID;
	int 				targetLaneID;
	bool 				bUpcomingLeft;
	bool 				bUpcomingRight;
	bool				bCanChangeLane;
	bool				bTargetLaneSafe;
	//-------------------------------------------//
	//Traffic Lights
	int 				currentStopSignID;
	int 				currentTrafficLightID;
	bool 				bTrafficIsRed; //On , off status
	//-------------------------------------------//
	//Swerving
	int 				iCurrSafeTrajectory;
	int 				iCentralTrajectory;
	bool 				bRePlan;
	bool				bFullyBlock;
	LIGHT_INDICATOR 	indicator;

	//-------------------------------------------//
	//General
	double 				currentVelocity;
	double				minStoppingDistance; //comfortably
	bool 				bGoalReached;
	int 				bOutsideControl; // 0 waiting, 1 start, 2 Stop, 3 Green Traffic Light, 4 Red Traffic Light
	bool				bGreenOutsideControl;
	std::vector<double> stoppingDistances;


	double distanceToStop()
	{
		if(stoppingDistances.size()==0) return 0;
		double minS = stoppingDistances.at(0);
		for(unsigned int i=0; i< stoppingDistances.size(); i++)
		{
			if(stoppingDistances.at(i) < minS)
				minS = stoppingDistances.at(i);
		}
		return minS;
	}

	PreCalculatedConditions()
	{

		currentVelocity 		= 0;
		minStoppingDistance		= 1;
		bOutsideControl			= 0;
		bGreenOutsideControl	= false;
		//distance to stop
		distanceToNext			= 0;
		velocityOfNext			= 0;
		currentTrafficLightID	= -1;
		bTrafficIsRed			= false;
		iCurrSafeTrajectory		= -1;
		bFullyBlock				= false;

		iCentralTrajectory		= -1;
		bRePlan					= false;

		bCanChangeLane			= false;
		distanceToGoBack		= 0;
		timeToGoBack			= 0;
		distanceToChangeLane	= 0;
		timeToChangeLane		= 0;
		bGoalReached			= false;
		bTargetLaneSafe			= true;
		bUpcomingLeft			= false;
		bUpcomingRight			= false;
		targetLaneID			= -1;
		currentLaneID			= -1;
		originalLaneID			= -1;
		currentStopSignID		= -1;

		indicator 				= INDICATOR_NONE;
	}

	virtual ~PreCalculatedConditions(){}

	std::string ToStringHeader()
	{
		return "General>>:currentVelocity:distanceToStop:minStoppingDistance:bStartBehaviorGenerator:bGoalReached:"
				"Following>>:velocityOfNext:distanceToNext:"
				"TrafficLight>>:currentTrafficLightID:bTrafficIsRed:"
				"Swerving>>:iSafeTrajectory:bFullyBlock:";
	}

	std::string ToString(STATE_TYPE beh)
	{
		std::ostringstream str;
		if(beh == FORWARD_STATE)
		{
			str << "GoToGoal>>:"<<currentVelocity<<":"<<distanceToStop()<<":"<<minStoppingDistance<<":"<<bGreenOutsideControl<<":"<<bGoalReached<<":" <<
					">>:"<<velocityOfNext<<":"<<distanceToNext<<":" <<
					">>:"<<currentTrafficLightID<<":"<<bTrafficIsRed<<":" <<
					">>:"<<iCurrSafeTrajectory<<":"<<bFullyBlock<<":";
		}
		else if(beh == FOLLOW_STATE)
		{
			str << "General>>:"<<currentVelocity<<":"<<distanceToStop()<<":"<<minStoppingDistance<<":"<<bGreenOutsideControl<<":"<<bGoalReached<<":" <<
					"Following>>:"<<velocityOfNext<<":"<<distanceToNext<<":" <<
					">>:"<<currentTrafficLightID<<":"<<bTrafficIsRed<<":" <<
					">>:"<<iCurrSafeTrajectory<<":"<<bFullyBlock<<":";
		}
		else if(beh == OBSTACLE_AVOIDANCE_STATE)
		{
			str << "General>>:"<<currentVelocity<<":"<<distanceToStop()<<":"<<minStoppingDistance<<":"<<bGreenOutsideControl<<":"<<bGoalReached<<":" <<
					">>:"<<velocityOfNext<<":"<<distanceToNext<<":" <<
					">>:"<<currentTrafficLightID<<":"<<bTrafficIsRed<<":" <<
					"Swerving>>:"<<iCurrSafeTrajectory<<":"<<bFullyBlock<<":";

		}
		else if(beh == TRAFFIC_LIGHT_STOP_STATE)
		{
			str << "General>>:"<<currentVelocity<<":"<<distanceToStop()<<":"<<minStoppingDistance<<":"<<bGreenOutsideControl<<":"<<bGoalReached<<":" <<
					">>:"<<velocityOfNext<<":"<<distanceToNext<<":" <<
					"TL Stop>>:"<<currentTrafficLightID<<":"<<bTrafficIsRed<<":" <<
					">>:"<<iCurrSafeTrajectory<<":"<<bFullyBlock<<":";

		}
		else if(beh == WAITING_STATE)
		{
			str << "General>>:"<<currentVelocity<<":"<<distanceToStop()<<":"<<minStoppingDistance<<":"<<bGreenOutsideControl<<":"<<bGoalReached<<":" <<
					">>:"<<velocityOfNext<<":"<<distanceToNext<<":" <<
					"TL Wait>>:"<<currentTrafficLightID<<":"<<bTrafficIsRed<<":" <<
					">>:"<<iCurrSafeTrajectory<<":"<<bFullyBlock<<":";
		}
		else
		{
			str << ">>:"<<currentVelocity<<":"<<distanceToStop()<<":"<<minStoppingDistance<<":"<<bGreenOutsideControl<<":"<<bGoalReached<<":" <<
					">>:"<<velocityOfNext<<":"<<distanceToNext<<":" <<
					">>:"<<currentTrafficLightID<<":"<<bTrafficIsRed<<":" <<
					">>:"<<iCurrSafeTrajectory<<":"<<bFullyBlock<<":";
		}

		return str.str();
	}
};

class BehaviorStateMachine
{
public:
	virtual BehaviorStateMachine* GetNextState() = 0;
	virtual void Init();
	virtual void Reset();
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

class StopState : public BehaviorStateMachine
{
public:
	StopState(BehaviorStateMachine* pNextState)
	: BehaviorStateMachine(pNextState){m_Behavior = STOPPING_STATE;}
	virtual ~StopState(){}
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

} /* namespace PlannerHNS */

#endif /* BEHAVIORSTATEMACHINE_H_ */
