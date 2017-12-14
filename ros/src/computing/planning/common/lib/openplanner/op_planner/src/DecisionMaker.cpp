/*
 * CarState.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: hatem
 */

#include "DecisionMaker.h"
#include "UtilityH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include "MatrixOperations.h"


namespace PlannerHNS
{

DecisionMaker::DecisionMaker()
{
	m_iCurrentTotalPathId = 0;
	pLane = 0;
	m_pCurrentBehaviorState = 0;
	m_pGoToGoalState = 0;
	m_pStopState= 0;
	m_pWaitState= 0;
	m_pMissionCompleteState= 0;
	m_pAvoidObstacleState = 0;
	m_pTrafficLightStopState = 0;
	m_pTrafficLightWaitState = 0;
	m_pStopSignStopState = 0;
	m_pStopSignWaitState = 0;
	m_pFollowState = 0;
	m_MaxLaneSearchDistance = 3.0;
	m_pStopState = 0;
	m_pMissionCompleteState  =0;
	m_pGoalState			=0;
	m_pGoToGoalState 		=0;
	m_pWaitState 			=0;
	m_pInitState 			=0;
	m_pFollowState			=0;
	m_pAvoidObstacleState	=0;
	m_pTrafficLightStopState=0;
	m_pTrafficLightWaitState=0;
	m_pStopSignWaitState	=0;
	m_pStopSignStopState=0;
}

DecisionMaker::~DecisionMaker()
{
	delete m_pStopState;
	delete m_pMissionCompleteState ;
	delete m_pGoalState			;
	delete m_pGoToGoalState 		;
	delete m_pWaitState 			;
	delete m_pInitState 			;
	delete m_pFollowState			;
	delete m_pAvoidObstacleState	;
	delete m_pTrafficLightStopState;
	delete m_pTrafficLightWaitState;
	delete m_pStopSignWaitState	;
	delete m_pStopSignStopState;
}

void DecisionMaker::Init(const ControllerParams& ctrlParams, const PlannerHNS::PlanningParams& params,const CAR_BASIC_INFO& carInfo)
 	{
 		m_CarInfo = carInfo;
 		m_ControlParams = ctrlParams;
 		m_params = params;

 		m_pidVelocity.Init(0.01, 0.004, 0.01);
		m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

		m_pidStopping.Init(0.05, 0.05, 0.1);
		m_pidStopping.Setlimit(m_params.horizonDistance, 0);

		InitBehaviorStates();

		if(m_pCurrentBehaviorState)
			m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);
 	}

void DecisionMaker::InitBehaviorStates()
{

	m_pStopState 				= new StopState(&m_params, 0, 0);
	m_pMissionCompleteState 	= new MissionAccomplishedStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
	m_pGoalState				= new GoalStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
	m_pGoToGoalState 			= new ForwardStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
	m_pInitState 				= new InitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

//	m_pWaitState 				= new WaitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pFollowState				= new FollowStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pAvoidObstacleState		= new SwerveStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
//	m_pTrafficLightStopState	= new TrafficLightStopState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
//	m_pTrafficLightWaitState	= new TrafficLightWaitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignWaitState		= new StopSignWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignStopState		= new StopSignStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

//	m_pGoToGoalState->InsertNextState(m_pStopState);
//	m_pGoToGoalState->InsertNextState(m_pWaitState);
//	m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);

	m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
	m_pGoToGoalState->InsertNextState(m_pStopSignStopState);
	m_pGoToGoalState->InsertNextState(m_pFollowState);
	m_pGoToGoalState->decisionMakingCount = m_params.nReliableCount;

	m_pGoalState->InsertNextState(m_pGoToGoalState);

//	m_pStopState->InsertNextState(m_pGoToGoalState);
//
//	m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);
//	m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);
//
	m_pStopSignWaitState->decisionMakingTime = 5.0;
	m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);
	m_pStopSignWaitState->InsertNextState(m_pGoalState);
//
//	m_pFollowState->InsertNextState(m_pStopState);
//	m_pFollowState->InsertNextState(m_pTrafficLightStopState);

	m_pFollowState->InsertNextState(m_pAvoidObstacleState);
	m_pFollowState->InsertNextState(m_pStopSignStopState);
	m_pFollowState->InsertNextState(m_pGoalState);
	m_pFollowState->decisionMakingCount = m_params.nReliableCount;


	m_pInitState->decisionMakingCount = m_params.nReliableCount;

	m_pCurrentBehaviorState = m_pInitState;
}

 bool DecisionMaker::GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<PlannerHNS::TrafficLight>& trafficLights, PlannerHNS::TrafficLight& trafficL)
 {
	 for(unsigned int i = 0; i < trafficLights.size(); i++)
	 {
		 double d = hypot(trafficLights.at(i).pos.y - state.pos.y, trafficLights.at(i).pos.x - state.pos.x);
		 if(d <= trafficLights.at(i).stoppingDistance)
		 {
			 double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(UtilityHNS::UtilityH::FixNegativeAngle(trafficLights.at(i).pos.a) , UtilityHNS::UtilityH::FixNegativeAngle(state.pos.a));

			 if(a_diff < M_PI_2 && trafficLights.at(i).id != prevTrafficLightId)
			 {
				 //std::cout << "Detected Light, ID = " << trafficLights.at(i).id << ", Distance = " << d << ", Angle = " << trafficLights.at(i).pos.a*RAD2DEG << ", Car Heading = " << state.pos.a*RAD2DEG << ", Diff = " << a_diff*RAD2DEG << std::endl;
				 trafficL = trafficLights.at(i);
				 return true;
			 }
		 }
	 }

	 return false;
 }

 void DecisionMaker::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
		 const int& goalID, const bool& bEmergencyStop, const std::vector<TrafficLight>& detectedLights,
		 const TrajectoryCost& bestTrajectory)
 {
	 if(m_TotalPath.size() == 0) return;

 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	pValues->minStoppingDistance = -pow(car_state.speed, 2)/(m_CarInfo.max_deceleration);
 	pValues->iCentralTrajectory		= m_pCurrentBehaviorState->m_pParams->rollOutNumber/2;

	if(pValues->iPrevSafeTrajectory < 0)
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;

 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= car_state.speed;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
 	pValues->bFullyBlock 			= false;

 	pValues->distanceToNext = bestTrajectory.closest_obj_distance;
 	pValues->velocityOfNext = bestTrajectory.closest_obj_velocity;

 	if(bestTrajectory.index >=0 &&  bestTrajectory.index < (int)m_RollOuts.size())
 		pValues->iCurrSafeTrajectory = bestTrajectory.index;
 	else
 		pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;

	pValues->bFullyBlock = bestTrajectory.bBlocked;

 	if(bestTrajectory.lane_index >=0)
 		pValues->iCurrSafeLane = bestTrajectory.lane_index;
 	else
 	{
 		PlannerHNS::RelativeInfo info;
 		PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_TotalPath, state, m_params.rollOutDensity*m_params.rollOutNumber/2.0 + 0.1, info);
 		pValues->iCurrSafeLane = info.iGlobalPath;
 	}

 	double critical_long_front_distance =  m_CarInfo.wheel_base/2.0 + m_CarInfo.length/2.0 + m_params.verticalSafetyDistance;

	if(ReachEndOfGlobalPath(pValues->minStoppingDistance + critical_long_front_distance, pValues->iCurrSafeLane))
		pValues->currentGoalID = -1;
	else
		pValues->currentGoalID = goalID;

 	m_iCurrentTotalPathId = pValues->iCurrSafeLane;

 	int stopLineID = -1;
 	int stopSignID = -1;
 	int trafficLightID = -1;
 	double distanceToClosestStopLine = 0;
 	bool bGreenTrafficLight = true;


  	distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_TotalPath.at(pValues->iCurrSafeLane), state, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

 	if(distanceToClosestStopLine > 0 && distanceToClosestStopLine < pValues->minStoppingDistance)
 	{
 		if(m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
 		{
 			pValues->currentTrafficLightID = trafficLightID;
 			//cout << "Detected Traffic Light: " << trafficLightID << endl;
 			for(unsigned int i=0; i< detectedLights.size(); i++)
 			{
 				if(detectedLights.at(i).id == trafficLightID)
 					bGreenTrafficLight = (detectedLights.at(i).lightState == GREEN_LIGHT);
 			}
 		}

 		if(m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior)
 			pValues->currentStopSignID = stopSignID;

		pValues->stoppingDistances.push_back(distanceToClosestStopLine);
		//std::cout << "LP => D: " << pValues->distanceToStop() << ", PrevSignID: " << pValues->prevTrafficLightID << ", CurrSignID: " << pValues->currentTrafficLightID << ", Green: " << bGreenTrafficLight << endl;
 	}


 	//std::cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop SignID: " << stopSignID << ", TFID: " << trafficLightID << std::endl;

 	pValues->bTrafficIsRed = !bGreenTrafficLight;

 	if(bEmergencyStop)
	{
		pValues->bFullyBlock = true;
		pValues->distanceToNext = 1;
		pValues->velocityOfNext = 0;
	}
 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

 void DecisionMaker::UpdateCurrentLane(const double& search_distance)
 {
	 PlannerHNS::Lane* pMapLane = 0;
	PlannerHNS::Lane* pPathLane = 0;
	pPathLane = MappingHelpers::GetLaneFromPath(state, m_TotalPath.at(m_iCurrentTotalPathId));
	if(!pPathLane)
	{
		std::cout << "Performance Alert: Can't Find Lane Information in Global Path, Searching the Map :( " << std::endl;
		pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, m_Map, search_distance);
	}

	if(pPathLane)
		pLane = pPathLane;
	else if(pMapLane)
		pLane = pMapLane;
	else
		pLane = 0;
 }

 bool DecisionMaker::ReachEndOfGlobalPath(const double& min_distance, const int& iGlobalPathIndex)
 {
	 if(m_TotalPath.size()==0) return false;

	 PlannerHNS::RelativeInfo info;
	 PlanningHelpers::GetRelativeInfo(m_TotalPath.at(iGlobalPathIndex), state, info);

	 double d = 0;
	 for(unsigned int i = info.iFront; i < m_TotalPath.at(iGlobalPathIndex).size()-1; i++)
	 {
		 d+= hypot(m_TotalPath.at(iGlobalPathIndex).at(i+1).pos.y - m_TotalPath.at(iGlobalPathIndex).at(i).pos.y, m_TotalPath.at(iGlobalPathIndex).at(i+1).pos.x - m_TotalPath.at(iGlobalPathIndex).at(i).pos.x);
		 if(d > min_distance)
			 return false;
	 }

	 return true;
 }

 void DecisionMaker::SetNewGlobalPath(const std::vector<std::vector<WayPoint> >& globalPath)
 {
	 if(m_pCurrentBehaviorState)
	 {
		 m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
		 m_TotalOriginalPath = globalPath;
	 }
 }

 bool DecisionMaker::SelectSafeTrajectory()
 {
	 bool bNewTrajectory = false;
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	 if(!preCalcPrams || m_RollOuts.size() == 0) return bNewTrajectory;

	int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
	int index_limit = 0;
	if(index_limit<=0)
		index_limit =  m_Path.size()/2.0;
	if(currIndex > index_limit
			|| preCalcPrams->bRePlan
			|| preCalcPrams->bNewGlobalPath)
	{
		//std::cout << "New Local Plan !! " << currIndex << ", "<< preCalcPrams->bRePlan << ", " << preCalcPrams->bNewGlobalPath  << std::endl;

		preCalcPrams->bRePlan = false;
		preCalcPrams->bNewGlobalPath = false;
		bNewTrajectory = true;
		m_Path = m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
	}

	return bNewTrajectory;
 }

 PlannerHNS::BehaviorState DecisionMaker::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
 {
	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
	if(m_pCurrentBehaviorState==0)
		m_pCurrentBehaviorState = m_pInitState;

	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	currentBehavior.followDistance = preCalcPrams->distanceToNext;

	if(preCalcPrams->bUpcomingRight)
		currentBehavior.indicator = PlannerHNS::INDICATOR_RIGHT;
	else if(preCalcPrams->bUpcomingLeft)
		currentBehavior.indicator = PlannerHNS::INDICATOR_LEFT;
	else
		currentBehavior.indicator = PlannerHNS::INDICATOR_NONE;

	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;
	if(preCalcPrams->iPrevSafeTrajectory<0 || preCalcPrams->iPrevSafeTrajectory >= m_RollOuts.size())
		currentBehavior.iTrajectory		= preCalcPrams->iCurrSafeTrajectory;
	else
		currentBehavior.iTrajectory		= preCalcPrams->iPrevSafeTrajectory;

	return currentBehavior;
 }

 double DecisionMaker::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
 {
	if(m_TotalOriginalPath.size() ==0 ) return 0;

	RelativeInfo info, total_info;
	PlanningHelpers::GetRelativeInfo(m_TotalOriginalPath.at(m_iCurrentTotalPathId), state, total_info);
	PlanningHelpers::GetRelativeInfo(m_Path, state, info);
	double average_braking_distance = -pow(CurrStatus.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;
	double max_velocity	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalOriginalPath.at(m_iCurrentTotalPathId), total_info, total_info.iBack, average_braking_distance);

	unsigned int point_index = 0;
	double critical_long_front_distance = m_CarInfo.length/2.0;


	if(m_Path.size() <= 5)
	{
		double target_velocity = 0;
		for(unsigned int i = 0; i < m_Path.size(); i++)
			m_Path.at(i).v = target_velocity;
	}
	else if(beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE || beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
	{
		PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.stopDistance - critical_long_front_distance, point_index);

		double e = -beh.stopDistance;
		double desiredVelocity = m_pidStopping.getPID(e);

		for(unsigned int i =  0; i < m_Path.size(); i++)
			m_Path.at(i).v = desiredVelocity;

		return desiredVelocity;
	}
	else if(beh.state == FOLLOW_STATE)
	{
		double follow_d = beh.followDistance;
		double min_follow_distance = m_params.minFollowingDistance;
//		if(CurrStatus.speed < 1 )
//			min_follow_distance = critical_long_front_distance;

		double targe_acceleration = -pow(CurrStatus.speed, 2)/(2.0*(follow_d));
//		if(targe_acceleration <= 0 &&  targe_acceleration > m_CarInfo.max_deceleration/2.0)
		{
			double target_velocity = (targe_acceleration * dt);
			//double target_velocity = beh.followVelocity;

//			double e = target_velocity - CurrStatus.speed;
//			double desiredVelocity = m_pidVelocity.getPID(e);

			double e = beh.followDistance - min_follow_distance;
			double desiredVelocity = m_pidStopping.getPID(e);

			if(desiredVelocity > max_velocity)
				desiredVelocity = max_velocity;
			else if(desiredVelocity < 0.25)
				desiredVelocity = 0;

			for(unsigned int i = 0; i < m_Path.size(); i++)
				m_Path.at(i).v = desiredVelocity;

			//std::cout << "Acc: V: " << desiredVelocity << ", Object V: " <<  target_velocity << ", Accel: " << targe_acceleration << std::endl;

			return desiredVelocity;
		}
//		else
//		{
//			//WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, follow_d - critical_long_front_distance, point_index);
//
//			double e = follow_d - min_follow_distance;
//			double desiredVelocity = m_pidStopping.getPID(e);
//
//			if(desiredVelocity > max_velocity)
//				desiredVelocity = max_velocity;
//
//			for(unsigned int i =  0; i < m_Path.size(); i++)
//			{
//				if(i < m_Path.size() && i >= 0)
//					m_Path.at(i).v = desiredVelocity;
//			}
//
//			std::cout << "Follow Dec: Target V: " << desiredVelocity << ", Object D: " <<  follow_d << ", E: " << e << std::endl;
//			return desiredVelocity;
//		}

	}
	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
	{
		double target_velocity = max_velocity;
		bool bSlowBecauseChange=false;
		if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory != m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
		{
			target_velocity*=0.5;
			bSlowBecauseChange = true;
		}

		double e = target_velocity - CurrStatus.speed;
		double desiredVelocity = m_pidVelocity.getPID(e);

		if(desiredVelocity>max_velocity)
			desiredVelocity = max_velocity;
		else if(desiredVelocity < 0.25)
			desiredVelocity = 0;

		for(unsigned int i = 0; i < m_Path.size(); i++)
			m_Path.at(i).v = desiredVelocity;

		//std::cout << "Target Velocity: " << desiredVelocity << ", Change Slowdown: " << bSlowBecauseChange  << std::endl;

		return desiredVelocity;
	}
	else
	{
		double target_velocity = 0;
		for(unsigned int i = info.iBack; i < m_Path.size(); i++)
			m_Path.at(i).v = target_velocity;

		return target_velocity;
	}

	return max_velocity;
 }

 PlannerHNS::BehaviorState DecisionMaker::DoOneStep(
		const double& dt,
		const PlannerHNS::WayPoint currPose,
		const PlannerHNS::VehicleState& vehicleState,
		const int& goalID,
		const std::vector<TrafficLight>& trafficLight,
		const TrajectoryCost& tc,
		const bool& bEmergencyStop)
{
	 PlannerHNS::BehaviorState beh;
	 state = currPose;
	 m_TotalPath.clear();
	for(unsigned int i = 0; i < m_TotalOriginalPath.size(); i++)
	{
		t_centerTrajectorySmoothed.clear();
		PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_TotalOriginalPath.at(i), state, m_params.horizonDistance ,	m_params.pathDensity , t_centerTrajectorySmoothed);
		m_TotalPath.push_back(t_centerTrajectorySmoothed);
	}

	UpdateCurrentLane(m_MaxLaneSearchDistance);

	CalculateImportantParameterForDecisionMaking(vehicleState, goalID, bEmergencyStop, trafficLight, tc);

	beh = GenerateBehaviorState(vehicleState);

	beh.bNewPlan = SelectSafeTrajectory();

	beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);

	//std::cout << "Eval_i: " << tc.index << ", Curr_i: " <<  m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ", Prev_i: " << m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << std::endl;

	return beh;
 }

} /* namespace PlannerHNS */
