/// \file LocalPlannerH.cpp
/// \brief OpenPlanner's local planing functions combines in one process, used in simulation vehicle and OpenPlanner old implementation like dp_planner node.
/// \author Hatem Darweesh
/// \date Dec 14, 2016

#include "op_planner/LocalPlannerH.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"
#include "op_planner/PlannerH.h"

using namespace UtilityHNS;

namespace PlannerHNS
{

LocalPlannerH::LocalPlannerH()
{

	m_PrevBrakingWayPoint = 0;
	m_iSafeTrajectory = 0;
	m_iCurrentTotalPathId = 0;
	pLane = 0;
	m_CurrentVelocity =  m_CurrentVelocityD =0;
	m_CurrentSteering = m_CurrentSteeringD =0;
	m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;
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
	m_SimulationSteeringDelayFactor = 0.1;
	UtilityH::GetTickCount(m_SteerDelayTimer);
	m_PredictionTime = 0;

	InitBehaviorStates();
}

LocalPlannerH::~LocalPlannerH()
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

void LocalPlannerH::Init(const ControllerParams& ctrlParams, const PlannerHNS::PlanningParams& params,const CAR_BASIC_INFO& carInfo)
 	{

 		m_CarInfo = carInfo;
 		m_ControlParams = ctrlParams;
 		m_CurrentVelocity =  m_CurrentVelocityD =0;
 		m_CurrentSteering = m_CurrentSteeringD =0;
 		m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
 		m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;
 		m_params = params;
 		m_InitialFollowingDistance = m_params.minFollowingDistance;

 		m_pidVelocity.Init(0.005, 0.005, 0.05);
		m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

		m_pidStopping.Init(0.1, 0.05, 0.1);
		m_pidStopping.Setlimit(m_params.horizonDistance, 0);

		if(m_pCurrentBehaviorState)
			m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);

 	}

void LocalPlannerH::InitBehaviorStates()
{

	m_pStopState 				= new StopState(&m_params, 0, 0);
	m_pMissionCompleteState 	= new MissionAccomplishedState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
	m_pGoalState				= new GoalState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
	m_pGoToGoalState 			= new ForwardState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
	m_pWaitState 				= new WaitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pInitState 				= new InitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pFollowState				= new FollowState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pAvoidObstacleState		= new SwerveState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pTrafficLightStopState	= new TrafficLightStopState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pTrafficLightWaitState	= new TrafficLightWaitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignWaitState		= new StopSignWaitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	m_pStopSignStopState		= new StopSignStopState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

	m_pGoToGoalState->InsertNextState(m_pStopState);
	m_pGoToGoalState->InsertNextState(m_pWaitState);
	m_pGoToGoalState->InsertNextState(m_pFollowState);
	m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
	m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);
	m_pGoToGoalState->InsertNextState(m_pStopSignStopState);

	m_pStopState->InsertNextState(m_pGoToGoalState);

	m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);
	m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);

	m_pStopSignWaitState->decisionMakingTime = 5.0;
	m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);

	m_pFollowState->InsertNextState(m_pStopState);
	m_pFollowState->InsertNextState(m_pTrafficLightStopState);
	m_pFollowState->InsertNextState(m_pStopSignStopState);

	m_pAvoidObstacleState->decisionMakingTime = 0.1;

	m_pCurrentBehaviorState = m_pInitState;
}

void LocalPlannerH::InitPolygons()
{
	double l2 = m_CarInfo.length/2.0;
	double w2 = m_CarInfo.width/2.0;

	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 0,0));
}

void LocalPlannerH::ReInitializePlanner(const WayPoint& start_pose)
{

	m_pidVelocity.Init(0.005, 0.005, 0.05);
	m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

	m_pidStopping.Init(0.1, 0.05, 0.1);
	m_pidStopping.Setlimit(m_params.horizonDistance, 0);

	m_PrevBrakingWayPoint = 0;
	m_iSafeTrajectory = 0;
	m_iCurrentTotalPathId = 0;
	m_CurrentVelocity =  m_CurrentVelocityD =0;
	m_CurrentSteering = m_CurrentSteeringD =0;
	m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;

	m_pCurrentBehaviorState = m_pFollowState;
	m_TotalPath.clear();
	m_OriginalLocalPath.clear();
	m_TotalOriginalPath.clear();
	m_Path.clear();
	m_RollOuts.clear();
	m_pCurrentBehaviorState->m_Behavior = PlannerHNS::FORWARD_STATE;
	m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 1;
	FirstLocalizeMe(start_pose);
	LocalizeMe(0);
}

 void LocalPlannerH::FirstLocalizeMe(const WayPoint& initCarPos)
 {
	pLane = initCarPos.pLane;
	state = initCarPos;
	m_OdometryState.pos.a = initCarPos.pos.a;
	m_OdometryState.pos.x = initCarPos.pos.x + (m_CarInfo.wheel_base/2.0 * cos(initCarPos.pos.a));
	m_OdometryState.pos.y = initCarPos.pos.y + (m_CarInfo.wheel_base/2.0 * sin(initCarPos.pos.a));
 }

 void LocalPlannerH::LocalizeMe(const double& dt)
{
	//calculate the new x, y ,
	 WayPoint currPose = state;

	if(m_CurrentShift == SHIFT_POS_DD)
	{
		m_OdometryState.pos.x	 +=  m_CurrentVelocity * dt * cos(currPose.pos.a);
		m_OdometryState.pos.y	 +=  m_CurrentVelocity * dt * sin(currPose.pos.a);
		m_OdometryState.pos.a	 +=  m_CurrentVelocity * dt * tan(m_CurrentSteering)  / m_CarInfo.wheel_base;

	}
	else if(m_CurrentShift == SHIFT_POS_RR )
	{
		m_OdometryState.pos.x	 +=  -m_CurrentVelocity * dt * cos(currPose.pos.a);
		m_OdometryState.pos.y	 +=  -m_CurrentVelocity * dt * sin(currPose.pos.a);
		m_OdometryState.pos.a	 +=  -m_CurrentVelocity * dt * tan(m_CurrentSteering);
	}

	m_OdometryState.pos.a = atan2(sin(m_OdometryState.pos.a), cos(m_OdometryState.pos.a));
	m_OdometryState.pos.a = UtilityH::FixNegativeAngle(m_OdometryState.pos.a);

	state.pos.a = m_OdometryState.pos.a;
	state.pos.x = m_OdometryState.pos.x	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base) * cos (m_OdometryState.pos.a));
	state.pos.y = m_OdometryState.pos.y	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base/2.0) * sin (m_OdometryState.pos.a));
}

 void LocalPlannerH::UpdateState(const PlannerHNS::VehicleState& state, const bool& bUseDelay)
  {
	 if(!bUseDelay)
	 {
		 m_CurrentSteering 	= m_CurrentSteeringD;
		// std::cout << " No Delay " << std::endl;
	 }
	 else
	 {
		 double currSteerDeg = RAD2DEG * m_CurrentSteering;
		 double desiredSteerDeg = RAD2DEG * m_CurrentSteeringD;

		 double mFact = UtilityH::GetMomentumScaleFactor(state.speed);
		 double diff = desiredSteerDeg - currSteerDeg;
		 double diffSign = UtilityH::GetSign(diff);
		 double inc = 1.0*diffSign;
		 if(fabs(diff) < 1.0 )
			 inc = diff;

//		 std::cout << "Delay: " << m_SimulationSteeringDelayFactor
//				 << ", Fact: " << mFact
//				 << ", Diff: " << diff
//				 << ", inc: " << inc << std::endl;
		 if(UtilityH::GetTimeDiffNow(m_SteerDelayTimer) > m_SimulationSteeringDelayFactor*mFact)
		 {
			 UtilityH::GetTickCount(m_SteerDelayTimer);
			 currSteerDeg += inc;
		 }

		 m_CurrentSteering = DEG2RAD * currSteerDeg;
	 }

	 m_CurrentShift 	= m_CurrentShiftD;
	 m_CurrentVelocity = m_CurrentVelocityD;
  }

 void LocalPlannerH::AddAndTransformContourPoints(const PlannerHNS::DetectedObject& obj, std::vector<PlannerHNS::WayPoint>& contourPoints)
 {
	 contourPoints.clear();
	 WayPoint  p, p_center = obj.center;
	 p_center.pos.a += M_PI_2;
	 for(unsigned int i=0; i< obj.contour.size(); i++)
	 {
		 p.pos = obj.contour.at(i);
		 //TransformPoint(p_center, p.pos);
		 contourPoints.push_back(p);
	 }

	 contourPoints.push_back(obj.center);
 }

 void LocalPlannerH::TransformPoint(const PlannerHNS::WayPoint& refPose, PlannerHNS::GPSPoint& p)
 {
 	PlannerHNS::Mat3 rotationMat(refPose.pos.a);
 	PlannerHNS::Mat3 translationMat(refPose.pos.x, refPose.pos.y);
	p = rotationMat*p;
	p = translationMat*p;
 }

 bool LocalPlannerH::GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<PlannerHNS::TrafficLight>& trafficLights, PlannerHNS::TrafficLight& trafficL)
 {
	 for(unsigned int i = 0; i < trafficLights.size(); i++)
	 {
		 double d = hypot(trafficLights.at(i).pos.y - state.pos.y, trafficLights.at(i).pos.x - state.pos.x);
		 if(d <= trafficLights.at(i).stoppingDistance)
		 {
			 double a_diff = UtilityH::AngleBetweenTwoAnglesPositive(UtilityH::FixNegativeAngle(trafficLights.at(i).pos.a) , UtilityH::FixNegativeAngle(state.pos.a));

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

 void LocalPlannerH::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
		 const int& goalID, const bool& bEmergencyStop, const vector<TrafficLight>& detectedLights,
		 const TrajectoryCost& bestTrajectory)
 {
 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	double critical_long_front_distance =  m_CarInfo.wheel_base/2.0 + m_CarInfo.length/2.0 + m_params.verticalSafetyDistance;

 	pValues->minStoppingDistance = -pow(car_state.speed, 2)/(m_CarInfo.max_deceleration);

 	pValues->iCentralTrajectory		= m_pCurrentBehaviorState->m_pParams->rollOutNumber/2;

 	if(pValues->iCurrSafeTrajectory < 0)
 			pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;

	if(pValues->iPrevSafeTrajectory < 0)
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;

 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= car_state.speed;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
 	pValues->bRePlan 				= false;
 	pValues->bFullyBlock 			= false;


 	pValues->distanceToNext = bestTrajectory.closest_obj_distance;
 	pValues->velocityOfNext = bestTrajectory.closest_obj_velocity;

 	if(pValues->distanceToNext > m_params.minDistanceToAvoid)
 		pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;
 	else if(bestTrajectory.index>=0)
 		pValues->iCurrSafeTrajectory = bestTrajectory.index;

	pValues->bFullyBlock = bestTrajectory.bBlocked;

 	if(bestTrajectory.lane_index >=0)
 		pValues->iCurrSafeLane = bestTrajectory.lane_index;
 	else
 	{
 		PlannerHNS::RelativeInfo info;
 		PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_TotalPath, state, m_params.rollOutDensity*m_params.rollOutNumber/2.0 + 0.1, info);
 		pValues->iCurrSafeLane = info.iGlobalPath;
 	}


	if(NoWayTest(pValues->minStoppingDistance, pValues->iCurrSafeLane))
		pValues->currentGoalID = -1;
	else
		pValues->currentGoalID = goalID;

 	m_iSafeTrajectory = pValues->iCurrSafeTrajectory;
 	m_iCurrentTotalPathId = pValues->iCurrSafeLane;

 	int stopLineID = -1;
 	int stopSignID = -1;
 	int trafficLightID = -1;
 	double distanceToClosestStopLine = 0;
 	bool bGreenTrafficLight = true;

 	if(m_TotalPath.size()>0)
 		distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_TotalPath.at(pValues->iCurrSafeLane), state, 0, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

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


// 	cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop SignID: " << stopSignID << ", TFID: " << trafficLightID << endl;

 	pValues->bTrafficIsRed = !bGreenTrafficLight;

 	if(bEmergencyStop)
	{
		pValues->bFullyBlock = true;
		pValues->distanceToNext = 1;
		pValues->velocityOfNext = 0;
	}
 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

 void LocalPlannerH::UpdateCurrentLane(PlannerHNS::RoadNetwork& map, const double& search_distance)
 {
	 PlannerHNS::Lane* pMapLane = 0;
	PlannerHNS::Lane* pPathLane = 0;
	pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
	if(!pPathLane)
	{
		cout << "Performance Alert: Can't Find Lane Information in Global Path, Searching the Map :( " << endl;
		pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, search_distance);

	}

	if(pPathLane)
		pLane = pPathLane;
	else if(pMapLane)
		pLane = pMapLane;
	else
		pLane = 0;
 }

 void LocalPlannerH::SimulateOdoPosition(const double& dt, const PlannerHNS::VehicleState& vehicleState)
 {
	SetSimulatedTargetOdometryReadings(vehicleState.speed, vehicleState.steer, vehicleState.shift);
	UpdateState(vehicleState, false);
	LocalizeMe(dt);
 }

 bool LocalPlannerH::NoWayTest(const double& min_distance, const int& iGlobalPathIndex)
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

 bool LocalPlannerH::SelectSafeTrajectoryAndSpeedProfile(const PlannerHNS::VehicleState& vehicleState)
 {
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	bool bNewTrajectory = false;

	if(m_TotalPath.size()>0)
	{
		int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
		int index_limit = 0;//m_Path.size() - 20;
		if(index_limit<=0)
			index_limit =  m_Path.size()/2.0;
		if(m_RollOuts.size() == 0
				|| currIndex > index_limit
				|| m_pCurrentBehaviorState->GetCalcParams()->bRePlan
				|| m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath
				|| m_pCurrentBehaviorState->m_Behavior == OBSTACLE_AVOIDANCE_STATE)
		{
			PlannerHNS::PlannerH planner;
			planner.GenerateRunoffTrajectory(m_TotalPath, state,
					m_pCurrentBehaviorState->m_pParams->enableLaneChange,
					vehicleState.speed,
					m_pCurrentBehaviorState->m_pParams->microPlanDistance,
					m_pCurrentBehaviorState->m_pParams->maxSpeed,
					m_pCurrentBehaviorState->m_pParams->minSpeed,
					m_pCurrentBehaviorState->m_pParams->carTipMargin,
					m_pCurrentBehaviorState->m_pParams->rollInMargin,
					m_pCurrentBehaviorState->m_pParams->rollInSpeedFactor,
					m_pCurrentBehaviorState->m_pParams->pathDensity,
					m_pCurrentBehaviorState->m_pParams->rollOutDensity,
					m_pCurrentBehaviorState->m_pParams->rollOutNumber,
					m_pCurrentBehaviorState->m_pParams->smoothingDataWeight,
					m_pCurrentBehaviorState->m_pParams->smoothingSmoothWeight,
					m_pCurrentBehaviorState->m_pParams->smoothingToleranceError,
					m_pCurrentBehaviorState->m_pParams->speedProfileFactor,
					m_pCurrentBehaviorState->m_pParams->enableHeadingSmoothing,
					preCalcPrams->iCurrSafeLane , preCalcPrams->iCurrSafeTrajectory,
					m_RollOuts, m_SampledPoints);

			m_pCurrentBehaviorState->GetCalcParams()->bRePlan = false;
			m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = false;

			if(m_pCurrentBehaviorState->m_Behavior == OBSTACLE_AVOIDANCE_STATE)
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCurrSafeTrajectory;
			else
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCentralTrajectory;

			preCalcPrams->iPrevSafeLane = preCalcPrams->iCurrSafeLane;

			if(preCalcPrams->iPrevSafeLane >= 0
					&& preCalcPrams->iPrevSafeLane < (int)m_RollOuts.size()
					&& preCalcPrams->iPrevSafeTrajectory >= 0
					&& preCalcPrams->iPrevSafeTrajectory < (int)m_RollOuts.at(preCalcPrams->iPrevSafeLane).size())
			{
				m_Path = m_RollOuts.at(preCalcPrams->iPrevSafeLane).at(preCalcPrams->iPrevSafeTrajectory);
				m_OriginalLocalPath = m_TotalPath.at(m_iCurrentTotalPathId);
				bNewTrajectory = true;
			}
		}
	}

	return bNewTrajectory;
 }

 PlannerHNS::BehaviorState LocalPlannerH::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
 {
	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
	if(m_pCurrentBehaviorState==0)
		m_pCurrentBehaviorState = m_pInitState;

	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	currentBehavior.followDistance = preCalcPrams->distanceToNext;


	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;

	double average_braking_distance = -pow(vehicleState.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;

	if(average_braking_distance  < 15)
		average_braking_distance = 15;

	currentBehavior.indicator = PlanningHelpers::GetIndicatorsFromPath(m_Path, state, average_braking_distance );

	return currentBehavior;
 }

// double LocalPlannerH::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
// {
//	RelativeInfo info, total_info;
//	PlanningHelpers::GetRelativeInfo(m_TotalPath.at(m_iCurrentTotalPathId), state, total_info);
//	PlanningHelpers::GetRelativeInfo(m_Path, state, info);
//	double average_braking_distance = -pow(CurrStatus.speed, 2)/(m_CarInfo.max_deceleration);
//	double max_velocity	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalPath.at(m_iCurrentTotalPathId), total_info, average_braking_distance);
//
//	unsigned int point_index = 0;
//	double critical_long_front_distance = 2.0;
//
//	if(m_Path.size() <= 5)
//	{
//		double target_velocity = 0;
//		for(unsigned int i = 0; i < m_Path.size(); i++)
//			m_Path.at(i).v = target_velocity;
//	}
//	else if(beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE || beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
//	{
//		PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.stopDistance - critical_long_front_distance, point_index);
//
//		double inc = CurrStatus.speed;
//		int iRange = point_index - info.iBack;
//		if(iRange > 0)
//			inc = inc / (double)iRange;
//		else
//			inc = 0;
//
//		double target_velocity = CurrStatus.speed - inc;
//		for(unsigned int i =  info.iBack; i < point_index; i++)
//		{
//			 if(i < m_Path.size() && i >= 0)
//				 m_Path.at(i).v = target_velocity;
//			 target_velocity -= inc;
//		}
//	}
//	else if(beh.state == FOLLOW_STATE)
//	{
//		double targe_acceleration = -pow(CurrStatus.speed, 2)/(2.0*(beh.followDistance - critical_long_front_distance));
//		if(targe_acceleration <= 0 &&  targe_acceleration > m_CarInfo.max_deceleration/2.0)
//		{
//			double target_velocity = (targe_acceleration * dt) + CurrStatus.speed;
//			for(unsigned int i = 0; i < m_Path.size(); i++)
//			{
//				if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory == m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
//					m_Path.at(i).v = target_velocity;
//				else
//					m_Path.at(i).v = target_velocity*AVOIDANCE_SPEED_FACTOR;
//			}
//
//			//cout << "Accelerate -> Target V: " << target_velocity << ", Brake D: " <<  average_braking_distance << ", Acceleration: " << targe_acceleration << endl;
//		}
//		else
//		{
//			WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.followDistance - critical_long_front_distance, point_index);
//			double inc = CurrStatus.speed;
//			int iRange = point_index - info.iBack;
//
//			if(iRange > 0)
//				inc = inc / (double)iRange;
//			else
//				inc = 0;
//
//			double target_velocity = CurrStatus.speed - inc;
//			for(unsigned int i =  info.iBack; i < point_index; i++)
//			{
//				 if(i < m_Path.size() && i >= 0)
//				 {
//					 target_velocity = target_velocity < 0 ? 0 : target_velocity;
//					 if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory == m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
//						 m_Path.at(i).v = target_velocity;
//					 else
//						 m_Path.at(i).v = target_velocity*AVOIDANCE_SPEED_FACTOR;
//				 }
//
//				 target_velocity -= inc;
//			}
//
//			//cout << "Decelerate -> Target V: " << target_velocity << ", Brake D: " <<  average_braking_distance << ", Start I" << info.iBack << endl;
//		}
//
//	}
//	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
//	{
//		double target_velocity = max_velocity;
//
//		for(unsigned int i = 0; i < m_Path.size(); i++)
//		{
//			if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory == m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
//				m_Path.at(i).v = target_velocity;
//			else
//				m_Path.at(i).v = target_velocity*AVOIDANCE_SPEED_FACTOR;
//		}
//	}
//	else
//	{
//		double target_velocity = 0;
//		for(unsigned int i = 0; i < m_Path.size(); i++)
//			m_Path.at(i).v = target_velocity;
//	}
//
//	return max_velocity;
// }

 double LocalPlannerH::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
 {
	if(m_TotalOriginalPath.size() ==0 ) return 0;

	RelativeInfo info, total_info;
	PlanningHelpers::GetRelativeInfo(m_TotalOriginalPath.at(m_iCurrentTotalPathId), state, total_info);
	PlanningHelpers::GetRelativeInfo(m_Path, state, info);
	double average_braking_distance = -pow(CurrStatus.speed, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;
	double max_velocity	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_TotalOriginalPath.at(m_iCurrentTotalPathId), total_info, m_PrevBrakingWayPoint, average_braking_distance);

	unsigned int point_index = 0;
	double critical_long_front_distance = 2.0;

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

		for(unsigned int i =  info.iBack; i < point_index; i++)
		{
			 if(i < m_Path.size() && i >= 0)
				 m_Path.at(i).v = desiredVelocity;
		}

		return desiredVelocity;
	}
	else if(beh.state == FOLLOW_STATE)
	{
		double targe_acceleration = -pow(CurrStatus.speed, 2)/(2.0*(beh.followDistance - critical_long_front_distance));
		if(targe_acceleration <= 0 &&  targe_acceleration > m_CarInfo.max_deceleration/2.0)
		{
			double target_velocity = (targe_acceleration * dt) + CurrStatus.speed;

			double e = target_velocity - CurrStatus.speed;
			double desiredVelocity = m_pidVelocity.getPID(e);

			for(unsigned int i = info.iBack; i < m_Path.size(); i++)
			{
				if(i < m_Path.size() && i >= 0)
					m_Path.at(i).v = desiredVelocity;
			}

			return desiredVelocity;
			//cout << "Accelerate -> Target V: " << target_velocity << ", Brake D: " <<  average_braking_distance << ", Acceleration: " << targe_acceleration << endl;
		}
		else
		{
			WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.followDistance - critical_long_front_distance, point_index);

			double e = beh.followDistance - m_params.minFollowingDistance;
			double desiredVelocity = m_pidStopping.getPID(e);

			for(unsigned int i =  info.iBack; i < point_index; i++)
			{
				if(i < m_Path.size() && i >= 0)
					m_Path.at(i).v = desiredVelocity;
			}

			return desiredVelocity;
			//cout << "Decelerate -> Target V: " << target_velocity << ", Brake D: " <<  average_braking_distance << ", Start I" << info.iBack << endl;
		}

	}
	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
	{
		double target_velocity = max_velocity;
		if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory != m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
			target_velocity*=AVOIDANCE_SPEED_FACTOR;

		double e = target_velocity - CurrStatus.speed;
		double desiredVelocity = m_pidVelocity.getPID(e);

		for(unsigned int i = info.iBack; i < m_Path.size(); i++)
		{
			m_Path.at(i).v = desiredVelocity;
		}

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

 void LocalPlannerH::ExtractHorizonAndCalculateRecommendedSpeed()
 {
	 if(m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath && m_TotalOriginalPath.size() > 0)
		{
		 	m_PrevBrakingWayPoint = 0;
		 	PlanningHelpers::FixPathDensity(m_TotalOriginalPath.at(m_iCurrentTotalPathId), m_pCurrentBehaviorState->m_pParams->pathDensity);
		 	PlanningHelpers::SmoothPath(m_TotalOriginalPath.at(m_iCurrentTotalPathId), 0.49, 0.25, 0.05);
		 	
			PlanningHelpers::GenerateRecommendedSpeed(m_TotalOriginalPath.at(m_iCurrentTotalPathId), m_CarInfo.max_speed_forward, m_pCurrentBehaviorState->m_pParams->speedProfileFactor);
			m_TotalOriginalPath.at(m_iCurrentTotalPathId).at(m_TotalOriginalPath.at(m_iCurrentTotalPathId).size()-1).v = 0;

		}

		 m_TotalPath.clear();

		 for(unsigned int i = 0; i < m_TotalOriginalPath.size(); i++)
		{
			vector<WayPoint> centerTrajectorySmoothed;
			PlanningHelpers::ExtractPartFromPointToDistanceFast(m_TotalOriginalPath.at(i), state,
					m_pCurrentBehaviorState->m_pParams->horizonDistance ,
					m_pCurrentBehaviorState->m_pParams->pathDensity ,
					centerTrajectorySmoothed,
					m_pCurrentBehaviorState->m_pParams->smoothingDataWeight,
					m_pCurrentBehaviorState->m_pParams->smoothingSmoothWeight,
					m_pCurrentBehaviorState->m_pParams->smoothingToleranceError);

			m_TotalPath.push_back(centerTrajectorySmoothed);
		}
 }

 PlannerHNS::BehaviorState LocalPlannerH::DoOneStep(
		 const double& dt,
		const PlannerHNS::VehicleState& vehicleState,
		const std::vector<PlannerHNS::DetectedObject>& obj_list,
		const int& goalID, PlannerHNS::RoadNetwork& map	,
		const bool& bEmergencyStop,
		const std::vector<TrafficLight>& trafficLight,
		const bool& bLive)
{
	 PlannerHNS::BehaviorState beh;

	 m_params.minFollowingDistance = m_InitialFollowingDistance + vehicleState.speed*1.5;

	 if(!bLive)
		 SimulateOdoPosition(dt, vehicleState);

	UpdateCurrentLane(map, 3.0);


	ExtractHorizonAndCalculateRecommendedSpeed();


	m_PredictedTrajectoryObstacles = obj_list;

	timespec t;
	UtilityH::GetTickCount(t);
	TrajectoryCost tc = m_TrajectoryCostsCalculatotor.DoOneStep(m_RollOuts, m_TotalPath, state,
			m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory, m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeLane, *m_pCurrentBehaviorState->m_pParams,
			m_CarInfo,vehicleState, m_PredictedTrajectoryObstacles);
	m_CostCalculationTime = UtilityH::GetTimeDiffNow(t);


	UtilityH::GetTickCount(t);
	CalculateImportantParameterForDecisionMaking(vehicleState, goalID, bEmergencyStop, trafficLight, tc);

	beh = GenerateBehaviorState(vehicleState);
	m_BehaviorGenTime = UtilityH::GetTimeDiffNow(t);

	UtilityH::GetTickCount(t);
	beh.bNewPlan = SelectSafeTrajectoryAndSpeedProfile(vehicleState);

	m_RollOutsGenerationTime = UtilityH::GetTimeDiffNow(t);

	beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);

	return beh;
 }

} /* namespace PlannerHNS */
