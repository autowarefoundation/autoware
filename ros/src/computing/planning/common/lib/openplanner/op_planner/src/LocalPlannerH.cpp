/*
 * CarState.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: hatem
 */

#include "LocalPlannerH.h"
#include "UtilityH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include "MatrixOperations.h"
#include "PlannerH.h"

using namespace UtilityHNS;

namespace PlannerHNS
{

LocalPlannerH::LocalPlannerH()
{
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
			 //double a = UtilityH::FixNegativeAngle(atan2(trafficLights.at(i).pos.y - state.pos.y, trafficLights.at(i).pos.x - state.pos.x));
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
		 const int& goalID, const bool& bEmergencyStop, const bool& bGreenTrafficLight,
		 const TrajectoryCost& bestTrajectory)
 {
 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	double critical_long_front_distance =  m_CarInfo.wheel_base/2.0 + m_CarInfo.length/2.0 + m_params.verticalSafetyDistance;
	//double critical_long_back_distance =  m_CarInfo.length/2.0 + m_params.verticalSafetyDistance - m_CarInfo.wheel_base/2.0;

 	pValues->minStoppingDistance = -pow(car_state.speed, 2)/m_CarInfo.max_deceleration;

 	pValues->iCentralTrajectory		= m_pCurrentBehaviorState->m_pParams->rollOutNumber/2;

 	if(pValues->iCurrSafeTrajectory < 0)
 			pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;

	if(pValues->iPrevSafeTrajectory < 0)
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;

 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= car_state.speed;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
// 	pValues->currentStopSignID		= -1;
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


// 	if(bestTrajectory.index == -1 && pValues->distanceToNext < m_pCurrentBehaviorState->m_pParams->minFollowingDistance)
// 		pValues->bFullyBlock = true;



 	int stopLineID = -1;
 	int stopSignID = -1;
 	int trafficLightID = -1;
 	double distanceToClosestStopLine = 0;

 	if(m_TotalPath.size()>0)
 		distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_TotalPath.at(pValues->iCurrSafeLane), state, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

 	if(distanceToClosestStopLine > 0 && distanceToClosestStopLine < pValues->minStoppingDistance)
 	{
 		if(m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
 			pValues->currentTrafficLightID = trafficLightID;

 		if(m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior)
 			pValues->currentStopSignID = stopSignID;

		pValues->stoppingDistances.push_back(distanceToClosestStopLine);
		//std::cout << "From Local Planner => D: " << pValues->distanceToStop() << ", Prev SignID: " << pValues->prevStopSignID << ", Curr SignID: " << pValues->currentStopSignID << endl;
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

double LocalPlannerH::PredictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::VehicleState& vstatus, const PlannerHNS::WayPoint& currState)
{
	PlanningParams* pParams = m_pCurrentBehaviorState->m_pParams;

		//1- Calculate time prediction for each trajectory
	if(path.size() == 0) return 0;

//	SimulatedTrajectoryFollower predControl;
//	ControllerParams params;
//	params.Steering_Gain = PID_CONST(1.5, 0.0, 0.0);
//	params.Velocity_Gain = PID_CONST(0.2, 0.01, 0.1);
//	params.minPursuiteDistance = 3.0;
//
//	predControl.Init(params, m_CarInfo);
//	//double totalDistance = 0;
//	VehicleState CurrentState = vstatus;
//	VehicleState CurrentSteeringD;
//	bool bNewPath = true;
//	WayPoint localState = currState;
//	WayPoint prevState = currState;
//	int iPrevIndex = 0;
	double accum_time = 0;
//	double pred_max_time = 10.0;
//	double endDistance = pParams->microPlanDistance/2.0;
//
//	for(unsigned int i = 0 ; i < path.size(); i++)
//	{
//		path.at(i).collisionCost = 0;
//		path.at(i).timeCost = -1;
//	}
//
//	int startIndex = PlanningHelpers::GetClosestPointIndex(path, state);
//	double total_distance = 0;
//	path.at(startIndex).timeCost = 0;
//	for(unsigned int i=startIndex+1; i<path.size(); i++)
//	{
//		total_distance += hypot(path.at(i).pos.x- path.at(i-1).pos.x,path.at(i).pos.y- path.at(i-1).pos.y);
//		if(m_CurrentVelocity > 0.1 && total_distance > 0.1)
//			accum_time = total_distance/m_CurrentVelocity;
//		path.at(i).timeCost = accum_time;
//		if(total_distance > endDistance)
//			break;
//	}

//	while(totalDistance < pParams->microPlanDistance/2.0 && accum_time < pred_max_time)
//	{
//		double dt = 0.05;
//		PlannerHNS::BehaviorState currMessage;
//		currMessage.state = FORWARD_STATE;
//		currMessage.maxVelocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, state, 1.5*CurrentState.speed*3.6);
//
//		ControllerParams c_params = m_ControlParams;
//		c_params.SteeringDelay = m_ControlParams.SteeringDelay / (1.0-UtilityH::GetMomentumScaleFactor(CurrentState.speed));
//		predControl.Init(c_params, m_CarInfo);
//		CurrentSteeringD = predControl.DoOneStep(dt, currMessage, path, localState, CurrentState, bNewPath);
//
//		if(bNewPath) // first call
//		{
//			if(predControl.m_iCalculatedIndex > 0)
//			{
//				for(unsigned int j=0; j < predControl.m_iCalculatedIndex; j++)
//					path.at(j).timeCost = -1;
//			}
//		}
//		else
//		{
//			if(predControl.m_iCalculatedIndex != iPrevIndex)
//				path.at(iPrevIndex).timeCost = accum_time;
//		}
//
//		accum_time+=dt;
//		bNewPath = false;
//
//		//Update State
//		CurrentState = CurrentSteeringD;
//
//		//Localize Me
//		localState.pos.x	 +=  CurrentState.speed * dt * cos(localState.pos.a);
//		localState.pos.y	 +=  CurrentState.speed * dt * sin(localState.pos.a);
//		localState.pos.a	 +=  CurrentState.speed * dt * tan(CurrentState.steer)  / m_CarInfo.wheel_base;
//
//		totalDistance += distance2points(prevState.pos, localState.pos);
//
//		prevState = localState;
//		iPrevIndex = predControl.m_iCalculatedIndex;
//	}

	return accum_time;
}

void LocalPlannerH::PredictObstacleTrajectory(PlannerHNS::RoadNetwork& map, const PlannerHNS::WayPoint& pos, const double& predTime, std::vector<std::vector<PlannerHNS::WayPoint> >& paths)
{
	PlannerHNS::PlanningParams planningDefaultParams;
	planningDefaultParams.rollOutNumber = 0;
	planningDefaultParams.microPlanDistance = predTime*pos.v;

	planningDefaultParams.pathDensity = 0.5;
	//PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMapDirectionBased(pos, map, 3.0);
	std::vector<PlannerHNS::Lane*> pMapLanes = MappingHelpers::GetClosestMultipleLanesFromMap(pos, map, 1.5);

	PlannerHNS::PlannerH planner;
	std::vector<int> LanesIds;
	std::vector< std::vector<PlannerHNS::WayPoint> >  rollOuts;
	std::vector<std::vector<PlannerHNS::WayPoint> > generatedPath;

	if(planningDefaultParams.microPlanDistance > 0)
	{
		for(unsigned int i = 0; i < pMapLanes.size(); i++)
		{
			std::vector<std::vector<PlannerHNS::WayPoint> > loca_generatedPath;
			planner.PredictPlanUsingDP(pMapLanes.at(i), pos, planningDefaultParams.microPlanDistance, loca_generatedPath);
			if(loca_generatedPath.size() > 0)
				generatedPath.insert(generatedPath.begin(),loca_generatedPath.begin(), loca_generatedPath.end());
		}
	}

//	planner.GenerateRunoffTrajectory(generatedPath, pos,
//			planningDefaultParams.enableLaneChange,
//			pos.v,
//			planningDefaultParams.microPlanDistance,
//			m_CarInfo.max_speed_forward,
//			planningDefaultParams.minSpeed,
//			planningDefaultParams.carTipMargin,
//			planningDefaultParams.rollInMargin,
//			planningDefaultParams.rollInSpeedFactor,
//			planningDefaultParams.pathDensity,
//			planningDefaultParams.rollOutDensity,
//			planningDefaultParams.rollOutNumber,
//			planningDefaultParams.smoothingDataWeight,
//			planningDefaultParams.smoothingSmoothWeight,
//			planningDefaultParams.smoothingToleranceError,
//			planningDefaultParams.speedProfileFactor,
//			planningDefaultParams.enableHeadingSmoothing,
//			rollOuts);

	if(generatedPath.size() > 0)
	{
		//path = rollOuts.at(0);
		paths = generatedPath;

//		PlanningHelpers::GenerateRecommendedSpeed(path,
//				m_CarInfo.max_speed_forward,
//				planningDefaultParams.speedProfileFactor);
//		PlanningHelpers::SmoothSpeedProfiles(path, 0.15,0.35, 0.1);
	}

	if(pMapLanes.size() ==0 || paths.size() == 0)
	{
		paths.clear();
		generatedPath.clear();
	}
	else
	{
		//std::cout << "------------------------------------------------" <<  std::endl;
		//std::cout << "Predicted Trajectories for Distance : " <<  planningDefaultParams.microPlanDistance << std::endl;
		for(unsigned int j=0; j < paths.size(); j++)
		{
			if(paths.at(j).size()==0)
				continue;

			double timeDelay = 0;
			double total_distance = 0;
			paths.at(j).at(0).timeCost = 0;
			paths.at(j).at(0).v = pos.v;
			for(unsigned int i=1; i<paths.at(j).size(); i++)
			{
				paths.at(j).at(i).v = pos.v;
				paths.at(j).at(i).pos.a = atan2(paths.at(j).at(i).pos.y - paths.at(j).at(i-1).pos.y, paths.at(j).at(i).pos.x - paths.at(j).at(i-1).pos.x);
				total_distance += distance2points(paths.at(j).at(i).pos, paths.at(j).at(i-1).pos);
				if(pos.v > 0.1 && total_distance > 0.1)
					timeDelay = total_distance/pos.v;
				paths.at(j).at(i).timeCost = timeDelay;
			}

			//std::cout << "ID : " <<  j << ", timeDelay : " << timeDelay << ", Distance : " << total_distance << std::endl;
		}

		//std::cout << "------------------------------------------------" <<  std::endl;
	}
}

bool LocalPlannerH::CalculateIntersectionVelocities(std::vector<PlannerHNS::WayPoint>& ego_path, std::vector<std::vector<PlannerHNS::WayPoint> >& predctedPath, const PlannerHNS::DetectedObject& obj)
{
	bool bCollisionDetected = false;
	for(unsigned int k = 0; k < predctedPath.size(); k++)
	{
		for(unsigned int j = 0; j < predctedPath.at(k).size(); j++)
		{
			bool bCollisionFound =false;
			for(unsigned int i = 0; i < ego_path.size(); i++)
			{
				if(ego_path.at(i).timeCost > 0.0)
				{
					double collision_distance = hypot(ego_path.at(i).pos.x-predctedPath.at(k).at(j).pos.x, ego_path.at(i).pos.y-predctedPath.at(k).at(j).pos.y);
					double contact_distance = hypot(state.pos.x - ego_path.at(i).pos.x,state.pos.y - ego_path.at(i).pos.y);
					if(collision_distance <= m_CarInfo.width  && fabs(ego_path.at(i).timeCost - predctedPath.at(k).at(j).timeCost)<4.0)
					{
						ego_path.at(i).collisionCost = 1;
						double a = UtilityH::AngleBetweenTwoAnglesPositive(ego_path.at(i).pos.a, predctedPath.at(k).at(j).pos.a);
						if(a < M_PI_4/2.0)
							ego_path.at(i).v = obj.center.v;
						else
							ego_path.at(i).v = 0;
						predctedPath.at(k).at(j).collisionCost = 1;
						bCollisionFound = true;
						bCollisionDetected = true;
						break;
					}
				}
			}

			if(bCollisionFound)
				break;
		}
	}

	return bCollisionDetected;
}

bool LocalPlannerH::CalculateObstacleCosts(PlannerHNS::RoadNetwork& map, const PlannerHNS::VehicleState& vstatus, const std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	double predTime = PredictTimeCostForTrajectory(m_Path, vstatus, state);
	m_PredictedPath.clear();
	bool bObstacleDetected = false;
	for(unsigned int i = 0; i < obj_list.size(); i++)
	{
		//std::vector<WayPoint> predPath;
		PredictObstacleTrajectory(map, obj_list.at(i).center, 10.0, m_PredictedPath);
		bool bObstacle = CalculateIntersectionVelocities(m_Path, m_PredictedPath, obj_list.at(i));
		if(bObstacle)
			bObstacleDetected = true;
	}

	return bObstacleDetected;
}

 void LocalPlannerH::UpdateCurrentLane(PlannerHNS::RoadNetwork& map, const double& search_distance)
 {
	 PlannerHNS::Lane* pMapLane = 0;
	PlannerHNS::Lane* pPathLane = 0;
	pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
	if(!pPathLane)
		pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, search_distance);

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
	UpdateState(vehicleState, true);
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
		int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndex(m_Path, state);
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
					m_RollOuts, m_PathSection, m_SampledPoints);

			m_pCurrentBehaviorState->GetCalcParams()->bRePlan = false;
			m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = false;

			//cout << "Generating New Trajectories ! iPrev: " << preCalcPrams->iPrevSafeTrajectory << " , iSafe: " << preCalcPrams->iCurrSafeTrajectory << endl;

			if(m_pCurrentBehaviorState->m_Behavior == OBSTACLE_AVOIDANCE_STATE)
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCurrSafeTrajectory;
			else
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCentralTrajectory;

			preCalcPrams->iPrevSafeLane = preCalcPrams->iCurrSafeLane;

			if(preCalcPrams->iPrevSafeLane >= 0
					&& preCalcPrams->iPrevSafeLane < m_RollOuts.size()
					&& preCalcPrams->iPrevSafeTrajectory >= 0
					&& preCalcPrams->iPrevSafeTrajectory < m_RollOuts.at(preCalcPrams->iPrevSafeLane).size())
			{
				//cout << "Select New Trajectories ! iPrev: " << preCalcPrams->iPrevSafeTrajectory << " , iSafe: " << preCalcPrams->iCurrSafeTrajectory << endl;

				m_Path = m_RollOuts.at(preCalcPrams->iPrevSafeLane).at(preCalcPrams->iPrevSafeTrajectory);
//				PlanningHelpers::GenerateRecommendedSpeed(m_Path,
//						m_pCurrentBehaviorState->m_pParams->maxSpeed,
//						m_pCurrentBehaviorState->m_pParams->speedProfileFactor);
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
	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	//if(currentBehavior.state == PlannerHNS::FOLLOW_STATE)
		currentBehavior.followDistance = preCalcPrams->distanceToNext;
	//else
	//	currentBehavior.followDistance = 0;

	if(preCalcPrams->bUpcomingRight)
		currentBehavior.indicator = PlannerHNS::INDICATOR_RIGHT;
	else if(preCalcPrams->bUpcomingLeft)
		currentBehavior.indicator = PlannerHNS::INDICATOR_LEFT;
	else
		currentBehavior.indicator = PlannerHNS::INDICATOR_NONE;
	currentBehavior.maxVelocity 	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, state, vehicleState.speed*3.6);
	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;

	return currentBehavior;
 }

 void LocalPlannerH::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const VehicleState& CurrStatus, const double& dt)
 {

	 RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(m_Path, state, info);
	unsigned int point_index = 0;
	double critical_long_front_distance = 2.0;
	for(unsigned int i = 0; i < m_Path.size(); i++)
		m_Path.at(i).v = m_CarInfo.min_speed_forward;

	if(beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE)
	{
		PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.stopDistance - critical_long_front_distance, point_index);

		double inc = CurrStatus.speed;
		int iRange = point_index - info.iBack;
		//cout << "Range : " << iRange;
		if(iRange > 0)
			inc = inc / (double)iRange;
		else
			inc = 0;

	//	cout << "Target Stopping Velocity: "  <<  endl ;
		double target_velocity = CurrStatus.speed - inc;
		for(unsigned int i =  info.iBack; i < point_index; i++)
		{
			if(target_velocity > m_CarInfo.max_speed_forward)
				target_velocity = m_CarInfo.max_speed_forward;

			if(target_velocity < m_CarInfo.min_speed_forward)
				target_velocity = m_CarInfo.min_speed_forward;

			 if(i < m_Path.size() && i >= 0)
			 {
			//	 cout << target_velocity << ", " ;
				 m_Path.at(i).v = target_velocity;
			 }

			 target_velocity -= inc;
		}

		//cout << endl << endl;
	}
	else if(beh.state == FOLLOW_STATE)
	{
		double targe_acceleration = -pow(CurrStatus.speed, 2)/(2.0*(beh.followDistance - critical_long_front_distance));
		if(targe_acceleration <= 0 &&  targe_acceleration > m_CarInfo.max_deceleration/2.0)
		{
			PlanningHelpers::GenerateRecommendedSpeed(m_Path, m_pCurrentBehaviorState->m_pParams->maxSpeed, m_pCurrentBehaviorState->m_pParams->speedProfileFactor);

			double follow_distance = fabs(CurrStatus.speed) * (m_ControlParams.SteeringDelay+1);
			if(follow_distance < m_ControlParams.minPursuiteDistance)
				follow_distance = m_ControlParams.minPursuiteDistance;

			RelativeInfo info;
			PlanningHelpers::GetRelativeInfo(m_Path, state, info);
			unsigned int point_index = 0;
			WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, follow_distance, point_index);
			double target_velocity = pursuite_point.v;
			if(target_velocity > m_CarInfo.max_speed_forward)
				target_velocity = m_CarInfo.max_speed_forward;


			for(unsigned int i = 0; i < m_Path.size(); i++)
			{
				if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory == m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
					m_Path.at(i).v = target_velocity;
				else
					m_Path.at(i).v = target_velocity*0.75;
			}
		}
		else
		{
			PlanningHelpers::GenerateRecommendedSpeed(m_Path, m_pCurrentBehaviorState->m_pParams->maxSpeed, m_pCurrentBehaviorState->m_pParams->speedProfileFactor);

			WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.followDistance - critical_long_front_distance, point_index);

			double inc = CurrStatus.speed;
			int iRange = point_index - info.iBack;
			//cout << "Range : " << iRange;
			if(iRange > 0)
				inc = inc / (double)iRange;
			else
				inc = 0;

			//cout << "Target Follow Velocity: " <<  endl ;
			double target_velocity = CurrStatus.speed - inc;
			for(unsigned int i =  info.iBack; i < point_index; i++)
			{
				if(target_velocity > m_CarInfo.max_speed_forward)
					target_velocity = m_CarInfo.max_speed_forward;

				if(target_velocity < m_CarInfo.min_speed_forward)
					target_velocity = m_CarInfo.min_speed_forward;

				 if(i < m_Path.size() && i >= 0)
				 {
				//	 cout << target_velocity << ", " ;
					 if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory == m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
						 m_Path.at(i).v = target_velocity;
					 else
						 m_Path.at(i).v = target_velocity*0.75;
				 }

				 target_velocity -= inc;
			}
		}
	}
	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
	{
		PlanningHelpers::GenerateRecommendedSpeed(m_Path, m_pCurrentBehaviorState->m_pParams->maxSpeed, m_pCurrentBehaviorState->m_pParams->speedProfileFactor);

		double follow_distance = fabs(CurrStatus.speed) * (m_ControlParams.SteeringDelay+1);
		if(follow_distance < m_ControlParams.minPursuiteDistance)
			follow_distance = m_ControlParams.minPursuiteDistance;

		RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(m_Path, state, info);
		unsigned int point_index = 0;
		WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, follow_distance, point_index);
		double target_velocity = pursuite_point.v;
		if(target_velocity > m_CarInfo.max_speed_forward)
			target_velocity = m_CarInfo.max_speed_forward;

		for(unsigned int i = 0; i < m_Path.size(); i++)
		{
			if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory == m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
				m_Path.at(i).v = target_velocity;
			else
				m_Path.at(i).v = target_velocity*0.75;

		}
	}
	else
	{
		double target_velocity = 0;
		for(unsigned int i = 0; i < m_Path.size(); i++)
			m_Path.at(i).v = target_velocity;
	}
 }

 PlannerHNS::BehaviorState LocalPlannerH::DoOneStep(
		 const double& dt,
		const PlannerHNS::VehicleState& vehicleState,
		const std::vector<PlannerHNS::DetectedObject>& obj_list,
		const int& goalID, PlannerHNS::RoadNetwork& map	,
		const bool& bEmergencyStop,
		const bool& bGreenTrafficLight,
		const bool& bLive)
{

	 if(!bLive)
		 SimulateOdoPosition(dt, vehicleState);

	UpdateCurrentLane(map, 3.0);

	timespec costTimer;
	UtilityH::GetTickCount(costTimer);
	TrajectoryCost tc = m_TrajectoryCostsCalculatotor.DoOneStep(m_RollOuts, m_TotalPath, state,
			m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory, m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeLane, *m_pCurrentBehaviorState->m_pParams,
			m_CarInfo,vehicleState, obj_list);
	m_CostCalculationTime = UtilityH::GetTimeDiffNow(costTimer);


	timespec behTimer;
	UtilityH::GetTickCount(behTimer);
	CalculateImportantParameterForDecisionMaking(vehicleState, goalID, bEmergencyStop, bGreenTrafficLight, tc);

	PlannerHNS::BehaviorState beh = GenerateBehaviorState(vehicleState);
	m_BehaviorGenTime = UtilityH::GetTimeDiffNow(behTimer);

	timespec t;
	UtilityH::GetTickCount(t);
	beh.bNewPlan = SelectSafeTrajectoryAndSpeedProfile(vehicleState);
	m_RollOutsGenerationTime = UtilityH::GetTimeDiffNow(t);

	if(m_pCurrentBehaviorState->m_pParams->enabTrajectoryVelocities)
	{
		UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);
	}
	else if(beh.bNewPlan == true)
	{
		if(m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory == m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
			PlanningHelpers::GenerateRecommendedSpeed(m_Path, m_pCurrentBehaviorState->m_pParams->maxSpeed, m_pCurrentBehaviorState->m_pParams->speedProfileFactor);
		else
			PlanningHelpers::GenerateRecommendedSpeed(m_Path, m_pCurrentBehaviorState->m_pParams->maxSpeed*0.25, m_pCurrentBehaviorState->m_pParams->speedProfileFactor);
	}

/**
 * Usage of predictive planning
 */
//	timespec predictionTime;
//	UtilityH::GetTickCount(predictionTime);
//	if(UtilityH::GetTimeDiffNow(m_PredictionTimer) > 0.5 || beh.bNewPlan)
//	{
//		CalculateObstacleCosts(map, vehicleState, obj_list);
//		m_PredictionTime = UtilityH::GetTimeDiffNow(predictionTime);
//	}
//	bool bCollision = false;
//	int wp_id = -1;
//	for(unsigned int i=0; i < m_Path.size(); i++)
//	{
//		if(m_Path.at(i).collisionCost > 0)
//		{
//			bCollision = true;
//			wp_id = i;
//			beh.maxVelocity = m_Path.at(i).v;//PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, state, 1.5*vehicleState.speed*3.6);
//			break;
//		}
//	}
//	std::cout << "------------------------------------------------" <<  std::endl;
//	std::cout << "Max Velocity = " << beh.maxVelocity << ", New Plan : " << beh.bNewPlan <<  std::endl;
//	std::cout << "Collision = " << bCollision << ", @ WayPoint : " << wp_id <<  std::endl;
//	std::cout << "------------------------------------------------" <<  std::endl;

	return beh;
 }

} /* namespace PlannerHNS */
