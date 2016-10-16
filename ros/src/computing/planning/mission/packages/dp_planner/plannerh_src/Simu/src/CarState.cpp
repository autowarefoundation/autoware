/*
 * CarState.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: hatem
 */

#include "CarState.h"
#include "UtilityH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include "MatrixOperations.h"
#include "PlannerH.h"
#include "TrajectoryFollower.h"
#include <cmath>

using namespace PlannerHNS;
using namespace UtilityHNS;

namespace SimulationNS
{

CarState::CarState()
{
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
	m_pFollowState = 0;
	m_delayFactor = 0.1;
	UtilityH::GetTickCount(m_SteerDelayTimer);
	UtilityH::GetTickCount(m_PredictionTimer);

	InitBehaviorStates();
}

CarState::~CarState()
{

}

void CarState::Init(const SimulationNS::ControllerParams ctrlParams, const PlannerHNS::PlanningParams& params,const CAR_BASIC_INFO& carInfo)
 	{
 		m_CarInfo = carInfo;
 		m_ControlParams = ctrlParams;
 		m_CurrentVelocity =  m_CurrentVelocityD =0;
 		m_CurrentSteering = m_CurrentSteeringD =0;
 		m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
 		m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;

 		if(m_pCurrentBehaviorState)
 			m_pCurrentBehaviorState->SetBehaviorsParams(params);
 	}

void CarState::InitBehaviorStates()
{

	m_pStopState 				= new StopState(0);
	m_pMissionCompleteState 	= new MissionAccomplishedState(0);
	m_pGoToGoalState 			= new ForwardState(m_pMissionCompleteState);
	m_pWaitState 				= new WaitState(m_pGoToGoalState);
	m_pInitState 				= new InitState(m_pGoToGoalState);
	m_pFollowState				= new FollowState(m_pGoToGoalState);
	m_pAvoidObstacleState		= new SwerveState(m_pGoToGoalState);

	m_pGoToGoalState->InsertNextState(m_pStopState);
	m_pGoToGoalState->InsertNextState(m_pWaitState);
	m_pGoToGoalState->InsertNextState(m_pFollowState);
	m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);

	m_pAvoidObstacleState->InsertNextState(m_pStopState);
	m_pAvoidObstacleState->InsertNextState(m_pWaitState);
	m_pAvoidObstacleState->InsertNextState(m_pFollowState);

	m_pFollowState->InsertNextState(m_pStopState);
	m_pFollowState->InsertNextState(m_pWaitState);
	m_pFollowState->InsertNextState(m_pAvoidObstacleState);

	m_pCurrentBehaviorState = m_pInitState;

}

void CarState::InitPolygons()
{
	double l2 = m_CarInfo.length/2.0;
	double w2 = m_CarInfo.width/2.0;

	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 0,0));
	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 0,0));

//	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 1,0));
//	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 1,0));
//	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 1,0));
//	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 1,0));
}

 void CarState::FirstLocalizeMe(const WayPoint& initCarPos)
 {
	pLane = initCarPos.pLane;
	state = initCarPos;
	m_OdometryState.pos.a = initCarPos.pos.a;
	m_OdometryState.pos.x = initCarPos.pos.x + (m_CarInfo.wheel_base/2.0 * cos(initCarPos.pos.a));
	m_OdometryState.pos.y = initCarPos.pos.y + (m_CarInfo.wheel_base/2.0 * sin(initCarPos.pos.a));
 }

 void CarState::LocalizeMe(const double& dt)
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

 void CarState::UpdateState(const PlannerHNS::VehicleState& state, const bool& bUseDelay)
  {
	 if(!bUseDelay)
	 {
		 m_CurrentSteering 	= m_CurrentSteeringD;
	 }
	 else
	 {
		 double currSteerDeg = RAD2DEG * m_CurrentSteering;
		 double desiredSteerDeg = RAD2DEG * m_CurrentSteeringD;

		 double mFact = 1.0 - UtilityH::GetMomentumScaleFactor(state.speed);
		 double diff = desiredSteerDeg - currSteerDeg;
		 double diffSign = UtilityH::GetSign(diff);
		 double inc = 1.0*diffSign;
		 if(abs(diff) < 1.0 )
			 inc = diff;

		 if(UtilityH::GetTimeDiffNow(m_SteerDelayTimer) > m_delayFactor*mFact)
		 {
			 UtilityH::GetTickCount(m_SteerDelayTimer);
			 currSteerDeg += inc;
		 }

		 m_CurrentSteering = DEG2RAD * currSteerDeg;
	 }

	 m_CurrentShift 	= m_CurrentShiftD;
	 m_CurrentVelocity = m_CurrentVelocityD;
  }

 void CarState::AddAndTransformContourPoints(const PlannerHNS::DetectedObject& obj, std::vector<PlannerHNS::WayPoint>& contourPoints)
 {
	 contourPoints.clear();
	 WayPoint  p;
	 for(unsigned int i=0; i< obj.contour.size(); i++)
	 {
		 p.pos = obj.contour.at(i);
		 TransformPoint(obj.center, p.pos);
		 contourPoints.push_back(p);
	 }

	 contourPoints.push_back(obj.center);
 }

 void CarState::TransformPoint(const PlannerHNS::WayPoint& refPose, PlannerHNS::GPSPoint& p)
 {
 	PlannerHNS::Mat3 rotationMat(refPose.pos.a);
 	PlannerHNS::Mat3 translationMat(refPose.pos.x, refPose.pos.y);
	p = rotationMat*p;
	p = translationMat*p;
 }

 void CarState::CalculateImportantParameterForDecisionMaking(const PlannerHNS::VehicleState& car_state,
		 const PlannerHNS::GPSPoint& goal)
 {
 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	//Mission Complete
 	pValues->bGoalReached = IsGoalAchieved(goal);
 	pValues->minStoppingDistance	= car_state.speed * 3.6 * 1.5;
 	if(pValues->distanceToNext > 0 || pValues->distanceToStop()>0)
 		pValues->minStoppingDistance += 1.0;
 	pValues->iCentralTrajectory		= m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber/2;
 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= car_state.speed;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
 	pValues->bRePlan 				= false;
 	FindSafeTrajectory(pValues->iCurrSafeTrajectory, pValues->distanceToNext, pValues->velocityOfNext);
 	if(pValues->iCurrSafeTrajectory == -1)
 		pValues->bFullyBlock = true;
 	else
 		pValues->bFullyBlock = false;

 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

void CarState::InitializeTrajectoryCosts()
{
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;
	int centralIndex = pParams->rollOutNumber/2;
	std::vector<double> end_distance_list;

	m_TrajectoryCosts.clear();

	//double totalCost = 1.0 / (double)pParams->rollOutNumber;
	double totalDistance = 0;
	for(int i=0; i< pParams->rollOutNumber+1; i++)
	{
		PlannerHNS::TrajectoryCost tc;
		tc.index = i;
		tc.relative_index = i - centralIndex;
		tc.distance_from_center = pParams->rollOutDensity*tc.relative_index;
		tc.priority_cost = fabs(tc.distance_from_center);
		totalDistance += tc.priority_cost;
		m_TrajectoryCosts.push_back(tc);
	}

	if(totalDistance==0) return ;

	//Normalize cost
	for(unsigned int i = 0; i< m_TrajectoryCosts.size(); i++)
	{
		m_TrajectoryCosts.at(i).priority_cost = m_TrajectoryCosts.at(i).priority_cost/totalDistance;
	}
}

void CarState::CalculateTransitionCosts()
{
	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;

	double totalDistance = 0;
	//pValues->iCurrSafeTrajectory = 4;
	if(pValues->iCentralTrajectory < 0)
		pValues->iCentralTrajectory = pParams->rollOutNumber / 2;

	if(pValues->iCurrSafeTrajectory < 0)
		pValues->iCurrSafeTrajectory = pValues->iCentralTrajectory;

	if(pValues->iPrevSafeTrajectory < 0)
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;

	for(unsigned int i = 0; i< m_TrajectoryCosts.size(); i++)
	{
		m_TrajectoryCosts.at(i).transition_cost = fabs(pParams->rollOutDensity* (m_TrajectoryCosts.at(i).index - pValues->iCurrSafeTrajectory));
		totalDistance += m_TrajectoryCosts.at(i).transition_cost;
	}

	if(totalDistance==0) return ;

	//Normalize cost
	for(unsigned int i = 0; i< m_TrajectoryCosts.size(); i++)
	{
		m_TrajectoryCosts.at(i).transition_cost = m_TrajectoryCosts.at(i).transition_cost/totalDistance;
	}
}

double CarState::PredictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::VehicleState& vstatus, const PlannerHNS::WayPoint& currState)
{
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;

		//1- Calculate time prediction for each trajectory
	if(path.size() == 0) return 0;

	SimulationNS::TrajectoryFollower predControl;
	predControl.Init(m_ControlParams, m_CarInfo);
	double totalDistance = 0;
	VehicleState CurrentState = vstatus;
	VehicleState CurrentSteeringD;
	bool bNewPath = true;
	WayPoint localState = currState;
	WayPoint prevState = currState;
	int iPrevIndex = 0;
	double accum_time = 0;
	double pred_max_time = 5;

	while(totalDistance < pParams->microPlanDistance/2.0 && accum_time < pred_max_time)
	{
		double dt = 0.01;
		PlannerHNS::BehaviorState currMessage;
		currMessage.state = FORWARD_STATE;
		currMessage.maxVelocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, state, 1.5*CurrentState.speed*3.6);

		SimulationNS::ControllerParams c_params = m_ControlParams;
		c_params.SteeringDelay = m_ControlParams.SteeringDelay / (1.0-UtilityH::GetMomentumScaleFactor(CurrentState.speed));
		predControl.Init(c_params, m_CarInfo);
		CurrentSteeringD = predControl.DoOneStep(dt, currMessage, path, localState, CurrentState, bNewPath);

		if(bNewPath) // first call
		{
			if(predControl.m_iCalculatedIndex > 0)
			{
				for(unsigned int j=0; j < predControl.m_iCalculatedIndex; j++)
				{
					path.at(j).timeCost = 0;
					path.at(j).collisionCost = 0;
				}
			}
		}
		else
		{
			if(predControl.m_iCalculatedIndex != iPrevIndex && iPrevIndex > 0)
			{
				path.at(iPrevIndex-1).timeCost = accum_time;
				path.at(iPrevIndex-1).collisionCost = 0;
			}
		}



		accum_time+=dt;
		bNewPath = false;

		//Update State
		CurrentState = CurrentSteeringD;

		//Localize Me
		localState.pos.x	 +=  CurrentState.speed * dt * cos(localState.pos.a);
		localState.pos.y	 +=  CurrentState.speed * dt * sin(localState.pos.a);
		localState.pos.a	 +=  CurrentState.speed * dt * tan(CurrentState.steer)  / m_CarInfo.wheel_base;

		totalDistance += distance2points(prevState.pos, localState.pos);

		prevState = localState;
		iPrevIndex = predControl.m_iCalculatedIndex;
	}

	return accum_time;
}

void CarState::PredictObstacleTrajectory(PlannerHNS::RoadNetwork& map, const PlannerHNS::WayPoint& pos, const double& predTime, std::vector<PlannerHNS::WayPoint>& path)
{
	PlannerHNS::PlanningParams planningDefaultParams;
	planningDefaultParams.rollOutNumber = 0;
	planningDefaultParams.microPlanDistance = predTime*pos.v;
	planningDefaultParams.pathDensity = 0.5;
	PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMap(pos, map, 5.0);

	PlannerHNS::PlannerH planner;
	std::vector<int> LanesIds;
	std::vector< std::vector<PlannerHNS::WayPoint> >  rollOuts;
	std::vector<PlannerHNS::WayPoint> generatedPath;

	planner.PlanUsingDP(pMapLane, pos, PlannerHNS::WayPoint(),
			pos, 150, LanesIds, generatedPath);

	planner.GenerateRunoffTrajectory(generatedPath, pos,
			planningDefaultParams.enableLaneChange,
			pos.v,
			planningDefaultParams.microPlanDistance,
			m_CarInfo.max_speed_forward,
			planningDefaultParams.minSpeed,
			planningDefaultParams.carTipMargin,
			planningDefaultParams.rollInMargin,
			planningDefaultParams.rollInSpeedFactor,
			planningDefaultParams.pathDensity,
			planningDefaultParams.rollOutDensity,
			planningDefaultParams.rollOutNumber,
			planningDefaultParams.smoothingDataWeight,
			planningDefaultParams.smoothingSmoothWeight,
			planningDefaultParams.smoothingToleranceError,
			planningDefaultParams.speedProfileFactor,
			planningDefaultParams.enableHeadingSmoothing,
			rollOuts);

	if(rollOuts.size() > 0)
	{
		path = rollOuts.at(0);

//		PlanningHelpers::GenerateRecommendedSpeed(path,
//				m_CarInfo.max_speed_forward,
//				planningDefaultParams.speedProfileFactor);
//		PlanningHelpers::SmoothSpeedProfiles(path, 0.15,0.35, 0.1);
	}

	if(!pMapLane || generatedPath.size() < 3 || path.size() < 3)
	{
		path.clear();
		generatedPath.clear();
	}
	else
	{
		double timeDelay = 0;
		path.at(0).timeCost = 0;
		path.at(0).v = pos.v;
		for(unsigned int i=1; i<path.size(); i++)
		{
			path.at(i).v = pos.v;
			path.at(i).pos.a = atan2(path.at(i).pos.y - path.at(i-1).pos.y, path.at(i).pos.x - path.at(i-1).pos.x);
			timeDelay += planningDefaultParams.pathDensity/pos.v;
			path.at(i).timeCost = timeDelay;
		}
	}
}

void CarState::CalculateIntersectionVelocities(std::vector<PlannerHNS::WayPoint>& path, std::vector<PlannerHNS::WayPoint>& predctedPath, const PlannerHNS::DetectedObject& obj)
{
	for(unsigned int i = 0; i < path.size(); i++)
	{
		for(unsigned int j = 0; j < predctedPath.size(); j++)
		{
			double rd = distance2points(path.at(i).pos, predctedPath.at(j).pos);
			if(rd < m_CarInfo.width+obj.w)
			{
				path.at(i).collisionCost = 1;
				double a = UtilityH::AngleBetweenTwoAnglesPositive(path.at(i).pos.a, predctedPath.at(j).pos.a);
				if(a < M_PI_2)
				{
					path.at(i).v = predctedPath.at(j).v;
				}
				else
				{
					path.at(i).v = 0;
				}

				return;
			}
		}
	}
}

void CarState::CalculateObstacleCosts(PlannerHNS::RoadNetwork& map, const PlannerHNS::VehicleState& vstatus, const std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	double predTime = PredictTimeCostForTrajectory(m_Path, vstatus, state);
	for(unsigned int i = 0; i < obj_list.size(); i++)
	{
		std::vector<WayPoint> predPath;
		PredictObstacleTrajectory(map, obj_list.at(i).center, predTime, predPath);
		CalculateIntersectionVelocities(m_Path, predPath, obj_list.at(i));
	}
}

void CarState::CalculateDistanceCosts(const PlannerHNS::VehicleState& vstatus, const std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;
	double critical_lateral_distance = pParams->rollOutDensity/2.0 + m_CarInfo.width/2.0;

	if(m_TotalPath.size()==0) return;

	//First Filtering
	int iEgoIndex = PlanningHelpers::GetClosestPointIndex(m_TotalPath, state);
	for(unsigned int i = 0 ; i < obj_list.size(); i++)
	{
		std::vector<WayPoint> contourPoints;
		AddAndTransformContourPoints(obj_list.at(i), contourPoints);

		double distance_direct_smallest = 9999999;
		double distance_on_trajectory_smallest = 9999999;
		for(unsigned int j = 0; j < contourPoints.size(); j++)
		{
			double distance_direct = distance2points(state.pos, contourPoints.at(j).pos);
			if(distance_direct < distance_direct_smallest)
				distance_direct_smallest = distance_direct;

			double distance_on_trajectory  = PlanningHelpers::GetDistanceOnTrajectory(m_TotalPath, iEgoIndex, contourPoints.at(j)) - m_CarInfo.length/2.0;
			if(distance_on_trajectory > 0 && distance_on_trajectory < distance_on_trajectory_smallest)
				distance_on_trajectory_smallest = distance_on_trajectory;
		}

		for(unsigned int j = 0; j < contourPoints.size(); j++)
		{
			PlannerHNS::WayPoint wp;
			wp = contourPoints.at(j);

			double distance_lateral = PlanningHelpers::GetPerpDistanceToTrajectorySimple(m_TotalPath, wp);

//			if(distance_direct > pParams->horizonDistance)
//				continue;

			for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
			{
				double normalized_cost = 1.0 - (distance_on_trajectory_smallest / pParams->horizonDistance);
				double d_diff = fabs(distance_lateral - m_TrajectoryCosts.at(c).distance_from_center);
				m_TrajectoryCosts.at(c).lateral_costs.push_back(std::make_pair(j,d_diff));

				if(d_diff < critical_lateral_distance && (distance_on_trajectory_smallest < m_TrajectoryCosts.at(c).closest_obj_distance || m_TrajectoryCosts.at(c).closest_obj_distance <= 0))
				{
					//if(normalized_cost > m_TrajectoryCosts.at(c).closest_obj_cost)
					{
						m_TrajectoryCosts.at(c).closest_obj_cost = normalized_cost;
						m_TrajectoryCosts.at(c).closest_obj_distance = distance_on_trajectory_smallest;
						m_TrajectoryCosts.at(c).closest_obj_velocity = obj_list.at(i).center.v;

					}
				}
			}
		}
	}
}

void  CarState::FindSafeTrajectory(int& safe_index, double& closest_distance, double& closest_velocity)
{
	PlanningParams* pParams = &m_pCurrentBehaviorState->m_PlanningParams;

	//if the  closest_obj_cost is less than 0.9 (12 meter) consider this trajectory blocked
	closest_distance = pParams->horizonDistance;
//	std::cout << "----------------------------------------------------" << std::endl;
//	std::cout << ">> Costs: " << std::endl;
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
//		std::cout << m_TrajectoryCosts.at(c).ToString() << std::endl;

		if(m_TrajectoryCosts.at(c).closest_obj_cost >= 0.8)
			m_TrajectoryCosts.at(c).cost = 1;
		else
			m_TrajectoryCosts.at(c).cost =
					(m_TrajectoryCosts.at(c).closest_obj_cost +
					m_TrajectoryCosts.at(c).priority_cost +
					m_TrajectoryCosts.at(c).transition_cost)/3.0;

		if(m_TrajectoryCosts.at(c).closest_obj_distance > 0 && m_TrajectoryCosts.at(c).closest_obj_distance < closest_distance)
		{
			closest_distance = m_TrajectoryCosts.at(c).closest_obj_distance;
			closest_velocity = m_TrajectoryCosts.at(c).closest_obj_velocity;
		}
	}

	int smallestIndex = -1;
	double smallestCost = 1;
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
		if(m_TrajectoryCosts.at(c).cost < smallestCost)
		{
			smallestCost = m_TrajectoryCosts.at(c).cost;
			smallestIndex = c;
		}
	}

	safe_index = smallestIndex;

//	std::cout << "Selected Trajectory: " << safe_index << ", Closest Distance: " << closest_distance << std::endl;
//	std::cout << "----------------------------------------------------" << std::endl;
}

void CarState::FindNextBestSafeTrajectory(int& safe_index)
{
	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
		m_TrajectoryCosts.at(c).cost =
				(m_TrajectoryCosts.at(c).priority_cost +
				m_TrajectoryCosts.at(c).transition_cost)/2.0;
	}

	int smallestIndex = pValues->iCentralTrajectory;
	double smallestCost = 1;
	for(unsigned int c = 0; c < m_TrajectoryCosts.size(); c++)
	{
		if(m_TrajectoryCosts.at(c).cost < smallestCost)
		{
			smallestCost = m_TrajectoryCosts.at(c).cost;
			smallestIndex = c;
		}
	}

	safe_index = smallestIndex;
}

 void CarState::UpdateCurrentLane(PlannerHNS::RoadNetwork& map, const double& search_distance)
 {
	PlannerHNS::Lane* pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
	PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, search_distance);

	if(pPathLane)
		pLane = pPathLane;
	else if(pMapLane)
		pLane = pMapLane;
	else
		pLane = 0;
 }

 void CarState::SimulateOdoPosition(const double& dt, const PlannerHNS::VehicleState& vehicleState)
 {
	SetSimulatedTargetOdometryReadings(vehicleState.speed, vehicleState.steer, vehicleState.shift);
	UpdateState(vehicleState, true);
	LocalizeMe(dt);
 }

 bool CarState::IsGoalAchieved(const PlannerHNS::GPSPoint& goal)
 {
	double distance_to_goal = distance2points(state.pos , goal);
	if(distance_to_goal < 1.5)
		return true;
	else
		return false;
 }

 bool CarState::SelectSafeTrajectoryAndSpeedProfile(const PlannerHNS::VehicleState& vehicleState)
 {
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	bool bNewTrajectory = false;
	if(m_TotalPath.size()>0)
	{
		int currIndex = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_Path, state);
		int index_limit = 0;//m_Path.size() - 20;
		if(index_limit<=0)
			index_limit =  m_Path.size()/2.0;
		if(m_RollOuts.size() == 0
				|| currIndex > index_limit
				|| m_pCurrentBehaviorState->GetCalcParams()->bRePlan
				|| m_pCurrentBehaviorState->m_Behavior == OBSTACLE_AVOIDANCE_STATE)
		{
			PlannerHNS::PlannerH planner;
			planner.GenerateRunoffTrajectory(m_TotalPath, state,
					m_pCurrentBehaviorState->m_PlanningParams.enableLaneChange,
					vehicleState.speed,
					m_pCurrentBehaviorState->m_PlanningParams.microPlanDistance,
					m_pCurrentBehaviorState->m_PlanningParams.maxSpeed,
					m_pCurrentBehaviorState->m_PlanningParams.minSpeed,
					m_pCurrentBehaviorState->m_PlanningParams.carTipMargin,
					m_pCurrentBehaviorState->m_PlanningParams.rollInMargin,
					m_pCurrentBehaviorState->m_PlanningParams.rollInSpeedFactor,
					m_pCurrentBehaviorState->m_PlanningParams.pathDensity,
					m_pCurrentBehaviorState->m_PlanningParams.rollOutDensity,
					m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber,
					m_pCurrentBehaviorState->m_PlanningParams.smoothingDataWeight,
					m_pCurrentBehaviorState->m_PlanningParams.smoothingSmoothWeight,
					m_pCurrentBehaviorState->m_PlanningParams.smoothingToleranceError,
					m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor,
					m_pCurrentBehaviorState->m_PlanningParams.enableHeadingSmoothing,
					m_RollOuts);

			m_pCurrentBehaviorState->GetCalcParams()->bRePlan = false;

			//FindNextBestSafeTrajectory(pValues->iCurrSafeTrajectory);
			if(preCalcPrams->iCurrSafeTrajectory >= 0
					&& preCalcPrams->iCurrSafeTrajectory < m_RollOuts.size()
					&& m_pCurrentBehaviorState->m_Behavior == OBSTACLE_AVOIDANCE_STATE)
			{
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCurrSafeTrajectory;
				m_Path = m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
				bNewTrajectory = true;
			}
			else
			{
				preCalcPrams->iPrevSafeTrajectory = preCalcPrams->iCentralTrajectory;
				m_Path = m_RollOuts.at(preCalcPrams->iCentralTrajectory);
				bNewTrajectory = true;
			}

			PlanningHelpers::GenerateRecommendedSpeed(m_Path,
					m_pCurrentBehaviorState->m_PlanningParams.maxSpeed,
					m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor);
			PlanningHelpers::SmoothSpeedProfiles(m_Path, 0.15,0.35, 0.1);
			std::ostringstream str_out;
			str_out << UtilityH::GetHomeDirectory();
			str_out << DataRW::LoggingMainfolderName;
			str_out << DataRW::PathLogFolderName;
			str_out << "_";
			PlanningHelpers::WritePathToFile(str_out.str(), m_Path);
//			}
//			else if(m_RollOuts.size() > 0)
//				std::cout << "Error .. Error .. Slected Trajectory is out of range !! ( " << preCalcPrams->iCurrSafeTrajectory << ")" << std::endl;

		}
	}

	return bNewTrajectory;
 }

 PlannerHNS::BehaviorState CarState::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
 {
	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	if(currentBehavior.state == PlannerHNS::FOLLOW_STATE)
		currentBehavior.followDistance = preCalcPrams->distanceToNext;
	else
		currentBehavior.followDistance = 0;

	if(preCalcPrams->bUpcomingRight)
		currentBehavior.indicator = PlannerHNS::INDICATOR_RIGHT;
	else if(preCalcPrams->bUpcomingLeft)
		currentBehavior.indicator = PlannerHNS::INDICATOR_LEFT;
	else
		currentBehavior.indicator = PlannerHNS::INDICATOR_NONE;

	currentBehavior.maxVelocity 	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, state, 1.5*vehicleState.speed*3.6);
	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;

	return currentBehavior;
 }

 PlannerHNS::BehaviorState CarState::DoOneStep(const double& dt,
		 const PlannerHNS::VehicleState& vehicleState,
		 const std::vector<PlannerHNS::DetectedObject>& obj_list,
		 const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map	,const bool& bLive)
{
	 if(!bLive)
		 SimulateOdoPosition(dt, vehicleState);

	UpdateCurrentLane(map, 3.0);

	InitializeTrajectoryCosts();

	CalculateTransitionCosts();

	CalculateDistanceCosts(vehicleState, obj_list);

	CalculateImportantParameterForDecisionMaking(vehicleState, goal);

	PlannerHNS::BehaviorState beh = GenerateBehaviorState(vehicleState);

	beh.bNewPlan = SelectSafeTrajectoryAndSpeedProfile(vehicleState);


	if(UtilityH::GetTimeDiffNow(m_PredictionTimer) > 1.0)
	{
		CalculateObstacleCosts(map, vehicleState, obj_list);
		//double predTime = PredictTimeCostForTrajectory(m_Path, vehicleState, state);
		UtilityH::GetTickCount(m_PredictionTimer);
	}

	return beh;
 }


 SimulatedCarState::SimulatedCarState()
 {
 	pLane = 0;
 	m_CurrentVelocity =  m_CurrentVelocityD =0;
 	m_CurrentSteering = m_CurrentSteeringD =0;
 	m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
 	bDetected = false;
 }

 SimulatedCarState::~SimulatedCarState()
 {
 }

 void SimulatedCarState::Init(const CAR_BASIC_INFO& carInfo)
{
	m_CarInfo = carInfo;
	m_CurrentVelocity =  m_CurrentVelocityD =0;
	m_CurrentSteering = m_CurrentSteeringD =0;
	m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
}


 void SimulatedCarState::InitPolygons()
 {
 	double l2 = m_CarInfo.length/2.0;
 	double w2 = m_CarInfo.width/2.0;

 	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 0,0));
 	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 0,0));
 	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 0,0));
 	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 0,0));

// 	m_CarShapePolygon.push_back(GPSPoint(-w2, -l2, 1,0));
// 	m_CarShapePolygon.push_back(GPSPoint(w2, -l2, 1,0));
// 	m_CarShapePolygon.push_back(GPSPoint(w2, l2, 1,0));
// 	m_CarShapePolygon.push_back(GPSPoint(-w2, l2, 1,0));
 }

  void SimulatedCarState::FirstLocalizeMe(const WayPoint& initCarPos)
  {
 	pLane = initCarPos.pLane;
 	state = initCarPos;
 	m_OdometryState.pos.a = initCarPos.pos.a;
 	m_OdometryState.pos.x = initCarPos.pos.x + (m_CarInfo.wheel_base/2.0 * cos(initCarPos.pos.a));
 	m_OdometryState.pos.y = initCarPos.pos.y + (m_CarInfo.wheel_base/2.0 * sin(initCarPos.pos.a));
  }

  void SimulatedCarState::LocalizeMe(const double& dt)
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

  void SimulatedCarState::UpdateState(const bool& bUseDelay)
   {
 	 m_CurrentSteering 	= m_CurrentSteeringD;
 	 m_CurrentShift 	= m_CurrentShiftD;
 	 m_CurrentVelocity = m_CurrentVelocityD;
   }

  void SimulatedCarState::CalculateImportantParameterForDecisionMaking(const std::vector<PlannerHNS::DetectedObject>& obj_list,
 		 const PlannerHNS::VehicleState& car_state, const PlannerHNS::GPSPoint& goal, PlannerHNS::RoadNetwork& map)
  {

  	PlannerHNS::Lane* pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
  	PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, 3.0);

  	if(pPathLane)
  		pLane = pPathLane;
  	else if(pMapLane)
  		pLane = pMapLane;
  	else
  		pLane = 0;

  	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;

  }

  void SimulatedCarState::SimulateOdoPosition(const double& dt, const PlannerHNS::VehicleState& vehicleState)
  {
 	SetSimulatedTargetOdometryReadings(vehicleState.speed, vehicleState.steer, vehicleState.shift);
 	UpdateState(false);
 	LocalizeMe(dt);
  }

  void SimulatedCarState::UpdateCurrentLane(PlannerHNS::RoadNetwork& map, const double& search_distance)
  {
 	PlannerHNS::Lane* pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
 	PlannerHNS::Lane* pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, map, search_distance);

 	if(pPathLane)
 		pLane = pPathLane;
 	else if(pMapLane)
 		pLane = pMapLane;
 	else
 		pLane = 0;
  }

PlannerHNS::BehaviorState SimulatedCarState::GenerateBehaviorState(const PlannerHNS::VehicleState& vehicleState)
{
	PlannerHNS::BehaviorState currentBehavior;
	currentBehavior.state = PlannerHNS::FORWARD_STATE;

	/**
	 * Use for future simulation of other detecing other cars indicator and act accordingly
	 */
	//    	if(preCalcPrams->bUpcomingRight)
	//    		currentBehavior.indicator = PlannerHNS::INDICATOR_RIGHT;
	//    	else if(preCalcPrams->bUpcomingLeft)
	//    		currentBehavior.indicator = PlannerHNS::INDICATOR_LEFT;
	//    	else
	//    		currentBehavior.indicator = PlannerHNS::INDICATOR_NONE;

	currentBehavior.maxVelocity 	= PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, state, 1.5*vehicleState.speed*3.6);
	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= 0;
	currentBehavior.followVelocity 	= 0;

	return currentBehavior;
}

bool SimulatedCarState::SelectSafeTrajectoryAndSpeedProfile(const PlannerHNS::VehicleState& vehicleState)
{
	PlannerHNS::PlanningParams planningDefaultParams;
	planningDefaultParams.rollOutNumber = 0;

	int currIndex = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_Path, state);
	int index_limit = 0;//m_Path.size() - 15;
	if(index_limit<=0)
		index_limit =  m_Path.size()/2.0;
	if(m_RollOuts.size() == 0 || currIndex > index_limit)
	{
		PlannerHNS::PlannerH planner;
		std::vector<int> LanesIds;

		std::vector<PlannerHNS::WayPoint> generatedPath;
		planner.PlanUsingDP(pLane, state, PlannerHNS::WayPoint(),
				state, 150, LanesIds, generatedPath);
		m_RollOuts.clear();
		m_TotalPath = generatedPath;


		planner.GenerateRunoffTrajectory(m_TotalPath, state,
				planningDefaultParams.enableLaneChange,
				vehicleState.speed,
				planningDefaultParams.microPlanDistance,
				m_CarInfo.max_speed_forward,
				planningDefaultParams.minSpeed,
				planningDefaultParams.carTipMargin,
				planningDefaultParams.rollInMargin,
				planningDefaultParams.rollInSpeedFactor,
				planningDefaultParams.pathDensity,
				planningDefaultParams.rollOutDensity,
				planningDefaultParams.rollOutNumber,
				planningDefaultParams.smoothingDataWeight,
				planningDefaultParams.smoothingSmoothWeight,
				planningDefaultParams.smoothingToleranceError,
				planningDefaultParams.speedProfileFactor,
				planningDefaultParams.enableHeadingSmoothing,
				m_RollOuts);

		if(m_RollOuts.size() > 0)
		{
			m_Path = m_RollOuts.at(0);
			PlanningHelpers::GenerateRecommendedSpeed(m_Path,
					m_CarInfo.max_speed_forward,
								planningDefaultParams.speedProfileFactor);
			PlanningHelpers::SmoothSpeedProfiles(m_Path, 0.15,0.35, 0.1);
		}
	}

	if(!pLane || m_TotalPath.size() < 3 || m_Path.size() < 3)
	{
		m_Path.clear();
		m_TotalPath.clear();
		return false;
	}

	return m_Path.size() > 0;
}

  PlannerHNS::BehaviorState SimulatedCarState::DoOneStep(
		  const double& dt,
		  const PlannerHNS::VehicleState& vehicleState,
		  const PlannerHNS::WayPoint& currPose,
		  const PlannerHNS::GPSPoint& goal,
		  PlannerHNS::RoadNetwork& map)
{


	SimulateOdoPosition(dt, vehicleState);

	UpdateCurrentLane(map, 3.0);

	PlannerHNS::BehaviorState beh = GenerateBehaviorState(vehicleState);

	beh.bNewPlan = SelectSafeTrajectoryAndSpeedProfile(vehicleState);

	return beh;
}

} /* namespace SimulationNS */
