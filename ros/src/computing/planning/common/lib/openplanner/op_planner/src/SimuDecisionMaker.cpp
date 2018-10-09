/// \file  SimuDecisionMaker.cpp
/// \brief Decision Maker for Simulated Vehicles
/// \author Hatem Darweesh
/// \date Jan 10, 2018

#include "op_planner/SimuDecisionMaker.h"
#include "op_utility/UtilityH.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"
#include "op_planner/PlannerH.h"


namespace PlannerHNS
{

SimuDecisionMaker::SimuDecisionMaker()
{
	m_CurrentVelocity =  m_CurrentVelocityD =0;
	m_CurrentSteering = m_CurrentSteeringD =0;
	m_CurrentShift 		=  m_CurrentShiftD = PlannerHNS::SHIFT_POS_NN;
	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;

	m_SimulationSteeringDelayFactor = 0.1;
	UtilityHNS::UtilityH::GetTickCount(m_SteerDelayTimer);
}

SimuDecisionMaker::~SimuDecisionMaker()
{
}

void SimuDecisionMaker::ReInitializePlanner(const WayPoint& start_pose)
{
	m_pidVelocity.Init(0.01, 0.004, 0.01);
	m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

	m_pidStopping.Init(0.05, 0.05, 0.1);
	m_pidStopping.Setlimit(m_params.horizonDistance, 0);

	m_pidFollowing.Init(0.05, 0.05, 0.01);
	m_pidFollowing.Setlimit(m_params.minFollowingDistance, 0);

	m_iCurrentTotalPathId = 0;
	m_CurrentVelocity =  m_CurrentVelocityD =0;
	m_CurrentSteering = m_CurrentSteeringD =0;
	m_CurrentShift 		=  m_CurrentShiftD = SHIFT_POS_NN;
	m_CurrentAccSteerAngle = m_CurrentAccVelocity = 0;

	m_pCurrentBehaviorState = m_pFollowState;
	m_TotalPath.clear();
	m_TotalOriginalPath.clear();
	m_Path.clear();
	m_RollOuts.clear();
	m_pCurrentBehaviorState->m_Behavior = PlannerHNS::FORWARD_STATE;
	FirstLocalizeMe(start_pose);
	LocalizeMe(0);
}

void SimuDecisionMaker::SetSimulatedTargetOdometryReadings(const double& velocity_d, const double& steering_d, const SHIFT_POS& shift_d)
{
	m_CurrentVelocityD = velocity_d;
	m_CurrentSteeringD = steering_d;
	m_CurrentShiftD = shift_d;
}

void SimuDecisionMaker::FirstLocalizeMe(const WayPoint& initCarPos)
 {
	pLane = initCarPos.pLane;
	state = initCarPos;
	m_OdometryState.pos.a = initCarPos.pos.a;
	m_OdometryState.pos.x = initCarPos.pos.x + (m_CarInfo.wheel_base/2.0 * cos(initCarPos.pos.a));
	m_OdometryState.pos.y = initCarPos.pos.y + (m_CarInfo.wheel_base/2.0 * sin(initCarPos.pos.a));
 }

 void SimuDecisionMaker::LocalizeMe(const double& dt)
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
	m_OdometryState.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(m_OdometryState.pos.a);

	state.pos.a = m_OdometryState.pos.a;
	state.pos.x = m_OdometryState.pos.x	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base) * cos (m_OdometryState.pos.a));
	state.pos.y = m_OdometryState.pos.y	 - (m_CurrentVelocity*dt* (m_CarInfo.wheel_base/2.0) * sin (m_OdometryState.pos.a));
}

 void SimuDecisionMaker::UpdateState(const PlannerHNS::VehicleState& state, const bool& bUseDelay)
  {
	 if(!bUseDelay)
	 {
		 m_CurrentSteering 	= m_CurrentSteeringD;
	 }
	 else
	 {
		 double currSteerDeg = RAD2DEG * m_CurrentSteering;
		 double desiredSteerDeg = RAD2DEG * m_CurrentSteeringD;

		 double mFact = UtilityHNS::UtilityH::GetMomentumScaleFactor(state.speed);
		 double diff = desiredSteerDeg - currSteerDeg;
		 double diffSign = UtilityHNS::UtilityH::GetSign(diff);
		 double inc = 1.0*diffSign;
		 if(fabs(diff) < 1.0 )
			 inc = diff;

		 if(UtilityHNS::UtilityH::GetTimeDiffNow(m_SteerDelayTimer) > m_SimulationSteeringDelayFactor*mFact)
		 {
			 UtilityHNS::UtilityH::GetTickCount(m_SteerDelayTimer);
			 currSteerDeg += inc;
		 }

		 m_CurrentSteering = DEG2RAD * currSteerDeg;
	 }

	 m_CurrentShift 	= m_CurrentShiftD;
	 m_CurrentVelocity = m_CurrentVelocityD;
  }

 void SimuDecisionMaker::GenerateLocalRollOuts()
 {
	std::vector<PlannerHNS::WayPoint> sampledPoints_debug;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > _roll_outs;
	PlannerHNS::PlannerH _planner;
	_planner.GenerateRunoffTrajectory(m_TotalPath, state,
						m_params.enableLaneChange,
						state.v,
						m_params.microPlanDistance,
						m_params.maxSpeed,
						m_params.minSpeed,
						m_params.carTipMargin,
						m_params.rollInMargin,
						m_params.rollInSpeedFactor,
						m_params.pathDensity,
						m_params.rollOutDensity,
						m_params.rollOutNumber,
						m_params.smoothingDataWeight,
						m_params.smoothingSmoothWeight,
						m_params.smoothingToleranceError,
						m_params.speedProfileFactor,
						m_params.enableHeadingSmoothing,
						-1 , -1,
						_roll_outs, sampledPoints_debug);

	if(_roll_outs.size()>0)
		m_RollOuts.clear();
	for(unsigned int i=0; i < _roll_outs.size(); i++)
	{
		for(unsigned int j=0; j < _roll_outs.at(i).size(); j++)
		{
			PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(_roll_outs.at(i).at(j), state, m_params.minSpeed, m_params.microPlanDistance);
			m_RollOuts.push_back(_roll_outs.at(i).at(j));
		}
	}
 }

 bool SimuDecisionMaker::SelectSafeTrajectory()
 {
	 bool bNewTrajectory = false;
	 PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	 if(!preCalcPrams || m_TotalPath.size()==0) return bNewTrajectory;

	int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_Path, state);
	int index_limit = 0;
	if(index_limit<=0)
		index_limit =  m_Path.size()/2.0;
	if(currIndex > index_limit
			|| preCalcPrams->bRePlan
			|| preCalcPrams->bNewGlobalPath)
	{
		GenerateLocalRollOuts();
		if(m_RollOuts.size() <= preCalcPrams->iCurrSafeTrajectory)
			return false;

		std::cout << "New Local Plan !! " << currIndex << ", "<< preCalcPrams->bRePlan << ", " << preCalcPrams->bNewGlobalPath  << ", " <<  m_TotalOriginalPath.at(0).size() << ", PrevLocal: " << m_Path.size();
		std::cout << ", NewLocal: " << m_Path.size() << std::endl;

		m_Path = m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);
		preCalcPrams->bNewGlobalPath = false;
		preCalcPrams->bRePlan = false;
		bNewTrajectory = true;
	}

	return bNewTrajectory;
 }

 PlannerHNS::VehicleState SimuDecisionMaker::LocalizeStep(const double& dt, const PlannerHNS::VehicleState& desiredStatus)
 {
	SetSimulatedTargetOdometryReadings(desiredStatus.speed, desiredStatus.steer, desiredStatus.shift);
	UpdateState(desiredStatus, false);
	LocalizeMe(dt);
	PlannerHNS::VehicleState currStatus;
	currStatus.shift = desiredStatus.shift;
	currStatus.steer = m_CurrentSteering;
	currStatus.speed = m_CurrentVelocity;
	return currStatus;
 }

 PlannerHNS::BehaviorState SimuDecisionMaker::DoOneStep(const double& dt,
		const PlannerHNS::VehicleState& vehicleState,
		const int& goalID,
		const std::vector<TrafficLight>& trafficLight,
		const std::vector<PlannerHNS::DetectedObject>& objects,
		const bool& bEmergencyStop)
{
	 PlannerHNS::BehaviorState beh;
	 state.v = vehicleState.speed;
	 m_TotalPath.clear();
	for(unsigned int i = 0; i < m_TotalOriginalPath.size(); i++)
	{
		t_centerTrajectorySmoothed.clear();
		PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_TotalOriginalPath.at(i), state, m_params.horizonDistance ,	m_params.pathDensity , t_centerTrajectorySmoothed);
		m_TotalPath.push_back(t_centerTrajectorySmoothed);
	}

	if(m_TotalPath.size()==0) return beh;

	UpdateCurrentLane(m_MaxLaneSearchDistance);

	PlannerHNS::TrajectoryCost tc = m_TrajectoryCostsCalculator.DoOneStepStatic(m_RollOuts, m_TotalPath.at(m_iCurrentTotalPathId), state,	m_params, m_CarInfo, vehicleState, objects);

	//std::cout << "Detected Objects Distance: " << tc.closest_obj_distance << ", N RollOuts: " << m_RollOuts.size() << std::endl;

	CalculateImportantParameterForDecisionMaking(vehicleState, goalID, bEmergencyStop, trafficLight, tc);

	beh = GenerateBehaviorState(vehicleState);

	beh.bNewPlan = SelectSafeTrajectory();

	beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, vehicleState, dt);

	//std::cout << "Eval_i: " << tc.index << ", Curr_i: " <<  m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ", Prev_i: " << m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << std::endl;

	return beh;
 }
}
