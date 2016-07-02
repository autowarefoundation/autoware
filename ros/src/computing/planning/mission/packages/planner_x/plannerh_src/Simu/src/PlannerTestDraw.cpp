/*
 * PlannerTestDraw.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#include "PlannerTestDraw.h"
#include "PlannerH.h"
#include "TrajectoryFollower.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"

using namespace std;
using namespace PlannerHNS;
using namespace SimulationNS;
using namespace UtilityHNS;

#define VectorMapPath "/home/hatem/workspace/Data/VectorMap/"
#define kmlTemplateFile "/home/hatem/workspace/Data/templates/PlannerX_MapTemplate.kml"

namespace Graphics
{

PlannerTestDraw::PlannerTestDraw()
{
	planning_mutex =  PTHREAD_MUTEX_INITIALIZER;
	behaviors_mutex = PTHREAD_MUTEX_INITIALIZER;
	control_mutex = PTHREAD_MUTEX_INITIALIZER;

	planning_thread_tid = 0;
	control_thread_tid = 0;
	m_bCancelThread = false;
	m_PlanningCycleTime = 0.01;
	m_ControlCycleTime  = 0.01;

	m_CarModel = 0;

	MappingHelpers::ConstructRoadNetworkFromDataFiles(VectorMapPath, m_RoadMap);

//	ostringstream fileName;
//	fileName << UtilityH::GetFilePrefixHourMinuteSeconds();
//	fileName << "_RoadNetwork.kml";
//	MappingHelpers::WriteKML(fileName.str(),kmlTemplateFile , m_RoadMap);

	m_pMap = new GridMap(0,0,60,60,1.0, true);

	CAR_BASIC_INFO carInfo;
	m_State.Init(1.9, 4.2,carInfo);


	m_start = WayPoint(20, 2, 0, M_PI);
	m_start.bDir = FORWARD;

	m_State.FirstLocalizeMe(m_start);
	m_State.InitPolygons();

	m_goal = WayPoint(2, 40, 0, M_PI_2);
	m_goal.bDir = FORWARD;

	m_followX = m_start.pos.x;
	m_followY = m_start.pos.y;
	m_followZ = m_start.pos.z;
	m_followA = m_start.pos.a;

	m_bNewPath = false;

	pthread_create(&planning_thread_tid, NULL, &PlannerTestDraw::PlanningThreadStaticEntryPoint, this);
	pthread_create(&control_thread_tid, NULL, &PlannerTestDraw::ControlThreadStaticEntryPoint, this);
}

void PlannerTestDraw::LoadMaterials()
{
	if(!m_CarModel)
		m_CarModel = DrawingHelpers::LoadModel("libs/data/porsche.obj");
}

PlannerTestDraw::~PlannerTestDraw()
{
	m_bCancelThread = true;
	PlannerTestDraw* pRet = 0;
	pthread_join(planning_thread_tid, (void**)&pRet);
	pthread_join(control_thread_tid, (void**)&pRet);

	if(m_pMap)
	{
		delete m_pMap;
		m_pMap = 0;
	}
}

void PlannerTestDraw::Reset()
{
//	pthread_mutex_lock(&planning_mutex);
//	m_State.m_Path.clear();
//	m_State.m_RollOuts.clear();
//	pthread_mutex_unlock(&planning_mutex);
//
//	m_followX = m_start.pos.x;
//	m_followY = m_start.pos.y;
//	m_followZ = m_start.pos.z;
//	m_followA = m_start.pos.a;
//
//	m_bNewPath = false;
//
//	m_State.state = m_start;
//
//	pthread_mutex_lock(&behaviors_mutex);
//	m_CurrentBehavior.state = INITIAL_STATE ;
//	pthread_mutex_unlock(&behaviors_mutex);
}

void PlannerTestDraw::PrepareVectorMapForDrawing()
{
//	double distance_to_nearest_lane= 1;
//	int j=0;
//	int max_number_of_lanes = 500;
//	double width_ratio = 2.0;
//
//	vector<Lane*> currLane;
//	vector<Lane*> lanes_list;
//	vector<Lane*> traversed_lanes;
//
//	while(distance_to_nearest_lane < 500 && currLane.size() == 0)
//	{
//		currLane = m_pRoadNetwork->getLane(point, distance_to_nearest_lane);
//		distance_to_nearest_lane += 5;
//	}
//
//	for(unsigned int i=0; i< currLane.size(); i++)
//		lanes_list.push_back(currLane[i]);
//
//	m_ReadyToDrawLanes.clear();
//	m_ReadyToDrawCenterLines.clear();
//	m_TrafficLights.clear();
//	m_StopSigns.clear();
//	m_OtherSigns.clear();
//
//	if(currLane.size() > 0)
//	{
//		vector<vector<Vector2D> > ready_to_draw;
//
//		while(lanes_list.size()>0 && j <max_number_of_lanes)
//		{
//			ready_to_draw.clear();
//			Lane* l = lanes_list[0];
//
//
//			vector<Vector2D> path_local;
//			MappingHelpers::GetLanePointsCart(l, path_local);
//
//			//if(l->id.compare("89")==0 || l->id.compare("92")==0 || l->id.compare("94")==0 ||l->id.compare("96")==0)
//			{
//				OpenGlUtil::PreparePathForDrawing(path_local,ready_to_draw, l->width / width_ratio, 2);
//				m_ReadyToDrawLanes.push_back(ready_to_draw);
//
//				ready_to_draw.clear();
//				OpenGlUtil::PreparePathForDrawing(path_local,ready_to_draw, 0.1, 2);
//				m_ReadyToDrawCenterLines.push_back(ready_to_draw);
//			}
//
//			j++;
//
//			MappingHelpers::GetUniqueNextLanes(l, &lanes_list, &traversed_lanes);
//
//			//Get the traffic lights pos
//
//			for(unsigned int itl = 0; itl < l->trafficLights.size(); itl++)
//			{
//
//				Vector2D p(l->trafficLights[itl]->position.x, l->trafficLights[itl]->position.y, l->trafficLights[itl]->position.z,0);
//				int iNextIndex = MappingHelpers::GetClosestNextPointIndex(path_local, p);
//				if(iNextIndex > 0)
//					p.a = MathUtil::AngleBetweenVectorsPositive(path_local.at(iNextIndex), path_local.at(iNextIndex-1));
//				else
//					p.a = MathUtil::AngleBetweenVectorsPositive(path_local.at(iNextIndex+1), path_local.at(iNextIndex));
//
//				m_TrafficLights.push_back(p);
//			}
//
//			for(unsigned int itl = 0; itl < l->roadSigns.size(); itl++)
//			{
//				Vector2D p(l->roadSigns[itl]->position.x, l->roadSigns[itl]->position.y, l->roadSigns[itl]->position.z,0);
//				int iNextIndex = MappingHelpers::GetClosestNextPointIndex(path_local, p);
//				if(iNextIndex > 0)
//					p.a = MathUtil::AngleBetweenVectorsPositive(path_local.at(iNextIndex), path_local.at(iNextIndex-1));
//				else
//					p.a = MathUtil::AngleBetweenVectorsPositive(path_local.at(iNextIndex+1), path_local.at(iNextIndex));
//
//				if(l->roadSigns[itl]->signtype == STOP_SIGN)
//					m_StopSigns.push_back(p);
//				else if(l->roadSigns[itl]->signtype == STOP_LINE)
//				{
//					vector<Vector2D> path_temp;
//					for(unsigned int k=0; k< l->roadSigns[itl]->points.size(); k++)
//					{
//						path_temp.push_back(Vector2D(l->roadSigns[itl]->points.at(k).x, l->roadSigns[itl]->points.at(k).y, l->roadSigns[itl]->points.at(k).z,p.a));
//					}
//					m_StopLines.push_back(path_temp);
//				}
//				else
//					m_OtherSigns.push_back(p);
//			}
//		}
//	}
}

void PlannerTestDraw::DrawVectorMap()
{

}

void PlannerTestDraw::DrawSimu()
{

	float PlannedPathColor[3] = {0.1, 0.9, 0.1};
	float ActualPathColor[3] = {0.1, 0.1, 0.9};

	DrawingHelpers::DrawWidePath(m_State.m_Path, 0.1, 0.25, PlannedPathColor);
	DrawingHelpers::DrawWidePath(m_ActualPath, 0.11, 0.15, ActualPathColor);

	float RollOutsColor[3] = {0.9, 0.2, 0.1};
	for(unsigned int i=0; i < m_State.m_RollOuts.size();i++)
	{
		DrawingHelpers::DrawWidePath(m_State.m_RollOuts.at(i), 0.2, 0.1, RollOutsColor);
	}

	glColor3f(1,0,0);
	DrawingHelpers::DrawFilledEllipse(m_FollowPoint.x, m_FollowPoint.y, 0.2, 0.2, 0.2);

	glColor3f(0,0,1);
	DrawingHelpers::DrawFilledEllipse(m_PerpPoint.x, m_PerpPoint.y, 0.2, 0.2, 0.2);

	if(m_pMap)
		DrawingHelpers::DrawGrid(m_pMap->origin_x, m_pMap->origin_y, m_pMap->w, m_pMap->h, m_pMap->cell_l);


	m_followX = m_State.state.pos.x;
	m_followY = m_State.state.pos.y;
	m_followZ = m_State.state.pos.z;
	m_followA = m_State.state.pos.a;

	if(m_CarModel)
	{
		DrawingHelpers::DrawModel(m_CarModel, m_State.m_CarInfo.wheel_base *0.9,
				m_State.m_CarInfo.wheel_base*0.9, m_State.m_CarInfo.wheel_base*0.9,
				m_State.m_OdometryState.pos.x,m_State.m_OdometryState.pos.y,
				m_State.m_OdometryState.pos.z+0.275, m_State.m_OdometryState.pos.a, 0,0);
	}
	float CarColor[3] = {0.1, 0.9, 0.9};
	DrawingHelpers::DrawCustomCarModel(m_State.state, m_State.m_CarShapePolygon, CarColor, 90);

}

void PlannerTestDraw::DrawInfo()
{}

void PlannerTestDraw::OnLeftClick(const double& x, const double& y)
{}

void PlannerTestDraw::OnRightClick(const double& x, const double& y)
{}

void PlannerTestDraw::OnKeyboardPress(const SPECIAL_KEYS_TYPE& sKey, const unsigned char& key)
{
	//std::cout << "key" << std::endl;

	switch(key)
	{
	case 's':
		if(m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl == 0)
			m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 1;
		else if(m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl == 1)
			m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 0;
		break;
	default:
		break;

	}
}

void* PlannerTestDraw::PlanningThreadStaticEntryPoint(void* pThis)
{
	PlannerTestDraw* pR = (PlannerTestDraw*)pThis;
	struct timespec moveTimer;
	UtilityH::GetTickCount(moveTimer);
	vector<string> logData;
	PlanningInternalParams params;
	PlannerH planner(params);
	vector<Obstacle> dummyObstacles;


	while(!pR->m_bCancelThread)
	{
		double time_elapsed  = UtilityH::GetTimeDiffNow(moveTimer);

		if(time_elapsed >= pR->m_PlanningCycleTime)
		{
			UtilityH::GetTickCount(moveTimer);


			pthread_mutex_lock(&pR->control_mutex);
			pR->m_State.SetSimulatedTargetOdometryReadings(pR->m_VehicleState.speed, pR->m_VehicleState.steer, pR->m_VehicleState.shift);
			pthread_mutex_unlock(&pR->control_mutex);


			pR->m_State.UpdateState(false);
			pR->m_State.LocalizeMe(time_elapsed);
			pR->m_State.CalculateImportantParameterForDecisionMaking(dummyObstacles, pR->m_VehicleState, pR->m_goal.pos);
			pR->m_State.m_pCurrentBehaviorState = pR->m_State.m_pCurrentBehaviorState->GetNextState();

			pthread_mutex_lock(&pR->behaviors_mutex);

			PreCalculatedConditions *preCalcPrams = pR->m_State.m_pCurrentBehaviorState->GetCalcParams();

			pR->m_CurrentBehavior.state = pR->m_State.m_pCurrentBehaviorState->m_Behavior;
			if(pR->m_CurrentBehavior.state == FOLLOW_STATE)
				pR->m_CurrentBehavior.followDistance = preCalcPrams->distanceToNext;
			else
				pR->m_CurrentBehavior.followDistance = 0;

			if(preCalcPrams->bUpcomingRight)
				pR->m_CurrentBehavior.indicator = INDICATOR_RIGHT;
			else if(preCalcPrams->bUpcomingLeft)
				pR->m_CurrentBehavior.indicator = INDICATOR_LEFT;
			else
				pR->m_CurrentBehavior.indicator = INDICATOR_NONE;

			//TODO fix this , make get lookahead velocity work
			double max_velocity = 2; //BehaviorsNS::MappingHelpers::GetLookAheadVelocity(m_CarState.m_Path, GetCarPos(), 25);

			pR->m_CurrentBehavior.maxVelocity   = max_velocity;
			pR->m_CurrentBehavior.minVelocity	= 0;
			pR->m_CurrentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
			pR->m_CurrentBehavior.followVelocity = preCalcPrams->velocityOfNext;

			pthread_mutex_unlock(&pR->behaviors_mutex);

			if(pR->m_CurrentBehavior.state == INITIAL_STATE && pR->m_State.m_Path.size() == 0)
			{
				vector<WayPoint> generatedPath;
				planner.PlanUsingReedShepp(pR->m_State.state, pR->m_goal, generatedPath);
				planner.GenerateRunoffTrajectory(generatedPath, pR->m_State.state, false, 5, 20, 10, 0,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.carTipMargin,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.rollInMargin,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.rollInSpeedFactor,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.pathDensity,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.rollOutDensity,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingDataWeight,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingSmoothWeight,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingToleranceError,
						pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor,
						false, pR->m_State.m_RollOuts);


				if(generatedPath.size()> 0 )
					pR->m_bNewPath = true;

				cout << "Generated Path Length = " << generatedPath.size();

				pthread_mutex_lock(&pR->planning_mutex);
				pR->m_State.m_Path = generatedPath;
				pthread_mutex_unlock(&pR->planning_mutex);
			}

			if(pR->m_ActualPath.size() > 0 && distance2points(pR->m_ActualPath.at(pR->m_ActualPath.size()-1).pos, pR->m_State.state.pos) > 1 )
			{

				pR->m_ActualPath.push_back(pR->m_State.state);
			}
			else if(pR->m_ActualPath.size()==0)
			{
				pR->m_ActualPath.push_back(pR->m_State.state);
			}
		}
	}

	return pR;
}

void* PlannerTestDraw::ControlThreadStaticEntryPoint(void* pThis)
{
	PlannerTestDraw* pR = (PlannerTestDraw*)pThis;
	struct timespec moveTimer;
	TrajectoryFollower predControl;
	UtilityH::GetTickCount(moveTimer);
	vector<string> logData;


	while(!pR->m_bCancelThread)
	{
		double time_elapsed  = UtilityH::GetTimeDiffNow(moveTimer);

		if(time_elapsed >= pR->m_ControlCycleTime)
		{
			UtilityH::GetTickCount(moveTimer);

			pthread_mutex_lock(&pR->behaviors_mutex);
			BehaviorState currMessage = pR->m_CurrentBehavior;
			pthread_mutex_unlock(&pR->behaviors_mutex);


			vector<WayPoint> generatedPath;
			pthread_mutex_lock(&pR->planning_mutex);
			generatedPath = pR->m_State.m_Path;
			pthread_mutex_unlock(&pR->planning_mutex);

			if(pR->m_bNewPath)
			{
				if(generatedPath.size()>0)
				{
					predControl.UpdateCurrentPath(generatedPath);
					pR->m_bNewPath = false;
				}
			}

			VehicleState currState;

			if(currMessage.state == FORWARD_STATE)
			{
				if(generatedPath.size()>0)
				{
					predControl.PrepareNextWaypoint(pR->m_State.state, pR->m_State.m_CurrentVelocity, pR->m_State.m_CurrentSteering);
					//predControl.VeclocityControllerUpdate(time_elapsed, pR->m_GlobalVehicleStatus, currMessage,velocity_d);
					predControl.SteerControllerUpdate(currState, currMessage, currState.steer);

					currState.speed = 5;
					currState.shift = SHIFT_POS_DD;
				}
			}
			else if(currMessage.state == STOPPING_STATE)
			{
				currState.speed = 0;
				currState.shift = SHIFT_POS_DD;
			}
			else
			{
				currState.speed = 0;
				currState.shift = SHIFT_POS_NN;
			}

			pR->m_FollowPoint  = predControl.m_FollowMePoint.pos;
			pR->m_PerpPoint    = predControl.m_PerpendicularPoint.pos;

			pthread_mutex_lock(&pR->control_mutex);
			pR->m_VehicleState = currState;
			pthread_mutex_unlock(&pR->control_mutex);
		}
	}

	return pR;
}

} /* namespace Graphics */
