/*
 * PlannerTestDraw.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#include "PlannerTestDraw.h"
#include "PlannerH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include <sstream>

using namespace std;
using namespace SimulationNS;
using namespace UtilityHNS;

#define VectorMapPath "/home/hatem/workspace/Data/VectorMap/"
#define kmlTemplateFile "/home/hatem/workspace/Data/templates/PlannerX_MapTemplate.kml"
#define PreDefinedPath  "11,333,1090,1704,147, 1791,801, 431, 1522, 372, 791, 1875, 1872,171,108,21,"


namespace Graphics
{

#define EnableThreadBody

PlannerTestDraw::PlannerTestDraw()
{
	planning_mutex =  PTHREAD_MUTEX_INITIALIZER;
	control_mutex = PTHREAD_MUTEX_INITIALIZER;
	simulation_mutex = PTHREAD_MUTEX_INITIALIZER;

	planning_thread_tid = 0;
	control_thread_tid = 0;
	m_bCancelThread = false;
	m_PlanningCycleTime = 0.01;
	m_ControlCycleTime  = 0.01;

	m_CarModel = 0;

	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(VectorMapPath, m_RoadMap);
	/**
	 * Writing the kml file for the RoadNetwork Map
	 */
//	ostringstream fileName;
//	fileName << UtilityH::GetFilePrefixHourMinuteSeconds();
//	fileName << "_RoadNetwork.kml";
//	PlannerHNS::MappingHelpers::WriteKML(fileName.str(),kmlTemplateFile , m_RoadMap);

	m_pMap = new PlannerHNS::GridMap(0,0,60,60,1.0, true);

	CAR_BASIC_INFO carInfo;
	m_State.Init(1.9, 4.2,carInfo);
	m_State.InitPolygons();

	/**
	 * Planning using predefind path (sequence of lane IDs)
	 */
//	stringstream str_stream(PreDefinedPath);
//	string strLine, innerToken;
//	m_LanesIds.clear();
//	while(getline(str_stream, innerToken, ','))
//	{
//		int id = strtol(innerToken.c_str(), NULL, 10);
//		m_LanesIds.push_back(id);
//	}

	m_followX = m_start.pos.x;
	m_followY = m_start.pos.y;
	m_followZ = m_start.pos.z;
	m_followA = m_start.pos.a;

	m_bMakeNewPlan = false;

	pthread_create(&planning_thread_tid, NULL, &PlannerTestDraw::PlanningThreadStaticEntryPoint, this);
	pthread_create(&control_thread_tid, NULL, &PlannerTestDraw::ControlThreadStaticEntryPoint, this);
	pthread_create(&simulation_thread_tid, NULL, &PlannerTestDraw::SimulationThreadStaticEntryPoint, this);


	//InitStartAndGoal(2, -50, M_PI, 100, 100, M_PI_2);

	PrepareVectorMapForDrawing();
}

void PlannerTestDraw::InitStartAndGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2)
{
	m_start = PlannerHNS::WayPoint(x1, y1, 0, a1);
	//m_start = PlannerHNS::MappingHelpers::GetFirstWaypoint(m_RoadMap);
	PlannerHNS::WayPoint* pWS = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(m_start, m_RoadMap);
	if(pWS)
		m_start = *pWS;
	else
		cout << "#Planning Error: Start Position is too far from the road network map!" << endl;

	m_start.pos.z = 0;
	m_start.bDir = PlannerHNS::FORWARD_DIR;

	m_State.FirstLocalizeMe(m_start);



	/**
	 * Planning using goad point
	 */
	m_goal = PlannerHNS::WayPoint(x2, y2, 0, a2);
	PlannerHNS::WayPoint* pW = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(m_goal, m_RoadMap);
	if(pW)
		m_goal.pos = pW->pos;
	else
		cout << "#Planning Error: Goal Position is too far from the road network map!" << endl;

	m_goal.bDir = PlannerHNS::FORWARD_DIR;

	m_bMakeNewPlan = true;
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
	if(planning_thread_tid>0)
		pthread_join(planning_thread_tid, (void**)&pRet);
	if(control_thread_tid>0)
		pthread_join(control_thread_tid, (void**)&pRet);
	if(simulation_thread_tid>0)
		pthread_join(simulation_thread_tid, (void**)&pRet);

	if(m_pMap)
	{
		delete m_pMap;
		m_pMap = 0;
	}
}

bool PlannerTestDraw::IsInitState()
{
	if(m_State.m_TotalPath.size() > 0)
		return false;
	else
		return true;
}

void PlannerTestDraw::UpdatePlaneStartGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2)
{
	InitStartAndGoal(x1, y1, a1, x2, y2, a2);
}

void PlannerTestDraw::AddSimulatedCarPos(const double& x,const double& y, const double& a)
{
	PlannerHNS::DetectedObject ob;
	ob.id = m_dummyObstacles.size() + 1;
	ob.l = 4.0;
	ob.w = 2.0;
	ob.center.pos.x = x;
	ob.center.pos.y = y;
	ob.center.pos.a = a;
	ob.center.v = (rand()%70+3)/10.0/3.6;


	SimulatedCarState car;
	CAR_BASIC_INFO carInfo;
	car.Init(ob.w, ob.l,carInfo);
	car.InitPolygons();

	PlannerHNS::WayPoint* pWS = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(ob.center, m_RoadMap);
	if(pWS)
	{
		ob.center.pos = pWS->pos;
		ob.center.pLane = pWS->pLane;
		car.pLane = pWS->pLane;
	}
	else
		cout << "#Planning Error: Start Position is too far from the road network map!" << endl;

	ob.center.pos.z = 0;
	ob.center.bDir = PlannerHNS::FORWARD_DIR;

	car.FirstLocalizeMe(ob.center);

	pthread_mutex_lock(&simulation_mutex);
	m_SimulatedCars.push_back(car);
	m_dummyObstacles.push_back(ob);
	m_SimulatedBehaviors.push_back(PlannerHNS::BehaviorState());
	m_SimulatedVehicleState.push_back(PlannerHNS::VehicleState());
	m_SimulatedPrevTrajectory.push_back(vector<PlannerHNS::WayPoint>());
	m_SimulatedPathFollower.push_back(SimulatedTrajectoryFollower());
	pthread_mutex_unlock(&simulation_mutex);
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
	double distance_to_nearest_lane = 1;
	int j=0;
	int max_number_of_lanes = 500;
	double width_ratio = 2.0;

	vector<PlannerHNS::Lane*> currLane;
	vector<PlannerHNS::Lane*> lanes_list;
	vector<PlannerHNS::Lane*> traversed_lanes;

	while(distance_to_nearest_lane < 100 && currLane.size() == 0)
	{
		PlannerHNS::Lane* pL = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(m_start, m_RoadMap, distance_to_nearest_lane);
		if(pL)
			currLane.push_back(pL);
		distance_to_nearest_lane += 2;
	}

	for(unsigned int i=0; i< currLane.size(); i++)
		lanes_list.push_back(currLane[i]);

	m_ReadyToDrawLanes.clear();
	m_ReadyToDrawCenterLines.clear();
//	m_TrafficLights.clear();
//	m_StopSigns.clear();
//	m_OtherSigns.clear();

	if(currLane.size() > 0)
	{
		std::vector<std::vector<PlannerHNS::WayPoint> > ready_to_draw;

		while(lanes_list.size()>0 && j <max_number_of_lanes)
		{
			ready_to_draw.clear();
			PlannerHNS::Lane* l = lanes_list.at(0);
			lanes_list.erase(lanes_list.begin()+0);
			traversed_lanes.push_back(l);


			vector<PlannerHNS::WayPoint> path_local = l->points;

			DrawingHelpers::PreparePathForDrawing(path_local,ready_to_draw, 2.8 / width_ratio);
			m_ReadyToDrawLanes.push_back(ready_to_draw);

			ready_to_draw.clear();
			DrawingHelpers::PreparePathForDrawing(path_local,ready_to_draw, 0.1);
			m_ReadyToDrawCenterLines.push_back(ready_to_draw);


			j++;

			PlannerHNS::MappingHelpers::GetUniqueNextLanes(l, traversed_lanes, lanes_list);

			//Get the traffic lights pos
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
		}
	}
}

void PlannerTestDraw::DrawVectorMap()
{
	glDisable(GL_LIGHTING);
	float PathColor[3];
	float Color1[3]; Color1[0] = 1.0; Color1[1] = 204.0/256.0; Color1[2] = 51.0/256.0;
	float Color2[3]; Color2[0] = 1.0; Color2[1] = 102.0/256.0; Color2[2] = 51.0/256.0;
	float Color3[3]; Color3[0] = 1.0; Color3[1] = 51.0/256.0;  Color3[2] = 102.0/256.0;
	float Color4[3]; Color4[0] = 204.0/256.0; Color4[1] = 51.0/256.0; Color4[2] = 1.0;

	const float mapdata_z = 0.005;

	for(unsigned int i=0; i<m_ReadyToDrawLanes.size(); i++)
	{
		//PathColor[0]=0.5;PathColor[1] = j/20.0; PathColor[2] = j;
		if(i==0)
		{
			PathColor[0]=0.25 ;PathColor[1] = 0.25; PathColor[2] = 0.25;
		}
		else
		{
			double inc_color = (double)i/25.0;
			PathColor[0]=0.25 + inc_color; ;PathColor[1] = 0.25+inc_color; PathColor[2] = 0.25 + inc_color;
		}

		//PathColor[0]=0.25 ;PathColor[1] = 0.25; PathColor[2] = 0.25;
		if(i%4 == 0)
		{
			PathColor[0] = Color1[0]; PathColor[1] = Color1[1]; PathColor[2] = Color1[2];
		}
		else if(i%4 == 1)
		{
			PathColor[0] = Color2[0]; PathColor[1] = Color2[1]; PathColor[2] = Color2[2];
		}
		else if(i%4 == 2)
		{
			PathColor[0] = Color3[0]; PathColor[1] = Color3[1]; PathColor[2] = Color3[2];
		}
		else if(i%4 == 3)
		{
			PathColor[0] = Color4[0]; PathColor[1] = Color4[1]; PathColor[2] = Color4[2];
		}

		PathColor[0] = PathColor[0]*0.15;
		PathColor[1] = PathColor[1]*0.95;
		PathColor[2] = PathColor[2]*0.95;
		PathColor[0]=0.4;PathColor[1] = 0.4; PathColor[2] = 0.4;
		DrawingHelpers::DrawPrePreparedPolygons(m_ReadyToDrawLanes[i], mapdata_z, PathColor);

		PathColor[0]=0.97;PathColor[1] = 0.97; PathColor[2] = 0.97;
		DrawingHelpers::DrawPrePreparedPolygons(m_ReadyToDrawCenterLines[i], mapdata_z+0.015, PathColor, 1);
	}

	glEnable(GL_LIGHTING);
}

void PlannerTestDraw::DrawSimu()
{

	DrawVectorMap();

	float TotalPathColor[3] = {0.99, 0.99, 0.0};
	float PlannedPathColor[3] = {0.0, 0.99, 0.0};
	float ActualPathColor[3] = {0.1, 0.1, 0.9};

	pthread_mutex_lock(&planning_mutex);
	DrawingHelpers::DrawWidePath(m_State.m_TotalPath, 0.08, 0.25, TotalPathColor);
	DrawingHelpers::DrawWidePath(m_State.m_Path, 0.16, 0.2, PlannedPathColor);
	DrawingHelpers::DrawWidePath(m_ActualPath, 0.18, 0.15, ActualPathColor);

	float RollOutsColor[3] = {0.9, 0.2, 0.1};
	for(unsigned int i=0; i < m_State.m_RollOuts.size();i++)
	{
		DrawingHelpers::DrawWidePath(m_State.m_RollOuts.at(i), 0.14, 0.1, RollOutsColor);
	}
	pthread_mutex_unlock(&planning_mutex);

	float CarColor[3] = {0.1, 0.9, 0.9};
	pthread_mutex_lock(&simulation_mutex);

	for(unsigned int i =0; i <m_SimulatedCars.size(); i++)
	{
		DrawingHelpers::DrawCustomCarModel(m_SimulatedCars.at(i).state, m_State.m_CarShapePolygon, CarColor, 90);
		float TotalPathColor[3] = {0.99, 0.0, 0.99};
		DrawingHelpers::DrawWidePath(m_SimulatedCars.at(i).m_TotalPath, 0.08, 0.25, TotalPathColor);
	}
	pthread_mutex_unlock(&simulation_mutex);

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
				m_State.state.pos.x,m_State.state.pos.y,
				m_State.state.pos.z+0.275, m_State.state.pos.a, 0,0);
	}

	DrawingHelpers::DrawCustomCarModel(m_State.state, m_State.m_CarShapePolygon, CarColor, 90);
}

void PlannerTestDraw::DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY)
{
	double left_shift = 25;
	glDisable(GL_LIGHTING);
	glPushMatrix();
	glTranslated(centerX-left_shift, 70, 0);
	glRotated(-1*m_VehicleState.steer*RAD2DEG*16, 0,0,1);
	glTranslated(-(centerX-left_shift), -70, 0);

	float wheel_color[3] = {0.1, 0.2, 0.3};
	DrawingHelpers::DrawWideEllipse(centerX-left_shift, 70, 0.5, 60, 55, 54, wheel_color);

	glColor3f(0.3,0.2, 0.1);
	PlannerHNS::GPSPoint p1(centerX-left_shift, 70, 0.52, 0), p2(centerX-left_shift+38, 70-38, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p1, p2, 5);

	PlannerHNS::GPSPoint p11(centerX-left_shift, 70, 0.52, 0), p22(centerX-left_shift-38, 70-38, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p11, p22, 5);

	PlannerHNS::GPSPoint p111(centerX-left_shift, 70, 0.52, 0), p222(centerX-left_shift, 70+52, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p111, p222, 5);
	glPopMatrix();

	glPushMatrix();
	glTranslated(centerX-left_shift-15, 70+85, 0);
	std::ostringstream str_out ;
	str_out.precision(2);
	str_out <<  m_VehicleState.steer*RAD2DEG;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
	glPopMatrix();

	double speed = m_VehicleState.speed*3.6;
	float pedal_color[3] = {0.1, 0.7, 0.3};
	DrawingHelpers::DrawPedal(centerX + 70, 70, 0, 30.0, 100.0, speed*2.5,pedal_color );
	glPushMatrix();
	glTranslated(centerX+60, 70+85, 0);
	std::ostringstream v_out ;
	v_out.precision(2);
	v_out <<  speed;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)v_out.str().c_str());
	glPopMatrix();

	glEnable(GL_LIGHTING);
//
//	INITIAL_STATE, WAITING_STATE, FORWARD_STATE, STOPPING_STATE, EMERGENCY_STATE,
//		TRAFFIC_LIGHT_STOP_STATE, STOP_SIGN_STOP_STATE, FOLLOW_STATE, LANE_CHANGE_STATE, OBSTACLE_AVOIDANCE_STATE, FINISH_STATE

	std::ostringstream state_out ;
	state_out << "Behavior: ";
	string str = "Unknown State";
	switch(m_CurrentBehavior.state)
	{
	case PlannerHNS::INITIAL_STATE:
		str = "Init State";
		break;
	case PlannerHNS::WAITING_STATE:
		str = "Waiting State";
		break;
	case PlannerHNS::FORWARD_STATE:
		str = "Forward State";
		break;
	case PlannerHNS::STOPPING_STATE:
		str = "Stop State";
		break;
	case PlannerHNS::FINISH_STATE:
		str = "End State";
		break;
	default:
		str = "Unknown State";
		break;
	}
	state_out << str;
	glPushMatrix();
	glColor3f(0.2, 0.2, 0.9);
	glTranslated(10, 200, 0);
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)state_out.str().c_str());

	std::ostringstream sim_n_out ;
	sim_n_out << "Simulated Cars:";
	sim_n_out << m_SimulatedCars.size();
	glColor3f(0.6, 0.6, 0.9);
	DrawingHelpers::DrawString(0, 30, GLUT_BITMAP_TIMES_ROMAN_24, (char*)sim_n_out.str().c_str());
	glPopMatrix();
}

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
	PlannerHNS::PlanningInternalParams params;
	PlannerHNS::PlannerH planner(params);

	while(!pR->m_bCancelThread)
	{
		double time_elapsed  = UtilityH::GetTimeDiffNow(moveTimer);

		if(time_elapsed >= pR->m_PlanningCycleTime)
		{
			double dt = time_elapsed;
			UtilityH::GetTickCount(moveTimer);
#ifdef EnableThreadBody
			vector<PlannerHNS::WayPoint> generatedTotalPath = pR->m_State.m_TotalPath;
			vector<vector<PlannerHNS::WayPoint> > rollOuts;
			bool bNewPlan = false;
			bool bNewRollOuts = false;
			PlannerHNS::VehicleState currState;
			pthread_mutex_lock(&pR->control_mutex);
			currState = pR->m_VehicleState;
			pthread_mutex_unlock(&pR->control_mutex);

			/**
			 * Path Planning Step (Global Planning)
			 */
			int currIndexToal = PlannerHNS::PlanningHelpers::GetClosestPointIndex(pR->m_State.m_TotalPath, pR->m_State.state);
			int index_limit_total = pR->m_State.m_TotalPath.size() - 20;
			if(index_limit_total<=0)
				index_limit_total =  pR->m_State.m_TotalPath.size()/2.0;

			if(currIndexToal > index_limit_total)
			{
				pR->m_bMakeNewPlan = true;
				PlannerHNS::WayPoint g_p = pR->m_goal;
				pR->m_goal = pR->m_start;
				pR->m_start = g_p;
			}

			if((pR->m_CurrentBehavior.state == PlannerHNS::INITIAL_STATE && pR->m_State.m_Path.size() == 0 && pR->m_bMakeNewPlan) || pR->m_bMakeNewPlan)
			{
				//planner.PlanUsingReedShepp(pR->m_State.state, pR->m_goal, generatedPath);
				planner.PlanUsingDP(pR->m_State.pLane, pR->m_State.state, pR->m_goal, pR->m_State.state, 1000000,pR->m_LanesIds, generatedTotalPath);
				pR->m_goal = generatedTotalPath.at(generatedTotalPath.size()-1);
				pR->m_bMakeNewPlan = false;
				bNewPlan = true;
			}

			//bool bNewTrajectory = false;
			/**
			 * Trajectory Planning Step, Roll outs , (Micro planner)
			 */
			if(generatedTotalPath.size()>0)
			{
				int currIndex = PlannerHNS::PlanningHelpers::GetClosestPointIndex(pR->m_State.m_Path, pR->m_State.state);
				int index_limit = pR->m_State.m_Path.size() - 20;
				if(index_limit<=0)
					index_limit =  pR->m_State.m_Path.size()/2.0;
				if(pR->m_State.m_RollOuts.size() == 0 || currIndex > index_limit)
				{
					planner.GenerateRunoffTrajectory(pR->m_State.m_TotalPath, pR->m_State.state, false,
							currState.speed,
							pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.microPlanDistance,
							pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.maxSpeed,
							pR->m_State.m_pCurrentBehaviorState->m_PlanningParams.minSpeed,
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
							false, rollOuts);
					bNewRollOuts = true;
				}
			}

			/**
			 * Behavior Generator , State Machine , Decision making Step
			 */


			pthread_mutex_lock(&pR->planning_mutex);
			if(bNewRollOuts)
				pR->m_State.m_RollOuts  = rollOuts;
			if(bNewPlan)
				pR->m_State.m_TotalPath = generatedTotalPath;
			pR->m_CurrentBehavior = pR->m_State.DoOneStep(dt, currState, pR->m_dummyObstacles, pR->m_goal.pos, pR->m_RoadMap);
			pthread_mutex_unlock(&pR->planning_mutex);

			if(pR->m_ActualPath.size() > 0 && distance2points(pR->m_ActualPath.at(pR->m_ActualPath.size()-1).pos, pR->m_State.state.pos) > 1 )
			{
				pR->m_ActualPath.push_back(pR->m_State.state);
			}
			else if(pR->m_ActualPath.size()==0 && pR->m_State.m_Path.size() > 0)
			{
				pR->m_ActualPath.push_back(pR->m_State.state);
			}
#endif
		}
	}

	return 0;
}

void* PlannerTestDraw::ControlThreadStaticEntryPoint(void* pThis)
{
	PlannerTestDraw* pR = (PlannerTestDraw*)pThis;
	struct timespec moveTimer;
	TrajectoryFollower predControl;
	UtilityH::GetTickCount(moveTimer);
	vector<string> logData;
	vector<PlannerHNS::WayPoint> generatedPath;

	vector<vector<PlannerHNS::WayPoint> > simulationGeneratedPaths;


	while(!pR->m_bCancelThread)
	{
		double time_elapsed  = UtilityH::GetTimeDiffNow(moveTimer);

		if(time_elapsed >= pR->m_ControlCycleTime)
		{
			double dt = time_elapsed;
			UtilityH::GetTickCount(moveTimer);

#ifdef EnableThreadBody
			pthread_mutex_lock(&pR->planning_mutex);

			PlannerHNS::BehaviorState currMessage = pR->m_CurrentBehavior;

			PlannerHNS::VehicleState currState, targetState;

			bool bNewPath = false;
			if(PlannerHNS::PlanningHelpers::CompareTrajectories(generatedPath, pR->m_State.m_Path) == false && pR->m_State.m_Path.size()>0)
			{
				generatedPath = pR->m_State.m_Path;
				bNewPath = true;
				cout << "Path is Updated in the controller .. " << pR->m_State.m_Path.size() << endl;
			}

			currState.steer = pR->m_State.m_CurrentSteering;
			currState.speed = pR->m_State.m_CurrentVelocity;
			currState.shift = pR->m_State.m_CurrentShift;

			pthread_mutex_unlock(&pR->planning_mutex);

			targetState = predControl.DoOneStep(dt, currMessage, generatedPath, pR->m_State.state, currState, bNewPath);

			//cout << pR->m_State.m_pCurrentBehaviorState->GetCalcParams()->ToString(currMessage.state) << endl;

			pR->m_FollowPoint  = predControl.m_FollowMePoint.pos;
			pR->m_PerpPoint    = predControl.m_PerpendicularPoint.pos;

			pthread_mutex_lock(&pR->control_mutex);
			pR->m_VehicleState = targetState;
			pthread_mutex_unlock(&pR->control_mutex);
#endif
		}
	}

	return 0;
}

void* PlannerTestDraw::SimulationThreadStaticEntryPoint(void* pThis)
{
	PlannerTestDraw* pR = (PlannerTestDraw*)pThis;
	struct timespec moveTimer;
	UtilityH::GetTickCount(moveTimer);
	vector<string> logData;
	PlannerHNS::PlanningInternalParams params;
	PlannerHNS::PlannerH planner(params);
	std::vector<PlannerHNS::DetectedObject> dummyObstacles;


	while(!pR->m_bCancelThread)
	{
		double time_elapsed  = UtilityH::GetTimeDiffNow(moveTimer);

		if(time_elapsed >= 0.02)
		{
			double dt = time_elapsed;
			UtilityH::GetTickCount(moveTimer);

#ifdef EnableThreadBody
			pthread_mutex_lock(&pR->simulation_mutex);
			for(unsigned int i = 0 ; i < pR->m_SimulatedCars.size(); i++)
			{
				int currIndex = PlannerHNS::PlanningHelpers::GetClosestPointIndex(pR->m_SimulatedCars.at(i).m_Path, pR->m_SimulatedCars.at(i).state);
				int index_limit = pR->m_SimulatedCars.at(i).m_Path.size() - 10;
				if(index_limit<=0)
					index_limit =  pR->m_SimulatedCars.at(i).m_Path.size()/2.0;

				if(pR->m_SimulatedCars.at(i).m_RollOuts.size() == 0 || currIndex > index_limit)
				{
					vector<PlannerHNS::WayPoint> generatedPath;
					planner.PlanUsingDP(pR->m_SimulatedCars.at(i).pLane, pR->m_SimulatedCars.at(i).state, pR->m_goal,
							pR->m_SimulatedCars.at(i).state, 150,pR->m_LanesIds, generatedPath);
					pR->m_SimulatedCars.at(i).m_RollOuts.clear();
					pR->m_SimulatedCars.at(i).m_TotalPath = generatedPath;
					PlannerHNS::PlanningParams planningDefaultParams;
					planner.GenerateRunoffTrajectory(pR->m_SimulatedCars.at(i).m_TotalPath, pR->m_SimulatedCars.at(i).state, false,  3, 30, 3, 0,
							planningDefaultParams.carTipMargin,
							planningDefaultParams.rollInMargin,
							planningDefaultParams.rollInSpeedFactor,
							planningDefaultParams.pathDensity,
							planningDefaultParams.rollOutDensity,
							0,
							planningDefaultParams.smoothingDataWeight,
							planningDefaultParams.smoothingSmoothWeight,
							planningDefaultParams.smoothingToleranceError,
							planningDefaultParams.speedProfileFactor, false, pR->m_SimulatedCars.at(i).m_RollOuts);

					if(pR->m_SimulatedCars.at(i).m_RollOuts.size()>0)
						pR->m_SimulatedCars.at(i).m_Path = pR->m_SimulatedCars.at(i).m_RollOuts.at(0);
				}




				pR->m_SimulatedBehaviors.at(i) = pR->m_SimulatedCars.at(i).DoOneStep(dt, pR->m_SimulatedVehicleState.at(i),
										pR->m_SimulatedCars.at(i).state, dummyObstacles, pR->m_goal.pos, pR->m_RoadMap);

				bool bNewPath = false;
				if(PlannerHNS::PlanningHelpers::CompareTrajectories(pR->m_SimulatedPrevTrajectory.at(i), pR->m_SimulatedCars.at(i).m_Path) == false && pR->m_SimulatedCars.at(i).m_Path.size()>0)
				{
					pR->m_SimulatedPrevTrajectory.at(i) = pR->m_SimulatedCars.at(i).m_Path;
					bNewPath = true;
					cout << "Path is Updated in the controller .. " << pR->m_SimulatedCars.at(i).m_Path.size() << endl;
				}

				PlannerHNS::VehicleState currState;
				currState.steer = pR->m_SimulatedCars.at(i).m_CurrentSteering;
				currState.speed = 3;
				currState.shift = PlannerHNS::SHIFT_POS_DD;

				pR->m_SimulatedVehicleState.at(i) = pR->m_SimulatedPathFollower.at(i).DoOneStep(dt, pR->m_SimulatedBehaviors.at(i), pR->m_SimulatedPrevTrajectory.at(i),
						pR->m_SimulatedCars.at(i).state, currState, bNewPath);

				pR->m_SimulatedVehicleState.at(i).speed = 3;

			}
			pthread_mutex_unlock(&pR->simulation_mutex);
#endif
		}
	}

	return 0;
}

} /* namespace Graphics */
