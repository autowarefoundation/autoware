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
#include "MatrixOperations.h"
#include "SimpleTracker.h"
#include "DataRW.h"


using namespace std;
using namespace SimulationNS;
using namespace UtilityHNS;

#define VectorMap "ToyotaCity/"
#define kmlMap	"ToyotaKML.kml"
#define kmlTemplateFile "PlannerX_MapTemplate.kml"
#define kmltargetFile "ToyotaKML.kml"
#define PreDefinedPath  "11,333,1090,1704,147, 1791,801, 431, 1522, 372, 791, 1875, 1872,171,108,21,"

namespace Graphics
{

#define EnableThreadBody

PlannerTestDraw::PlannerTestDraw()
{
	planning_mutex =  PTHREAD_MUTEX_INITIALIZER;
	control_mutex = PTHREAD_MUTEX_INITIALIZER;
	simulation_mutex = PTHREAD_MUTEX_INITIALIZER;

	m_LateralError = 0;
	m_pVelocityGraph = 0;
	planning_thread_tid = 0;
	control_thread_tid = 0;
	simulation_thread_tid = 0;
	m_bCancelThread = false;
	m_PlanningCycleTime = 0.01;
	m_ControlCycleTime  = 0.01;
	m_SimulationCycleTime = 0.02;
	m_bResetForSimulation = false;
	m_GlobalPlanningTime = 0;
	m_LocalPlanningTime = 0;
	m_ControllingTime = 0;
	m_ObjectTrakingTime = 0;
	m_SimulationTime = 0;

	m_CarModel = 0;

	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(UtilityH::GetHomeDirectory()+
			DataRW::LoggingMainfolderName + DataRW::VectorMapsFolderName+VectorMap, m_RoadMap);

//	string kml_templateFilePath = UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmlTemplateFile;
//	string kml_fileToSave =UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmltargetFile;
//	PlannerHNS::MappingHelpers::WriteKML(kml_fileToSave, kml_templateFilePath, m_RoadMap);

//	PlannerHNS::MappingHelpers::LoadKML(UtilityH::GetHomeDirectory() +
//			DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName + kmlMap, m_RoadMap);
	/**
	 * Writing the kml file for the RoadNetwork Map
	 */
//	ostringstream fileName;
//	fileName << UtilityH::GetFilePrefixHourMinuteSeconds();
//	fileName << "_RoadNetwork.kml";
//	PlannerHNS::MappingHelpers::WriteKML(fileName.str(),UtilityH::GetHomeDirectory()+
//			DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmlTemplateFile, m_RoadMap);

	m_pMap = new PlannerHNS::GridMap(0,0,60,60,1.0, true);

	m_ControlParams.Steering_Gain = PID_CONST(0.07, 0.02, 0.01);
	m_ControlParams.SteeringDelay = 0.85;
	m_ControlParams.Steering_Gain.kD = 0.5;
	m_ControlParams.Steering_Gain.kP = 0.1;
	m_ControlParams.Steering_Gain.kI = 0.03;
	m_ControlParams.SteeringDelayPercent = 17.5;

	m_ControlParams.Velocity_Gain = PID_CONST(0.1, 0.005, 0.1);

//	m_PlanningParams.microPlanDistance = 50;

	m_CarInfo.width = 1.9;
	m_CarInfo.length = 4.2;
	m_CarInfo.max_speed_forward = 2.0;
	m_PlanningParams.pathDensity = 0.5;
	m_State.Init(m_ControlParams, m_PlanningParams, m_CarInfo);
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

	double axes_color[3] = {0.1, 0.1, 0.8};
	double graph_color[3] = {0.9, 0.2, 0.1};
	m_pVelocityGraph = new Graph2dBase(20, 200,1000, 12, 0, "Car Velocity", "T s", "V km/h", axes_color, graph_color );
	m_pSteeringGraph = new Graph2dBase(20, 200,1000, m_CarInfo.max_steer_angle*RAD2DEG, -m_CarInfo.max_steer_angle*RAD2DEG, "Car Steering", "T s", "A deg", axes_color, graph_color );
	m_pLateralErrGraph  = new Graph2dBase(20, 200,1000, 1.0, -1.0, "Lateral Error", "T s", "D meter", axes_color, graph_color );


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

void PlannerTestDraw::SaveSimulationData()
{
	std::vector<std::string> simulationDataPoints;
	std::ostringstream startStr;
	startStr << m_start.pos.x << "," << m_start.pos.y << "," << m_start.pos.z << ","
			<< m_start.pos.a << "," << m_start.cost << "," << m_start.v << "," ;
	simulationDataPoints.push_back(startStr.str());

	std::ostringstream goalStr;
	goalStr << m_goal.pos.x << "," << m_goal.pos.y << "," << m_goal.pos.z << ","
			<< m_goal.pos.a << "," << m_goal.cost << "," << m_goal.v << "," ;
	simulationDataPoints.push_back(goalStr.str());

	for(unsigned int i = 0; i < m_SimulatedCars.size(); i++)
	{
		std::ostringstream carStr;
		carStr << m_SimulatedCars.at(i).state.pos.x << ","
				<< m_SimulatedCars.at(i).state.pos.y << ","
				<< m_SimulatedCars.at(i).state.pos.z << ","
				<< m_SimulatedCars.at(i).state.pos.a << ","
				<< m_SimulatedCars.at(i).state.cost << ","
				<< m_SimulatedCars.at(i).m_CarInfo.max_speed_forward << "," ;
		simulationDataPoints.push_back(carStr.str());
	}

	DataRW::WriteLogData(UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName+DataRW::SimulationFolderName, "SimulationFile",
				"X,Y,Z,A,C,V,",	simulationDataPoints);
}

void PlannerTestDraw::LoadSimulationData()
{
	m_bCancelThread = true;
	PlannerTestDraw* pRet = 0;
	if(planning_thread_tid>0)
		pthread_join(planning_thread_tid, (void**)&pRet);
	if(control_thread_tid>0)
		pthread_join(control_thread_tid, (void**)&pRet);
	if(simulation_thread_tid>0)
		pthread_join(simulation_thread_tid, (void**)&pRet);

	m_State = CarState();

	m_State.Init(m_ControlParams, m_PlanningParams, m_CarInfo);
	m_State.InitPolygons();
	m_ActualPath.clear();

	string simuDataFileName = UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName+DataRW::SimulationFolderName + "s1.csv";
	SimulationFileReader sfr(simuDataFileName);
	SimulationFileReader::SimulationData data;

	sfr.ReadAllData(data);
	InitStartAndGoal(data.startPoint.x, data.startPoint.y, data.startPoint.a,
			data.goalPoint.x, data.goalPoint.y, data.goalPoint.a);

	for(unsigned  int i=0; i < data.simuCars.size(); i++)
	{
		AddSimulatedCar(data.simuCars.at(i).x, data.simuCars.at(i).y,
				data.simuCars.at(i).a, (6.0 + rand()%6)/3.6);
	}

	//m_bResetForSimulation = true;

	m_bCancelThread = false;
	pthread_create(&planning_thread_tid, NULL, &PlannerTestDraw::PlanningThreadStaticEntryPoint, this);
	pthread_create(&control_thread_tid, NULL, &PlannerTestDraw::ControlThreadStaticEntryPoint, this);
	pthread_create(&simulation_thread_tid, NULL, &PlannerTestDraw::SimulationThreadStaticEntryPoint, this);
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

void PlannerTestDraw::DetectSimulatedObstacles(std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	obj_list.clear();
	for(unsigned int i =0; i < m_SimulatedCars.size(); i++)
	{
		PlannerHNS::DetectedObject ob;
		ob.id = 0;
		ob.l = m_SimulatedCars.at(i).m_CarInfo.length;
		ob.w = m_SimulatedCars.at(i).m_CarInfo.width;
		ob.center.pos = m_SimulatedCars.at(i).state.pos;
		ob.center.v = m_SimulatedCars.at(i).state.v;
		ob.contour = m_SimulatedCars.at(i).m_CarShapePolygon;
		ob.center.pLane = m_SimulatedCars.at(i).state.pLane;
		obj_list.push_back(ob);
	}
}

void PlannerTestDraw::AddSimulatedCarPos(const double& x,const double& y, const double& a)
{
	double v  = 4.0;//(6.0 + rand()%6)/3.6;
	AddSimulatedCar(x,y,a,v);
}

void PlannerTestDraw::AddSimulatedCar(const double& x,const double& y, const double& a, const double& v)
{
	SimulatedCarState car;
	CAR_BASIC_INFO carInfo;
	ControllerParams params;


	carInfo.width  = 1.8;
	carInfo.length = 4.1;
	carInfo.max_speed_forward = v;
	carInfo.max_steer_angle = 0.42;
	carInfo.min_steer_angle = -0.42;
	carInfo.turning_radius = 4.0;
	carInfo.wheel_base = 2.0;

	params.Steering_Gain = PID_CONST(1.5, 0.0, 0.0);
	params.Velocity_Gain = PID_CONST(0.2, 0.05, 0.1);
	params.minPursuiteDistance = 3.0;


	car.Init(carInfo);
	car.InitPolygons();
	car.state.pos = PlannerHNS::GPSPoint(x,y,0,a);

	PlannerHNS::WayPoint* pWS = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(car.state, m_RoadMap);
	if(pWS)
	{
		car.pLane = pWS->pLane;
	}
	else
		cout << "#Planning Error: Start Position is too far from the road network map!" << endl;


	car.state.bDir = PlannerHNS::FORWARD_DIR;
	car.FirstLocalizeMe(car.state);

	pthread_mutex_lock(&simulation_mutex);
	m_SimulatedCars.push_back(car);
	m_SimulatedBehaviors.push_back(PlannerHNS::BehaviorState());
	m_SimulatedVehicleState.push_back(PlannerHNS::VehicleState());
	m_SimulatedPrevTrajectory.push_back(vector<PlannerHNS::WayPoint>());
	SimulatedTrajectoryFollower pf;
	pf.Init(params, carInfo);
	m_SimulatedPathFollower.push_back(pf);
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

	if(currLane.size()==0)
	{
		if(m_RoadMap.roadSegments.size() > 0)
			if(m_RoadMap.roadSegments.at(0).Lanes.size()>0)
				currLane.push_back(&m_RoadMap.roadSegments.at(0).Lanes.at(0));
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

	glDisable(GL_LIGHTING);
	for(unsigned int i = 0; i < m_State.m_Path.size(); i+=2 )
	{
		if(m_State.m_Path.at(i).collisionCost >= 1)
		{
			glColor3f(1, 0, 0);
			float collisionColor[3] = {1, 0, 0};
			DrawingHelpers::DrawWideEllipse(m_State.m_Path.at(i).pos.x,
					m_State.m_Path.at(i).pos.y, 0.2, 1.0, 1.0, 0.8, collisionColor);
		}
		else
		{
			glColor3f(PlannedPathColor[0], PlannedPathColor[1], PlannedPathColor[2]);
			DrawingHelpers::DrawSimpleEllipse(m_State.m_Path.at(i).pos.x,
					m_State.m_Path.at(i).pos.y, 0.2, 1.0, 1.0);
		}

		glPointSize(10);
		glBegin(GL_POINTS);
			glColor3f(0,0,1);
			glVertex3f(m_State.m_Path.at(i).pos.x, m_State.m_Path.at(i).pos.y, 0.21);
		glEnd();
		glPointSize(1);

		glPushMatrix();
		glTranslated(m_State.m_Path.at(i).pos.x, m_State.m_Path.at(i).pos.y, 0.25);
		std::ostringstream str_out ;
		str_out.precision(4);
		str_out <<  m_State.m_Path.at(i).timeCost;
		glColor3f(1,0.9,1);
		DrawingHelpers::DrawString(0, 0,
				GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
		glPopMatrix();
	}


	float RollOutsColor[3] = {0.9, 0.2, 0.1};
	for(unsigned int oi=0; oi < m_State.m_RollOuts.size();oi++)
	{
		glDisable(GL_LIGHTING);
		DrawingHelpers::DrawWidePath(m_State.m_RollOuts.at(oi), 0.14, 0.075, RollOutsColor);

		for(unsigned int i = 0; i < m_State.m_RollOuts.at(oi).size(); i+=2 )
		{
			if(m_State.m_RollOuts.at(oi).at(i).collisionCost >= 1)
			{
				glColor3f(1, 0, 0);
				float collisionColor[3] = {1, 0, 0};
				DrawingHelpers::DrawWideEllipse(m_State.m_RollOuts.at(oi).at(i).pos.x,
						m_State.m_RollOuts.at(oi).at(i).pos.y, 0.2, 1.0, 1.0, 0.8, collisionColor);
			}
			else
			{
				glColor3f(PlannedPathColor[0], PlannedPathColor[1], PlannedPathColor[2]);
				DrawingHelpers::DrawSimpleEllipse(m_State.m_RollOuts.at(oi).at(i).pos.x,
						m_State.m_RollOuts.at(oi).at(i).pos.y, 0.2, 1.0, 1.0);
			}

//			glDisable(GL_LIGHTING);
//			glPointSize(10);
//			glBegin(GL_POINTS);
//			int j_dec = 0;
//			int r_inc = i*8;
//			if(r_inc > 255)
//			{
//				//j_dec = i*5 - 255;
//				r_inc = 255;
//			}
//				glColor3ub(r_inc,255-j_dec,0);
//				glVertex3f(m_State.m_RollOuts.at(oi).at(i).pos.x, m_State.m_RollOuts.at(oi).at(i).pos.y, 0.21);
//			glEnd();
//			glPointSize(1);

			glPushMatrix();
			glTranslated(m_State.m_RollOuts.at(oi).at(i).pos.x, m_State.m_RollOuts.at(oi).at(i).pos.y, 0.25);
			std::ostringstream str_out ;
			str_out.precision(4);
			str_out <<  m_State.m_RollOuts.at(oi).at(i).timeCost;
			glColor3f(1,0.9,1);
			DrawingHelpers::DrawString(0, 0,
					GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
			glPopMatrix();
		}
	}

	for(unsigned int i =0; i <m_dummyObstacles.size(); i++)
	{
		float CarColor[3] = {0.9, 0.1, 0.9};
		DrawingHelpers::DrawCustomCarModel(m_dummyObstacles.at(i).center, m_State.m_CarShapePolygon, CarColor, 90);
		//std::cout << " >>> Calculated Angle : " << (m_dummyObstacles.at(i).center.pos.a*RAD2DEG + 90)*DEG2RAD << std::endl;
		glPushMatrix();
		glTranslated(m_dummyObstacles.at(i).center.pos.x, m_dummyObstacles.at(i).center.pos.y, 1.3);
		std::ostringstream str_out ;
		str_out.precision(4);
		str_out <<  m_dummyObstacles.at(i).center.v *3.6;
		DrawingHelpers::DrawString(0, 0,
				GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
		glPopMatrix();
	}

	for(unsigned int ii = 0; ii < m_State.m_PredictedPath.size(); ii++ )
	{
		DrawingHelpers::DrawWidePath(m_State.m_PredictedPath.at(ii), 0.08, 0.25, TotalPathColor);

		for(unsigned int k = 0; k < m_State.m_PredictedPath.at(ii).size(); k++ )
		{
			if(m_State.m_PredictedPath.at(ii).at(k).collisionCost >= 1)
				glColor3f(1, 0, 0);
			else
				glColor3f(TotalPathColor[0], TotalPathColor[1], TotalPathColor[2]);
			DrawingHelpers::DrawSimpleEllipse(m_State.m_PredictedPath.at(ii).at(k).pos.x,
					m_State.m_PredictedPath.at(ii).at(k).pos.y, 0.25, 1.0, 1.0);

			glPushMatrix();
			glTranslated(m_State.m_PredictedPath.at(ii).at(k).pos.x, m_State.m_PredictedPath.at(ii).at(k).pos.y, 0.25);
			std::ostringstream str_out ;
			str_out.precision(4);
			str_out <<  m_State.m_PredictedPath.at(ii).at(k).timeCost;
			glColor3f(1,0.9,1);
			DrawingHelpers::DrawString(0, 0,
					GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
			glPopMatrix();
		}

	}

	pthread_mutex_unlock(&planning_mutex);

	float CarColor[3] = {0.1, 0.9, 0.9};

	pthread_mutex_lock(&simulation_mutex);
	for(unsigned int i =0; i <m_SimulatedCars.size(); i++)
	{
		DrawingHelpers::DrawCustomCarModel(m_SimulatedCars.at(i).state, m_State.m_CarShapePolygon, CarColor, 90);
//
//		glPushMatrix();
//		glTranslated(m_SimulatedCars.at(i).state.pos.x, m_SimulatedCars.at(i).state.pos.y, 1.3);
//		std::ostringstream str_out ;
//		str_out.precision(2);
//		str_out <<  m_SimulatedVehicleState.at(i).speed *3.6;
//		DrawingHelpers::DrawString(0, 1,
//				GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//		glPopMatrix();

		float TotalPathColor[3] = {0.5, 0.0, 0.99};
		DrawingHelpers::DrawWidePath(m_SimulatedCars.at(i).m_TotalPath, 0.08, 0.15, TotalPathColor);

//		for(unsigned int ii = 0; ii < m_SimulatedCars.at(i).m_TotalPath.size(); ii++ )
//		{
//			if(m_SimulatedCars.at(i).m_TotalPath.at(ii).collisionCost >= 1)
//				glColor3f(1, 0, 0);
//			else
//				glColor3f(TotalPathColor[0], TotalPathColor[1], TotalPathColor[2]);
//			DrawingHelpers::DrawSimpleEllipse(m_SimulatedCars.at(i).m_TotalPath.at(ii).pos.x,
//					m_SimulatedCars.at(i).m_TotalPath.at(ii).pos.y, 0.25, 1.0, 1.0);
//
//
//		}





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
	glRotated(-1*m_VehicleCurrentState.steer*RAD2DEG*16, 0,0,1);
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
	glColor3f(0.8, 0.1, 0.7);
	std::ostringstream str_out ;
	str_out.precision(2);
	str_out <<  m_VehicleCurrentState.steer*RAD2DEG;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
	glPopMatrix();

	double speed = m_VehicleCurrentState.speed*3.6;
	float pedal_color[3] = {0.1, 0.7, 0.8};
	DrawingHelpers::DrawPedal(centerX + 70, 70, 0, 30.0, 100.0, speed*5.5,pedal_color );
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
	state_out.precision(4);
	state_out << "State-> ";
	string str = "Unknown";
	switch(m_CurrentBehavior.state)
	{
	case PlannerHNS::INITIAL_STATE:
		str = "Init";
		break;
	case PlannerHNS::WAITING_STATE:
		str = "Waiting";
		break;
	case PlannerHNS::FORWARD_STATE:
		str = "Forward";
		break;
	case PlannerHNS::STOPPING_STATE:
		str = "Stop";
		break;
	case PlannerHNS::FINISH_STATE:
		str = "End";
		break;
	case PlannerHNS::FOLLOW_STATE:
		str = "Follow";
		break;
	case PlannerHNS::OBSTACLE_AVOIDANCE_STATE:
		str = "Swerving";
		break;
	default:
		str = "Unknown";
		break;
	}
	state_out << str;
	state_out << " (" << m_CurrentBehavior.followDistance << ";"
			<< m_CurrentBehavior.followVelocity*3.6 << ";"
			<< m_CurrentBehavior.stopDistance << ";"
			<< m_State.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ";"
			<< m_State.m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << ")";

	glPushMatrix();
	glColor3f(0.8, 0.5, 0.7);
	glTranslated(10, 200, 0);
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)state_out.str().c_str());

	double y = 30;
	for(unsigned int i = 0; i < m_dummyObstacles.size(); i++)
	{
		std::ostringstream sim_n_out ;
		sim_n_out << "Simu Car (:" << m_dummyObstacles.at(i).id << ") -> ";
		sim_n_out << m_dummyObstacles.at(i).center.v;
		glColor3f(0.6, 0.6, 0.9);
		DrawingHelpers::DrawString(0, y, GLUT_BITMAP_TIMES_ROMAN_24, (char*)sim_n_out.str().c_str());
		y+=30;
	}

	glPopMatrix();

	glPushMatrix();
	glTranslated(10, 500, 0);
	if(m_pVelocityGraph)
	{
		double axes_color[3] = {0.1, 0.1, 0.8};
		double graph_color[3] = {0.9, 0.2, 0.1};
		m_pVelocityGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
		//if(m_VehicleCurrentState.speed>0)
		{
			m_pVelocityGraph->InsertPointTimeStamp(m_VehicleCurrentState.tStamp, m_VehicleCurrentState.speed*3.6);
		}
		m_pVelocityGraph->DrawGraph();
	}
	glPopMatrix();

	glPushMatrix();
	glTranslated(10, 750, 0);
	if(m_pSteeringGraph)
	{
		double axes_color[3] = {0.1, 0.1, 0.8};
		double graph_color[3] = {0.9, 0.2, 0.1};
		m_pSteeringGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
		//if(m_VehicleCurrentState.steer>0)
		{
			m_pSteeringGraph->InsertPointTimeStamp(m_VehicleCurrentState.tStamp, m_VehicleCurrentState.steer*RAD2DEG);
		}
		m_pSteeringGraph->DrawGraph();
	}
	glPopMatrix();

	glPushMatrix();
	glTranslated(10, 1000, 0);
	if(m_pLateralErrGraph)
	{
		double axes_color[3] = {0.1, 0.1, 0.8};
		double graph_color[3] = {0.9, 0.2, 0.1};
		m_pLateralErrGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
		//if(m_VehicleCurrentState.steer>0)
		{
			m_pLateralErrGraph->InsertPointTimeStamp(m_VehicleCurrentState.tStamp, m_LateralError);
		}
		m_pLateralErrGraph->DrawGraph();
	}
	glPopMatrix();

	glPushMatrix();
	std::ostringstream performance_str ;
	performance_str.precision(6);
	glColor3f(0.8, 0.5, 0.7);
	glTranslated(10, 1250, 0);
	performance_str << 				"DP Time 		= ";
	performance_str << m_GlobalPlanningTime;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)performance_str.str().c_str());
	glPopMatrix();
	glPushMatrix();
	std::ostringstream local_performance_str ;
	local_performance_str.precision(6);
	glColor3f(0.8, 0.5, 0.7);
	glTranslated(10, 1280, 0);
	local_performance_str << 		"Local Planner 	= ";
	local_performance_str << m_LocalPlanningTime;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)local_performance_str.str().c_str());
	glPopMatrix();
	glPushMatrix();
	std::ostringstream track_performance_str ;
	track_performance_str.precision(6);
	glColor3f(0.8, 0.5, 0.7);
	glTranslated(10, 1310, 0);
	track_performance_str << 		"Tracking Timer	= ";
	track_performance_str << m_ObjectTrakingTime;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)track_performance_str.str().c_str());
	glPopMatrix();
	glPushMatrix();
	std::ostringstream control_performance_str ;
	control_performance_str.precision(6);
	glColor3f(0.8, 0.5, 0.7);
	glTranslated(10, 1340, 0);
	control_performance_str << 		"Control Time	= ";
	control_performance_str << m_ControllingTime;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)control_performance_str.str().c_str());
	glPopMatrix();
	glPushMatrix();
	std::ostringstream simu_performance_str ;
	simu_performance_str.precision(6);
	glColor3f(0.8, 0.5, 0.7);
	glTranslated(10, 1370, 0);
	simu_performance_str << 		"Simu Time 		= ";
	simu_performance_str << m_SimulationTime;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)simu_performance_str.str().c_str());
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
	case 'v':
	{
		SaveSimulationData();
	}
	break;
	case 'l':
	{
		LoadSimulationData();
	}
	break;
	case 'n':
	{
		pthread_mutex_lock(&simulation_mutex);
		m_SimulatedCars.clear();
		m_SimulatedBehaviors.clear();
		m_SimulatedVehicleState.clear();
		m_SimulatedPrevTrajectory.clear();
		m_SimulatedPathFollower.clear();
		pthread_mutex_unlock(&simulation_mutex);
	}
	break;
	default:
		break;

	}
}

void PlannerTestDraw::TransToCarCoordinates(const PlannerHNS::WayPoint& currPose, std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	PlannerHNS::Mat3 rotationMat(-currPose.pos.a);
	PlannerHNS::Mat3 translationMat(-currPose.pos.x, -currPose.pos.y);
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		obj_list.at(i).center.pos = translationMat*obj_list.at(i).center.pos;
		obj_list.at(i).center.pos = rotationMat*obj_list.at(i).center.pos;
		for(unsigned int j = 0 ; j < obj_list.at(i).contour.size(); j++)
		{
			obj_list.at(i).contour.at(j) = translationMat*obj_list.at(i).contour.at(j);
			obj_list.at(i).contour.at(j) = rotationMat*obj_list.at(i).contour.at(j);
		}
	}
}

void PlannerTestDraw::TransToWorldCoordinates(const PlannerHNS::WayPoint& currPose, std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	PlannerHNS::Mat3 rotationMat(currPose.pos.a);
	PlannerHNS::Mat3 translationMat(currPose.pos.x, currPose.pos.y);
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		obj_list.at(i).center.pos = rotationMat*obj_list.at(i).center.pos;
		obj_list.at(i).center.pos = translationMat*obj_list.at(i).center.pos;

		for(unsigned int j = 0 ; j < obj_list.at(i).contour.size(); j++)
		{
			obj_list.at(i).contour.at(j) = translationMat*obj_list.at(i).contour.at(j);
			obj_list.at(i).contour.at(j) = rotationMat*obj_list.at(i).contour.at(j);
		}
	}
}

void* PlannerTestDraw::PlanningThreadStaticEntryPoint(void* pThis)
{
	PlannerTestDraw* pR = (PlannerTestDraw*)pThis;
	struct timespec moveTimer;
	UtilityH::GetTickCount(moveTimer);
	vector<string> logData;
	PlannerHNS::PlannerH planner;
	SimpleTracker obstacleTracking;
	obstacleTracking.Initialize(pR->m_State.state);
	vector<string> behaviorsLogData;

	while(!pR->m_bCancelThread)
	{
		double time_elapsed  = UtilityH::GetTimeDiffNow(moveTimer);

		if(time_elapsed >= pR->m_PlanningCycleTime)
		{
			double dt = time_elapsed;
			UtilityH::GetTickCount(moveTimer);
#ifdef EnableThreadBody
			vector<PlannerHNS::WayPoint> generatedTotalPath = pR->m_State.m_TotalPath;
			bool bNewPlan = false;
			PlannerHNS::VehicleState currTargetState;
			pthread_mutex_lock(&pR->control_mutex);
			currTargetState = pR->m_VehicleTargetState;
			pthread_mutex_unlock(&pR->control_mutex);

			/**
			 * Path Planning Step (Global Planning)
			 */
			int currIndexToal = PlannerHNS::PlanningHelpers::GetClosestPointIndex(pR->m_State.m_TotalPath, pR->m_State.state);
			int index_limit_total = pR->m_State.m_TotalPath.size() - 25;
			if(index_limit_total<=0)
				index_limit_total =  pR->m_State.m_TotalPath.size()/2.0;

			if(currIndexToal > index_limit_total)
			{
				pR->m_bMakeNewPlan = true;
				PlannerHNS::WayPoint g_p = pR->m_goal;
				pR->m_goal = pR->m_start;
				pR->m_start = g_p;
			}

			if((pR->m_CurrentBehavior.state == PlannerHNS::INITIAL_STATE && pR->m_State.m_Path.size() == 0 && pR->m_bMakeNewPlan) || pR->m_bMakeNewPlan )
			{
				//planner.PlanUsingReedShepp(pR->m_State.state, pR->m_goal, generatedPath);
				timespec planTime;
				UtilityH::GetTickCount(planTime);
				planner.PlanUsingDP(pR->m_State.pLane, pR->m_State.state, pR->m_goal, pR->m_State.state, 1000000,pR->m_LanesIds, generatedTotalPath);
				pR->m_GlobalPlanningTime = UtilityH::GetTimeDiffNow(planTime);

				if(generatedTotalPath.size()>0)
					pR->m_goal = generatedTotalPath.at(generatedTotalPath.size()-1);
				pR->m_bMakeNewPlan = false;
				bNewPlan = true;
			}

			/**
			 * Behavior Generator , State Machine , Decision making Step
			 */
			pthread_mutex_lock(&pR->planning_mutex);
			if(bNewPlan)
				pR->m_State.m_TotalPath = generatedTotalPath;



			timespec trackingTimer;
			UtilityH::GetTickCount(trackingTimer);
			std::vector<PlannerHNS::DetectedObject> obj_list;
			pthread_mutex_lock(&pR->simulation_mutex);
			pR->DetectSimulatedObstacles(obj_list);
			pthread_mutex_unlock(&pR->simulation_mutex);

			obstacleTracking.DoOneStep(pR->m_State.state, obj_list);
			obj_list = obstacleTracking.m_DetectedObjects;


			pR->m_ObjectTrakingTime = UtilityH::GetTimeDiffNow(trackingTimer);

			timespec localPlannerTimer;
			UtilityH::GetTickCount(localPlannerTimer);
			pR->m_CurrentBehavior = pR->m_State.DoOneStep(dt, currTargetState, obj_list, pR->m_goal.pos, pR->m_RoadMap);
			pR->m_LocalPlanningTime = UtilityH::GetTimeDiffNow(localPlannerTimer);
			pR->m_VehicleCurrentState.steer = pR->m_State.m_CurrentSteering;
			pR->m_VehicleCurrentState.speed = pR->m_State.m_CurrentVelocity;
			pR->m_VehicleCurrentState.shift = pR->m_State.m_CurrentShift;
			UtilityH::GetTickCount(pR->m_VehicleCurrentState.tStamp);

			if(pR->m_CurrentBehavior.state != PlannerHNS::INITIAL_STATE)
				behaviorsLogData.push_back(pR->m_State.m_pCurrentBehaviorState->GetCalcParams()->ToString(pR->m_CurrentBehavior.state));

			pR->m_dummyObstacles = obj_list;

			pthread_mutex_unlock(&pR->planning_mutex);

			double d = 0;
			if(pR->m_ActualPath.size()>0)
				d = distance2points(pR->m_ActualPath.at(pR->m_ActualPath.size()-1).pos, pR->m_State.state.pos);


			if(pR->m_ActualPath.size() > 0 && d > 0.5 )
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

	DataRW::WriteLogData(UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName+DataRW::StatesLogFolderName, "BehaviorsLog",
			pR->m_State.m_pCurrentBehaviorState->GetCalcParams()->ToStringHeader(), behaviorsLogData );

	return 0;
}

void* PlannerTestDraw::ControlThreadStaticEntryPoint(void* pThis)
{
	PlannerTestDraw* pR = (PlannerTestDraw*)pThis;
	struct timespec moveTimer;
	TrajectoryFollower predControl;
	double timeOfHalf = 0;
	double timeTotal = 0;
	int counter = 0;
	PlannerHNS::VehicleState calibrationTargetState;

	bool bCalibrationMode = false;
	bool bStartCalibration = false;
	timespec delayTimer;
	double timeDelay = 0;
	double totalLateralError = 0;


	predControl.Init(pR->m_ControlParams, pR->m_CarInfo);

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
			if(pR->m_bResetForSimulation)
			{
				predControl.Init(pR->m_ControlParams, pR->m_CarInfo);
				generatedPath.clear();
				pR->m_bResetForSimulation = false;
			}

			currState.steer = pR->m_State.m_CurrentSteering;
			currState.speed = pR->m_State.m_CurrentVelocity;
			currState.shift = pR->m_State.m_CurrentShift;

			pthread_mutex_unlock(&pR->planning_mutex);

			if(bCalibrationMode)
			{
				if(!bStartCalibration)
				{
					calibrationTargetState.speed = 0;
					if(counter == 0)
						calibrationTargetState.steer = pR->m_State.m_CarInfo.max_steer_angle/2.0;
					else if(counter == 1)
						calibrationTargetState.steer = 0;
					else if(counter == 2)
						calibrationTargetState.steer = -pR->m_State.m_CarInfo.max_steer_angle/2.0;
					else if(counter == 3)
						calibrationTargetState.steer = 0;

					UtilityH::GetTickCount(delayTimer);
					timeOfHalf = 0;
					bStartCalibration = true;
				}
				else
				{
					if(abs(abs(currState.steer*RAD2DEG) - abs(calibrationTargetState.steer*RAD2DEG)) < 0.5)
					{
						timeOfHalf = UtilityH::GetTimeDiffNow(delayTimer);
						counter++;
						timeTotal += timeOfHalf;
						bStartCalibration = false;
						if(counter==4)
						{
							bCalibrationMode = false;
							timeDelay = (timeTotal / (double)counter) / (pR->m_State.m_CarInfo.max_steer_angle*RAD2DEG/2.0);
							timeDelay = timeDelay*pR->m_ControlParams.SteeringDelayPercent;

						}
					}
				}

				targetState = calibrationTargetState;
			}
			else if(!bCalibrationMode)
			{
				timespec controlTimer;
				UtilityH::GetTickCount(controlTimer);
				bool bNewPath = false;
				if(PlannerHNS::PlanningHelpers::CompareTrajectories(generatedPath, pR->m_State.m_Path) == false && pR->m_State.m_Path.size()>0)
				{
					generatedPath = pR->m_State.m_Path;
					bNewPath = true;
					cout << "Path is Updated in the controller .. " << pR->m_State.m_Path.size() << endl;
				}

				SimulationNS::ControllerParams c_params = pR->m_ControlParams;
				c_params.SteeringDelay = pR->m_ControlParams.SteeringDelay / (1.0-UtilityH::GetMomentumScaleFactor(currState.speed));
				predControl.Init(c_params, pR->m_CarInfo);
				targetState = predControl.DoOneStep(dt, currMessage, generatedPath, pR->m_State.state, currState, bNewPath);
				pR->m_ControllingTime = UtilityH::GetTimeDiffNow(controlTimer);
			}

			//cout << pR->m_State.m_pCurrentBehaviorState->GetCalcParams()->ToString(currMessage.state) << endl;

			pR->m_FollowPoint  = predControl.m_FollowMePoint.pos;
			pR->m_PerpPoint    = predControl.m_PerpendicularPoint.pos;
			pR->m_LateralError = predControl.m_LateralError;

			//if(abs(pR->m_LateralError) > 0.5)
				totalLateralError = abs(pR->m_LateralError);

//			cout << "S D P  = " << pR->m_ControlParams.SteeringDelayPercent
//				 << ", S D  = " << pR->m_ControlParams.SteeringDelay
//				 << ", T D  = " << timeDelay
//				 << ", L E  = " << pR->m_LateralError
//				 << ", T E  = " << totalLateralError
//				 << ", K P " << pR->m_ControlParams.Steering_Gain.kP
//				 << ", K I " << pR->m_ControlParams.Steering_Gain.kI << endl;


//			if(totalLateralError >= 3.0)
//			{
//				break;
//			}

			pthread_mutex_lock(&pR->control_mutex);
			pR->m_VehicleTargetState = targetState;
			pthread_mutex_unlock(&pR->control_mutex);
#endif
		}
	}

//	if(totalLateralError >= 3.0)
//	{
//		pR->LoadSimulationData();
//		totalLateralError = 0;
//		//pR->m_ControlParams.Steering_Gain.kP += 0.5;
//		//pR->m_ControlParams.Steering_Gain.kD-=0.001;
//		//pR->m_ControlParams.SteeringDelayPercent+=1.0;
//	}

	return 0;
}

void* PlannerTestDraw::SimulationThreadStaticEntryPoint(void* pThis)
{
	PlannerTestDraw* pR = (PlannerTestDraw*)pThis;
	struct timespec moveTimer;
	UtilityH::GetTickCount(moveTimer);
	vector<string> logData;
	PlannerHNS::PlannerH planner;
	std::vector<PlannerHNS::DetectedObject> dummyObstacles;


	while(!pR->m_bCancelThread)
	{
		double time_elapsed  = UtilityH::GetTimeDiffNow(moveTimer);

		if(time_elapsed >= 0.05)
		{
			double dt = time_elapsed;
			UtilityH::GetTickCount(moveTimer);

#ifdef EnableThreadBody
			PlannerHNS::BehaviorState currMessage = pR->m_CurrentBehavior;

			if(currMessage.state != PlannerHNS::INITIAL_STATE && currMessage.state != PlannerHNS::WAITING_STATE)
			{
				pthread_mutex_lock(&pR->simulation_mutex);
				for(unsigned int i = 0 ; i < pR->m_SimulatedCars.size(); i++)
				{
					pR->m_SimulatedBehaviors.at(i) = pR->m_SimulatedCars.at(i).DoOneStep(
							dt, pR->m_SimulatedVehicleState.at(i),
							pR->m_SimulatedCars.at(i).state,
							pR->m_goal.pos, pR->m_RoadMap);

					if(pR->m_SimulatedCars.at(i).m_Path.size() == 0)
					{
						pR->m_SimulatedCars.erase(pR->m_SimulatedCars.begin()+i);
						pR->m_SimulatedBehaviors.erase(pR->m_SimulatedBehaviors.begin()+i);
						pR->m_SimulatedVehicleState.erase(pR->m_SimulatedVehicleState.begin()+i);
						pR->m_SimulatedPrevTrajectory.erase(pR->m_SimulatedPrevTrajectory.begin()+i);
						pR->m_SimulatedPathFollower.erase(pR->m_SimulatedPathFollower.begin()+i);
						i--;
						continue;
					}

					bool bNewPath = false;
					if(PlannerHNS::PlanningHelpers::CompareTrajectories(pR->m_SimulatedPrevTrajectory.at(i), pR->m_SimulatedCars.at(i).m_Path) == false && pR->m_SimulatedCars.at(i).m_Path.size()>0)
					{
						pR->m_SimulatedPrevTrajectory.at(i) = pR->m_SimulatedCars.at(i).m_Path;

						bNewPath = true;
						cout << "Path is Updated in the controller .. " << pR->m_SimulatedCars.at(i).m_Path.size() << endl;
					}

					PlannerHNS::VehicleState currState;
					currState.steer = pR->m_SimulatedCars.at(i).m_CurrentSteering;
					currState.speed = pR->m_SimulatedCars.at(i).m_CurrentVelocity;
					currState.shift = PlannerHNS::SHIFT_POS_DD;

					pR->m_SimulatedVehicleState.at(i) = pR->m_SimulatedPathFollower.at(i).DoOneStep(dt, pR->m_SimulatedBehaviors.at(i), pR->m_SimulatedPrevTrajectory.at(i),
							pR->m_SimulatedCars.at(i).state, currState, bNewPath);
					//pR->m_SimulatedCars.at(i).state.v = pR->m_SimulatedVehicleState.at(i).speed;
				}
				pthread_mutex_unlock(&pR->simulation_mutex);
			}
			pR->m_SimulationTime = UtilityH::GetTimeDiffNow(moveTimer);
#endif
		}
	}

	return 0;
}

} /* namespace Graphics */
