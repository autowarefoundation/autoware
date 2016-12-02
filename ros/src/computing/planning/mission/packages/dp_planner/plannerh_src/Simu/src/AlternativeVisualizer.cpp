/*
 * AlternativeVisualizer.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#include "AlternativeVisualizer.h"
#include "PlannerH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include <sstream>
#include "MatrixOperations.h"
#include "SimpleTracker.h"
#include "DataRW.h"
#include <algorithm>



using namespace std;
using namespace SimulationNS;
using namespace UtilityHNS;
using namespace PlannerHNS;


namespace Graphics
{

AlternativeVisualizer::AlternativeVisualizer()
{

//	PlannerHNS::MappingHelpers::CreateKmlFromLocalizationPathFile("/home/hatem/Downloads/path_16_2.csv");

//	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(UtilityH::GetHomeDirectory()+
//			DataRW::LoggingMainfolderName + DataRW::VectorMapsFolderName+VectorMap, m_RoadMap);

//	string kml_templateFilePath = UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmlTemplateFile;
//	string kml_fileToSave =UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmltargetFile;
//	PlannerHNS::MappingHelpers::WriteKML(kml_fileToSave, kml_templateFilePath, m_RoadMap);

	PlannerHNS::MappingHelpers::LoadKML("/home/user/data/ToyotaCity1/map/kml/ToyotaKML.kml", m_RoadMap);
	/**
	 * Writing the kml file for the RoadNetwork Map
	 */
//	ostringstream fileName;
//	fileName << UtilityH::GetFilePrefixHourMinuteSeconds();
//	fileName << "_RoadNetwork.kml";
//	PlannerHNS::MappingHelpers::WriteKML(fileName.str(),UtilityH::GetHomeDirectory()+
//			DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmlTemplateFile, m_RoadMap);

	//Initialize Static Traffic Light


//	m_followX = m_start.pos.x;
//	m_followY = m_start.pos.y;
//	m_followZ = m_start.pos.z;
//	m_followA = m_start.pos.a;


	PrepareVectorMapForDrawing();


	}

void AlternativeVisualizer::LoadMaterials()
{
}

AlternativeVisualizer::~AlternativeVisualizer()
{
	if(m_pMap)
	{
		delete m_pMap;
		m_pMap = 0;
	}
}

bool AlternativeVisualizer::IsInitState()
{
	return false;
}

void AlternativeVisualizer::UpdatePlaneStartGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2)
{
}

void AlternativeVisualizer::AddSimulatedCarPos(const double& x,const double& y, const double& a)
{
}

void AlternativeVisualizer::Reset()
{
}

void AlternativeVisualizer::PrepareVectorMapForDrawing()
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

void AlternativeVisualizer::medianfilter(std::vector<double> signal, std::vector<double>& result, int nOrder)
{
	int nNewSize = signal.size()/nOrder;

	for(int i=0; i < nNewSize; i++)
	{
		int index = i*nOrder;
		vector<double> temp;
		temp.insert(temp.begin(), signal.begin()+index, signal.begin()+index+nOrder);
		std::sort(temp.begin(), temp.end());
		result.push_back(temp.at(nOrder/2));
	}

}

void AlternativeVisualizer::DrawGPSData()
{

	// 1- Load Data From file
	vector<UtilityHNS::GPSDataReader::GPSBasicData> gps_data;
	GPSDataReader gps_reader("/home/user/Temp_folder/GPSRawData-noisy.csv");
	gps_reader.ReadAllData(gps_data);
	vector<WayPoint> gpsDataPath;

	// 2- Convert to OpenPlanner data structure
	for(int i = gps_data.size()-1; i >=0 ; i--)
	{
		WayPoint p ;
		p.pos.lat = p.pos.x = gps_data.at(i).lat;
		p.pos.lon = p.pos.y = gps_data.at(i).lon;
		p.pos.alt = p.pos.z = gps_data.at(i).alt;
		gpsDataPath.push_back(p);
	}

	// 3- Specify Origini for transformation
	GPSPoint origin;
	if(gpsDataPath.size() > 0)
	{
		origin = gpsDataPath.at(0).pos;
		m_followX = gpsDataPath.at(0).pos.x;
		m_followY = gpsDataPath.at(0).pos.y;
		m_followZ = gpsDataPath.at(0).pos.z;
		m_followA = gpsDataPath.at(0).pos.a;
	}

	// 4- Convert to Cartesian and scale Up
	MappingHelpers::llaToxyz(origin, GPSPoint());
	GPSPoint prevP = origin;
	vector<double> x_signal;
	vector<double> y_signal;
	for(unsigned int i = 0 ; i < gpsDataPath.size(); i++)
	{
		MappingHelpers::llaToxyz(gpsDataPath.at(i).pos, origin);
		gpsDataPath.at(i).pos.x = gpsDataPath.at(i).pos.x * 100000;
		gpsDataPath.at(i).pos.y = gpsDataPath.at(i).pos.y * 100000;

		x_signal.push_back(gpsDataPath.at(i).pos.x);
		y_signal.push_back(gpsDataPath.at(i).pos.y);

		prevP = gpsDataPath.at(i).pos;
	}


	// 5- using cojugate grandient
	vector<WayPoint> gpsDataPathSmoothed;
	gpsDataPathSmoothed = gpsDataPath;
	PlanningHelpers::CalcAngleAndCost(gpsDataPathSmoothed);

	PlanningHelpers::SmoothPath(gpsDataPathSmoothed, 0.3, 0.46, 1.5);


	// 6- using kalman filter
	vector<WayPoint> gpsDataPathSmoothedKalman = gpsDataPath;
	KFTrack kf(origin.x, origin.y, origin.a, 0);
	for(unsigned int i = 0 ; i < gpsDataPathSmoothedKalman.size(); i++)
	{
		GPSPoint p = gpsDataPathSmoothedKalman.at(i).pos;
		kf.UpdateTrackingFullKalman(p.x, p.y, p.a, p.x, p.y, p.a, gpsDataPathSmoothedKalman.at(i).v);
		gpsDataPathSmoothedKalman.at(i).pos = p;
	}

	// 7- using median filter with order n
	vector<double> x_signal_res;
	vector<double> y_signal_res;
	vector<WayPoint> gpsDataPathSmoothedMedian;

	medianfilter(x_signal, x_signal_res, 3);
	medianfilter(y_signal, y_signal_res, 3);

	for(unsigned int i =0 ; i < x_signal_res.size(); i++)
	{
		WayPoint p ;
		p.pos.x = x_signal_res.at(i);
		p.pos.y = y_signal_res.at(i);

		gpsDataPathSmoothedMedian.push_back(p);
	}


	// 8- Visualizing results
	float PathColor[3] = {1.0, 0.0, 0.0};
	float SmoothPathColor[3] = {0.0, 1.0, 0.0};
	float kalmanPathColor[3] = {0.0, 0.0, 1.0};
	float medianPathColor[3] = {1.0, 1.0, 0.0};

	glDisable(GL_LIGHTING);
	glPushMatrix();
	glTranslated(3,10,0);
	DrawingHelpers::DrawWidePath(gpsDataPath, 0.2, 0.05, PathColor , false);
	glPopMatrix();

	glPushMatrix();
	glTranslated(6,10,0);
	DrawingHelpers::DrawWidePath(gpsDataPathSmoothed, 0.1, 0.05, SmoothPathColor , false);
	glPopMatrix();

	glPushMatrix();
	glTranslated(9,10,0);
	DrawingHelpers::DrawWidePath(gpsDataPathSmoothedKalman, 0.3, 0.05, kalmanPathColor , false);
	glPopMatrix();

	glPushMatrix();
	glTranslated(12,10,0);
	DrawingHelpers::DrawWidePath(gpsDataPathSmoothedMedian, 0.4, 0.05, medianPathColor , false);
	glPopMatrix();

	glEnable(GL_LIGHTING);




}

void AlternativeVisualizer::DrawVectorMap()
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

void AlternativeVisualizer::DrawSimu()
{

	DrawGPSData();
	//DrawVectorMap();

//	float TotalPathColor[3] = {0.99, 0.99, 0.0};
//	float PlannedPathColor[3] = {0.0, 0.99, 0.0};
//	float ActualPathColor[3] = {0.1, 0.1, 0.9};
//
//	pthread_mutex_lock(&planning_mutex);
//	DrawingHelpers::DrawWidePath(m_State.m_TotalPath, 0.08, 0.25, TotalPathColor);
//	DrawingHelpers::DrawWidePath(m_State.m_Path, 0.16, 0.2, PlannedPathColor);
//	DrawingHelpers::DrawWidePath(m_ActualPath, 0.18, 0.15, ActualPathColor);
//
//	glDisable(GL_LIGHTING);
//	for(unsigned int i = 0; i < m_State.m_Path.size(); i+=2 )
//	{
//		if(m_State.m_Path.at(i).collisionCost >= 1)
//		{
//			glColor3f(1, 0, 0);
//			float collisionColor[3] = {1, 0, 0};
//			DrawingHelpers::DrawWideEllipse(m_State.m_Path.at(i).pos.x,
//					m_State.m_Path.at(i).pos.y, 0.2, 1.0, 1.0, 0.8, collisionColor);
//		}
//		else
//		{
//			glColor3f(PlannedPathColor[0], PlannedPathColor[1], PlannedPathColor[2]);
//			DrawingHelpers::DrawSimpleEllipse(m_State.m_Path.at(i).pos.x,
//					m_State.m_Path.at(i).pos.y, 0.2, 1.0, 1.0);
//		}
//
//		glPointSize(10);
//		glBegin(GL_POINTS);
//			glColor3f(0,0,1);
//			glVertex3f(m_State.m_Path.at(i).pos.x, m_State.m_Path.at(i).pos.y, 0.21);
//		glEnd();
//		glPointSize(1);
//
//		glPushMatrix();
//		glTranslated(m_State.m_Path.at(i).pos.x, m_State.m_Path.at(i).pos.y, 0.25);
//		std::ostringstream str_out ;
//		str_out.precision(4);
//		str_out <<  m_State.m_Path.at(i).timeCost;
//		glColor3f(1,0.9,1);
//		DrawingHelpers::DrawString(0, 0,
//				GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//		glPopMatrix();
//	}
//
//
//	float RollOutsColor[3] = {0.9, 0.2, 0.1};
//	for(unsigned int oi=0; oi < m_State.m_RollOuts.size();oi++)
//	{
//		glDisable(GL_LIGHTING);
//		DrawingHelpers::DrawWidePath(m_State.m_RollOuts.at(oi), 0.14, 0.075, RollOutsColor);
//
//		for(unsigned int i = 0; i < m_State.m_RollOuts.at(oi).size(); i+=2 )
//		{
//			if(m_State.m_RollOuts.at(oi).at(i).collisionCost >= 1)
//			{
//				glColor3f(1, 0, 0);
//				float collisionColor[3] = {1, 0, 0};
//				DrawingHelpers::DrawWideEllipse(m_State.m_RollOuts.at(oi).at(i).pos.x,
//						m_State.m_RollOuts.at(oi).at(i).pos.y, 0.2, 1.0, 1.0, 0.8, collisionColor);
//			}
//			else
//			{
//				glColor3f(PlannedPathColor[0], PlannedPathColor[1], PlannedPathColor[2]);
//				DrawingHelpers::DrawSimpleEllipse(m_State.m_RollOuts.at(oi).at(i).pos.x,
//						m_State.m_RollOuts.at(oi).at(i).pos.y, 0.2, 1.0, 1.0);
//			}
//
////			glDisable(GL_LIGHTING);
////			glPointSize(10);
////			glBegin(GL_POINTS);
////			int j_dec = 0;
////			int r_inc = i*8;
////			if(r_inc > 255)
////			{
////				//j_dec = i*5 - 255;
////				r_inc = 255;
////			}
////				glColor3ub(r_inc,255-j_dec,0);
////				glVertex3f(m_State.m_RollOuts.at(oi).at(i).pos.x, m_State.m_RollOuts.at(oi).at(i).pos.y, 0.21);
////			glEnd();
////			glPointSize(1);
//
//			glPushMatrix();
//			glTranslated(m_State.m_RollOuts.at(oi).at(i).pos.x, m_State.m_RollOuts.at(oi).at(i).pos.y, 0.25);
//			std::ostringstream str_out ;
//			str_out.precision(4);
//			str_out <<  m_State.m_RollOuts.at(oi).at(i).timeCost;
//			glColor3f(1,0.9,1);
//			DrawingHelpers::DrawString(0, 0,
//					GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//			glPopMatrix();
//		}
//	}
//
//	for(unsigned int i =0; i <m_dummyObstacles.size(); i++)
//	{
//		float CarColor[3] = {0.9, 0.1, 0.9};
//		DrawingHelpers::DrawCustomCarModel(m_dummyObstacles.at(i).center, m_State.m_CarShapePolygon, CarColor, 90);
//		//std::cout << " >>> Calculated Angle : " << (m_dummyObstacles.at(i).center.pos.a*RAD2DEG + 90)*DEG2RAD << std::endl;
//		glPushMatrix();
//		glTranslated(m_dummyObstacles.at(i).center.pos.x, m_dummyObstacles.at(i).center.pos.y, 1.3);
//		std::ostringstream str_out ;
//		str_out.precision(4);
//		str_out <<  m_dummyObstacles.at(i).center.v *3.6;
//		DrawingHelpers::DrawString(0, 0,
//				GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//		glPopMatrix();
//	}
//
//	for(unsigned int ii = 0; ii < m_State.m_PredictedPath.size(); ii++ )
//	{
//		DrawingHelpers::DrawWidePath(m_State.m_PredictedPath.at(ii), 0.08, 0.25, TotalPathColor);
//
//		for(unsigned int k = 0; k < m_State.m_PredictedPath.at(ii).size(); k++ )
//		{
//			if(m_State.m_PredictedPath.at(ii).at(k).collisionCost >= 1)
//				glColor3f(1, 0, 0);
//			else
//				glColor3f(TotalPathColor[0], TotalPathColor[1], TotalPathColor[2]);
//			DrawingHelpers::DrawSimpleEllipse(m_State.m_PredictedPath.at(ii).at(k).pos.x,
//					m_State.m_PredictedPath.at(ii).at(k).pos.y, 0.25, 1.0, 1.0);
//
//			glPushMatrix();
//			glTranslated(m_State.m_PredictedPath.at(ii).at(k).pos.x, m_State.m_PredictedPath.at(ii).at(k).pos.y, 0.25);
//			std::ostringstream str_out ;
//			str_out.precision(4);
//			str_out <<  m_State.m_PredictedPath.at(ii).at(k).timeCost;
//			glColor3f(1,0.9,1);
//			DrawingHelpers::DrawString(0, 0,
//					GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//			glPopMatrix();
//		}
//
//	}
//
//	pthread_mutex_unlock(&planning_mutex);
//
//	float CarColor[3] = {0.1, 0.9, 0.9};
//
//	pthread_mutex_lock(&simulation_mutex);
//	for(unsigned int i =0; i <m_SimulatedCars.size(); i++)
//	{
//		DrawingHelpers::DrawCustomCarModel(m_SimulatedCars.at(i).state, m_State.m_CarShapePolygon, CarColor, 90);
////
////		glPushMatrix();
////		glTranslated(m_SimulatedCars.at(i).state.pos.x, m_SimulatedCars.at(i).state.pos.y, 1.3);
////		std::ostringstream str_out ;
////		str_out.precision(2);
////		str_out <<  m_SimulatedVehicleState.at(i).speed *3.6;
////		DrawingHelpers::DrawString(0, 1,
////				GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
////		glPopMatrix();
//
//		float TotalPathColor[3] = {0.5, 0.0, 0.99};
//		DrawingHelpers::DrawWidePath(m_SimulatedCars.at(i).m_TotalPath, 0.08, 0.15, TotalPathColor);
//
////		for(unsigned int ii = 0; ii < m_SimulatedCars.at(i).m_TotalPath.size(); ii++ )
////		{
////			if(m_SimulatedCars.at(i).m_TotalPath.at(ii).collisionCost >= 1)
////				glColor3f(1, 0, 0);
////			else
////				glColor3f(TotalPathColor[0], TotalPathColor[1], TotalPathColor[2]);
////			DrawingHelpers::DrawSimpleEllipse(m_SimulatedCars.at(i).m_TotalPath.at(ii).pos.x,
////					m_SimulatedCars.at(i).m_TotalPath.at(ii).pos.y, 0.25, 1.0, 1.0);
////
////
////		}
//
//
//
//
//
//	}
//	pthread_mutex_unlock(&simulation_mutex);
//
//	glColor3f(1,0,0);
//	DrawingHelpers::DrawFilledEllipse(m_FollowPoint.x, m_FollowPoint.y, 0.2, 0.2, 0.2);
//
//	glColor3f(0,0,1);
//	DrawingHelpers::DrawFilledEllipse(m_PerpPoint.x, m_PerpPoint.y, 0.2, 0.2, 0.2);
//
//	if(m_pMap)
//		DrawingHelpers::DrawGrid(m_pMap->origin_x, m_pMap->origin_y, m_pMap->w, m_pMap->h, m_pMap->cell_l);
//
//
//	m_followX = m_State.state.pos.x;
//	m_followY = m_State.state.pos.y;
//	m_followZ = m_State.state.pos.z;
//	m_followA = m_State.state.pos.a;
//
//	if(m_CarModel)
//	{
////		DrawingHelpers::DrawModel(m_CarModel, m_State.m_CarInfo.wheel_base *0.9,
////				m_State.m_CarInfo.wheel_base*0.9, m_State.m_CarInfo.wheel_base*0.9,
////				m_State.state.pos.x,m_State.state.pos.y,
////				m_State.state.pos.z+0.275, m_State.state.pos.a, 0,0);
//	}
//
//	DrawingHelpers::DrawCustomCarModel(m_State.state, m_State.m_CarShapePolygon, CarColor, 90);
//
//
//	//Draw Traffic Light :
//	glDisable(GL_LIGHTING);
//	for(unsigned int i=0 ; i < m_State.m_TrafficLights.size(); i++)
//	{
//		glColor3f(1,0,0);
//		DrawingHelpers::DrawFilledEllipse(m_State.m_TrafficLights.at(i).pos.x, m_State.m_TrafficLights.at(i).pos.y, 1, 1,1);
//		glColor3f(0,1,0);
//		DrawingHelpers::DrawFilledEllipse(m_State.m_TrafficLights.at(i).pos.x+1.2, m_State.m_TrafficLights.at(i).pos.y, 1, 1,1);
//		DrawingHelpers::DrawArrow(m_State.m_TrafficLights.at(i).pos.x+2.5, m_State.m_TrafficLights.at(i).pos.y, m_State.m_TrafficLights.at(i).pos.a);
//	}
//
//	for(unsigned int i=0 ; i < m_goals.size(); i++)
//	{
//		glColor3f(1,0,1);
//		DrawingHelpers::DrawFilledEllipse(m_goals.at(i).pos.x, m_goals.at(i).pos.y, 1.2, 0.2,0.2);
//		DrawingHelpers::DrawArrow(m_goals.at(i).pos.x+2.5, m_goals.at(i).pos.y, m_goals.at(i).pos.a);
//	}
//
//	glColor3f(0,1,1);
//	DrawingHelpers::DrawFilledEllipse(287.504799, 112.376465, 1.2, 0.5,0.5);
//	DrawingHelpers::DrawFilledEllipse(257.398966, 146.998802, 1.2, 0.5,0.5);
//
//	glEnable(GL_LIGHTING);
}

void AlternativeVisualizer::DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY)
{
//	double left_shift = 25;
//	glDisable(GL_LIGHTING);
//	glPushMatrix();
//	glTranslated(centerX-left_shift, 70, 0);
//	glRotated(-1*m_VehicleCurrentState.steer*RAD2DEG*16, 0,0,1);
//	glTranslated(-(centerX-left_shift), -70, 0);
//
//	float wheel_color[3] = {0.6, 0.7, 0.8};
//	DrawingHelpers::DrawWideEllipse(centerX-left_shift, 70, 0.5, 60, 55, 54, wheel_color);
//
//	glColor3f(0.5,0.4, 0.3);
//	PlannerHNS::GPSPoint p1(centerX-left_shift, 70, 0.52, 0), p2(centerX-left_shift+38, 70-38, 0.52, 0);
//	DrawingHelpers::DrawLinePoygonline(p1, p2, 5);
//
//	PlannerHNS::GPSPoint p11(centerX-left_shift, 70, 0.52, 0), p22(centerX-left_shift-38, 70-38, 0.52, 0);
//	DrawingHelpers::DrawLinePoygonline(p11, p22, 5);
//
//	PlannerHNS::GPSPoint p111(centerX-left_shift, 70, 0.52, 0), p222(centerX-left_shift, 70+52, 0.52, 0);
//	DrawingHelpers::DrawLinePoygonline(p111, p222, 5);
//	glPopMatrix();
//
//	double speed = m_VehicleCurrentState.speed*3.6;
//	float pedal_color[3] = {0.5,0.4, 0.3};
//	glColor3f(wheel_color[0],wheel_color[1],wheel_color[2]);
//	DrawingHelpers::DrawPedal(centerX + 70, 70, 0, 30.0, 100.0, speed*5.5,pedal_color );
//
//	glPushMatrix();
//	glTranslated(centerX-left_shift-15, 70+85, 0);
//	glColor3f(0.8, 0.1, 0.7);
//	std::ostringstream str_out ;
//	str_out.precision(2);
//	str_out <<  m_VehicleCurrentState.steer*RAD2DEG;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//	glPopMatrix();
//
//	glPushMatrix();
//	glTranslated(centerX+60, 70+85, 0);
//	glColor3f(0.8, 0.1, 0.7);
//	std::ostringstream v_out ;
//	v_out.precision(2);
//	v_out <<  speed;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)v_out.str().c_str());
//	glPopMatrix();
//
//	glEnable(GL_LIGHTING);
////
////	INITIAL_STATE, WAITING_STATE, FORWARD_STATE, STOPPING_STATE, EMERGENCY_STATE,
////		TRAFFIC_LIGHT_STOP_STATE, STOP_SIGN_STOP_STATE, FOLLOW_STATE, LANE_CHANGE_STATE, OBSTACLE_AVOIDANCE_STATE, FINISH_STATE
//
//	std::ostringstream state_out ;
//	state_out.precision(4);
//	state_out << "State-> ";
//	string str = "Unknown";
//	switch(m_CurrentBehavior.state)
//	{
//	case PlannerHNS::INITIAL_STATE:
//		str = "Init";
//		break;
//	case PlannerHNS::WAITING_STATE:
//		str = "Waiting";
//		break;
//	case PlannerHNS::FORWARD_STATE:
//		str = "Forward";
//		break;
//	case PlannerHNS::STOPPING_STATE:
//		str = "Stop";
//		break;
//	case PlannerHNS::FINISH_STATE:
//		str = "End";
//		break;
//	case PlannerHNS::FOLLOW_STATE:
//		str = "Follow";
//		break;
//	case PlannerHNS::OBSTACLE_AVOIDANCE_STATE:
//		str = "Swerving";
//		break;
//	case PlannerHNS::TRAFFIC_LIGHT_STOP_STATE:
//		str = "Light Stop";
//		break;
//	case PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE:
//		str = "Light Wait";
//		break;
//	default:
//		str = "Unknown";
//		break;
//	}
//	state_out << str;
//	state_out << " (" << m_CurrentBehavior.followDistance << ";"
//			<< m_CurrentBehavior.followVelocity*3.6 << ";"
//			<< m_CurrentBehavior.stopDistance << ";"
//			<< m_State.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ";"
//			<< m_State.m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << ")";
//
//	glPushMatrix();
//	glColor3f(0.8, 0.5, 0.7);
//	glTranslated(10, 200, 0);
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)state_out.str().c_str());
//
//	double y = 30;
//	for(unsigned int i = 0; i < m_dummyObstacles.size(); i++)
//	{
//		std::ostringstream sim_n_out ;
//		sim_n_out << "Simu Car (:" << m_dummyObstacles.at(i).id << ") -> ";
//		sim_n_out << m_dummyObstacles.at(i).center.v;
//		glColor3f(0.6, 0.6, 0.9);
//		DrawingHelpers::DrawString(0, y, GLUT_BITMAP_TIMES_ROMAN_24, (char*)sim_n_out.str().c_str());
//		y+=30;
//	}
//
//	glPopMatrix();
//
//	glPushMatrix();
//	glTranslated(10, 500, 0);
//	if(m_pVelocityGraph)
//	{
//		double axes_color[3] = {0.1, 0.1, 0.8};
//		double graph_color[3] = {0.9, 0.2, 0.1};
//		m_pVelocityGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
//		//if(m_VehicleCurrentState.speed>0)
//		{
//			m_pVelocityGraph->InsertPointTimeStamp(m_VehicleCurrentState.tStamp, m_VehicleCurrentState.speed*3.6);
//		}
//		m_pVelocityGraph->DrawGraph();
//	}
//	glPopMatrix();
//
//	glPushMatrix();
//	glTranslated(10, 750, 0);
//	if(m_pSteeringGraph)
//	{
//		double axes_color[3] = {0.1, 0.1, 0.8};
//		double graph_color[3] = {0.9, 0.2, 0.1};
//		m_pSteeringGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
//		//if(m_VehicleCurrentState.steer>0)
//		{
//			m_pSteeringGraph->InsertPointTimeStamp(m_VehicleCurrentState.tStamp, m_VehicleCurrentState.steer*RAD2DEG);
//		}
//		m_pSteeringGraph->DrawGraph();
//	}
//	glPopMatrix();
//
//	glPushMatrix();
//	glTranslated(10, 1000, 0);
//	if(m_pLateralErrGraph)
//	{
//		double axes_color[3] = {0.1, 0.1, 0.8};
//		double graph_color[3] = {0.9, 0.2, 0.1};
//		m_pLateralErrGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
//		//if(m_VehicleCurrentState.steer>0)
//		{
//			m_pLateralErrGraph->InsertPointTimeStamp(m_VehicleCurrentState.tStamp, m_LateralError);
//		}
//		m_pLateralErrGraph->DrawGraph();
//	}
//	glPopMatrix();
//
//	glPushMatrix();
//	std::ostringstream performance_str ;
//	performance_str.precision(6);
//	glColor3f(0.8, 0.5, 0.7);
//	glTranslated(10, 1250, 0);
//	performance_str << 				"DP Time 		= ";
//	performance_str << m_GlobalPlanningTime;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)performance_str.str().c_str());
//	glPopMatrix();
//	glPushMatrix();
//	std::ostringstream local_performance_str ;
//	local_performance_str.precision(6);
//	glColor3f(0.8, 0.5, 0.7);
//	glTranslated(10, 1280, 0);
//	local_performance_str << 		"Local Planner 	= ";
//	local_performance_str << m_LocalPlanningTime;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)local_performance_str.str().c_str());
//	glPopMatrix();
//	glPushMatrix();
//	std::ostringstream track_performance_str ;
//	track_performance_str.precision(6);
//	glColor3f(0.8, 0.5, 0.7);
//	glTranslated(10, 1310, 0);
//	track_performance_str << 		"Tracking Timer	= ";
//	track_performance_str << m_ObjectTrakingTime;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)track_performance_str.str().c_str());
//	glPopMatrix();
//	glPushMatrix();
//	std::ostringstream control_performance_str ;
//	control_performance_str.precision(6);
//	glColor3f(0.8, 0.5, 0.7);
//	glTranslated(10, 1340, 0);
//	control_performance_str << 		"Control Time	= ";
//	control_performance_str << m_ControllingTime;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)control_performance_str.str().c_str());
//	glPopMatrix();
//	glPushMatrix();
//	std::ostringstream simu_performance_str ;
//	simu_performance_str.precision(6);
//	glColor3f(0.8, 0.5, 0.7);
//	glTranslated(10, 1370, 0);
//	simu_performance_str << 		"Simu Time 		= ";
//	simu_performance_str << m_SimulationTime;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)simu_performance_str.str().c_str());
//	glPopMatrix();

}

void AlternativeVisualizer::OnLeftClick(const double& x, const double& y)
{}

void AlternativeVisualizer::OnRightClick(const double& x, const double& y)
{}

void AlternativeVisualizer::OnKeyboardPress(const SPECIAL_KEYS_TYPE& sKey, const unsigned char& key)
{
	//std::cout << "key" << std::endl;

	switch(key)
	{
	case 's':
		break;
	case 'v':
	{
	}
	break;
	case 'l':
	{
	}
	break;
	case 'n':
	{
	}
	break;
	case 'g':
	{
	}
	break;
	default:
		break;

	}
}

void AlternativeVisualizer::TransToCarCoordinates(const PlannerHNS::WayPoint& currPose, std::vector<PlannerHNS::DetectedObject>& obj_list)
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

void AlternativeVisualizer::TransToWorldCoordinates(const PlannerHNS::WayPoint& currPose, std::vector<PlannerHNS::DetectedObject>& obj_list)
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

} /* namespace Graphics */
