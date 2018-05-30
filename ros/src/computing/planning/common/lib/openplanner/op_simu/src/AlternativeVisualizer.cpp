/*
 * AlternativeVisualizer.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#include "op_simu/AlternativeVisualizer.h"
#include "op_planner/PlannerH.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include <sstream>
#include "op_planner/MatrixOperations.h"
#include "op_simu/SimpleTracker.h"
#include "op_utility/DataRW.h"
#include <algorithm>


using namespace std;
using namespace SimulationNS;
using namespace UtilityHNS;
using namespace PlannerHNS;


namespace Graphics
{

AlternativeVisualizer::AlternativeVisualizer()
{
	/**
	 * Writing the kml file for the RoadNetwork Map
	 */
//	std::vector<TrafficLight> trafficLights;
//	std::vector<GPSPoint> stopLines;
//	PlannerHNS::MappingHelpers::CreateKmlFromLocalizationPathFile("/home/user/Downloads/pose.csv", 50, 0.5, trafficLights, stopLines);
//	string kml_templateFilePath = UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmlTemplateFile;
//	string kml_fileToSave =UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmltargetFile;
//	PlannerHNS::MappingHelpers::WriteKML(kml_fileToSave, kml_templateFilePath, m_RoadMap);
//	PlannerHNS::MappingHelpers::LoadKML("/home/user/SimuLogs/road_network_test.kml", m_RoadMap);

	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles("/media/hatem/8ac0c5d5-8793-4b98-8728-55f8d67ec0f4/data/ToyotaCity2/map/vector_map/", m_RoadMap);
	m_start =  PlannerHNS::MappingHelpers::GetFirstWaypoint(m_RoadMap);
	PrepareVectorMapForDrawing();
}

void AlternativeVisualizer::LoadMaterials()
{
}

AlternativeVisualizer::~AlternativeVisualizer()
{
}

bool AlternativeVisualizer::IsInitState()
{
	return false;
}

void AlternativeVisualizer::UpdatePlaneStartGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2)
{
	m_start.pos.x = x1;
	m_start.pos.y = y1;
	m_start.pos.a = a1;

	m_goal.pos.x = x2;
	m_goal.pos.y = y2;
	m_goal.pos.a = a2;

	PlannerHNS::PlannerH planner;
	m_GeneratedPath.clear();
	planner.PlanUsingReedShepp(m_start, m_goal, m_GeneratedPath, 0.5, 20);
	cout << "Path is Generated: " << m_GeneratedPath.size() << endl;
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

			DrawingHelpers::PreparePathForDrawing(path_local,ready_to_draw, 2.8 / width_ratio, 0.5);
			m_ReadyToDrawLanes.push_back(ready_to_draw);

			ready_to_draw.clear();
			DrawingHelpers::PreparePathForDrawing(path_local,ready_to_draw, 0.1, 0.5);
			m_ReadyToDrawCenterLines.push_back(ready_to_draw);


			j++;

			PlannerHNS::MappingHelpers::GetUniqueNextLanes(l, traversed_lanes, lanes_list);
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
//	MappingHelpers::llaToxyz(origin, GPSPoint());
//	GPSPoint prevP = origin;
//	vector<double> x_signal;
//	vector<double> y_signal;
//	for(unsigned int i = 0 ; i < gpsDataPath.size(); i++)
//	{
//		MappingHelpers::llaToxyz(gpsDataPath.at(i).pos, origin);
//		gpsDataPath.at(i).pos.x = gpsDataPath.at(i).pos.x * 100000;
//		gpsDataPath.at(i).pos.y = gpsDataPath.at(i).pos.y * 100000;
//
//		x_signal.push_back(gpsDataPath.at(i).pos.x);
//		y_signal.push_back(gpsDataPath.at(i).pos.y);
//
//		prevP = gpsDataPath.at(i).pos;
//	}


	// 5- using cojugate grandient
	vector<WayPoint> gpsDataPathSmoothed;
	gpsDataPathSmoothed = gpsDataPath;
	PlanningHelpers::CalcAngleAndCost(gpsDataPathSmoothed);

	PlanningHelpers::SmoothPath(gpsDataPathSmoothed, 0.3, 0.46, 1.5);


	// 6- using kalman filter
	vector<WayPoint> gpsDataPathSmoothedKalman = gpsDataPath;
/*	KFTrackV kf(origin.x, origin.y, origin.a, 0, 1);
	for(unsigned int i = 0 ; i < gpsDataPathSmoothedKalman.size(); i++)
	{
		GPSPoint p = gpsDataPathSmoothedKalman.at(i).pos;
		kf.UpdateTracking(0.1, p.x, p.y, p.a, p.x, p.y, p.a, gpsDataPathSmoothedKalman.at(i).v);
		gpsDataPathSmoothedKalman.at(i).pos = p;
	}
*/
	// 7- using median filter with order n
	vector<double> x_signal_res;
	vector<double> y_signal_res;
	vector<WayPoint> gpsDataPathSmoothedMedian;

	//medianfilter(x_signal, x_signal_res, 3);
	//medianfilter(y_signal, y_signal_res, 3);

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


	//DrawGPSData();
	//DrawVectorMap();
	float color[] = {0, 1, 0};
	DrawingHelpers::DrawWidePath(m_GeneratedPath, 0.5, 0.5, color);
}

void AlternativeVisualizer::DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY)
{
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
