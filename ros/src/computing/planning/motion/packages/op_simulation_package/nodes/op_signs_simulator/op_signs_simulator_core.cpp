/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "op_signs_simulator_core.h"
#include "op_utility/UtilityH.h"
#include "math.h"
#include "op_planner/MatrixOperations.h"
#include "op_ros_helpers/op_ROSHelpers.h"


namespace SignsSimulatorNS
{

OpenPlannerSimulatorSigns::OpenPlannerSimulatorSigns()
{
	bMap = false;

	std::string first_str, second_str;
	ros::NodeHandle _nh("~");
	_nh.getParam("first_signs_list_ids" , first_str);
	_nh.getParam("second_signs_list_ids" , second_str);

	_nh.getParam("first_green_time" , m_Params.firstGreenTime);
	_nh.getParam("second_green_time" , m_Params.secondGreenTime);
	_nh.getParam("first_yellow_time" , m_Params.firstyellowTime);
	_nh.getParam("second_yellow_time" , m_Params.secondyellowTime);

	int iSource = 0;
	_nh.getParam("mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if(iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("mapFileName" , m_MapPath);

	std::cout << first_str  << " | " <<   second_str << std::endl;
	m_Params.SetCommandParams(first_str, second_str);

	for(unsigned int i = 0; i < m_Params.firstSignsIds.size(); i++)
	{
		autoware_msgs::ExtractedPosition s;
		s.signalId = m_Params.firstSignsIds.at(i);
		s.type = 1;
		m_FirstSignals.Signals.push_back(s);
	}

	for(unsigned int i=0; i < m_Params.secondSignsIds.size(); i++)
	{
		autoware_msgs::ExtractedPosition s;
		s.signalId = m_Params.secondSignsIds.at(i);
		s.type = 0;
		m_SecondSignals.Signals.push_back(s);
	}


	pub_TrafficLightsRviz = nh.advertise<visualization_msgs::MarkerArray>("op_traffic_lights_rviz", 1);

	pub_trafficLights 	= nh.advertise<autoware_msgs::Signals>("roi_signal",1);

	UtilityHNS::UtilityH::GetTickCount(m_Timer);
	m_CurrLightState = PlannerHNS::GREEN_LIGHT;


	//Mapping Section
	sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &OpenPlannerSimulatorSigns::callbackGetVMLanes,  this);
	sub_points = nh.subscribe("/vector_map_info/point", 1, &OpenPlannerSimulatorSigns::callbackGetVMPoints,  this);
	sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &OpenPlannerSimulatorSigns::callbackGetVMdtLanes,  this);
	sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &OpenPlannerSimulatorSigns::callbackGetVMIntersections,  this);
	sup_area = nh.subscribe("/vector_map_info/area", 1, &OpenPlannerSimulatorSigns::callbackGetVMAreas,  this);
	sub_lines = nh.subscribe("/vector_map_info/line", 1, &OpenPlannerSimulatorSigns::callbackGetVMLines,  this);
	sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &OpenPlannerSimulatorSigns::callbackGetVMStopLines,  this);
	sub_signals = nh.subscribe("/vector_map_info/signal", 1, &OpenPlannerSimulatorSigns::callbackGetVMSignal,  this);
	sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &OpenPlannerSimulatorSigns::callbackGetVMVectors,  this);
	sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &OpenPlannerSimulatorSigns::callbackGetVMCurbs,  this);
	sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &OpenPlannerSimulatorSigns::callbackGetVMRoadEdges,  this);
	sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &OpenPlannerSimulatorSigns::callbackGetVMWayAreas,  this);
	sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &OpenPlannerSimulatorSigns::callbackGetVMCrossWalks,  this);
	sub_nodes = nh.subscribe("/vector_map_info/node", 1, &OpenPlannerSimulatorSigns::callbackGetVMNodes,  this);

	std::cout << "OpenPlannerSimulatorSigns initialized successfully " << std::endl;

}

OpenPlannerSimulatorSigns::~OpenPlannerSimulatorSigns()
{
}

void OpenPlannerSimulatorSigns::VisualizeTrafficLight(autoware_msgs::Signals& _signals)
{
	std::vector<PlannerHNS::TrafficLight> simulatedLights;
	for(unsigned int i = 0 ; i < _signals.Signals.size() ; i++)
	{
		PlannerHNS::TrafficLight tl;
		tl.id = _signals.Signals.at(i).signalId;

		for(unsigned int k = 0; k < m_Map.trafficLights.size(); k++)
		{
			if(m_Map.trafficLights.at(k).id == tl.id)
			{
				tl.pos = m_Map.trafficLights.at(k).pos;
				break;
			}
		}

		if(_signals.Signals.at(i).type == 1)
		{
			tl.lightState = PlannerHNS::GREEN_LIGHT;
		}
		else
		{
			tl.lightState = PlannerHNS::RED_LIGHT;
		}

		simulatedLights.push_back(tl);
	}

	//visualize traffic light
	visualization_msgs::MarkerArray lights;
	PlannerHNS::ROSHelpers::GetTrafficLightForVisualization(simulatedLights, lights);
	pub_TrafficLightsRviz.publish(lights);
}

void OpenPlannerSimulatorSigns::MainLoop()
{

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		ros::spinOnce();

		if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_MapPath, m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_MapPath, m_Map, true);
		}
		else if (m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap)
		{
			std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

			if(m_MapRaw.GetVersion()==2)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
						m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map V2 Is Loaded successfully from the Sign Simulator!! " << std::endl;
				}
			}
			else if(m_MapRaw.GetVersion()==1)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map V1 Is Loaded successfully from the Sign Simulator !! " << std::endl;
				}
			}
		}

		if(m_CurrLightState == PlannerHNS::GREEN_LIGHT)
		{
			//std::cout << "Greeeeeen" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.firstGreenTime)
			{
				for(unsigned int i =0; i<m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 1;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 0;
			}
			else
			{
				m_CurrLightState = PlannerHNS::YELLOW_LIGHT;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}
		else if(m_CurrLightState == PlannerHNS::YELLOW_LIGHT)
		{
			//std::cout << "Yelowwwwww" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.firstyellowTime)
			{
				for(unsigned int i =0; i< m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 0;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 0;
			}
			else
			{
				m_CurrLightState = PlannerHNS::RED_LIGHT;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}
		else if(m_CurrLightState == PlannerHNS::RED_LIGHT)
		{
			//std::cout << "Reeeeeeed" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.secondGreenTime)
			{
				for(unsigned int i =0; i<m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 0;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 1;
			}
			else
			{
				m_CurrLightState = PlannerHNS::FLASH_YELLOW;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}
		else if(m_CurrLightState == PlannerHNS::FLASH_YELLOW)
		{
			//std::cout << "Yelowwwwww" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.secondyellowTime)
			{
				for(unsigned int i =0; i<m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 0;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 0;
			}
			else
			{
				m_CurrLightState = PlannerHNS::GREEN_LIGHT;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}

		autoware_msgs::Signals all_signals;
		all_signals.Signals.insert(all_signals.Signals.end(), m_FirstSignals.Signals.begin(), m_FirstSignals.Signals.end());
		all_signals.Signals.insert(all_signals.Signals.end(), m_SecondSignals.Signals.begin(), m_SecondSignals.Signals.end());

		//std::cout << "Number of Signals before send: " << all_signals.Signals.size()  << std::endl;
		pub_trafficLights.publish(all_signals);

		if(bMap)
			VisualizeTrafficLight(all_signals);

		loop_rate.sleep();
	}
}

//Mapping Section

void OpenPlannerSimulatorSigns::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void OpenPlannerSimulatorSigns::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

}
