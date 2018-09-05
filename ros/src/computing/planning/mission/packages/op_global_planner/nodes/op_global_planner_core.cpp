/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "op_global_planner_core.h"
#include "op_ros_helpers/op_RosHelpers.h"

namespace GlobalPlanningNS
{

GlobalPlanner::GlobalPlanner()
{
	m_pCurrGoal = 0;
	m_iCurrentGoalIndex = 0;
	m_bKmlMap = false;
	m_bFirstStart = false;
	m_GlobalPathID = 1;
	UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);

	nh.getParam("/op_global_planner/pathDensity" , m_params.pathDensity);
	nh.getParam("/op_global_planner/enableSmoothing" , m_params.bEnableSmoothing);
	nh.getParam("/op_global_planner/enableLaneChange" , m_params.bEnableLaneChange);
	nh.getParam("/op_global_planner/enableRvizInput" , m_params.bEnableRvizInput);
	nh.getParam("/op_global_planner/enableReplan" , m_params.bEnableReplanning);
	nh.getParam("/op_global_planner/enableDynamicMapUpdate" , m_params.bEnableDynamicMapUpdate);
	nh.getParam("/op_global_planner/mapFileName" , m_params.KmlMapPath);

	int iSource = 0;
	nh.getParam("/op_global_planner/mapSource", iSource);
	if(iSource == 0)
		m_params.mapSource = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_params.mapSource = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_params.mapSource = PlannerHNS::MAP_KML_FILE;

	tf::StampedTransform transform;
	PlannerHNS::RosHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
	pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
	pub_MapRviz  = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 1, true);
	pub_GoalsListRviz = nh.advertise<visualization_msgs::MarkerArray>("op_destinations_rviz", 1, true);

	if(m_params.bEnableRvizInput)
	{
		sub_start_pose = nh.subscribe("/initialpose", 1, &GlobalPlanner::callbackGetStartPose, this);
		sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &GlobalPlanner::callbackGetGoalPose, this);
	}
	else
	{
		LoadSimulationData();
	}

	sub_current_pose = nh.subscribe("/current_pose", 10, &GlobalPlanner::callbackGetCurrentPose, this);

	int bVelSource = 1;
	nh.getParam("/op_global_planner/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom = nh.subscribe("/odom", 10, &GlobalPlanner::callbackGetRobotOdom, this);
	else if(bVelSource == 1)
		sub_current_velocity = nh.subscribe("/current_velocity", 10, &GlobalPlanner::callbackGetVehicleStatus, this);
	else if(bVelSource == 2)
		sub_can_info = nh.subscribe("/can_info", 10, &GlobalPlanner::callbackGetCanInfo, this);

	if(m_params.bEnableDynamicMapUpdate)
	  sub_road_status_occupancy = nh.subscribe<>("/occupancy_road_status", 1, &GlobalPlanner::callbackGetRoadStatusOccupancyGrid, this);

	//Mapping Section
	sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &GlobalPlanner::callbackGetVMLanes,  this);
	sub_points = nh.subscribe("/vector_map_info/point", 1, &GlobalPlanner::callbackGetVMPoints,  this);
	sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &GlobalPlanner::callbackGetVMdtLanes,  this);
	sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &GlobalPlanner::callbackGetVMIntersections,  this);
	sup_area = nh.subscribe("/vector_map_info/area", 1, &GlobalPlanner::callbackGetVMAreas,  this);
	sub_lines = nh.subscribe("/vector_map_info/line", 1, &GlobalPlanner::callbackGetVMLines,  this);
	sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &GlobalPlanner::callbackGetVMStopLines,  this);
	sub_signals = nh.subscribe("/vector_map_info/signal", 1, &GlobalPlanner::callbackGetVMSignal,  this);
	sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &GlobalPlanner::callbackGetVMVectors,  this);
	sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &GlobalPlanner::callbackGetVMCurbs,  this);
	sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &GlobalPlanner::callbackGetVMRoadEdges,  this);
	sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &GlobalPlanner::callbackGetVMWayAreas,  this);
	sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &GlobalPlanner::callbackGetVMCrossWalks,  this);
	sub_nodes = nh.subscribe("/vector_map_info/node", 1, &GlobalPlanner::callbackGetVMNodes,  this);

}

GlobalPlanner::~GlobalPlanner()
{
	if(m_params.bEnableRvizInput)
		SaveSimulationData();
}

void GlobalPlanner::callbackGetRoadStatusOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg)
{
//	std::cout << "Occupancy Grid Origin (" << msg->info.origin.position.x << ", " << msg->info.origin.position.x << ") , " << msg->header.frame_id << ", Res: " << msg->info.resolution <<  std::endl;

	m_GridMapIntType.clear();

	//std::cout << "Found Map Data: Zero " <<  std::endl;
	for(unsigned int i=0; i < msg->data.size(); i++)
	{
		if((int8_t)msg->data.at(i) == 0)
			m_GridMapIntType.push_back(0);
		else if((int8_t)msg->data.at(i) == 50)
			m_GridMapIntType.push_back(75);
		else if((int8_t)msg->data.at(i) == 100)
			m_GridMapIntType.push_back(255);
		else
			m_GridMapIntType.push_back(128);

			//std::cout << msg->data.at(i) << ",";
	}
	//std::cout << std::endl << "--------------------------------------------------------" << std::endl;

	//std::cout << "Found Map Data: Zero : " << m_GridMapIntType.size() <<  std::endl;
	PlannerHNS::WayPoint center(msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z, tf::getYaw(msg->info.origin.orientation));
	PlannerHNS::OccupancyToGridMap grid(msg->info.width,msg->info.height, msg->info.resolution, center);
	std::vector<PlannerHNS::WayPoint*> modified_nodes;
	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	PlannerHNS::MappingHelpers::UpdateMapWithOccupancyGrid(grid, m_GridMapIntType, m_Map, modified_nodes);
	m_ModifiedMapItemsTimes.push_back(std::make_pair(modified_nodes, t));

	visualization_msgs::MarkerArray map_marker_array;
	PlannerHNS::RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

//	visualization_msgs::Marker mkr = PlannerHNS::RosHelpers::CreateGenMarker(center.pos.x, center.pos.y, center.pos.z, 0, 0,0,1,0.5, 1000, "TestCenter", visualization_msgs::Marker::SPHERE);
//
//	map_marker_array.markers.push_back(mkr);

	pub_MapRviz.publish(map_marker_array);
}

void GlobalPlanner::ClearOldCostFromMap()
{
	for(int i=0; i < (int)m_ModifiedMapItemsTimes.size(); i++)
	{
		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_ModifiedMapItemsTimes.at(i).second) > CLEAR_COSTS_TIME)
		{
			for(unsigned int j= 0 ; j < m_ModifiedMapItemsTimes.at(i).first.size(); j++)
			{
				for(unsigned int i_action=0; i_action < m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.size(); i_action++)
				{
					if(m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).first == PlannerHNS::FORWARD_ACTION)
					{
						m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).second = 0;
					}
				}
			}

			m_ModifiedMapItemsTimes.erase(m_ModifiedMapItemsTimes.begin()+i);
			i--;
		}
	}
}

void GlobalPlanner::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y, msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
	m_GoalsPos.push_back(wp);
	ROS_INFO("Received Goal Pose");
}

void GlobalPlanner::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	m_CurrentPose = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x, msg->pose.pose.position.y+m_OriginPos.position.y, msg->pose.pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.pose.orientation));
	ROS_INFO("Received Start pose");
}

void GlobalPlanner::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
}

void GlobalPlanner::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.twist.linear.x;
	m_CurrentPose.v = m_VehicleState.speed;
	if(fabs(msg->twist.twist.linear.x) > 0.25)
		m_VehicleState.steer += atan(2.7 * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
}

void GlobalPlanner::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.linear.x;
	m_CurrentPose.v = m_VehicleState.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleState.steer = atan(2.7 * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

void GlobalPlanner::callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
	m_VehicleState.speed = msg->speed/3.6;
	m_CurrentPose.v = m_VehicleState.speed;
	m_VehicleState.steer = msg->angle * 0.45 / 660;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

bool GlobalPlanner::GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths)
{
	std::vector<int> predefinedLanesIds;
	double ret = 0;

	ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint, MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange, predefinedLanesIds, m_Map, generatedTotalPaths);

	if(ret == 0)
	{
		std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
												<< ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
		return false;
	}


	if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
	{
		if(m_params.bEnableSmoothing)
		{
			for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
				PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35 , 0.01);
			}
		}

		for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
		{
			PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
			if(m_GlobalPathID > 10000)
				m_GlobalPathID = 1;

			for(unsigned int j=0; j < generatedTotalPaths.at(i).size(); j++)
				generatedTotalPaths.at(i).at(j).gid = m_GlobalPathID;

			m_GlobalPathID++;

			std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;
		}
		return true;
	}
	else
	{
		std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
	}
	return false;
}

void GlobalPlanner::VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths)
{
	autoware_msgs::LaneArray lane_array;
	visualization_msgs::MarkerArray pathsToVisualize;

	for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
	{
		autoware_msgs::lane lane;
		PlannerHNS::RosHelpers::ConvertFromLocalLaneToAutowareLane(generatedTotalPaths.at(i), lane);
		lane_array.lanes.push_back(lane);
	}

	std_msgs::ColorRGBA total_color;
	total_color.r = 0;
	total_color.g = 0.7;
	total_color.b = 1.0;
	total_color.a = 0.9;
	PlannerHNS::RosHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
	PlannerHNS::RosHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
	PlannerHNS::RosHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
	pub_PathsRviz.publish(pathsToVisualize);
	if((m_bFirstStart && m_params.bEnableHMI) || !m_params.bEnableHMI)
		pub_Paths.publish(lane_array);

	for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
	{
		std::ostringstream str_out;
		str_out << UtilityHNS::UtilityH::GetHomeDirectory();
		str_out << UtilityHNS::DataRW::LoggingMainfolderName;
		str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
		str_out << "GlobalPath_";
		str_out << i;
		str_out << "_";
		PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), generatedTotalPaths.at(i));
	}
}

void GlobalPlanner::VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected)
{
	visualization_msgs::MarkerArray goals_array;

	for(unsigned int i=0; i< destinations.size(); i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "HMI_Destinations";
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 3.25;
		marker.scale.y = 3.25;
		marker.scale.z = 3.25;
		marker.color.a = 0.9;
		marker.id = i;
		if(i == iSelected)
		{
			marker.color.r = 1;
			marker.color.g = 0;
			marker.color.b = 0;
		}
		else
		{
			marker.color.r = 0.2;
			marker.color.g = 0.8;
			marker.color.b = 0.2;
		}
		marker.pose.position.x = destinations.at(i).pos.x;
		marker.pose.position.y = destinations.at(i).pos.y;
		marker.pose.position.z = destinations.at(i).pos.z;
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(destinations.at(i).pos.a);

		std::ostringstream str_out;
		str_out << "G";
		marker.text = str_out.str();

		goals_array.markers.push_back(marker);
	}
	pub_GoalsListRviz.publish(goals_array);
}

void GlobalPlanner::SaveSimulationData()
{
	std::vector<std::string> simulationDataPoints;
	std::ostringstream startStr;
	startStr << m_CurrentPose.pos.x << "," << m_CurrentPose.pos.y << "," << m_CurrentPose.pos.z << "," << m_CurrentPose.pos.a << ","<< m_CurrentPose.cost << "," << 0 << ",";
	simulationDataPoints.push_back(startStr.str());

	for(unsigned int i=0; i < m_GoalsPos.size(); i++)
	{
		std::ostringstream goalStr;
		goalStr << m_GoalsPos.at(i).pos.x << "," << m_GoalsPos.at(i).pos.y << "," << m_GoalsPos.at(i).pos.z << "," << m_GoalsPos.at(i).pos.a << "," << 0 << "," << 0 << ",destination_" << i+1 << ",";
		simulationDataPoints.push_back(goalStr.str());
	}

	std::string header = "X,Y,Z,A,C,V,name,";

	std::ostringstream fileName;
	fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName;
	fileName << "EgoCar.csv";
	std::ofstream f(fileName.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < simulationDataPoints.size(); i++)
			f << simulationDataPoints.at(i) << "\r\n";
	}

	f.close();
}

int GlobalPlanner::LoadSimulationData()
{
	std::ostringstream fileName;
	fileName << "EgoCar.csv";

	std::string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName + fileName.str();
	UtilityHNS::SimulationFileReader sfr(simuDataFileName);
	UtilityHNS::SimulationFileReader::SimulationData data;

	int nData = sfr.ReadAllData(data);
	if(nData == 0)
		return 0;

	m_CurrentPose = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
	m_GoalsPos.push_back(PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a));

	for(unsigned int i=0; i < data.simuCars.size(); i++)
	{
		m_GoalsPos.push_back(PlannerHNS::WayPoint(data.simuCars.at(i).x, data.simuCars.at(i).y, data.simuCars.at(i).z, data.simuCars.at(i).a));
	}

	return nData;
}

void GlobalPlanner::MainLoop()
{
	ros::Rate loop_rate(25);
	timespec animation_timer;
	UtilityHNS::UtilityH::GetTickCount(animation_timer);

	while (ros::ok())
	{
		ros::spinOnce();
		bool bMakeNewPlan = false;

		if(m_params.mapSource == PlannerHNS::MAP_KML_FILE && !m_bKmlMap)
		{
			m_bKmlMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_params.KmlMapPath, m_Map);
			visualization_msgs::MarkerArray map_marker_array;
			PlannerHNS::RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
			pub_MapRviz.publish(map_marker_array);
		}
		else if (m_params.mapSource == PlannerHNS::MAP_FOLDER && !m_bKmlMap)
		{
			m_bKmlMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_params.KmlMapPath, m_Map, true);
			visualization_msgs::MarkerArray map_marker_array;
			PlannerHNS::RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

			pub_MapRviz.publish(map_marker_array);
		}
		else if (m_params.mapSource == PlannerHNS::MAP_AUTOWARE && !m_bKmlMap)
		{
			std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

			if(m_MapRaw.GetVersion()==2)
			{
				std::cout << "Map Version 2" << endl;
				m_bKmlMap = true;
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
						m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
			}
			else if(m_MapRaw.GetVersion()==1)
			{
				std::cout << "Map Version 1" << endl;
				m_bKmlMap = true;
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
			}

			if(m_bKmlMap)
			{
				visualization_msgs::MarkerArray map_marker_array;
				PlannerHNS::RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
				pub_MapRviz.publish(map_marker_array);
			}
		}

		ClearOldCostFromMap();

		if(m_GoalsPos.size() > 0)
		{
			if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3)
			{
				if(m_params.bEnableReplanning)
				{
					PlannerHNS::RelativeInfo info;
					bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
					if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
					{
						double remaining_distance =    m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
						if(remaining_distance <= REPLANNING_DISTANCE)
						{
							bMakeNewPlan = true;
							if(m_GoalsPos.size() > 0)
								m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
							std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
						}
					}
				}
			}
			else
				bMakeNewPlan = true;

			if(bMakeNewPlan || (m_params.bEnableDynamicMapUpdate && UtilityHNS::UtilityH::GetTimeDiffNow(m_ReplnningTimer) > REPLANNING_TIME))
			{
				UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);
				PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);
				bool bNewPlan = GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths);


				if(bNewPlan)
				{
					bMakeNewPlan = false;
					VisualizeAndSend(m_GeneratedTotalPaths);
				}
			}
			VisualizeDestinations(m_GoalsPos, m_iCurrentGoalIndex);
		}

		loop_rate.sleep();
	}
}


//Mapping Section

void GlobalPlanner::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << msg.data.size() << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void GlobalPlanner::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << msg.data.size() << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void GlobalPlanner::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << msg.data.size() << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void GlobalPlanner::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << msg.data.size() << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void GlobalPlanner::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << msg.data.size() << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void GlobalPlanner::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << msg.data.size() << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void GlobalPlanner::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << msg.data.size() << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void GlobalPlanner::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << msg.data.size() << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void GlobalPlanner::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << msg.data.size() << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void GlobalPlanner::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << msg.data.size() << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void GlobalPlanner::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << msg.data.size() << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void GlobalPlanner::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << msg.data.size() << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void GlobalPlanner::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << msg.data.size() << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void GlobalPlanner::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << msg.data.size() << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

}
