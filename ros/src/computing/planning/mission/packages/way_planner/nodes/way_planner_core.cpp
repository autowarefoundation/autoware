/*
 *  Copyright (c) 2016, Nagoya University
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

#include "way_planner_core.h"

namespace WayPlannerNS {

void way_planner_core::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
	static tf::TransformListener listener;

	while (1)
	{
		try
		{
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

way_planner_core::way_planner_core()
{

	m_bKmlMap = false;
	bStartPos = false;
	nh.getParam("/way_planner/pathDensity" 			, m_params.pathDensity);
	nh.getParam("/way_planner/enableSmoothing" 		, m_params.bEnableSmoothing);
	nh.getParam("/way_planner/enableLaneChange" 	, m_params.bEnableLaneChange);
	nh.getParam("/way_planner/enableRvizInput" 		, m_params.bEnableRvizInput);
	int iSource = 0;
	nh.getParam("/way_planner/mapSource" 			, iSource);
	if(iSource == 0)
		m_params.mapSource = MAP_LOADER;
	else if (iSource == 1)
		m_params.mapSource = MAP_SERVER;
	else if(iSource == 2)
		m_params.mapSource = KML_MAP;
	nh.getParam("/way_planner/mapFileName" 			, m_params.KmlMapPath);

	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_Paths = nh.advertise<waypoint_follower::LaneArray>("lane_waypoints_array", 1, true);
	pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
	pub_StartPointRviz = nh.advertise<visualization_msgs::Marker>("Global_StartPoint_rviz", 1, true);
	pub_GoalPointRviz = nh.advertise<visualization_msgs::MarkerArray>("Global_GoalPoints_rviz", 1, true);
	pub_NodesListRviz = nh.advertise<visualization_msgs::MarkerArray>("Goal_Nodes_Points_rviz", 1, true);
	pub_MapRviz  = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 100, true);

	/** @todo To achieve perfection , you need to start sometime */

	if(m_params.bEnableRvizInput)
	{
		sub_start_pose 	= nh.subscribe("/initialpose", 					1, &way_planner_core::callbackGetStartPose, 		this);
		sub_goal_pose 	= nh.subscribe("move_base_simple/goal", 		1, &way_planner_core::callbackGetGoalPose, 		this);
	}
	else
	{
		sub_start_pose 	= nh.subscribe("/GlobalStartPose", 				1, &way_planner_core::callbackGetStartPose, 		this);
		sub_goal_pose 	= nh.subscribe("/GlobalGoalPose", 				1, &way_planner_core::callbackGetGoalPose, 		this);
	}

	sub_nodes_list 		= nh.subscribe("/GlobalNodesList", 				1, &way_planner_core::callbackGetNodesList, 		this);

	if(m_params.mapSource == MAP_LOADER || m_params.mapSource == MAP_SERVER)
	{
		sub_map_points 	= nh.subscribe("/vector_map_info/point", 		1, &way_planner_core::callbackGetVMPoints, 		this);
		sub_map_lanes 	= nh.subscribe("/vector_map_info/lane", 		1, &way_planner_core::callbackGetVMLanes, 		this);
		sub_map_nodes 	= nh.subscribe("/vector_map_info/node", 		1, &way_planner_core::callbackGetVMNodes, 		this);
		sup_stop_lines 	= nh.subscribe("/vector_map_info/stop_line",	1, &way_planner_core::callbackGetVMStopLines, 	this);
		sub_dtlanes 	= nh.subscribe("/vector_map_info/dtlane", 		1, &way_planner_core::callbackGetVMCenterLines,	this);
	}
	else if(m_params.mapSource == KML_MAP)
	{

	}
}

way_planner_core::~way_planner_core(){
}

void way_planner_core::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	if(bStartPos)
	{
		m_GoalPos = msg->pose;

		bool bNewPlan = false;
		PlannerHNS::WayPoint startPoint(m_StartPos.position.x+m_OriginPos.position.x,
				m_StartPos.position.y+m_OriginPos.position.y,
				m_StartPos.position.z+m_OriginPos.position.z, tf::getYaw(m_StartPos.orientation));
		PlannerHNS::WayPoint goalPoint(m_GoalPos.position.x+m_OriginPos.position.x,
				m_GoalPos.position.y+m_OriginPos.position.y,
				m_GoalPos.position.z+m_OriginPos.position.z, tf::getYaw(m_GoalPos.orientation));

		PlannerHNS::WayPoint* pStart = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(startPoint, m_Map);
		PlannerHNS::WayPoint* pGoal = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(goalPoint, m_Map);
		std::vector<int> predefinedLanesIds;
		std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths;
		if(pStart && pGoal && bStartPos)
		{
			double ret = m_PlannerH.PlanUsingDP(pStart->pLane,
					*pStart, *pGoal,
					*pStart, MAX_GLOBAL_PLAN_DISTANCE,
					predefinedLanesIds, generatedTotalPaths);

			if(ret == 0) generatedTotalPaths.clear();

			if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
			{
				for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
				{
					PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
					if(m_params.bEnableSmoothing)
					{
						PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35 , 0.01);
						PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
					}


				}
				std::cout << "New DP Path -> " << generatedTotalPaths.size() << std::endl;
				bNewPlan = true;
			}
		}

		if(bNewPlan)
		{
			bStartPos = false;
			waypoint_follower::LaneArray lane_array;
			for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
				RosHelpers::ConvertFromPlannerHToAutowarePathFormat(generatedTotalPaths.at(i), lane_array);
			pub_Paths.publish(lane_array);

			visualization_msgs::MarkerArray pathsToVisualize;
			std_msgs::ColorRGBA total_color;
			total_color.r = 0;
			total_color.g = 0.7;
			total_color.b = 1.0;
			total_color.a = 0.2;
			RosHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
			RosHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
			RosHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
			//RosHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(generatedTotalPaths, pathsToVisualize);
			pub_PathsRviz.publish(pathsToVisualize);
		}
		else
		{
			std::cout << "Can;t Generate Global Path for Start (" << startPoint.pos.ToString()
					<< ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
		}

	}

}

void way_planner_core::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	if(!bStartPos)
	{
		m_StartPos = msg->pose.pose;
		bStartPos = true;
	}
}

void way_planner_core::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	ROS_INFO("Received Map Points");
	m_AwMap.points = msg;
	m_AwMap.bPoints = true;
}

void way_planner_core::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	ROS_INFO("Received Map Lane Array");
	m_AwMap.lanes = msg;
	m_AwMap.bLanes = true;
}

void way_planner_core::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	//ROS_INFO("Received Map Nodes");


}

void way_planner_core::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	//ROS_INFO("Received Map Stop Lines");
}

void way_planner_core::callbackGetVMCenterLines(const vector_map_msgs::DTLaneArray& msg)
{
	ROS_INFO("Received Map Center Lines");
	m_AwMap.dtlanes = msg;
	m_AwMap.bDtLanes = true;
}

void way_planner_core::callbackGetNodesList(const vector_map_msgs::NodeArray& msg)
{

}

void way_planner_core::UpdateRoadMap(const AutowareRoadNetwork& src_map, PlannerHNS::RoadNetwork& out_map)
{
	std::vector<UtilityHNS::AisanLanesFileReader::AisanLane> lanes;
	for(unsigned int i=0; i < src_map.lanes.data.size();i++)
	{
		UtilityHNS::AisanLanesFileReader::AisanLane l;
		l.BLID 		=  src_map.lanes.data.at(i).blid;
		l.BLID2 	=  src_map.lanes.data.at(i).blid2;
		l.BLID3 	=  src_map.lanes.data.at(i).blid3;
		l.BLID4 	=  src_map.lanes.data.at(i).blid4;
		l.BNID 		=  src_map.lanes.data.at(i).bnid;
		l.ClossID 	=  src_map.lanes.data.at(i).clossid;
		l.DID 		=  src_map.lanes.data.at(i).did;
		l.FLID 		=  src_map.lanes.data.at(i).flid;
		l.FLID2 	=  src_map.lanes.data.at(i).flid2;
		l.FLID3 	=  src_map.lanes.data.at(i).flid3;
		l.FLID4 	=  src_map.lanes.data.at(i).flid4;
		l.FNID 		=  src_map.lanes.data.at(i).fnid;
		l.JCT 		=  src_map.lanes.data.at(i).jct;
		l.LCnt 		=  src_map.lanes.data.at(i).lcnt;
		l.LnID 		=  src_map.lanes.data.at(i).lnid;
		l.Lno 		=  src_map.lanes.data.at(i).lno;
		l.Span 		=  src_map.lanes.data.at(i).span;
		l.RefVel	=  src_map.lanes.data.at(i).refvel;
		l.LimitVel	=  src_map.lanes.data.at(i).limitvel;

//		l.LaneChgFG =  src_map.lanes.at(i).;
//		l.LaneType 	=  src_map.lanes.at(i).blid;
//		l.LimitVel 	=  src_map.lanes.at(i).;
//		l.LinkWAID 	=  src_map.lanes.at(i).blid;
//		l.RefVel 	=  src_map.lanes.at(i).blid;
//		l.RoadSecID =  src_map.lanes.at(i).;

		lanes.push_back(l);
	}

	std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints> points;

	for(unsigned int i=0; i < src_map.points.data.size();i++)
	{
		UtilityHNS::AisanPointsFileReader::AisanPoints p;
		double integ_part = src_map.points.data.at(i).l;
		double deg = trunc(src_map.points.data.at(i).l);
		double min = trunc((src_map.points.data.at(i).l - deg) * 100.0) / 60.0;
		double sec = modf((src_map.points.data.at(i).l - deg) * 100.0, &integ_part)/36.0;
		double L =  deg + min + sec;

		deg = trunc(src_map.points.data.at(i).b);
		min = trunc((src_map.points.data.at(i).b - deg) * 100.0) / 60.0;
		sec = modf((src_map.points.data.at(i).b - deg) * 100.0, &integ_part)/36.0;
		double B =  deg + min + sec;

		p.B 		= B;
		p.Bx 		= src_map.points.data.at(i).bx;
		p.H 		= src_map.points.data.at(i).h;
		p.L 		= L;
		p.Ly 		= src_map.points.data.at(i).ly;
		p.MCODE1 	= src_map.points.data.at(i).mcode1;
		p.MCODE2 	= src_map.points.data.at(i).mcode2;
		p.MCODE3 	= src_map.points.data.at(i).mcode3;
		p.PID 		= src_map.points.data.at(i).pid;
		p.Ref 		= src_map.points.data.at(i).ref;

		points.push_back(p);
	}


	std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine> dts;
	for(unsigned int i=0; i < src_map.dtlanes.data.size();i++)
	{
		UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine dt;

		dt.Apara 	= src_map.dtlanes.data.at(i).apara;
		dt.DID 		= src_map.dtlanes.data.at(i).did;
		dt.Dir 		= src_map.dtlanes.data.at(i).dir;
		dt.Dist 	= src_map.dtlanes.data.at(i).dist;
		dt.LW 		= src_map.dtlanes.data.at(i).lw;
		dt.PID 		= src_map.dtlanes.data.at(i).pid;
		dt.RW 		= src_map.dtlanes.data.at(i).rw;
		dt.cant 	= src_map.dtlanes.data.at(i).cant;
		dt.r 		= src_map.dtlanes.data.at(i).r;
		dt.slope 	= src_map.dtlanes.data.at(i).slope;

		dts.push_back(dt);
	}

	PlannerHNS::GPSPoint origin;//(m_OriginPos.position.x, m_OriginPos.position.y, m_OriginPos.position.z, 0);
	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(lanes, points, dts, origin, out_map);
}

void way_planner_core::PlannerMainLoop()
{
	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		ros::spinOnce();

		//std::cout << "Main Loop ! " << std::endl;
		if(m_params.mapSource == KML_MAP && !m_bKmlMap)
		{
			m_bKmlMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_params.KmlMapPath, m_Map);
		}
		else if(m_params.mapSource == MAP_LOADER || m_params.mapSource == MAP_SERVER)
		{
			 if(m_AwMap.bDtLanes && m_AwMap.bLanes && m_AwMap.bPoints)
			 {
				 m_AwMap.bDtLanes = m_AwMap.bLanes = m_AwMap.bPoints = false;
				 UpdateRoadMap(m_AwMap,m_Map);
			 }
		}

		visualization_msgs::MarkerArray map_marker_array;
		RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
		pub_MapRviz.publish(map_marker_array);

		//ROS_INFO("Main Loop Step");
		loop_rate.sleep();
	}
}

}
