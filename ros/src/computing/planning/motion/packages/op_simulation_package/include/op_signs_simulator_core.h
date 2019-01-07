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

#ifndef OP_SIGNS_SIMULATOR
#define OP_SIGNS_SIMULATOR

#include <ros/ros.h>
//#include <runtime_manager/traffic_light.h>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ExtractedPosition.h>
#include <geometry_msgs/PoseArray.h>
#include <op_planner/RoadNetwork.h>
#include <op_planner/MappingHelpers.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_utility/DataRW.h"


namespace SignsSimulatorNS
{

class SignsCommandParams
{
public:
	std::vector<int>  	firstSignsIds;
	std::vector<int>	secondSignsIds;
	double firstGreenTime;
	double secondGreenTime;
	double firstyellowTime;
	double secondyellowTime;

	void SetCommandParams(const std::string& firstSet, const std::string& secondSet)
	{
		std::vector<std::string> s_list = PlannerHNS::MappingHelpers::SplitString(firstSet, ",");
		for(unsigned int i=0; i < s_list.size(); i++)
		{
			if(s_list.at(i).size()>0)
			{
				firstSignsIds.push_back(strtol(s_list.at(i).c_str(), NULL, 10));
			}
		}

		s_list = PlannerHNS::MappingHelpers::SplitString(secondSet, ",");
		for(unsigned int i=0; i < s_list.size(); i++)
			if(s_list.at(i).size()>0)
			{

				secondSignsIds.push_back(strtol(s_list.at(i).c_str(), NULL, 10));
			}
	}

	SignsCommandParams()
	{
		firstGreenTime = 10;
		secondGreenTime = 10;
		firstyellowTime = 1;
		secondyellowTime = 1;
	}
};

class OpenPlannerSimulatorSigns
{
public:
	autoware_msgs::Signals m_FirstSignals;
	autoware_msgs::Signals m_SecondSignals;

protected:
	ros::NodeHandle nh;
	timespec m_Timer;
	PlannerHNS::TrafficLightState m_CurrLightState;
	SignsCommandParams m_Params;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap;



	ros::Publisher pub_TrafficLightsRviz;
	ros::Publisher pub_trafficLights;

	void VisualizeTrafficLight(autoware_msgs::Signals& _signals);

public:
	OpenPlannerSimulatorSigns();
	virtual ~OpenPlannerSimulatorSigns();
	void MainLoop();

	//Mapping Section

	UtilityHNS::MapRaw m_MapRaw;

	ros::Subscriber sub_lanes;
	ros::Subscriber sub_points;
	ros::Subscriber sub_dt_lanes;
	ros::Subscriber sub_intersect;
	ros::Subscriber sup_area;
	ros::Subscriber sub_lines;
	ros::Subscriber sub_stop_line;
	ros::Subscriber sub_signals;
	ros::Subscriber sub_vectors;
	ros::Subscriber sub_curbs;
	ros::Subscriber sub_edges;
	ros::Subscriber sub_way_areas;
	ros::Subscriber sub_cross_walk;
	ros::Subscriber sub_nodes;


	void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
	void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
	void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
	void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
	void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
	void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
	void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
	void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
	void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
	void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
	void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
	void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
};

}

#endif  // OP_SIGNS_SIMULATOR
