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

#ifndef OP_MAP_CONVERTER
#define OP_MAP_CONVERTER

#include <ros/ros.h>

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
#include "vector_map_msgs/CrossRoadArray.h"
#include "vector_map_msgs/CrossWalkArray.h"
#include "vector_map_msgs/WayAreaArray.h"
#include "vector_map_msgs/CurbArray.h"
#include "vector_map_msgs/RoadEdgeArray.h"

#include "op_planner/RoadNetwork.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_utility/DataRW.h"


namespace MAPCONVERTERNS
{

class Vector2OP
{

protected:
//	UtilityHNS::AisanLanesFileReader* pLanes;
//	UtilityHNS::AisanPointsFileReader* pPoints;
//	UtilityHNS::AisanCenterLinesFileReader* pCenterLines;
//	UtilityHNS::AisanIntersectionFileReader* pIntersections;
//	UtilityHNS::AisanAreasFileReader* pAreas;
//	UtilityHNS::AisanLinesFileReader* pLines;
//	UtilityHNS::AisanStopLineFileReader* pStopLines;
//	UtilityHNS::AisanSignalFileReader* pSignals;
//	UtilityHNS::AisanVectorFileReader* pVectors;
//	UtilityHNS::AisanCurbFileReader* pCurbs;
//	UtilityHNS::AisanRoadEdgeFileReader* pRoadedges;
//	UtilityHNS::AisanWayareaFileReader* pWayAreas;
//	UtilityHNS::AisanCrossWalkFileReader* pCrossWalks;
//	UtilityHNS::AisanNodesFileReader* pNodes;
//	UtilityHNS::AisanDataConnFileReader* pConnections;

	UtilityHNS::MapRaw m_MapRaw;

	ros::NodeHandle nh;

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


	void callbackGetVMLanes(const vector_map_msgs::LaneArrayConstPtr& msg);
	void callbackGetVMPoints(const vector_map_msgs::PointArrayConstPtr& msg);
	void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArrayConstPtr& msg);
	void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArrayConstPtr& msg);
	void callbackGetVMAreas(const vector_map_msgs::AreaArrayConstPtr& msg);
	void callbackGetVMLines(const vector_map_msgs::LineArrayConstPtr& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArrayConstPtr& msg);
	void callbackGetVMSignal(const vector_map_msgs::SignalArrayConstPtr& msg);
	void callbackGetVMVectors(const vector_map_msgs::VectorArrayConstPtr& msg);
	void callbackGetVMCurbs(const vector_map_msgs::CurbArrayConstPtr& msg);
	void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArrayConstPtr& msg);
	void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArrayConstPtr& msg);
	void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArrayConstPtr& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArrayConstPtr& msg);


public:
	Vector2OP();
	virtual ~Vector2OP();
	void MainLoop();
};

}

#endif  // OP_MAP_CONVERTER
