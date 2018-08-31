/*
// *  Copyright (c) 2018, Nagoya University
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
