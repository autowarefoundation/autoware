/*
// *  Copyright (c) 2016, Nagoya University
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

#ifndef WAYPLANNERCORE_H_
#define WAYPLANNERCORE_H_

#include <ros/ros.h>

//#include <map_file/PointClassArray.h>
//#include <map_file/LaneArray.h>
//#include <map_file/NodeArray.h>
//#include <map_file/StopLineArray.h>
//#include <map_file/DTLaneArray.h>
#include <vector_map_msgs/PointArray.h>
#include <vector_map_msgs/LaneArray.h>
#include <vector_map_msgs/NodeArray.h>
#include <vector_map_msgs/StopLineArray.h>
#include <vector_map_msgs/DTLaneArray.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <std_msgs/Int8.h>
#include "waypoint_follower/libwaypoint_follower.h"
#include "waypoint_follower/LaneArray.h"
#include <visualization_msgs/MarkerArray.h>

#include "MappingHelpers.h"
#include "PlannerH.h"
#include "RosHelpers.h"

namespace WayPlannerNS {


#define MAX_GLOBAL_PLAN_DISTANCE 10000

class AutowareRoadNetwork
{
public:
	vector_map_msgs::PointArray 	points;
	vector_map_msgs::LaneArray	 	lanes;
	vector_map_msgs::NodeArray 		nodes;
	vector_map_msgs::StopLineArray 	stoplines;
	vector_map_msgs::DTLaneArray 	dtlanes; //center lines
	bool bPoints;
	bool bLanes;
	bool bNodes;
	bool bStopLines;
	bool bDtLanes;

	AutowareRoadNetwork()
	{
		bPoints 	= false;
		bLanes  	= false;
		bStopLines 	= false;
		bDtLanes  	= false;
		bNodes 		= false;
	}
};

enum MAP_SOURCE_TYPE{MAP_LOADER, MAP_SERVER, KML_MAP};

class WayPlannerParams
{
public:
	std::string 	KmlMapPath;
	bool 			bEnableSmoothing;
	bool 			bEnableLaneChange;
	bool 			bEnableRvizInput;
	double 			pathDensity;
	MAP_SOURCE_TYPE	mapSource;


	WayPlannerParams()
	{
		bEnableSmoothing 	= false;
		bEnableLaneChange 	= false;
		bEnableRvizInput 	= true;
		pathDensity			= 0.5;
		mapSource 			= KML_MAP;
	}
};

class way_planner_core
{
protected:

	WayPlannerParams m_params;
	AutowareRoadNetwork m_AwMap;
	geometry_msgs::Pose m_StartPos;
	geometry_msgs::Pose m_CurrentPose;
	bool bStartPos;
	bool bUsingCurrentPose;
	geometry_msgs::Pose m_GoalPos;
	//bool bGoalPos;
	geometry_msgs::Pose m_OriginPos;

	std::vector<geometry_msgs::PoseStamped> m_NodesList;

	ros::NodeHandle nh;

	ros::Publisher pub_MapRviz;
	ros::Publisher pub_Paths;
	ros::Publisher pub_PathsRviz;
	ros::Publisher pub_TrafficInfo;
	ros::Publisher pub_TrafficInfoRviz;
	ros::Publisher pub_StartPointRviz;
	ros::Publisher pub_GoalPointRviz;
	ros::Publisher pub_NodesListRviz;

	ros::Subscriber sub_start_pose;
	ros::Subscriber sub_goal_pose;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_nodes_list;
	ros::Subscriber sub_map_points;
	ros::Subscriber sub_map_lanes;
	ros::Subscriber sub_map_nodes;
	ros::Subscriber sup_stop_lines;
	ros::Subscriber sub_dtlanes;

public:
	way_planner_core();
  ~way_planner_core();
  void PlannerMainLoop();

private:

  void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);

  // Callback function for subscriber.
  void callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);

  void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
  void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
  void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
  void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
  void callbackGetVMCenterLines(const vector_map_msgs::DTLaneArray& msg);
  void callbackGetNodesList(const vector_map_msgs::NodeArray& msg);

  protected:
  	PlannerHNS::RoadNetwork m_Map;
  	bool	m_bKmlMap;
  	PlannerHNS::PlannerH m_PlannerH;
  	void UpdateRoadMap(const AutowareRoadNetwork& src_map, PlannerHNS::RoadNetwork& out_map);


};

}

#endif
