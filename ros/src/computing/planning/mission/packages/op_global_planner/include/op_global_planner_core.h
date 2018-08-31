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

#ifndef OP_GLOBAL_PLANNER
#define OP_GLOBAL_PLANNER

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

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <std_msgs/Int8.h>
#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/CanInfo.h"
#include <visualization_msgs/MarkerArray.h>

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerH.h"

namespace GlobalPlanningNS
{

#define MAX_GLOBAL_PLAN_DISTANCE 100000
#define _ENABLE_VISUALIZE_PLAN
#define REPLANNING_DISTANCE 30
#define REPLANNING_TIME 5
#define ARRIVE_DISTANCE 5
#define CLEAR_COSTS_TIME 15 // seconds

class WayPlannerParams
{
public:
	std::string KmlMapPath;
	bool bEnableSmoothing;
	bool bEnableLaneChange;
	bool bEnableHMI;
	bool bEnableRvizInput;
	bool bEnableReplanning;
	double pathDensity;
	PlannerHNS::MAP_SOURCE_TYPE	mapSource;
	bool bEnableDynamicMapUpdate;


	WayPlannerParams()
	{
	    bEnableDynamicMapUpdate = false;
		bEnableReplanning = false;
		bEnableHMI = false;
		bEnableSmoothing = false;
		bEnableLaneChange = false;
		bEnableRvizInput = true;
		pathDensity = 0.5;
		mapSource = PlannerHNS::MAP_KML_FILE;
	}
};


class GlobalPlanner
{

public:
	int m_iCurrentGoalIndex;
protected:

	WayPlannerParams m_params;
	PlannerHNS::WayPoint m_CurrentPose;
	std::vector<PlannerHNS::WayPoint> m_GoalsPos;
	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::VehicleState m_VehicleState;
	std::vector<int> m_GridMapIntType;
	std::vector<std::pair<std::vector<PlannerHNS::WayPoint*> , timespec> > m_ModifiedMapItemsTimes;
	timespec m_ReplnningTimer;

	int m_GlobalPathID;

	bool m_bFirstStart;

	ros::NodeHandle nh;

	ros::Publisher pub_MapRviz;
	ros::Publisher pub_Paths;
	ros::Publisher pub_PathsRviz;
	ros::Publisher pub_TrafficInfo;
	//ros::Publisher pub_TrafficInfoRviz;
	//ros::Publisher pub_StartPointRviz;
	//ros::Publisher pub_GoalPointRviz;
	//ros::Publisher pub_NodesListRviz;
	ros::Publisher pub_GoalsListRviz;

	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_start_pose;
	ros::Subscriber sub_goal_pose;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_road_status_occupancy;

public:
	GlobalPlanner();
  ~GlobalPlanner();
  void MainLoop();

private:
  PlannerHNS::WayPoint* m_pCurrGoal;

  void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);

  // Callback function for subscriber.
  void callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg);
  void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
  void callbackGetRoadStatusOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg);

  protected:
  	PlannerHNS::RoadNetwork m_Map;
  	bool	m_bKmlMap;
  	PlannerHNS::PlannerH m_PlannerH;
  	std::vector<std::vector<PlannerHNS::WayPoint> > m_GeneratedTotalPaths;

  	bool GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths);
  	void VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths);
  	void VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected);
  	void SaveSimulationData();
  	int LoadSimulationData();
  	void ClearOldCostFromMap();


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

#endif
