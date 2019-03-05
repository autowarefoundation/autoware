/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#ifndef OP_BEHAVIOR_SELECTOR_CORE
#define OP_BEHAVIOR_SELECTOR_CORE

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

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ControlCommand.h>
#include <visualization_msgs/MarkerArray.h>

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/DecisionMaker.h"
#include "op_utility/DataRW.h"


namespace BehaviorGeneratorNS
{

class BehaviorGen
{
protected: //Planning Related variables

	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool bVehicleStatus;

	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;
	bool bWayGlobalPath;
	bool bWayGlobalPathLogs;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_RollOuts;
	bool bRollOuts;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;

	PlannerHNS::RoadNetwork m_Map;
	bool bMap;

	PlannerHNS::TrajectoryCost m_TrajectoryBestCost;
	bool bBestCost;

	PlannerHNS::DecisionMaker m_BehaviorGenerator;
	PlannerHNS::BehaviorState m_CurrentBehavior;

  	std::vector<std::string>    m_LogData;

  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;

  	autoware_msgs::Lane m_CurrentTrajectoryToSend;
  	bool bNewLightStatus;
	bool bNewLightSignal;
	PlannerHNS::TrafficLightState  m_CurrLightStatus;
	std::vector<PlannerHNS::TrafficLight> m_CurrTrafficLight;
	std::vector<PlannerHNS::TrafficLight> m_PrevTrafficLight;

	geometry_msgs::TwistStamped m_Twist_raw;
	geometry_msgs::TwistStamped m_Twist_cmd;
	autoware_msgs::ControlCommand m_Ctrl_cmd;

	//ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_LocalPath;
	ros::Publisher pub_LocalBasePath;
	ros::Publisher pub_ClosestIndex;
	ros::Publisher pub_BehaviorState;
	ros::Publisher pub_SimuBoxPose;
	ros::Publisher pub_SelectedPathRviz;

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_LocalPlannerPaths;
	ros::Subscriber sub_TrafficLightStatus;
	ros::Subscriber sub_TrafficLightSignals;
	ros::Subscriber sub_Trajectory_Cost;
	ros::Publisher pub_BehaviorStateRviz;

	ros::Subscriber sub_twist_cmd;
	ros::Subscriber sub_twist_raw;
	ros::Subscriber sub_ctrl_cmd;

	// Callback function for subscriber.
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg);
	void callbackGetTrafficLightStatus(const autoware_msgs::TrafficLight & msg);
	void callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg);

	void callbackGetTwistCMD(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCommandCMD(const autoware_msgs::ControlCommandConstPtr& msg);

	//Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);
  void SendLocalPlanningTopics();
  void VisualizeLocalPlanner();
  void LogLocalPlanningInfo(double dt);

public:
  BehaviorGen();
  ~BehaviorGen();
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

#endif  // OP_BEHAVIOR_SELECTOR_CORE
