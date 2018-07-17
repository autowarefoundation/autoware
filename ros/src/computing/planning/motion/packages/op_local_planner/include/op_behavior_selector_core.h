/*
// *  Copyright (c) 2017, Nagoya University
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
#include <autoware_msgs/CanInfo.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/traffic_light.h>
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

  	autoware_msgs::lane m_CurrentTrajectoryToSend;
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
	void callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalTrajectoryCost(const autoware_msgs::laneConstPtr& msg);
	void callbackGetTrafficLightStatus(const autoware_msgs::traffic_light & msg);
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
