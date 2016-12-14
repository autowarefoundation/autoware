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

#ifndef dp_planner_CORE_H
#define dp_planner_CORE_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include "waypoint_follower/libwaypoint_follower.h"
#include "waypoint_follower/LaneArray.h"

#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "RoadNetwork.h"
#include "MappingHelpers.h"
#include "PlanningHelpers.h"
//#include "CarState.h"
#include "LocalPlannerH.h"
#include "RosHelpers.h"
#include "SimpleTracker.h"

namespace PlannerXNS
{

enum SIGNAL_TYPE{SIMULATION_SIGNAL, ROBOT_SIGNAL};

class PlannerX
{
protected:
	//For Testing
	timespec m_Timer;
	int m_counter;
	int m_frequency;

protected:
	SimulationNS::SimpleTracker m_ObstacleTracking;
	//SimulationNS::CarState m_State;
	PlannerHNS::LocalPlannerH m_LocalPlanner;

	geometry_msgs::Pose m_OriginPos;

	PlannerHNS::WayPoint m_InitPos;
	bool bInitPos;

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	SIGNAL_TYPE m_bSignal;

	std::vector<PlannerHNS::DetectedObject> m_DetectedClusters;
	std::vector<PlannerHNS::DetectedObject> m_DetectedBoxes;
	bool bNewClusters;
	jsk_recognition_msgs::BoundingBoxArray m_BoundingBoxes;
	bool bNewBoxes;

	PlannerHNS::VehicleState m_VehicleState;
	bool bVehicleState;

	bool bNewEmergency;
	int m_bEmergencyStop;

	bool bNewTrafficLigh;
	int m_bGreenLight;

	bool bNewOutsideControl;
	int m_bOutsideControl;

	std::vector<PlannerHNS::WayPoint> m_AStarPath;
	bool bNewAStarPath;
	timespec m_AStartPlanningTimer;

	std::vector<std::vector<PlannerHNS::WayPoint> > m_WayPlannerPaths;
	bool bWayPlannerPath;


	//Planning Related variables
	PlannerHNS::BehaviorState m_CurrentBehavior;
	PlannerHNS::BehaviorState m_PrevBehavior;
	//std::vector<PlannerHNS::WayPoint> m_goals;
	//int m_iCurrentGoal;
	PlannerHNS::WayPoint m_CurrentGoal;
	struct timespec m_PlanningTimer;
	AutowareRoadNetwork m_AwMap;
  	PlannerHNS::RoadNetwork m_Map;
  	bool	m_bKmlMap;
  	bool	bKmlMapLoaded;
  	std::string m_KmlMapPath;

  	bool m_bEnableTracking;
  	bool m_bEnableOutsideControl;


protected:
	//ROS messages (topics)

	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_LocalPath;
	ros::Publisher pub_BehaviorState;
	ros::Publisher pub_GlobalPlanNodes;
	ros::Publisher pub_StartPoint;
	ros::Publisher pub_GoalPoint;
	ros::Publisher pub_AStarStartPoint;
	ros::Publisher pub_AStarGoalPoint;

	ros::Publisher pub_DetectedPolygonsRviz;
	ros::Publisher pub_TrackedObstaclesRviz;
	ros::Publisher pub_LocalTrajectoriesRviz;

	// define subscribers.
	ros::Subscriber sub_initialpose			;
	ros::Subscriber sub_current_pose 		;
	ros::Subscriber sub_cluster_cloud		;
	ros::Subscriber sub_bounding_boxs		;
	ros::Subscriber sub_vehicle_status 		;
	ros::Subscriber sub_robot_odom			;
	ros::Subscriber sub_EmergencyStop		;
	ros::Subscriber sub_TrafficLight		;
	ros::Subscriber sub_OutsideControl		;
	ros::Subscriber sub_AStarPath			;
	ros::Subscriber sub_WayPlannerPaths		;

	//vector map subscription
	ros::Subscriber sub_map_points;
	ros::Subscriber sub_map_lanes;
	ros::Subscriber sub_map_nodes;
	ros::Subscriber sup_stop_lines;
	ros::Subscriber sub_dtlanes;

	ros::Subscriber sub_simulated_obstacle_pose_rviz;

	// Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetCloudClusters(const lidar_tracker::CloudClusterArrayConstPtr& msg);
	void callbackGetBoundingBoxes(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetEmergencyStop(const std_msgs::Int8& msg);
	void callbackGetTrafficLight(const std_msgs::Int8& msg);
	void callbackGetOutsideControl(const std_msgs::Int8& msg);
	void callbackGetAStarPath(const waypoint_follower::LaneArrayConstPtr& msg);
	void callbackGetWayPlannerPath(const waypoint_follower::LaneArrayConstPtr& msg);

	//Vector map callbacks
	void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
	void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
	void callbackGetVMCenterLines(const vector_map_msgs::DTLaneArray& msg);

	//for simulation
	void callbackGetRvizPoint(const geometry_msgs::PointStampedConstPtr& msg);


public:
  PlannerX();
  ~PlannerX();
  void PlannerMainLoop();

protected:
  //Helper Functions
  void UpdatePlanningParams();

  lidar_tracker::CloudCluster GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::PointStamped& centerPose);

};

}

#endif  // dp_planner_H
