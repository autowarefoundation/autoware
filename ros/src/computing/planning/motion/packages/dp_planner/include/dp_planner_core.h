/*
 * Copyright 2016-2019 Autoware Foundation. All rights reserved.
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

#ifndef dp_planner_CORE_H
#define dp_planner_CORE_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_can_msgs/CANInfo.h"

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "op_planner/RoadNetwork.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/LocalPlannerH.h"
#include "ROSHelpers.h"
#include "op_simu/SimpleTracker.h"

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#if (CV_MAJOR_VERSION < 3)
#include <opencv2/contrib/contrib.hpp>
#endif


namespace PlannerXNS
{

#define SIMU_OBSTACLE_WIDTH 3.5
#define SIMU_OBSTACLE_HEIGHT 0.5
#define SIMU_OBSTACLE_LENGTH 2.0

enum SIGNAL_TYPE{SIMULATION_SIGNAL, ROBOT_SIGNAL};
enum MAP_SOURCE_TYPE{MAP_AUTOWARE, MAP_FOLDER, MAP_KML_FILE};

class PlannerX
{
protected:
	//For Testing
	double m_TrackingTime;
	int m_nTrackObjects;
	int m_nContourPoints;
	int m_nOriginalPoints;

	timespec m_Timer;
	timespec m_TrafficLightTimer;
	int m_counter;
	int m_frequency;

protected:
	SimulationNS::SimpleTracker m_ObstacleTracking;
	PlannerHNS::LocalPlannerH m_LocalPlanner;

	geometry_msgs::Pose m_OriginPos;

	PlannerHNS::WayPoint m_InitPos;
	bool bInitPos;

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	std::vector<PlannerHNS::DetectedObject> m_OriginalClusters;
	std::vector<PlannerHNS::DetectedObject> m_TrackedClusters;
	std::vector<PlannerHNS::DetectedObject> m_DetectedBoxes;
	bool bNewClusters;
	jsk_recognition_msgs::BoundingBoxArray m_BoundingBoxes;
	bool bNewBoxes;

	PlannerHNS::VehicleState m_VehicleState;
	bool bVehicleState;

	bool bNewEmergency;
	int m_bEmergencyStop;

	bool bNewTrafficLigh;
	bool m_bGreenLight;

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
	struct timespec m_PlanningTimer;
	AutowareRoadNetwork m_AwMap;
  	PlannerHNS::RoadNetwork m_Map;
  	MAP_SOURCE_TYPE	m_MapSource;
  	bool	bKmlMapLoaded;
  	std::string m_KmlMapPath;

  	bool m_bEnableTracking;
  	bool m_bEnableOutsideControl;

  	std::vector<std::string>    m_LogData;

protected:
	//ROS messages (topics)

	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_LocalPath;
	ros::Publisher pub_LocalBasePath;
	ros::Publisher pub_ClosestIndex;
	ros::Publisher pub_BehaviorState;
	ros::Publisher pub_GlobalPlanNodes;
	ros::Publisher pub_StartPoint;
	ros::Publisher pub_GoalPoint;
	ros::Publisher pub_AStarStartPoint;
	ros::Publisher pub_AStarGoalPoint;
	ros::Publisher pub_EnableLattice;

	ros::Publisher pub_DetectedPolygonsRviz;
	ros::Publisher pub_TrackedObstaclesRviz;
	ros::Publisher pub_LocalTrajectoriesRviz;
	ros::Publisher pub_LocalTrajectoriesRviz_dynamic;
	ros::Publisher pub_TestLineRviz;
	ros::Publisher pub_BehaviorStateRviz;
	ros::Publisher pub_SafetyBorderRviz;
	ros::Publisher pub_cluster_cloud;
	ros::Publisher pub_SimuBoxPose;

	// define subscribers.
	ros::Subscriber sub_initialpose;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_cluster_cloud;
	ros::Subscriber sub_bounding_boxs;
	ros::Subscriber sub_vehicle_simu_status ;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_EmergencyStop;
	ros::Subscriber sub_TrafficLight;
	ros::Subscriber sub_OutsideControl;
	ros::Subscriber sub_AStarPath;
	ros::Subscriber sub_WayPlannerPaths;

	ros::Subscriber sub_CostMap;

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
	void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg);
	void callbackGetBoundingBoxes(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetEmergencyStop(const std_msgs::Int8& msg);
	void callbackGetTrafficLight(const std_msgs::Int8& msg);
	void callbackGetOutsideControl(const std_msgs::Int8& msg);
	void callbackGetAStarPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetWayPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetCostMap(const nav_msgs::OccupancyGrid& msg);


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

  autoware_msgs::CloudCluster GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::PointStamped& centerPose);
};

}

#endif  // dp_planner_H
