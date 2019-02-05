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

#ifndef OP_MOTION_PREDICTION
#define OP_MOTION_PREDICTION

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
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/BehaviorPrediction.h"
#include "op_utility/DataRW.h"

namespace MotionPredictorNS
{

class MotionPrediction
{
protected:
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool bVehicleStatus;
	bool m_bGoNextStep;

	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::ControllerParams m_ControlParams;
	PlannerHNS::PlanningParams m_PlanningParams;
	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;

	std::vector<PlannerHNS::DetectedObject> m_TrackedObjects;
	bool bTrackedObjects;

	PlannerHNS::RoadNetwork m_Map;
	bool bMap;

	bool m_bEnableCurbObstacles;
	std::vector<PlannerHNS::DetectedObject> curr_curbs_obstacles;

	PlannerHNS::BehaviorPrediction m_PredictBeh;
	autoware_msgs::DetectedObjectArray m_PredictedResultsResults;

	timespec m_VisualizationTimer;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_all_pred_paths;
	std::vector<PlannerHNS::WayPoint> m_particles_points;

	visualization_msgs::MarkerArray m_PredictedTrajectoriesDummy;
	visualization_msgs::MarkerArray m_PredictedTrajectoriesActual;

	visualization_msgs::MarkerArray m_PredictedParticlesDummy;
	visualization_msgs::MarkerArray m_PredictedParticlesActual;

	visualization_msgs::MarkerArray m_CurbsDummy;
	visualization_msgs::MarkerArray m_CurbsActual;

	double m_DistanceBetweenCurbs;
	double m_VisualizationTime;

	timespec m_SensingTimer;


	ros::NodeHandle nh;
	ros::Publisher pub_predicted_objects_trajectories;
	ros::Publisher pub_PredictedTrajectoriesRviz ;
	ros::Publisher pub_CurbsRviz ;
	ros::Publisher pub_ParticlesRviz;

	// define subscribers.
	ros::Subscriber sub_tracked_objects		;
	ros::Subscriber sub_current_pose 		;
	ros::Subscriber sub_current_velocity	;
	ros::Subscriber sub_robot_odom			;
	ros::Subscriber sub_can_info			;
	ros::Subscriber sub_StepSignal;

	// Callback function for subscriber.
	void callbackGetTrackedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetStepForwardSignals(const geometry_msgs::TwistStampedConstPtr& msg);

	//Helper functions
	void VisualizePrediction();
	void UpdatePlanningParams(ros::NodeHandle& _nh);
	void GenerateCurbsObstacles(std::vector<PlannerHNS::DetectedObject>& curb_obstacles);

public:
	MotionPrediction();
	virtual ~MotionPrediction();
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

#endif  // OP_MOTION_PREDICTION
