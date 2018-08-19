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
#include <autoware_msgs/CanInfo.h>
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
	void callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg);
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
