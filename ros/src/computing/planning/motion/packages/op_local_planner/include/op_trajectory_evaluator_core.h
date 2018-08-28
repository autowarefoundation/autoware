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

#ifndef OP_TRAJECTORY_EVALUATOR_CORE
#define OP_TRAJECTORY_EVALUATOR_CORE

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/CanInfo.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "op_planner/PlannerCommonDef.h"
#include "op_planner/TrajectoryDynamicCosts.h"

namespace TrajectoryEvaluatorNS
{

class TrajectoryEval
{
protected:

	PlannerHNS::TrajectoryDynamicCosts m_TrajectoryCostsCalculator;
	bool m_bUseMoveingObjectsPrediction;

	geometry_msgs::Pose m_OriginPos;

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool bVehicleStatus;

	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathSections;
	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	bool bWayGlobalPath;
	bool bWayGlobalPathToUse;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GeneratedRollOuts;
	bool bRollOuts;

	std::vector<PlannerHNS::DetectedObject> m_PredictedObjects;
	bool bPredictedObjects;


	struct timespec m_PlanningTimer;
  	std::vector<std::string>    m_LogData;

  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;

  	PlannerHNS::BehaviorState m_CurrentBehavior;


  	visualization_msgs::MarkerArray m_CollisionsDummy;
	visualization_msgs::MarkerArray m_CollisionsActual;

	//ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_CollisionPointsRviz;
	ros::Publisher pub_LocalWeightedTrajectoriesRviz;
	ros::Publisher pub_LocalWeightedTrajectories;
	ros::Publisher pub_TrajectoryCost;
	ros::Publisher pub_SafetyBorderRviz;

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_GlobalPlannerPaths;
	ros::Subscriber sub_LocalPlannerPaths;
	ros::Subscriber sub_predicted_objects;
	ros::Subscriber sub_current_behavior;



	// Callback function for subscriber.
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);
	void callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr & msg);

	//Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);

public:
  TrajectoryEval();
  ~TrajectoryEval();
  void MainLoop();
};

}

#endif  // OP_TRAJECTORY_EVALUATOR_CORE
