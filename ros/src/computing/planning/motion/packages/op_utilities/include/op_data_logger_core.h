/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef OP_DATALOGGER
#define OP_DATALOGGER


#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ExtractedPosition.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/Lane.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "op_planner/RoadNetwork.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerCommonDef.h"


namespace DataLoggerNS
{

class VehicleDataContainer
{
public:
	int id;
	std::vector<PlannerHNS::WayPoint> path;
	PlannerHNS::BehaviorState beh;
	PlannerHNS::WayPoint pose;
	ros::Time path_time;
	ros::Time pose_time;
};

class OpenPlannerDataLogger
{

protected:

	ros::Subscriber sub_predicted_objects;
	ros::Subscriber sub_behavior_state;
	std::vector<ros::Subscriber> sub_simu_paths;
	std::vector<ros::Subscriber> sub_objs;

	ros::NodeHandle nh;
	timespec m_Timer;

	std::vector<VehicleDataContainer>  m_SimulatedVehicle;
	std::vector<PlannerHNS::DetectedObject> m_PredictedObjects;
	ros::Time m_pred_time;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap;
	int m_iSimuCarsNumber;

	std::vector<std::vector<std::string> >  m_LogData;

	void callbackGetSimuPose(const geometry_msgs::PoseArray &msg);
	void callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetSimuCarsPathAndState(const autoware_msgs::LaneConstPtr& msg);
	void callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg );
	PlannerHNS::BehaviorState ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg);
	PlannerHNS::STATE_TYPE GetStateFromNumber(const int& iBehState);
	PlannerHNS::BEH_STATE_TYPE GetBehStateFromNumber(const int& iBehState);

	void CompareAndLog(VehicleDataContainer& ground_truth, PlannerHNS::DetectedObject& predicted);
	double CalculateRMS(std::vector<PlannerHNS::WayPoint>& path1, std::vector<PlannerHNS::WayPoint>& path2);

public:
	OpenPlannerDataLogger();
	virtual ~OpenPlannerDataLogger();
	void MainLoop();
};

}

#endif  // OP_DATALOGGER
