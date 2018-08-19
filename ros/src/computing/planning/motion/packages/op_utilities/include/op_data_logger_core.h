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
#include <autoware_msgs/lane.h>
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
	void callbackGetSimuCarsPathAndState(const autoware_msgs::laneConstPtr& msg);
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
