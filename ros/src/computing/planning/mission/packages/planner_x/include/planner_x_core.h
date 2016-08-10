/*
// *  Copyright (c) 2015, Nagoya University
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

#ifndef PLANNER_X_CORE_H
#define PLANNER_X_CORE_H

// ROS includes
#include <ros/ros.h>
#include <cv_tracker/obj_label.h>
#include <runtime_manager/traffic_light.h>

#include <map_file/PointClassArray.h>
#include <map_file/LaneArray.h>
#include <map_file/NodeArray.h>
#include <map_file/StopLineArray.h>
#include <map_file/DTLaneArray.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "PlannerXInterface.h"

namespace PlannerXNS
{

class PlannerX
{
protected:
	//bool m_bReadyToPlan;
	timespec m_Timer;
	int m_counter;
	int m_frequency;

	PlannerXNS::AutowareRoadNetwork m_AwMap;
	PlannerXNS::PlannerX_Interface* m_pPlanner;

	geometry_msgs::Pose m_InitPos;
	bool bInitPos;
	//geometry_msgs::Pose m_GoalPos;
	bool bGoalPos;
	geometry_msgs::Pose m_CurrentPos;
	bool bNewCurrentPos;
	geometry_msgs::Pose m_OriginPos;

	cv_tracker::obj_label m_DetectedObstacles;
	bool bNewDetectedObstacles;

	runtime_manager::traffic_light m_TrafficLights;
	bool bTrafficLights;

	AutowareVehicleState m_VehicleState;
	bool bVehicleState;

	ros::NodeHandle nh;

	ros::Publisher m_PositionPublisher;
	ros::Publisher m_PathPublisherRviz;
	ros::Publisher m_PathPublisher;
	ros::Publisher m_TrajectoryFinalWaypointPublisher;
	ros::Publisher m_BehaviorPublisher;

	// define subscribers.
	ros::Subscriber sub_current_pose 		;
	ros::Subscriber sub_traffic_light 		;
	ros::Subscriber sub_obj_pose 			;
	ros::Subscriber point_sub 				;
	ros::Subscriber lane_sub 				;
	ros::Subscriber node_sub 				;
	ros::Subscriber stopline_sub 			;
	ros::Subscriber dtlane_sub 				;
	ros::Subscriber initialpose_subscriber 	;
	ros::Subscriber goalpose_subscriber 	;
	ros::Subscriber sub_vehicle_status 	;



public:
  PlannerX(std::string plannerType);


  ~PlannerX();

  void PlannerMainLoop();

private:
  // Callback function for subscriber.
  void callbackSimuGoalPose(const geometry_msgs::PoseStamped &msg);
  void callbackSimuInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackFromLightColor(const runtime_manager::traffic_light& msg);
  void callbackFromObjCar(const cv_tracker::obj_label& msg);
  void callbackGetVMPoints(const map_file::PointClassArray& msg);
  void callbackGetVMLanes(const map_file::LaneArray& msg);
  void callbackGetVMNodes(const map_file::NodeArray& msg);
  void callbackGetVMStopLines(const map_file::StopLineArray& msg);
  void callbackGetVMCenterLines(const map_file::DTLaneArray& msg);
  void callbackFromVehicleStatus(const geometry_msgs::Vector3StampedConstPtr& msg);


  //void ConvertAndPulishDrivingTrajectory(const std::vector<PlannerHNS::WayPoint>& path);
};

}

#endif  // PLANNER_X_H
