/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef ff_waypoint_follower_CORE_H
#define ff_waypoint_follower_CORE_H

#define _ENABLE_ZMP_LIBRARY_LINK

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"
#include "op_simu/TrajectoryFollower.h"
//#include "CarState.h"
#include "op_planner/LocalPlannerH.h"
#include "op_planner/PlannerH.h"
#include "op_planner/MappingHelpers.h"


#ifdef ENABLE_ZMP_LIBRARY_LINK
#include "HevComm.h"
#endif

namespace FFSteerControlNS
{

enum STATUS_TYPE{CONTROL_BOX_STATUS, ROBOT_STATUS, SIMULATION_STATUS};

class ControlCommandParams
{
public:
	STATUS_TYPE statusSource;
	//0 -> Control box (zmp)
	//1 -> Autoware
	//2 -> Robot
	//3 -> Simulation
	int iMapping; // 1 create map
	double recordDistance;
	double recordDensity;
	bool bTorqueMode; // true -> torque and stroke mode, false -> angle and velocity mode
	bool bAngleMode;
	bool bVelocityMode;
	bool bEnableLogs;
	bool bCalibration;

	ControlCommandParams()
	{
		statusSource = SIMULATION_STATUS;
		bTorqueMode = false;
		iMapping = 0;
		recordDistance = 5.0;
		recordDensity = 0.5;
		bAngleMode = true;
		bVelocityMode = true;
		bEnableLogs = false;
		bCalibration = false;

	}
};

class FFSteerControl
{
protected:
#ifdef ENABLE_ZMP_LIBRARY_LINK
	HevComm* m_pComm;
#endif

	//bool m_bReadyToPlan;
	timespec m_Timer;
	int m_counter;
	int m_frequency;
	int m_bOutsideControl;


	PlannerHNS::WayPoint m_InitPos;
	bool bInitPos;
	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;
	geometry_msgs::Pose m_OriginPos;
	bool bNewTrajectory;
	std::vector<PlannerHNS::WayPoint> m_FollowingTrajectory;
	bool bNewBehaviorState;
	PlannerHNS::BehaviorState m_CurrentBehavior;
	bool bNewVelocity;
	//double m_CurrentVelocity;
	//SimulationNS::CarState m_State;
	PlannerHNS::LocalPlannerH m_State;
	struct timespec m_PlanningTimer;
	PlannerHNS::WayPoint m_FollowPoint;
	PlannerHNS::WayPoint m_PerpPoint;

	PlannerHNS::VehicleState m_PrevStepTargetStatus;
	PlannerHNS::VehicleState m_CurrVehicleStatus;


	//geometry_msgs::Vector3Stamped m_segway_status;
	bool bVehicleStatus;
	ControlCommandParams m_CmdParams;
	PlannerHNS::CAR_BASIC_INFO m_CarInfo;
	PlannerHNS::ControllerParams m_ControlParams;
	PlannerHNS::PlanningParams m_PlanningParams;

	SimulationNS::TrajectoryFollower m_PredControl;

	ros::NodeHandle nh;

	ros::Publisher pub_VelocityAutoware;
	ros::Publisher pub_StatusAutoware;
	ros::Publisher pub_SimuPose;
	ros::Publisher pub_SimuVelocity;
	ros::Publisher pub_CurrPoseRviz;
	ros::Publisher pub_FollowPointRviz;
	ros::Publisher pub_VehicleCommand;
	ros::Publisher pub_ControlBoxOdom;
	ros::Publisher pub_VelocityRviz;

	// define subscribers.
	ros::Subscriber sub_initialpose;
	ros::Subscriber sub_current_pose ;
	ros::Subscriber sub_current_velocity ;
	ros::Subscriber sub_behavior_state;
	ros::Subscriber sub_current_trajectory;
	//ros::Subscriber sub_autoware_odom;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_OutsideControl		;

	// Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg );
	void callbackGetCurrentTrajectory(const autoware_msgs::LaneConstPtr& msg);
	//void callbackGetAutowareOdom(const geometry_msgs::TwistStampedConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetOutsideControl(const std_msgs::Int8& msg);

public:
	FFSteerControl();

	virtual ~FFSteerControl();

	void PlannerMainLoop();

  void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);

  void ReadParamFromLaunchFile(PlannerHNS::CAR_BASIC_INFO& m_CarInfo,
		  PlannerHNS::ControllerParams& m_ControlParams);

  void displayFollowingInfo(PlannerHNS::WayPoint& curr_pose, PlannerHNS::WayPoint& perp_pose, PlannerHNS::WayPoint& follow_pose);

  PlannerHNS::BehaviorState ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg);

  //void ConvertAndPulishDrivingTrajectory(const std::vector<PlannerHNS::WayPoint>& path);
};

}

#endif  // ff_waypoint_follower_H
