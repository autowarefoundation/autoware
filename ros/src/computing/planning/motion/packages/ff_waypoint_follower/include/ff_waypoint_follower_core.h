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
	void callbackGetCurrentTrajectory(const autoware_msgs::laneConstPtr& msg);
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
