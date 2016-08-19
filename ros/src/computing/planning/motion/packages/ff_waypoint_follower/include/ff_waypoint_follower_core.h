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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "waypoint_follower/libwaypoint_follower.h"
#include "TrajectoryFollower.h"
#include "CarState.h"
#include "PlannerH.h"
#include "MappingHelpers.h"
#include "HevComm.h"

namespace FFSteerControlNS
{

class ControlCommandParams
{
public:
	bool bTest; // true -> Test parameter entered at command line
	bool bMode; // true -> mode parameter entered at command line
	bool bSignal; // true -> signal parameter entered at command line
	double targetSteer; // test steering angle
	double targetVelocity; // test velocity
	int iLocalizationSource;
	//0 -> control box (zmp)
	//1 -> autoware
	//2 -> segway
	//3 -> simulation
	bool bTorqueMode; // true -> torque and stroke mode, false -> angle and velocity mode

	ControlCommandParams()
	{
		bTest = false;
		bMode = false;
		bSignal = false;
		targetVelocity = 0;
		targetSteer = 0;
		iLocalizationSource = 3;
		bTorqueMode = false;
	}
};

class FFSteerControl
{
protected:
	HevComm* m_pComm;

	//bool m_bReadyToPlan;
	timespec m_Timer;
	int m_counter;
	int m_frequency;

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
	double m_CurrentVelocity;
	SimulationNS::CarState m_State;
	struct timespec m_PlanningTimer;
	PlannerHNS::WayPoint m_FollowPoint;
	PlannerHNS::WayPoint m_PerpPoint;

	PlannerHNS::VehicleState m_PrevStepTargetStatus;
	geometry_msgs::Vector3Stamped m_segway_status;
	bool bVehicleStatus;
	ControlCommandParams m_CmdParams;

	ros::NodeHandle nh;

	//ros::Publisher m_PositionPublisher;
	//ros::Publisher m_PathPublisherRviz;
	ros::Publisher m_velocity_publisher;
	ros::Publisher m_stat_pub;
	ros::Publisher m_target_pub;
	ros::Publisher m_simulated_pos_pub;
	ros::Publisher m_simulated_velocity_pub;
	ros::Publisher m_current_vehicle_status;
	ros::Publisher m_segway_rpm_cmd;

	// define subscribers.
	ros::Subscriber sub_current_pose ;
	ros::Subscriber sub_behavior_state;
	ros::Subscriber sub_current_trajectory;
	ros::Subscriber sub_twist_velocity;
	ros::Subscriber initialpose_subscriber 	;
	ros::Subscriber sub_segway_rpm_odom;



	SimulationNS::TrajectoryFollower m_PredControl;


public:
	FFSteerControl(const ControlCommandParams& params);

	virtual ~FFSteerControl();

	void PlannerMainLoop();

private:
  // Callback function for subscriber.

	void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackFromCurrentTrajectory(const waypoint_follower::laneConstPtr& msg);
	void callbackFromVector3Stamped(const geometry_msgs::Vector3StampedConstPtr &msg);
	void callbackSimuInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void callbackFromBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg );
	void callbackFromSegwayRPM(const nav_msgs::OdometryConstPtr& msg);

  void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);


  void displayNextTarget(PlannerHNS::WayPoint target);

  PlannerHNS::BehaviorState ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg);

  //void ConvertAndPulishDrivingTrajectory(const std::vector<PlannerHNS::WayPoint>& path);
};

}

#endif  // ff_waypoint_follower_H
