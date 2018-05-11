/*
// *  Copyright (c) 2016, Nagoya University
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

#ifndef OP_TRAJECTORY_GENERATOR_CORE
#define OP_TRAJECTORY_GENERATOR_CORE

// ROS includes
#include <ros/ros.h>
#include "op_planner/PlannerH.h"
#include "op_planner/PlannerCommonDef.h"

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/CanInfo.h>

namespace TrajectoryGeneratorNS
{

class TrajectoryGen
{
protected: //Planning Related variables

	PlannerHNS::PlannerH m_Planner;
	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::WayPoint m_InitPos;
	bool bInitPos;

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool bVehicleStatus;

	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathSections;
	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_RollOuts;
	bool bWayGlobalPath;

	struct timespec m_PlanningTimer;
  	std::vector<std::string>    m_LogData;

  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;


protected: //ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_LocalPath;
	ros::Publisher pub_LocalBasePath;
	ros::Publisher pub_LocalTrajectoriesRviz;

	// define subscribers.
	ros::Subscriber sub_initialpose			;
	ros::Subscriber sub_current_pose 		;
	ros::Subscriber sub_current_velocity	;
	ros::Subscriber sub_vehicle_simu_status ;
	ros::Subscriber sub_robot_odom			;
	ros::Subscriber sub_can_info			;
	ros::Subscriber sub_GlobalPlannerPaths	;


protected: // Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);

protected: //Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);

public:
	TrajectoryGen();
  ~TrajectoryGen();
  void MainLoop();
};

}

#endif  // OP_TRAJECTORY_GENERATOR_CORE
