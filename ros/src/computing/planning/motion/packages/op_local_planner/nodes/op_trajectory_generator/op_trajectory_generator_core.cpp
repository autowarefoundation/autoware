/*
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
#include "op_trajectory_generator_core.h"
#include "op_ros_helpers/op_RosHelpers.h"


namespace TrajectoryGeneratorNS
{

TrajectoryGen::TrajectoryGen()
{
	bInitPos = false;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bWayGlobalPath = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	PlannerHNS::RosHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_LocalPath = nh.advertise<autoware_msgs::lane>("/final_waypoints", 100,true);
	pub_LocalBasePath = nh.advertise<autoware_msgs::lane>("/base_waypoints", 100,true);
	pub_LocalTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories", 1);

	sub_initialpose 	= nh.subscribe("/initialpose", 				1,		&TrajectoryGen::callbackGetInitPose, 		this);
	sub_current_pose 	= nh.subscribe("/current_pose", 			1,		&TrajectoryGen::callbackGetCurrentPose, 		this);

	int bVelSource = 1;
	_nh.getParam("/op_trajectory_generator/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom 			= nh.subscribe("/odom", 					100,	&TrajectoryGen::callbackGetRobotOdom, 	this);
	else if(bVelSource == 1)
		sub_current_velocity 	= nh.subscribe("/current_velocity",		100,	&TrajectoryGen::callbackGetVehicleStatus, 	this);
	else if(bVelSource == 2)
		sub_can_info 			= nh.subscribe("/can_info",		100,	&TrajectoryGen::callbackGetCanInfo, 	this);

	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&TrajectoryGen::callbackGetGlobalPlannerPath, 	this);
}

TrajectoryGen::~TrajectoryGen()
{
}


void TrajectoryGen::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_trajectory_generator/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("/op_trajectory_generator/enableFollowing", m_PlanningParams.enableFollowing);

	_nh.getParam("/op_trajectory_generator/enableHeadingSmoothing", m_PlanningParams.enableHeadingSmoothing);
	_nh.getParam("/op_trajectory_generator/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_trajectory_generator/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

	_nh.getParam("/op_trajectory_generator/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_trajectory_generator/minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("/op_trajectory_generator/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);
	_nh.getParam("/op_trajectory_generator/samplingTipMargin", m_PlanningParams.carTipMargin);
	_nh.getParam("/op_trajectory_generator/samplingOutMargin", m_PlanningParams.rollInMargin);
	_nh.getParam("/op_trajectory_generator/samplingSpeedFactor", m_PlanningParams.rollInSpeedFactor);

	_nh.getParam("/op_trajectory_generator/pathDensity", m_PlanningParams.pathDensity);
	_nh.getParam("/op_trajectory_generator/rollOutDensity", m_PlanningParams.rollOutDensity);
	if(m_PlanningParams.enableSwerving)
		_nh.getParam("/op_trajectory_generator/rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;

	_nh.getParam("/op_trajectory_generator/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_trajectory_generator/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_trajectory_generator/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_trajectory_generator/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_trajectory_generator/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_trajectory_generator/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_trajectory_generator/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("/op_trajectory_generator/enableLaneChange", m_PlanningParams.enableLaneChange);
	_nh.getParam("/op_trajectory_generator/enabTrajectoryVelocities", m_PlanningParams.enabTrajectoryVelocities);

	_nh.getParam("/op_trajectory_generator/width", m_CarInfo.width);
	_nh.getParam("/op_trajectory_generator/length", m_CarInfo.length);
	_nh.getParam("/op_trajectory_generator/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_trajectory_generator/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_trajectory_generator/maxSteerAngle", m_CarInfo.max_steer_angle);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

}

void TrajectoryGen::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	if(!bInitPos)
	{
		m_InitPos = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x,
				msg->pose.pose.position.y+m_OriginPos.position.y,
				msg->pose.pose.position.z+m_OriginPos.position.z,
				tf::getYaw(msg->pose.pose.orientation));
		m_CurrentPos = m_InitPos;
		bInitPos = true;
	}
}

void TrajectoryGen::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	m_InitPos = m_CurrentPos;
	bNewCurrentPos = true;
	bInitPos = true;
}

void TrajectoryGen::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;

	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryGen::callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_steer_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryGen::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryGen::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{

		bool bOldGlobalPath = m_GlobalPaths.size() == msg->lanes.size();

		m_GlobalPaths.clear();

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			std::vector<PlannerHNS::WayPoint> path;
			for(unsigned int j = 0 ; j < msg->lanes.at(i).waypoints.size(); j++)
			{
				PlannerHNS::WayPoint wp(msg->lanes.at(i).waypoints.at(j).pose.pose.position.x,
						msg->lanes.at(i).waypoints.at(j).pose.pose.position.y,
						msg->lanes.at(i).waypoints.at(j).pose.pose.position.z,
						tf::getYaw(msg->lanes.at(i).waypoints.at(j).pose.pose.orientation));
				wp.v = msg->lanes.at(i).waypoints.at(j).twist.twist.linear.x;
				wp.laneId = msg->lanes.at(i).waypoints.at(j).twist.twist.linear.y;
				wp.stopLineID = msg->lanes.at(i).waypoints.at(j).twist.twist.linear.z;
				wp.laneChangeCost = msg->lanes.at(i).waypoints.at(j).twist.twist.angular.x;
				wp.LeftLaneId = msg->lanes.at(i).waypoints.at(j).twist.twist.angular.y;
				wp.RightLaneId = msg->lanes.at(i).waypoints.at(j).twist.twist.angular.z;

				if(msg->lanes.at(i).waypoints.at(j).dtlane.dir == 0)
					wp.bDir = PlannerHNS::FORWARD_DIR;
				else if(msg->lanes.at(i).waypoints.at(j).dtlane.dir == 1)
					wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
				else if(msg->lanes.at(i).waypoints.at(j).dtlane.dir == 2)
					wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;

				path.push_back(wp);
			}

			PlannerHNS::PlanningHelpers::CalcAngleAndCost(path);
			m_GlobalPaths.push_back(path);

			if(bOldGlobalPath)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(path, m_GlobalPaths.at(i));
			}
		}

		if(!bOldGlobalPath)
		{
			bWayGlobalPath = true;
			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
				PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.49, 0.25, 0.05);

				PlannerHNS::PlanningHelpers::GenerateRecommendedSpeed(m_GlobalPaths.at(i), m_CarInfo.max_speed_forward, m_PlanningParams.speedProfileFactor);
				m_GlobalPaths.at(i).at(m_GlobalPaths.at(i).size()-1).v = 0;
			}
		}
		else
		{
			m_GlobalPaths.clear();
		}
	}
}

void TrajectoryGen::MainLoop()
{
	ros::Rate loop_rate(100);

	PlannerHNS::WayPoint prevState, state_change;

	while (ros::ok())
	{
		ros::spinOnce();

		if(bInitPos && m_GlobalPaths.size()>0)
		{
			m_GlobalPathSections.clear();

			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				t_centerTrajectorySmoothed.clear();
				PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceFast(m_GlobalPaths.at(i), m_CurrentPos,
						m_PlanningParams.horizonDistance ,
						m_PlanningParams.pathDensity ,
						t_centerTrajectorySmoothed,
						m_PlanningParams.smoothingDataWeight,
						m_PlanningParams.smoothingSmoothWeight,
						m_PlanningParams.smoothingToleranceError);

				m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
			}

			std::vector<PlannerHNS::WayPoint> sampledPoints_debug;
			m_Planner.GenerateRunoffTrajectory(m_GlobalPathSections, m_CurrentPos,
								m_PlanningParams.enableLaneChange,
								m_VehicleStatus.speed,
								m_PlanningParams.microPlanDistance,
								m_PlanningParams.maxSpeed,
								m_PlanningParams.minSpeed,
								m_PlanningParams.carTipMargin,
								m_PlanningParams.rollInMargin,
								m_PlanningParams.rollInSpeedFactor,
								m_PlanningParams.pathDensity,
								m_PlanningParams.rollOutDensity,
								m_PlanningParams.rollOutNumber,
								m_PlanningParams.smoothingDataWeight,
								m_PlanningParams.smoothingSmoothWeight,
								m_PlanningParams.smoothingToleranceError,
								m_PlanningParams.speedProfileFactor,
								m_PlanningParams.enableHeadingSmoothing,
								-1 , -1,
								m_RollOuts, sampledPoints_debug);
		}
		else
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&TrajectoryGen::callbackGetGlobalPlannerPath, 	this);

		visualization_msgs::MarkerArray all_rollOuts;
		PlannerHNS::RosHelpers::TrajectoriesToMarkers(m_RollOuts, all_rollOuts);
		pub_LocalTrajectoriesRviz.publish(all_rollOuts);

		loop_rate.sleep();
	}
}

}
