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
#include "op_behavior_selector_core.h"
#include "op_RosHelpers.h"
#include "MappingHelpers.h"

namespace BehaviorGeneratorNS
{

BehaviorGen::BehaviorGen()
{
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bWayGlobalPath = false;
	bNewLightStatus = false;
	bNewLightSignal = false;
	bBestCost = false;
	bMap = false;
	bRollOuts = false;


	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	PlannerHNS::RosHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_LocalPath = nh.advertise<autoware_msgs::lane>("final_waypoints", 1,true);
	pub_LocalBasePath = nh.advertise<autoware_msgs::lane>("base_waypoints", 1,true);
	pub_ClosestIndex = nh.advertise<std_msgs::Int32>("closest_waypoint", 1,true);
	pub_BehaviorState = nh.advertise<geometry_msgs::TwistStamped>("op_behavior_state", 1);
	pub_SimuBoxPose	  = nh.advertise<geometry_msgs::PoseArray>("sim_box_pose_ego", 1);
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::Marker>("behavior_state", 1);

	sub_current_pose 	= nh.subscribe("/current_pose", 			1,		&BehaviorGen::callbackGetCurrentPose, 		this);

	int bVelSource = 1;
	_nh.getParam("/op_trajectory_evaluator/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom 			= nh.subscribe("/odom", 					100,	&BehaviorGen::callbackGetRobotOdom, 	this);
	else if(bVelSource == 1)
		sub_current_velocity 	= nh.subscribe("/current_velocity",		100,	&BehaviorGen::callbackGetVehicleStatus, 	this);
	else if(bVelSource == 2)
		sub_can_info 			= nh.subscribe("/can_info",		100,	&BehaviorGen::callbackGetCanInfo, 	this);

	sub_GlobalPlannerPaths 	= nh.subscribe("/lane_waypoints_array", 	1,		&BehaviorGen::callbackGetGlobalPlannerPath, 	this);
	sub_LocalPlannerPaths 	= nh.subscribe("/local_weighted_trajectories", 	1,		&BehaviorGen::callbackGetLocalPlannerPath, 	this);
	sub_TrafficLightStatus 		= nh.subscribe("/light_color", 		1,		&BehaviorGen::callbackGetTrafficLightStatus, 	this);
	sub_TrafficLightSignals		= nh.subscribe("/roi_signal", 		1,		&BehaviorGen::callbackGetTrafficLightSignals, 	this);
	sub_Trajectory_Cost			=  nh.subscribe("/local_trajectory_cost", 		1,		&BehaviorGen::callbackGetLocalTrajectoryCost, 	this);
}

BehaviorGen::~BehaviorGen()
{
}

void BehaviorGen::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

	_nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

	_nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

	_nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);

	_nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
	if(m_PlanningParams.enableSwerving)
		_nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;

	std::cout << "Rolls Number: " << m_PlanningParams.rollOutNumber << std::endl;

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_trajectory_evaluator/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_trajectory_evaluator/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

	m_PlanningParams.nReliableCount = 50;

	_nh.getParam("/op_common_params/width", m_CarInfo.width);
	_nh.getParam("/op_common_params/length", m_CarInfo.length);
	_nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_common_params/maxSteerAngle", m_CarInfo.max_steer_angle);
	_nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

	PlannerHNS::ControllerParams controlParams;
	controlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01);
	controlParams.Velocity_Gain = PlannerHNS::PID_CONST(0.1, 0.005, 0.1);
	nh.getParam("/op_common_params/steeringDelay", controlParams.SteeringDelay);
	nh.getParam("/op_common_params/minPursuiteDistance", controlParams.minPursuiteDistance );

	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" 			, iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("/op_common_params/mapFileName" 		, m_MapPath);

	m_BehaviorGenerator.Init(controlParams, m_PlanningParams, m_CarInfo);
	m_BehaviorGenerator.m_pCurrentBehaviorState->m_Behavior = PlannerHNS::INITIAL_STATE;

}

void BehaviorGen::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void BehaviorGen::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void BehaviorGen::callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_steer_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void BehaviorGen::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed ;
	if(msg->twist.twist.linear.x != 0)
		m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void BehaviorGen::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0 && bMap)
	{

		bool bOldGlobalPath = m_GlobalPaths.size() == msg->lanes.size();

		m_GlobalPaths.clear();

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			std::vector<PlannerHNS::WayPoint> path;
			PlannerHNS::Lane* pPrevValid = 0;
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

				PlannerHNS::Lane* pLane = 0;
				pLane = PlannerHNS::MappingHelpers::GetLaneById(wp.laneId, m_Map);
				if(!pLane)
				{
					pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMapDirectionBased(wp, m_Map, 1);

					if(!pLane && !pPrevValid)
					{
						ROS_ERROR("Map inconsistency between Global Path and Local Planer Map, Can't identify current lane.");
						return;
					}

					if(!pLane)
						wp.pLane = pPrevValid;
					else
					{
						wp.pLane = pLane;
						pPrevValid = pLane ;
					}

					wp.laneId = wp.pLane->id;
				}
				else
					wp.pLane = pLane;

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

			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;

			m_BehaviorGenerator.m_TotalOriginalPath = m_GlobalPaths;
		}
		else
		{
			m_GlobalPaths.clear();
		}
	}
}

void BehaviorGen::callbackGetLocalTrajectoryCost(const autoware_msgs::laneConstPtr& msg)
{
	bBestCost = true;
	m_TrajectoryBestCost.bBlocked = msg->is_blocked;
	m_TrajectoryBestCost.index = msg->lane_index;
	m_TrajectoryBestCost.cost = msg->cost;
	m_TrajectoryBestCost.closest_obj_distance = msg->closest_object_distance;
	m_TrajectoryBestCost.closest_obj_velocity = msg->closest_object_velocity;
}

void BehaviorGen::callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		int currIndex = PlannerHNS::PlanningHelpers::GetClosestNextPointIndexFast(m_BehaviorGenerator.m_Path, m_BehaviorGenerator.state);
		int index_limit = 0;//m_Path.size() - 20;
		if(index_limit<=0)
			index_limit =  m_BehaviorGenerator.m_Path.size()/2.0;
		if(m_RollOuts.size() == 0
				|| currIndex > index_limit
				|| m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bRePlan
				|| m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath
				|| m_BehaviorGenerator.m_pCurrentBehaviorState->m_Behavior == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
		{
			std::cout << "New Local Plan !! " << currIndex << ", "<< m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bRePlan << ", " << m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath  << std::endl;
			m_RollOuts.clear();
			m_TrajectoryCosts.clear();
			for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
			{
				std::vector<PlannerHNS::WayPoint> path;
				PlannerHNS::TrajectoryCost traj_cost;
				PlannerHNS::RosHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), path);

				traj_cost.bBlocked = msg->lanes.at(i).is_blocked;
				traj_cost.index = msg->lanes.at(i).lane_index;
				traj_cost.cost = msg->lanes.at(i).cost;
				traj_cost.closest_obj_distance = msg->lanes.at(i).closest_object_distance;
				traj_cost.closest_obj_velocity = msg->lanes.at(i).closest_object_velocity;

				m_RollOuts.push_back(path);
				m_TrajectoryCosts.push_back(traj_cost);
			}

			m_BehaviorGenerator.m_RollOuts = m_RollOuts;
			bRollOuts = true;
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bRePlan = false;
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = false;

			PlannerHNS::PreCalculatedConditions* pCParams = m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams();

			if(pCParams)
			{
				if(m_BehaviorGenerator.m_pCurrentBehaviorState->m_Behavior == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
					pCParams->iPrevSafeTrajectory = pCParams->iCurrSafeTrajectory;
				else
					pCParams->iPrevSafeTrajectory = pCParams->iCentralTrajectory;

				pCParams->iPrevSafeLane = pCParams->iCurrSafeLane;
			}
		}
	}
}

void BehaviorGen::callbackGetTrafficLightStatus(const autoware_msgs::traffic_light& msg)
{
	std::cout << "Received Traffic Light Status : " << msg.traffic_light << std::endl;
	bNewLightStatus = true;
	if(msg.traffic_light == 1) // green
		m_CurrLightStatus = PlannerHNS::GREEN_LIGHT;
	else //0 => RED , 2 => Unknown
		m_CurrLightStatus = PlannerHNS::RED_LIGHT;
}

void BehaviorGen::callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg)
{
	//std::cout << "Received Traffic Light Signals : " << msg.Signals.size() << std::endl;
	m_CurrTrafficLight.clear();
	bNewLightSignal = true;
	for(unsigned int i = 0 ; i < msg.Signals.size() ; i++)
	{
		PlannerHNS::TrafficLight tl;
		tl.id = msg.Signals.at(i).signalId;
		if(msg.Signals.at(i).type == 1)
			tl.lightState = PlannerHNS::GREEN_LIGHT;
		else
			tl.lightState = PlannerHNS::RED_LIGHT;

		m_CurrTrafficLight.push_back(tl);
	}
}

void BehaviorGen::VisualizeLocalPlanner()
{
	visualization_msgs::Marker behavior_rviz;
	int iDirection = 0;
	if(m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory > m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
		iDirection = 1;
	else if(m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory < m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
		iDirection = -1;
	PlannerHNS::RosHelpers::VisualizeBehaviorState(m_CurrentPos, m_CurrentBehavior, !m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bTrafficIsRed , iDirection, behavior_rviz, "beh_state");
	pub_BehaviorStateRviz.publish(behavior_rviz);
}

void BehaviorGen::SendLocalPlanningTopics()
{
	//Send Behavior State
	geometry_msgs::Twist t;
	geometry_msgs::TwistStamped behavior;
	t.linear.x = m_CurrentBehavior.bNewPlan;
	t.linear.y = m_CurrentBehavior.followDistance;
	t.linear.z = m_CurrentBehavior.followVelocity;
	t.angular.x = (int)m_CurrentBehavior.indicator;
	t.angular.y = (int)m_CurrentBehavior.state;
	t.angular.z = m_CurrentBehavior.iTrajectory;
	behavior.twist = t;
	behavior.header.stamp = ros::Time::now();
	pub_BehaviorState.publish(behavior);

	//Send Ego Vehicle Simulation Pose Data
	geometry_msgs::PoseArray sim_data;
	geometry_msgs::Pose p_id, p_pose, p_box;

	sim_data.header.frame_id = "map";
	sim_data.header.stamp = ros::Time();
	p_id.position.x = 0;
	p_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(m_BehaviorGenerator.state.pos.a));
	p_pose.position.x = m_BehaviorGenerator.state.pos.x;
	p_pose.position.y = m_BehaviorGenerator.state.pos.y;
	p_pose.position.z = m_BehaviorGenerator.state.pos.z;
	p_box.position.x = m_BehaviorGenerator.m_CarInfo.width;
	p_box.position.y = m_BehaviorGenerator.m_CarInfo.length;
	p_box.position.z = 2.2;
	sim_data.poses.push_back(p_id);
	sim_data.poses.push_back(p_pose);
	sim_data.poses.push_back(p_box);
	pub_SimuBoxPose.publish(sim_data);

	//Send Trajectory Data to path following nodes
	std_msgs::Int32 closest_waypoint;
	PlannerHNS::RelativeInfo info;
	PlannerHNS::PlanningHelpers::GetRelativeInfo(m_BehaviorGenerator.m_Path, m_BehaviorGenerator.state, info);
	PlannerHNS::RosHelpers::ConvertFromLocalLaneToAutowareLane(m_BehaviorGenerator.m_Path, m_CurrentTrajectoryToSend, info.iBack);
	//std::cout << "Path Size: " << m_BehaviorGenerator.m_Path.size() << ", Send Size: " << m_CurrentTrajectoryToSend << std::endl;

	closest_waypoint.data = 1;
	pub_ClosestIndex.publish(closest_waypoint);
	pub_LocalBasePath.publish(m_CurrentTrajectoryToSend);
	pub_LocalPath.publish(m_CurrentTrajectoryToSend);
}

void BehaviorGen::MainLoop()
{
	ros::Rate loop_rate(100);


	timespec planningTimer;
	UtilityHNS::UtilityH::GetTickCount(planningTimer);

	while (ros::ok())
	{
		ros::spinOnce();

		double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(planningTimer);
		UtilityHNS::UtilityH::GetTickCount(planningTimer);

		if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_MapPath, m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_MapPath, m_Map, true);
		}

		PlannerHNS::TrajectoryCost tc;
		if(bNewCurrentPos && m_GlobalPaths.size()>0)
		{
			m_BehaviorGenerator.m_TotalPath.clear();

			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				t_centerTrajectorySmoothed.clear();
				PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPaths.at(i), m_CurrentPos, m_PlanningParams.horizonDistance ,	m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed);
				m_BehaviorGenerator.m_TotalPath.push_back(t_centerTrajectorySmoothed);
			}

			if(bNewLightSignal)
			{
				m_PrevTrafficLight = m_CurrTrafficLight;
				bNewLightSignal = false;
			}

			if(bNewLightStatus)
			{
				bNewLightStatus = false;
				for(unsigned int itls = 0 ; itls < m_PrevTrafficLight.size() ; itls++)
					m_PrevTrafficLight.at(itls).lightState = m_CurrLightStatus;
			}

			m_CurrentBehavior = m_BehaviorGenerator.DoOneStep(dt, m_CurrentPos, m_VehicleStatus, 1, m_PrevTrafficLight, m_TrajectoryBestCost, 0 );

			SendLocalPlanningTopics();
			VisualizeLocalPlanner();
		}
		else
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&BehaviorGen::callbackGetGlobalPlannerPath, 	this);

		loop_rate.sleep();
	}
}

}
