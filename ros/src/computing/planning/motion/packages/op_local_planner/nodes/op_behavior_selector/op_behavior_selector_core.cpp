/*
 *  Copyright (c) 2018, Nagoya University
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
#include "op_ros_helpers/op_RosHelpers.h"
#include "op_planner/MappingHelpers.h"

namespace BehaviorGeneratorNS
{

BehaviorGen::BehaviorGen()
{
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bWayGlobalPath = false;
	bWayGlobalPathLogs = false;
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
	pub_BehaviorState = nh.advertise<geometry_msgs::TwistStamped>("current_behavior", 1);
	pub_SimuBoxPose	  = nh.advertise<geometry_msgs::PoseArray>("sim_box_pose_ego", 1);
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::MarkerArray>("behavior_state", 1);
	pub_SelectedPathRviz = nh.advertise<visualization_msgs::MarkerArray>("local_selected_trajectory_rviz", 1);

	sub_current_pose = nh.subscribe("/current_pose", 10,	&BehaviorGen::callbackGetCurrentPose, this);

	int bVelSource = 1;
	_nh.getParam("/op_trajectory_evaluator/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom = nh.subscribe("/odom", 10, &BehaviorGen::callbackGetRobotOdom, this);
	else if(bVelSource == 1)
		sub_current_velocity = nh.subscribe("/current_velocity", 10, &BehaviorGen::callbackGetVehicleStatus, this);
	else if(bVelSource == 2)
		sub_can_info = nh.subscribe("/can_info", 10, &BehaviorGen::callbackGetCanInfo, this);

	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &BehaviorGen::callbackGetGlobalPlannerPath, this);
	sub_LocalPlannerPaths = nh.subscribe("/local_weighted_trajectories", 1, &BehaviorGen::callbackGetLocalPlannerPath, this);
	sub_TrafficLightStatus = nh.subscribe("/light_color", 1, &BehaviorGen::callbackGetTrafficLightStatus, this);
	sub_TrafficLightSignals	= nh.subscribe("/roi_signal", 1, &BehaviorGen::callbackGetTrafficLightSignals, this);
	sub_Trajectory_Cost = nh.subscribe("/local_trajectory_cost", 1, &BehaviorGen::callbackGetLocalTrajectoryCost, this);

	sub_twist_raw = nh.subscribe("/twist_raw", 1, &BehaviorGen::callbackGetTwistRaw, this);
	sub_twist_cmd = nh.subscribe("/twist_cmd", 1, &BehaviorGen::callbackGetTwistCMD, this);
	//sub_ctrl_cmd = nh.subscribe("/ctrl_cmd", 1, &BehaviorGen::callbackGetCommandCMD, this);

	//Mapping Section
	sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &BehaviorGen::callbackGetVMLanes,  this);
	sub_points = nh.subscribe("/vector_map_info/point", 1, &BehaviorGen::callbackGetVMPoints,  this);
	sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &BehaviorGen::callbackGetVMdtLanes,  this);
	sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &BehaviorGen::callbackGetVMIntersections,  this);
	sup_area = nh.subscribe("/vector_map_info/area", 1, &BehaviorGen::callbackGetVMAreas,  this);
	sub_lines = nh.subscribe("/vector_map_info/line", 1, &BehaviorGen::callbackGetVMLines,  this);
	sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &BehaviorGen::callbackGetVMStopLines,  this);
	sub_signals = nh.subscribe("/vector_map_info/signal", 1, &BehaviorGen::callbackGetVMSignal,  this);
	sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &BehaviorGen::callbackGetVMVectors,  this);
	sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &BehaviorGen::callbackGetVMCurbs,  this);
	sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &BehaviorGen::callbackGetVMRoadEdges,  this);
	sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &BehaviorGen::callbackGetVMWayAreas,  this);
	sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &BehaviorGen::callbackGetVMCrossWalks,  this);
	sub_nodes = nh.subscribe("/vector_map_info/node", 1, &BehaviorGen::callbackGetVMNodes,  this);
}

BehaviorGen::~BehaviorGen()
{
	UtilityHNS::DataRW::WriteLogData(UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::StatesLogFolderName, "MainLog",
				"time,dt, Behavior_i, Behavior_str, RollOuts_n, Blocked_i, Central_i, Selected_i, StopSign_id, Light_id, Stop_Dist, Follow_Dist, Follow_Vel,"
				"Target_Vel, PID_Vel, T_cmd_Vel, C_cmd_Vel, Vel, Steer, X, Y, Z, Theta,"
				, m_LogData);
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

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_trajectory_evaluator/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_trajectory_evaluator/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

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
	nh.getParam("/op_common_params/additionalBrakingDistance", m_PlanningParams.additionalBrakingDistance );
	nh.getParam("/op_common_params/giveUpDistance", m_PlanningParams.giveUpDistance );

	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("/op_common_params/mapFileName" , m_MapPath);

	_nh.getParam("/op_behavior_selector/evidence_tust_number", m_PlanningParams.nReliableCount);

	//std::cout << "nReliableCount: " << m_PlanningParams.nReliableCount << std::endl;

	m_BehaviorGenerator.Init(controlParams, m_PlanningParams, m_CarInfo);
	m_BehaviorGenerator.m_pCurrentBehaviorState->m_Behavior = PlannerHNS::INITIAL_STATE;

}

void BehaviorGen::callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_Twist_raw = *msg;
}

void BehaviorGen::callbackGetTwistCMD(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_Twist_cmd = *msg;
}

void BehaviorGen::callbackGetCommandCMD(const autoware_msgs::ControlCommandConstPtr& msg)
{
	m_Ctrl_cmd = *msg;
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
			PlannerHNS::RosHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);
			PlannerHNS::Lane* pPrevValid = 0;
			for(unsigned int j = 0 ; j < m_temp_path.size(); j++)
			{
				PlannerHNS::Lane* pLane = 0;
				pLane = PlannerHNS::MappingHelpers::GetLaneById(m_temp_path.at(j).laneId, m_Map);
				if(!pLane)
				{
					pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMapDirectionBased(m_temp_path.at(j), m_Map, 1);

					if(!pLane && !pPrevValid)
					{
						ROS_ERROR("Map inconsistency between Global Path and Local Planer Map, Can't identify current lane.");
						return;
					}

					if(!pLane)
						m_temp_path.at(j).pLane = pPrevValid;
					else
					{
						m_temp_path.at(j).pLane = pLane;
						pPrevValid = pLane ;
					}

					m_temp_path.at(j).laneId = m_temp_path.at(j).pLane->id;
				}
				else
					m_temp_path.at(j).pLane = pLane;


				//std::cout << "StopLineInGlobalPath: " << m_temp_path.at(j).stopLineID << std::endl;
			}

			PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_temp_path);
			m_GlobalPaths.push_back(m_temp_path);

			if(bOldGlobalPath)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_temp_path, m_GlobalPaths.at(i));
			}
		}

		if(!bOldGlobalPath)
		{
			bWayGlobalPath = true;
			bWayGlobalPathLogs = true;
			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
				PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.35, 0.4, 0.05);

				PlannerHNS::PlanningHelpers::GenerateRecommendedSpeed(m_GlobalPaths.at(i), m_CarInfo.max_speed_forward, m_PlanningParams.speedProfileFactor);
				m_GlobalPaths.at(i).at(m_GlobalPaths.at(i).size()-1).v = 0;
			}

			std::cout << "Received New Global Path Selector! " << std::endl;
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
		m_RollOuts.clear();
		int globalPathId_roll_outs = -1;

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			std::vector<PlannerHNS::WayPoint> path;
			PlannerHNS::RosHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), path);
			m_RollOuts.push_back(path);

			if(path.size() > 0)
				globalPathId_roll_outs = path.at(0).gid;
		}

		if(bWayGlobalPath && m_GlobalPaths.size() > 0)
		{
			if(m_GlobalPaths.at(0).size() > 0)
			{
				int globalPathId = m_GlobalPaths.at(0).at(0).gid;
				std::cout << "Before Synchronization At Behavior Selector: GlobalID: " <<  globalPathId << ", LocalID: " << globalPathId_roll_outs << std::endl;

				if(globalPathId_roll_outs == globalPathId)
				{
					bWayGlobalPath = false;
					m_GlobalPathsToUse = m_GlobalPaths;
					m_BehaviorGenerator.SetNewGlobalPath(m_GlobalPathsToUse);
					std::cout << "Synchronization At Behavior Selector: GlobalID: " <<  globalPathId << ", LocalID: " << globalPathId_roll_outs << std::endl;
				}
			}
		}

		m_BehaviorGenerator.m_RollOuts = m_RollOuts;
		bRollOuts = true;
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
	bNewLightSignal = true;
	std::vector<PlannerHNS::TrafficLight> simulatedLights;
	for(unsigned int i = 0 ; i < msg.Signals.size() ; i++)
	{
		PlannerHNS::TrafficLight tl;
		tl.id = msg.Signals.at(i).signalId;

		for(unsigned int k = 0; k < m_Map.trafficLights.size(); k++)
		{
			if(m_Map.trafficLights.at(k).id == tl.id)
			{
				tl.pos = m_Map.trafficLights.at(k).pos;
				break;
			}
		}

		if(msg.Signals.at(i).type == 1)
		{
			tl.lightState = PlannerHNS::GREEN_LIGHT;
		}
		else
		{
			tl.lightState = PlannerHNS::RED_LIGHT;
		}

		simulatedLights.push_back(tl);
	}

	//std::cout << "Received Traffic Lights : " << lights.markers.size() << std::endl;

	m_CurrTrafficLight = simulatedLights;

//	int stopLineID = -1, stopSignID = -1, trafficLightID=-1;
//	PlannerHNS::PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_BehaviorGenerator.m_Path, m_CurrentPos, stopLineID, stopSignID, trafficLightID);
//	m_CurrTrafficLight.clear();
//	if(trafficLightID > 0)
//	{
//		for(unsigned int i=0; i < simulatedLights.size(); i++)
//		{
//			if(simulatedLights.at(i).id == trafficLightID)
//			{
//				m_CurrTrafficLight.push_back(simulatedLights.at(i));
//			}
//		}
//	}
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
	//pub_BehaviorStateRviz.publish(behavior_rviz);

	visualization_msgs::MarkerArray markerArray;

	//PlannerHNS::RosHelpers::GetIndicatorArrows(m_CurrentPos, m_CarInfo.width, m_CarInfo.length, m_CurrentBehavior.indicator, 0, markerArray);

	markerArray.markers.push_back(behavior_rviz);

	pub_BehaviorStateRviz.publish(markerArray);

	//To Test Synchronization Problem
//	visualization_msgs::MarkerArray selected_path;
//	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > paths;
//  paths.push_back(std::vector<std::vector<PlannerHNS::WayPoint> >());
//	paths.at(0).push_back(m_BehaviorGenerator.m_Path);
//	paths.push_back(m_GlobalPathsToUse);
//	paths.push_back(m_RollOuts);
//	PlannerHNS::RosHelpers::TrajectoriesToMarkers(paths, selected_path);
//	pub_SelectedPathRviz.publish(selected_path);
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

	PlannerHNS::WayPoint pose_center = PlannerHNS::PlanningHelpers::GetRealCenter(m_BehaviorGenerator.state, m_CarInfo.wheel_base);

	p_pose.position.x = pose_center.pos.x;
	p_pose.position.y = pose_center.pos.y;
	p_pose.position.z = pose_center.pos.z;
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

void BehaviorGen::LogLocalPlanningInfo(double dt)
{
	timespec log_t;
	UtilityHNS::UtilityH::GetTickCount(log_t);
	std::ostringstream dataLine;
	dataLine << UtilityHNS::UtilityH::GetLongTime(log_t) <<"," << dt << "," << m_CurrentBehavior.state << ","<< PlannerHNS::RosHelpers::GetBehaviorNameFromCode(m_CurrentBehavior.state) << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->m_pParams->rollOutNumber << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bFullyBlock << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->currentStopSignID << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->currentTrafficLightID << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->minStoppingDistance << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->distanceToNext << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->velocityOfNext << "," <<
			m_CurrentBehavior.maxVelocity << "," <<
			m_Twist_raw.twist.linear.x << "," <<
			m_Twist_cmd.twist.linear.x << "," <<
			m_Ctrl_cmd.linear_velocity << "," <<
			m_VehicleStatus.speed << "," <<
			m_VehicleStatus.steer << "," <<
			m_BehaviorGenerator.state.pos.x << "," << m_BehaviorGenerator.state.pos.y << "," << m_BehaviorGenerator.state.pos.z << "," << UtilityHNS::UtilityH::SplitPositiveAngle(m_BehaviorGenerator.state.pos.a)+M_PI << ",";
	m_LogData.push_back(dataLine.str());


	if(bWayGlobalPathLogs)
	{
		for(unsigned int i=0; i < m_GlobalPaths.size(); i++)
		{
			std::ostringstream str_out;
			str_out << UtilityHNS::UtilityH::GetHomeDirectory();
			str_out << UtilityHNS::DataRW::LoggingMainfolderName;
			str_out << UtilityHNS::DataRW::PathLogFolderName;
			str_out << "Global_Vel";
			str_out << i;
			str_out << "_";
			PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_GlobalPaths.at(i));
		}
		bWayGlobalPathLogs = false;
	}
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
		else if (m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap)
		{
			std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

			if(m_MapRaw.GetVersion()==2)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
						m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_PlanningParams.enableLaneChange, false);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map V2 Is Loaded successfully from the Behavior Selector !! " << std::endl;
				}
			}
			else if(m_MapRaw.GetVersion()==1)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true, m_PlanningParams.enableLaneChange, false);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map V1 Is Loaded successfully from the Behavior Selector !! " << std::endl;
				}
			}
		}

		if(bNewCurrentPos && m_GlobalPaths.size()>0)
		{
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

			m_CurrentBehavior = m_BehaviorGenerator.DoOneStep(dt, m_CurrentPos, m_VehicleStatus, 1, m_CurrTrafficLight, m_TrajectoryBestCost, 0 );

			SendLocalPlanningTopics();
			VisualizeLocalPlanner();
			LogLocalPlanningInfo(dt);
		}
		else
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&BehaviorGen::callbackGetGlobalPlannerPath, 	this);

		loop_rate.sleep();
	}
}

//Mapping Section

void BehaviorGen::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void BehaviorGen::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void BehaviorGen::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void BehaviorGen::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void BehaviorGen::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void BehaviorGen::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void BehaviorGen::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void BehaviorGen::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void BehaviorGen::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void BehaviorGen::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void BehaviorGen::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void BehaviorGen::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void BehaviorGen::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void BehaviorGen::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}
}
