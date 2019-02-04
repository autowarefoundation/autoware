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

#include "op_trajectory_evaluator_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"


namespace TrajectoryEvaluatorNS
{

TrajectoryEval::TrajectoryEval()
{
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bWayGlobalPath = false;
	bWayGlobalPathToUse = false;
	m_bUseMoveingObjectsPrediction = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_CollisionPointsRviz = nh.advertise<visualization_msgs::MarkerArray>("dynamic_collision_points_rviz", 1);
	pub_LocalWeightedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_eval_rviz", 1);
	pub_LocalWeightedTrajectories = nh.advertise<autoware_msgs::LaneArray>("local_weighted_trajectories", 1);
	pub_TrajectoryCost = nh.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
	pub_SafetyBorderRviz = nh.advertise<visualization_msgs::Marker>("safety_border", 1);

	sub_current_pose = nh.subscribe("/current_pose", 10, &TrajectoryEval::callbackGetCurrentPose, this);

	int bVelSource = 1;
	_nh.getParam("/op_trajectory_evaluator/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom = nh.subscribe("/odom", 10, &TrajectoryEval::callbackGetRobotOdom, this);
	else if(bVelSource == 1)
		sub_current_velocity = nh.subscribe("/current_velocity", 10, &TrajectoryEval::callbackGetVehicleStatus, this);
	else if(bVelSource == 2)
		sub_can_info = nh.subscribe("/can_info", 10, &TrajectoryEval::callbackGetCANInfo, this);

	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryEval::callbackGetGlobalPlannerPath, this);
	sub_LocalPlannerPaths = nh.subscribe("/local_trajectories", 1, &TrajectoryEval::callbackGetLocalPlannerPath, this);
	sub_predicted_objects = nh.subscribe("/predicted_objects", 1, &TrajectoryEval::callbackGetPredictedObjects, this);
	sub_current_behavior = nh.subscribe("/current_behavior", 1, &TrajectoryEval::callbackGetBehaviorState, this);

	PlannerHNS::ROSHelpers::InitCollisionPointsMarkers(50, m_CollisionsDummy);
}

TrajectoryEval::~TrajectoryEval()
{
}

void TrajectoryEval::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_trajectory_evaluator/enablePrediction", m_bUseMoveingObjectsPrediction);

	_nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);
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

}

void TrajectoryEval::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void TrajectoryEval::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEval::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_steer_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEval::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryEval::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{

		bool bOldGlobalPath = m_GlobalPaths.size() == msg->lanes.size();

		m_GlobalPaths.clear();

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);

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
			std::cout << "Received New Global Path Evaluator! " << std::endl;
		}
		else
		{
			m_GlobalPaths.clear();
		}
	}
}

void TrajectoryEval::callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		m_GeneratedRollOuts.clear();
		int globalPathId_roll_outs = -1;

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			std::vector<PlannerHNS::WayPoint> path;
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), path);
			m_GeneratedRollOuts.push_back(path);
			if(path.size() > 0)
				globalPathId_roll_outs = path.at(0).gid;
		}

		if(bWayGlobalPath && m_GlobalPaths.size() > 0 && m_GlobalPaths.at(0).size() > 0)
		{
			int globalPathId = m_GlobalPaths.at(0).at(0).gid;
			std::cout << "Before Synchronization At Trajectory Evaluator: GlobalID: " <<  globalPathId << ", LocalID: " << globalPathId_roll_outs << std::endl;

			if(globalPathId_roll_outs == globalPathId)
			{
				bWayGlobalPath = false;
				m_GlobalPathsToUse = m_GlobalPaths;
				std::cout << "Synchronization At Trajectory Evaluator: GlobalID: " <<  globalPathId << ", LocalID: " << globalPathId_roll_outs << std::endl;
			}
		}

		bRollOuts = true;
	}
}

void TrajectoryEval::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	m_PredictedObjects.clear();
	bPredictedObjects = true;

	PlannerHNS::DetectedObject obj;
	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		if(msg->objects.at(i).id > 0)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
			m_PredictedObjects.push_back(obj);
		}
//		else
//		{
//			std::cout << " Ego Car avoid detecting itself in trajectory evaluator node! ID: " << msg->objects.at(i).id << std::endl;
//		}
	}
}

void TrajectoryEval::callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_CurrentBehavior.iTrajectory = msg->twist.angular.z;
}

void TrajectoryEval::MainLoop()
{
	ros::Rate loop_rate(100);

	PlannerHNS::WayPoint prevState, state_change;

	while (ros::ok())
	{
		ros::spinOnce();
		PlannerHNS::TrajectoryCost tc;

		if(bNewCurrentPos && m_GlobalPaths.size()>0)
		{
			m_GlobalPathSections.clear();

			for(unsigned int i = 0; i < m_GlobalPathsToUse.size(); i++)
			{
				t_centerTrajectorySmoothed.clear();
				PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPathsToUse.at(i), m_CurrentPos, m_PlanningParams.horizonDistance , m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed);
				m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
			}

			if(m_GlobalPathSections.size()>0)
			{
				if(m_bUseMoveingObjectsPrediction)
					tc = m_TrajectoryCostsCalculator.DoOneStepDynamic(m_GeneratedRollOuts, m_GlobalPathSections.at(0), m_CurrentPos,m_PlanningParams,	m_CarInfo,m_VehicleStatus, m_PredictedObjects, m_CurrentBehavior.iTrajectory);
				else
					tc = m_TrajectoryCostsCalculator.DoOneStepStatic(m_GeneratedRollOuts, m_GlobalPathSections.at(0), m_CurrentPos,	m_PlanningParams,	m_CarInfo,m_VehicleStatus, m_PredictedObjects);

				autoware_msgs::Lane l;
				l.closest_object_distance = tc.closest_obj_distance;
				l.closest_object_velocity = tc.closest_obj_velocity;
				l.cost = tc.cost;
				l.is_blocked = tc.bBlocked;
				l.lane_index = tc.index;
				pub_TrajectoryCost.publish(l);
			}

			if(m_TrajectoryCostsCalculator.m_TrajectoryCosts.size() == m_GeneratedRollOuts.size())
			{
				autoware_msgs::LaneArray local_lanes;
				for(unsigned int i=0; i < m_GeneratedRollOuts.size(); i++)
				{
					autoware_msgs::Lane lane;
					PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_GeneratedRollOuts.at(i), lane);
					lane.closest_object_distance = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).closest_obj_distance;
					lane.closest_object_velocity = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).closest_obj_velocity;
					lane.cost = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).cost;
					lane.is_blocked = m_TrajectoryCostsCalculator.m_TrajectoryCosts.at(i).bBlocked;
					lane.lane_index = i;
					local_lanes.lanes.push_back(lane);
				}

				pub_LocalWeightedTrajectories.publish(local_lanes);
			}
			else
			{
				ROS_ERROR("m_TrajectoryCosts.size() Not Equal m_GeneratedRollOuts.size()");
			}

			if(m_TrajectoryCostsCalculator.m_TrajectoryCosts.size()>0)
			{
				visualization_msgs::MarkerArray all_rollOuts;
				PlannerHNS::ROSHelpers::TrajectoriesToColoredMarkers(m_GeneratedRollOuts, m_TrajectoryCostsCalculator.m_TrajectoryCosts, m_CurrentBehavior.iTrajectory, all_rollOuts);
				pub_LocalWeightedTrajectoriesRviz.publish(all_rollOuts);

				PlannerHNS::ROSHelpers::ConvertCollisionPointsMarkers(m_TrajectoryCostsCalculator.m_CollisionPoints, m_CollisionsActual, m_CollisionsDummy);
				pub_CollisionPointsRviz.publish(m_CollisionsActual);

				//Visualize Safety Box
				visualization_msgs::Marker safety_box;
				PlannerHNS::ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(m_TrajectoryCostsCalculator.m_SafetyBorder.points, safety_box);
				pub_SafetyBorderRviz.publish(safety_box);
			}
		}
		else
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&TrajectoryEval::callbackGetGlobalPlannerPath, 	this);

		loop_rate.sleep();
	}
}

}
