/*
 *  Copyright (c) 2017, Nagoya University
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

#include "op_motion_predictor_core.h"
#include "MappingHelpers.h"
#include "op_RosHelpers.h"

namespace MotionPredictorNS
{

MotionPrediction::MotionPrediction()
{
	bMap = false;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bTrackedObjects = false;
	m_bEnableCurbObstacles = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	PlannerHNS::RosHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_predicted_objects_trajectories = nh.advertise<autoware_msgs::DetectedObjectArray>("/predicted_objects", 1);
	pub_PredictedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("/predicted_trajectories_rviz", 1);
	pub_CurbsRviz					= nh.advertise<visualization_msgs::MarkerArray>("/map_curbs_rviz", 1);

	sub_tracked_objects	= nh.subscribe("/tracked_objects", 			1,		&MotionPrediction::callbackGetTrackedObjects, 		this);
	sub_current_pose 	= nh.subscribe("/current_pose", 			1,		&MotionPrediction::callbackGetCurrentPose, 		this);

	int bVelSource = 1;
	_nh.getParam("/op_motion_predictor/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom 			= nh.subscribe("/odom", 					100,	&MotionPrediction::callbackGetRobotOdom, 	this);
	else if(bVelSource == 1)
		sub_current_velocity 	= nh.subscribe("/current_velocity",		100,	&MotionPrediction::callbackGetVehicleStatus, 	this);
	else if(bVelSource == 2)
		sub_can_info 			= nh.subscribe("/can_info",		100,	&MotionPrediction::callbackGetCanInfo, 	this);

	UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
	PlannerHNS::RosHelpers::InitPredMarkers(100, m_PredictedTrajectoriesDummy);
	PlannerHNS::RosHelpers::InitCurbsMarkers(100, m_CurbsDummy);

	std::cout << "OpenPlanner Motion Predictor initialized successfully " << std::endl;
}

MotionPrediction::~MotionPrediction()
{
}

void MotionPrediction::UpdatePlanningParams(ros::NodeHandle& _nh)
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

	_nh.getParam("/op_trajectory_generator/width", m_CarInfo.width);
	_nh.getParam("/op_trajectory_generator/length", m_CarInfo.length);
	_nh.getParam("/op_trajectory_generator/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_trajectory_generator/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_trajectory_generator/maxSteerAngle", m_CarInfo.max_steer_angle);
	_nh.getParam("/op_trajectory_generator/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_trajectory_generator/maxDeceleration", m_CarInfo.max_deceleration);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

	int iSource = 0;
	_nh.getParam("/op_trajectory_generator/mapSource" 			, iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("/op_trajectory_generator/mapFileName" 		, m_MapPath);

	_nh.getParam("/op_motion_predictor/enableGenrateBranches" 		, m_PredictBeh.m_bGenerateBranches);
	_nh.getParam("/op_motion_predictor/max_distance_to_lane" 		, m_PredictBeh.m_MaxLaneDetectionDistance);
	_nh.getParam("/op_motion_predictor/prediction_distance" 		, m_PredictBeh.m_PredictionDistance);
	_nh.getParam("/op_motion_predictor/enableCurbObstacles"			, m_bEnableCurbObstacles);

	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);
}

void MotionPrediction::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void MotionPrediction::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_steer_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetTrackedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);
	m_TrackedObjects.clear();
	bTrackedObjects = true;

	PlannerHNS::DetectedObject obj;

	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		PlannerHNS::RosHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
		m_TrackedObjects.push_back(obj);
	}

	if(bMap)
	{

		m_PredictBeh.DoOneStep(m_TrackedObjects, m_CurrentPos, m_PlanningParams.minSpeed, m_CarInfo.max_deceleration,  m_Map);

		m_PredictedResultsResults.objects.clear();
		autoware_msgs::DetectedObject pred_obj;
		for(unsigned int i = 0 ; i <m_PredictBeh.m_PredictedObjects.size(); i++)
		{
			PlannerHNS::RosHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(m_PredictBeh.m_PredictedObjects.at(i), pred_obj);
			m_PredictedResultsResults.objects.push_back(pred_obj);
		}

		if(m_bEnableCurbObstacles)
		{
			curr_curbs_obstacles.clear();
			GenerateCurbsObstacles(curr_curbs_obstacles);
			for(unsigned int i = 0 ; i <curr_curbs_obstacles.size(); i++)
			{
				PlannerHNS::RosHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(curr_curbs_obstacles.at(i), pred_obj);
				m_PredictedResultsResults.objects.push_back(pred_obj);
			}
		}

		pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);
	}
}

void MotionPrediction::GenerateCurbsObstacles(std::vector<PlannerHNS::DetectedObject>& curb_obstacles)
{
	if(!bNewCurrentPos) return;

	for(unsigned int ic = 0; ic < m_Map.curbs.size(); ic++)
	{

		if(m_Map.curbs.at(ic).points.size() > 0)
		{
			PlannerHNS::DetectedObject obj;
			obj.center.pos = m_Map.curbs.at(ic).points.at(0);

			if(curb_obstacles.size()>0)
			{
				double distance_to_prev = hypot(curb_obstacles.at(curb_obstacles.size()-1).center.pos.y-obj.center.pos.y, curb_obstacles.at(curb_obstacles.size()-1).center.pos.x-obj.center.pos.x);
				if(distance_to_prev < 1.5)
					continue;
			}

			double longitudinalDist = hypot(m_CurrentPos.pos.y -obj.center.pos.y, m_CurrentPos.pos.x-obj.center.pos.x);

			if(longitudinalDist > m_PlanningParams.horizonDistance)
				continue;

			obj.bDirection = false;
			obj.bVelocity = false;
			obj.id = -1;
			obj.t  = PlannerHNS::SIDEWALK;
			obj.label = "curb";
			for(unsigned int icp=0; icp< m_Map.curbs.at(ic).points.size(); icp++)
			{
				obj.contour.push_back(m_Map.curbs.at(ic).points.at(icp));
			}

			curb_obstacles.push_back(obj);
		}
	}
}

void MotionPrediction::VisualizePrediction()
{
	m_all_pred_paths.clear();
	for(unsigned int i=0; i< m_PredictBeh.m_PredictedObjects.size(); i++)
		m_all_pred_paths.insert(m_all_pred_paths.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.end());

	PlannerHNS::RosHelpers::ConvertPredictedTrqajectoryMarkers(m_all_pred_paths, m_PredictedTrajectoriesActual, m_PredictedTrajectoriesDummy);
	pub_PredictedTrajectoriesRviz.publish(m_PredictedTrajectoriesActual);

	PlannerHNS::RosHelpers::ConvertCurbsMarkers(curr_curbs_obstacles, m_CurbsActual, m_CurbsDummy);
	pub_CurbsRviz.publish(m_CurbsActual);
}

void MotionPrediction::MainLoop()
{

	ros::Rate loop_rate(25);

	while (ros::ok())
	{
		ros::spinOnce();

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

		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_VisualizationTimer) > 0.25)
		{
			VisualizePrediction();
			UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
		}

//		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_SensingTimer) > 5)
//		{
//			ROS_INFO("op_motion_prediction sensing timeout, can't receive tracked object data ! Reset .. Reset");
//			m_PredictedResultsResults.objects.clear();
//			pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);
//		}

		loop_rate.sleep();
	}
}

}
