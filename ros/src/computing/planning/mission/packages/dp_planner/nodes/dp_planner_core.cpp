/*
 *  Copyright (c) 2015, Nagoya University
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
#include "dp_planner_core.h"
#include "RosHelpers.h"
#include <visualization_msgs/MarkerArray.h>
#include "geo_pos_conv.hh"
#include "PlannerHHandler.h"
#include "FreePlannerHandler.h"
#include "MappingHelpers.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <lidar_tracker/centroids.h>
#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>

namespace PlannerXNS
{

PlannerX_Interface* PlannerX_Interface::CreatePlannerInstance(const std::string& plannerName)
{
	if(plannerName.compare("DP") == 0)
		return new PlannerH_Handler;
	else if(plannerName.compare("Free") == 0)
		return new FreePlannerHandler;
	else
		return 0;
}

PlannerX::PlannerX(std::string plannerType, bool bAutoware, bool bKML, std::string kmlMapPath)
{
	clock_gettime(0, &m_Timer);
	m_counter = 0;
	m_frequency = 0;
	bInitPos = false;
	bGoalPos = false;
	bNewCurrentPos = false;
	bVehicleState = false;
	bNewDetectedObstacles = false;
	bTrafficLights = false;
	m_bAutoware = bAutoware;
	m_KmlMapPath = kmlMapPath;
	m_bKML_Map = bKML;
	bNewEmergency = false;
	m_bEmergencyStop = 0;
	bNewTrafficLigh = false;
	m_bGreenLight = 0;
	bNewOutsideControl = false;
	m_bOutsideControl = 0;
	bNewOutsideControl = false;
	m_bOutsideControl = 0;
	bNewAStarPath = true;
	m_bExternalPlanning = false;
	m_bStartAStartPlanner = false;
	m_bInitPoseFromMap = false;
	UtilityHNS::UtilityH::GetTickCount(m_AStartPlanningTimer);

	m_pPlanner = PlannerXNS::PlannerX_Interface::CreatePlannerInstance(plannerType);

	if(m_bAutoware)
	{
		tf::StampedTransform transform;
		RosHelpers::GetTransformFromTF("map", "world", transform);
		//ROS_INFO("Origin : x=%f, y=%f, z=%f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());

		m_OriginPos.position.x  = transform.getOrigin().x();
		m_OriginPos.position.y  = transform.getOrigin().y();
		m_OriginPos.position.z  = transform.getOrigin().z();
	}

	if(m_pPlanner)
		m_pPlanner->UpdateOriginTransformationPoint(m_OriginPos);


	m_DetectedPolygonsRviz = nh.advertise<visualization_msgs::MarkerArray>("detected_polygons", 100, true);
	m_PathPublisherRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_mark", 100, true);
	m_MapPublisherRviz = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines", 100, true);

	m_PathPublisher = nh.advertise<waypoint_follower::LaneArray>("lane_waypoints_array", 100, true);
	m_TrajectoryFinalWaypointPublisher = nh.advertise<waypoint_follower::lane>("final_waypoints", 100,true);
	m_BehaviorPublisher = nh.advertise<geometry_msgs::TwistStamped>("current_behavior", 100);
	m_TrackedObstaclesRviz  = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("dp_planner_tracked_boxes", 100);
	m_GlobalPlannerStart = nh.advertise<geometry_msgs::PoseStamped>("global_plan_start", 100);
	m_GlobalPlannerGoal = nh.advertise<geometry_msgs::PoseStamped>("global_plan_goal", 100);


//	sub_obj_pose 			= nh.subscribe("/obj_car/obj_label", 			100,
//			&PlannerX::callbackFromObjCar, 				this);
	sub_bounding_boxs = nh.subscribe("/bounding_boxes",						10,
			&PlannerX::callbackFromObjCar, 				this);
	sub_cluster_cloud = nh.subscribe("/cloud_clusters",						10,
			&PlannerX::callbackGetPointsClusters, 				this);
//	sub_centroids = nh.subscribe("/cluster_centroids",						1,
//			&PlannerX::callbackGetCentroids, 				this);
//	sub_AStarPlan = nh.subscribe("",						1,
//			&PlannerX::callbackGetAStarPath, 				this);
	sub_current_pose 		= nh.subscribe("/current_pose", 				100,
					&PlannerX::callbackFromCurrentPose, 		this);

	if(m_bAutoware)
	{
		point_sub 				= nh.subscribe("/vector_map_info/point_class", 	1,
				&PlannerX::callbackGetVMPoints, 			this);
		lane_sub 				= nh.subscribe("/vector_map_info/lane", 		1,
				&PlannerX::callbackGetVMLanes, 				this);
		node_sub 				= nh.subscribe("/vector_map_info/node", 		1,
				&PlannerX::callbackGetVMNodes, 				this);
		stopline_sub 			= nh.subscribe("/vector_map_info/stop_line", 	1,
				&PlannerX::callbackGetVMStopLines, 			this);
		dtlane_sub 				= nh.subscribe("/vector_map_info/dtlane", 		1,
				&PlannerX::callbackGetVMCenterLines, 		this);
		sub_traffic_light 		= nh.subscribe("/light_color", 					100,
				&PlannerX::callbackFromLightColor, 			this);
	}
	else
	{
		sub_EmergencyStop	 	= nh.subscribe("/emergency_stop_signal", 		100,
					&PlannerX::callbackGetEmergencyStop, 			this);
		sub_TrafficLight	 	= nh.subscribe("/traffic_signal_info", 			100,
					&PlannerX::callbackGetTrafficLight, 			this);
		sub_OutsideControl	 	= nh.subscribe("/usb_controller_r_signal", 		100,
					&PlannerX::callbackGetOutsideControl, 			this);

		m_bOutsideControl = 1;
	}

	initialpose_subscriber 	= nh.subscribe("initialpose", 					10,
			&PlannerX::callbackSimuInitPose, 			this);
	goalpose_subscriber 	= nh.subscribe("move_base_simple/goal", 		10,
			&PlannerX::callbackSimuGoalPose, 			this);
	sub_vehicle_status	 	= nh.subscribe("ff_vehicle_status", 			100,
				&PlannerX::callbackFromVehicleStatus, 			this);




	AutowarePlanningParams params;

	nh.getParam("/dp_planner/maxVelocity", params.maxSpeed);
	nh.getParam("/dp_planner/minVelocity", params.minSpeed);
	nh.getParam("/dp_planner/velocityProfileFactor", params.speedProfileFactor);
	nh.getParam("/dp_planner/maxPlanningDistance", params.planningDistance);
	nh.getParam("/dp_planner/maxLocalPlanDistance", params.microPlanDistance);
	nh.getParam("/dp_planner/samplingTipMargin", params.carTipMargin);
	nh.getParam("/dp_planner/samplingOutMargin", params.rollInMargin);
	nh.getParam("/dp_planner/samplingSpeedFactor", params.rollInSpeedFactor);

	nh.getParam("/dp_planner/pathDensity", params.pathDensity);
	nh.getParam("/dp_planner/rollOutDensity", params.rollOutDensity);
	nh.getParam("/dp_planner/rollOutsNumber", params.rollOutNumber);
	nh.getParam("/dp_planner/horizonDistance", params.horizonDistance);
	nh.getParam("/dp_planner/minFollowingDistance", params.minFollowingDistance);
	nh.getParam("/dp_planner/maxFollowingDistance", params.maxFollowingDistance);
	nh.getParam("/dp_planner/minDistanceToAvoid", params.minDistanceToAvoid);
	nh.getParam("/dp_planner/speedProfileFactor", params.speedProfileFactor);

	nh.getParam("/dp_planner/enableSwerving", params.enableSwerving);
	nh.getParam("/dp_planner/enableFollowing", params.enableFollowing);
	nh.getParam("/dp_planner/enableHeadingSmoothing", params.enableHeadingSmoothing);
	nh.getParam("/dp_planner/enableTrafficLightBehavior", params.enableTrafficLightBehavior);
	nh.getParam("/dp_planner/enableLaneChange", params.enableLaneChange);

	m_pPlanner->UpdatePlanningParams(params);


	double w = 0, l = 0, tr = 0, maxA = 0, wb = 0;

	nh.getParam("/dp_planner/width", w);
	nh.getParam("/dp_planner/length", l);
	nh.getParam("/dp_planner/wheelBaseLength", wb);
	nh.getParam("/dp_planner/turningRadius", tr);
	nh.getParam("/dp_planner/maxSteerAngle", maxA);

	m_pPlanner->UpdateVehicleInfo(w,l, wb, maxA, tr);

	std::string strInitPose, strGoals;
	nh.getParam("/dp_planner/InitialPose", strInitPose);
	nh.getParam("/dp_planner/GoalsPose", strGoals);

	if(strInitPose.compare("init") == 0)
		m_bInitPoseFromMap = true;

	if(strGoals.compare("rviz") == 0)
		m_bInitPoseFromMap = true;


}

PlannerX::~PlannerX()
{
	if(m_pPlanner)
		delete m_pPlanner;
}

void PlannerX::callbackGetPointsClusters(const lidar_tracker::CloudClusterArrayConstPtr& msg)
{
	//pcl::fromROSMsg(*msg, m_Points_Clusters);
	std::cout << " Number of Detected Clusters =" << msg->clusters.size() << std::endl;
	m_Points_Clusters = *msg;

}

void PlannerX::callbackGetAStarPath(const waypoint_follower::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0 && m_bExternalPlanning)
	{
		m_bExternalPlanning = false;
		bNewAStarPath = true;
		m_AStarPath = msg->lanes.at(0);
	}
}

void PlannerX::callbackGetOutsideControl(const std_msgs::Int8& msg)
{
	bNewOutsideControl = true;
	m_bOutsideControl  = msg.data;
	std::cout << "Received Outside Control : " << msg.data << std::endl;
}

void PlannerX::callbackGetEmergencyStop(const std_msgs::Int8& msg)
{
	bNewEmergency  = true;
	m_bEmergencyStop = msg.data;
	std::cout << "Received Emergency Stop : " << msg.data << std::endl;
}

void PlannerX::callbackGetTrafficLight(const std_msgs::Int8& msg)
{
	std::cout << "Received Traffic Light : " << msg.data << std::endl;

	bNewTrafficLigh = true;
	if(msg.data == 2)
		m_bGreenLight = 1;
	else
		m_bGreenLight = 0;
}

void PlannerX::callbackFromVehicleStatus(const geometry_msgs::Vector3StampedConstPtr& msg)
{
	m_VehicleState.steer = msg->vector.x;
	m_VehicleState.speed = msg->vector.y;
	if(msg->vector.z == 0x00)
		m_VehicleState.shift = AW_SHIFT_POS_BB;
	else if(msg->vector.z == 0x10)
		m_VehicleState.shift = AW_SHIFT_POS_DD;
	else if(msg->vector.z == 0x20)
		m_VehicleState.shift = AW_SHIFT_POS_NN;
	else if(msg->vector.z == 0x40)
		m_VehicleState.shift = AW_SHIFT_POS_RR;

	std::cout << "PlannerX: Read Status ("<<m_VehicleState.steer<< ", " << m_VehicleState.speed<<", " << msg->vector.z << ")" << std::endl;
}

void PlannerX::callbackSimuGoalPose(const geometry_msgs::PoseStamped &msg)
{
	if(!bGoalPos)
	{
		PlannerHNS::WayPoint p;
		ROS_INFO("Target Pose Data: x=%f, y=%f, z=%f, freq=%d", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, m_frequency);
		m_pPlanner->UpdateGlobalGoalPosition(msg.pose);
		bGoalPos = true;
	}
}

void PlannerX::callbackSimuInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	if(!bInitPos)
	{
		PlannerHNS::WayPoint p;
		ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, m_frequency);
		m_InitPos.position  = msg->pose.pose.position;
		m_InitPos.orientation = msg->pose.pose.orientation;
		bInitPos = true;
	}
}

void PlannerX::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // write procedure for current pose

	//PlannerHNS::WayPoint p;
	//std::cout<< "Pose Data: x=%f, y=%f, z=%f, freq=%d" << std::endl;//, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, m_frequency);
	m_counter++;
	double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer);
	if(dt >= 1.0)
	{
		m_frequency = m_counter;
		m_counter = 0;
		clock_gettime(0, &m_Timer);
	}

	geometry_msgs::Pose p = msg->pose;
	p.position.x = p.position.x - m_OriginPos.position.x;
	p.position.y = p.position.y - m_OriginPos.position.y;
	p.position.z = p.position.z - m_OriginPos.position.z;

	if(m_bAutoware)
	{
		double distance = hypot(m_CurrentPos.position.y-p.position.y, m_CurrentPos.position.x-p.position.x);
		m_VehicleState.speed = distance/dt;
		if(m_VehicleState.speed>0.2 || m_VehicleState.shift == AW_SHIFT_POS_DD )
			m_VehicleState.shift = AW_SHIFT_POS_DD;
		else if(m_VehicleState.speed<-0.2)
			m_VehicleState.shift = AW_SHIFT_POS_RR;
		else
			m_VehicleState.shift = AW_SHIFT_POS_NN;
	}

	m_CurrentPos.position  = p.position;
	m_CurrentPos.orientation = p.orientation;
	m_InitPos = m_CurrentPos;

	bNewCurrentPos = true;
	bInitPos = true;


}

void PlannerX::callbackFromLightColor(const runtime_manager::traffic_light& msg)
{
  // write procedure for traffic light
}

void PlannerX::callbackFromObjCar(const jsk_recognition_msgs::BoundingBoxArray& msg)
{
  // write procedure for car obstacle
	m_DetectedObstacles = msg;
	bNewDetectedObstacles = true;
	//std::cout << "Recieved Obstacles" << msg.boxes.size() << std::endl;
}

void PlannerX::callbackGetVMPoints(const map_file::PointClassArray& msg)
{
	ROS_INFO("Received Map Points");
	m_AwMap.points = msg;
	m_AwMap.bPoints = true;
}

void PlannerX::callbackGetVMLanes(const map_file::LaneArray& msg)
{
	ROS_INFO("Received Map Lane Array");
	m_AwMap.lanes = msg.lanes;
	m_AwMap.bLanes = true;
}

void PlannerX::callbackGetVMNodes(const map_file::NodeArray& msg)
{
	//ROS_INFO("Received Map Nodes");


}

void PlannerX::callbackGetVMStopLines(const map_file::StopLineArray& msg)
{
	//ROS_INFO("Received Map Stop Lines");
}

void PlannerX::callbackGetVMCenterLines(const map_file::DTLaneArray& msg)
{
	ROS_INFO("Received Map Center Lines");
	m_AwMap.dtlanes = msg.dtlanes;
	m_AwMap.bDtLanes = true;
}



void PlannerX::PlannerMainLoop()
{
	if(!m_pPlanner)
	{
		ROS_ERROR("Can't Create Planner Object ! ");
		return;
	}

	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		ros::spinOnce();

		if(!m_bAutoware)
		{
			visualization_msgs::MarkerArray map_marker_array;
			bool bNewMap = m_pPlanner->LoadRoadMap(m_KmlMapPath, m_bKML_Map, map_marker_array);
			if(bNewMap)
			{
				bInitPos = bGoalPos = false;
				m_MapPublisherRviz.publish(map_marker_array);
			}

		}
		else if(m_AwMap.bDtLanes && m_AwMap.bLanes && m_AwMap.bPoints)
		{
			m_pPlanner->UpdateRoadMap(m_AwMap);
			bInitPos = bGoalPos = false;
			m_AwMap.bDtLanes = m_AwMap.bLanes = m_AwMap.bPoints = false;
		}

		AutowareBehaviorState behState;
		visualization_msgs::MarkerArray marker_array;
		visualization_msgs::MarkerArray detectedPolygons;
		waypoint_follower::LaneArray lane_array;
		jsk_recognition_msgs::BoundingBoxArray trackedObjects;

		bool bNewPlan = false;

		if(bInitPos && bGoalPos)
		{
			if(m_VehicleState.shift == AW_SHIFT_POS_DD)
			{
				bNewPlan = m_pPlanner->GeneratePlan(m_CurrentPos,m_DetectedObstacles,
						m_Points_Clusters, m_TrafficLights, m_VehicleState,
						behState, marker_array, lane_array, trackedObjects, detectedPolygons, m_bEmergencyStop, m_bGreenLight, m_bOutsideControl,
						m_AStarPath, m_StartPoint, m_GoalPoints.at(0), m_bExternalPlanning);
			}
			else
			{
				bNewPlan = m_pPlanner->GeneratePlan(m_InitPos,m_DetectedObstacles,
						m_Points_Clusters, m_TrafficLights, m_VehicleState,
						behState, marker_array, lane_array, trackedObjects, detectedPolygons, m_bEmergencyStop, m_bGreenLight, m_bOutsideControl,
						m_AStarPath, m_StartPoint, m_GoalPoints.at(0), m_bExternalPlanning);

				std::cout << "Init Position Entry " << std::endl;
			}

			geometry_msgs::Twist t;
			geometry_msgs::TwistStamped behavior;
			t.linear.x = behState.followDistance;
			t.linear.y = behState.stopDistance;
			t.linear.z = (int)behState.indicator;

			t.angular.x = behState.followVelocity;
			t.angular.y = behState.maxVelocity;
			t.angular.z = (int)behState.state;

			behavior.twist = t;
			behavior.header.stamp = ros::Time::now();

			m_BehaviorPublisher.publish(behavior);
			m_DetectedPolygonsRviz.publish(detectedPolygons);
			m_TrackedObstaclesRviz.publish(trackedObjects);

			if(m_bExternalPlanning && !m_bStartAStartPlanner )
			{
				m_GlobalPlannerStart.publish(m_StartPoint);
				m_GlobalPlannerGoal.publish(m_GoalPoints.at(0));
				UtilityHNS::UtilityH::GetTickCount(m_AStartPlanningTimer);
				m_bStartAStartPlanner = true;

			}
		}

		if(bNewPlan)
		{
			if(lane_array.lanes.size()>0)
				std::cout << "New Plan , Path size = " << lane_array.lanes.at(0).waypoints.size() << std::endl;
			m_PathPublisher.publish(lane_array);
			m_PathPublisherRviz.publish(marker_array);
			m_TrajectoryFinalWaypointPublisher.publish(lane_array.lanes.at(0));
		}



		//ROS_INFO("Main Loop Step");
		loop_rate.sleep();
	}
}

}
