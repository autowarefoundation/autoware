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

#include <visualization_msgs/MarkerArray.h>
#include "geo_pos_conv.hh"


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "UtilityH.h"

namespace PlannerXNS
{

PlannerX::PlannerX()
{
	clock_gettime(0, &m_Timer);
	m_counter = 0;
	m_frequency = 0;
	m_bSignal = ROBOT_SIGNAL;

	bInitPos = false;
	bNewCurrentPos = false;
	bNewClusters = false;
	bNewBoxes = false;
	bVehicleState = false;
	bNewEmergency = false;
	m_bEmergencyStop = 0;
	bNewTrafficLigh = false;
	m_bGreenLight = 0;
	bNewOutsideControl = false;
	m_bOutsideControl = 0;
	bNewAStarPath = false;
	UtilityHNS::UtilityH::GetTickCount(m_AStartPlanningTimer);
	bWayPlannerPath = false;
	bKmlMapLoaded = false;
	m_bEnableTracking = true;
	m_nOriginalObstacles = 0;
	m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE = 5.5;
	m_ObstacleTracking.m_MAX_TRACKS_AFTER_LOSING = 5;
	m_ObstacleTracking.m_bUseCenterOnly = true;

	std::string str_signal;
	nh.getParam("/dp_planner/signal", str_signal);
	if(str_signal.compare("simulation")==0)
		m_bSignal = SIMULATION_SIGNAL;
	else
		m_bSignal = ROBOT_SIGNAL;

	std::string str_kml;
	nh.getParam("/dp_planner/map", str_kml);
	if(str_kml.compare("kml")==0)
		m_bKmlMap = true;
	else
		m_bKmlMap = false;
	nh.getParam("/dp_planner/mapDirectory", m_KmlMapPath);

	UpdatePlanningParams();

	tf::StampedTransform transform;
	RosHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();


	pub_LocalPath = nh.advertise<waypoint_follower::lane>("final_waypoints", 1,true);
	pub_BehaviorState = nh.advertise<geometry_msgs::TwistStamped>("current_behavior", 1);
	pub_GlobalPlanNodes = nh.advertise<geometry_msgs::PoseArray>("global_plan_nodes", 1);
	pub_StartPoint = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("GlobalStartpose", 1);
	pub_GoalPoint = nh.advertise<geometry_msgs::PoseStamped>("GlobalGoalPose", 1);
	pub_AStarStartPoint = nh.advertise<geometry_msgs::PoseStamped>("global_plan_start", 1);
	pub_AStarGoalPoint = nh.advertise<geometry_msgs::PoseStamped>("global_plan_goal", 1);

	pub_DetectedPolygonsRviz = nh.advertise<visualization_msgs::MarkerArray>("detected_polygons", 1, true);
	pub_TrackedObstaclesRviz = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("dp_planner_tracked_boxes", 1);
	pub_LocalTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories", 1);

	sub_initialpose 	= nh.subscribe("/initialpose", 				1,		&PlannerX::callbackGetInitPose, 		this);
	sub_current_pose 	= nh.subscribe("/current_pose", 			100,	&PlannerX::callbackGetCurrentPose, 		this);
	sub_cluster_cloud 	= nh.subscribe("/cloud_clusters",			1,		&PlannerX::callbackGetCloudClusters, 	this);
	sub_bounding_boxs  	= nh.subscribe("/bounding_boxes",			1,		&PlannerX::callbackGetBoundingBoxes, 	this);
	/**
	 * @todo This works only in simulation (Autoware or ff_Waypoint_follower), twist_cmd should be changed, consult team
	 */
	if(m_bSignal == SIMULATION_SIGNAL)
		sub_vehicle_status 	= nh.subscribe("/twist_cmd", 				100,	&PlannerX::callbackGetVehicleStatus, 	this);
	else
		sub_robot_odom 		= nh.subscribe("/odom", 					100,	&PlannerX::callbackGetRobotOdom, 	this);
	sub_EmergencyStop 	= nh.subscribe("/emergency_stop_signal", 	100,	&PlannerX::callbackGetEmergencyStop, 	this);
	sub_TrafficLight 	= nh.subscribe("/traffic_signal_info", 		10,		&PlannerX::callbackGetTrafficLight, 	this);
	sub_OutsideControl 	= nh.subscribe("/usb_controller_r_signal", 	10,		&PlannerX::callbackGetOutsideControl, 	this);
	sub_AStarPath 		= nh.subscribe("/astar_path", 				10,		&PlannerX::callbackGetAStarPath, 		this);
	sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&PlannerX::callbackGetWayPlannerPath, 	this);

	if(!m_bKmlMap)
	{
		sub_map_points 	= nh.subscribe("/vector_map_info/point", 		1, &PlannerX::callbackGetVMPoints, 		this);
		sub_map_lanes 	= nh.subscribe("/vector_map_info/lane", 		1, &PlannerX::callbackGetVMLanes, 		this);
		sub_map_nodes 	= nh.subscribe("/vector_map_info/node", 		1, &PlannerX::callbackGetVMNodes, 		this);
		sup_stop_lines 	= nh.subscribe("/vector_map_info/stop_line",	1, &PlannerX::callbackGetVMStopLines, 	this);
		sub_dtlanes 	= nh.subscribe("/vector_map_info/dtlane", 		1, &PlannerX::callbackGetVMCenterLines,	this);
	}

	sub_simulated_obstacle_pose_rviz = nh.subscribe("/clicked_point", 		1, &PlannerX::callbackGetRvizPoint,	this);

	if(!m_bEnableOutsideControl)
		m_bOutsideControl = 1;

//	PlannerHNS::WayPoint g1(557.1, 177.43, 0, 0);
//	PlannerHNS::WayPoint g2(553.03, 195.59, 0, 0);
//	PlannerHNS::WayPoint g3(-57.23, 60.67, 0, 0);
//	m_goals.push_back(g1);
//	m_goals.push_back(g2);
//	m_goals.push_back(g3);
//	m_iCurrentGoal = 0;

//	//Initialize Static Traffic Light
//	PlannerHNS::TrafficLight t1, t2;
//	PlannerHNS::GPSPoint stopT1(555.84, 181.89,0,0);
//	t1.id = 1;
//	t1.pos = PlannerHNS::GPSPoint(555.72,193.23, 0, 91.65 * DEG2RAD);
//	t1.stoppingDistance = hypot(t1.pos.y-stopT1.y, t1.pos.x - stopT1.x);
//	m_State.m_TrafficLights.push_back(t1);
//
//	PlannerHNS::GPSPoint stopT2(553.85,193.14,0,0);
//	t2.id = 2;
//	t2.pos = PlannerHNS::GPSPoint(552.33, 181.42, 0, 270 * DEG2RAD);
//	t2.stoppingDistance = hypot(t2.pos.y-stopT2.y, t2.pos.x - stopT2.x);
//	m_State.m_TrafficLights.push_back(t2);

}

PlannerX::~PlannerX()
{
}


void PlannerX::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	ROS_INFO("Received Map Points");
	m_AwMap.points = msg;
	m_AwMap.bPoints = true;
}

void PlannerX::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	ROS_INFO("Received Map Lane Array");
	m_AwMap.lanes = msg;
	m_AwMap.bLanes = true;
}

void PlannerX::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	//ROS_INFO("Received Map Nodes");


}

void PlannerX::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	//ROS_INFO("Received Map Stop Lines");
}

void PlannerX::callbackGetVMCenterLines(const vector_map_msgs::DTLaneArray& msg)
{
	ROS_INFO("Received Map Center Lines");
	m_AwMap.dtlanes = msg;
	m_AwMap.bDtLanes = true;
}

void PlannerX::UpdatePlanningParams()
{
	PlannerHNS::PlanningParams params;

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

	nh.getParam("/dp_planner/enableObjectTracking", m_bEnableTracking);
	nh.getParam("/dp_planner/enableOutsideControl", m_bEnableOutsideControl);

	PlannerHNS::ControllerParams controlParams;
	controlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01);
//	m_ControlParams.SteeringDelay = 0.85;
//	m_ControlParams.Steering_Gain.kD = 0.5;
//	m_ControlParams.Steering_Gain.kP = 0.1;
//	m_ControlParams.Steering_Gain.kI = 0.03;
	controlParams.Velocity_Gain = PlannerHNS::PID_CONST(0.1, 0.005, 0.1);


	PlannerHNS::CAR_BASIC_INFO vehicleInfo;

	nh.getParam("/dp_planner/width", vehicleInfo.width);
	nh.getParam("/dp_planner/length", vehicleInfo.length);
	nh.getParam("/dp_planner/wheelBaseLength", vehicleInfo.wheel_base);
	nh.getParam("/dp_planner/turningRadius", vehicleInfo.turning_radius);
	nh.getParam("/dp_planner/maxSteerAngle", vehicleInfo.max_steer_angle);

	m_LocalPlanner.m_SimulationSteeringDelayFactor = controlParams.SimulationSteeringDelay;
	m_LocalPlanner.Init(controlParams, params, vehicleInfo);
	m_LocalPlanner.m_pCurrentBehaviorState->m_Behavior = PlannerHNS::INITIAL_STATE;
}

void PlannerX::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	if(!bInitPos)
	{
		PlannerHNS::WayPoint p;
		ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, m_frequency);
		m_InitPos = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x,
				msg->pose.pose.position.y+m_OriginPos.position.y,
				msg->pose.pose.position.z+m_OriginPos.position.z,
				tf::getYaw(msg->pose.pose.orientation));
		m_CurrentPos = m_InitPos;
		bInitPos = true;
	}
}

void PlannerX::callbackGetRvizPoint(const geometry_msgs::PointStampedConstPtr& msg)
{
	PlannerHNS::WayPoint p(msg->point.x+m_OriginPos.position.x,
					msg->point.y+m_OriginPos.position.y,
					msg->point.z+m_OriginPos.position.z,0);

	//Add Simulated Obstacle polygon

	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);
	double width = ((double)(rand()%10)/10.0) * 1.5 + 0.25;
	double length = ((double)(rand()%10)/10.0) * 0.5 + 0.25;

	lidar_tracker::CloudClusterArray clusters_array;
	clusters_array.clusters.push_back(GenerateSimulatedObstacleCluster(width, length, 1.0, 75, *msg));
	RosHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(m_CurrentPos, m_LocalPlanner.m_CarInfo, clusters_array, m_DetectedClusters);

}

void PlannerX::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_counter++;
	double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer);
	if(dt >= 1.0)
	{
		m_frequency = m_counter;
		m_counter = 0;
		clock_gettime(0, &m_Timer);
	}

	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y,
					msg->pose.position.z, tf::getYaw(msg->pose.orientation));

	m_InitPos = m_CurrentPos;

	bNewCurrentPos = true;
	bInitPos = true;

//	geometry_msgs::Pose p = msg->pose;
//	p.position.x = p.position.x - m_OriginPos.position.x;
//	p.position.y = p.position.y - m_OriginPos.position.y;
//	p.position.z = p.position.z - m_OriginPos.position.z;
//
//	if(m_bAutoware)
//	{
//		double distance = hypot(m_CurrentPos.position.y-p.position.y, m_CurrentPos.position.x-p.position.x);
//		m_VehicleState.speed = distance/dt;
//		if(m_VehicleState.speed>0.2 || m_VehicleState.shift == AW_SHIFT_POS_DD )
//			m_VehicleState.shift = AW_SHIFT_POS_DD;
//		else if(m_VehicleState.speed<-0.2)
//			m_VehicleState.shift = AW_SHIFT_POS_RR;
//		else
//			m_VehicleState.shift = AW_SHIFT_POS_NN;
//	}

}

lidar_tracker::CloudCluster PlannerX::GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::PointStamped& centerPose)
{
	lidar_tracker::CloudCluster cluster;
	cluster.centroid_point.point = centerPose.point;
	cluster.dimensions.x = x_rand;
	cluster.dimensions.y = y_rand;
	cluster.dimensions.z = z_rand;
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	timespec t;
	for(int i=1; i < nPoints; i++)
	{
		UtilityHNS::UtilityH::GetTickCount(t);
		pcl::PointXYZ p;
		srand(t.tv_nsec/i);
		double x = (double)(rand()%100)/100.0 - 0.5;

		srand(t.tv_nsec/i*i);
		double y = (double)(rand()%100)/100.0 - 0.5;

		srand(t.tv_nsec);
		double z = (double)(rand()%100)/100.0 - 0.5;

		p.x = centerPose.point.x + x*x_rand;
		p.y = centerPose.point.y + y*y_rand;
		p.z = centerPose.point.z + z*z_rand;
		point_cloud.points.push_back(p);
	}

	pcl::toROSMsg(point_cloud, cluster.cloud);

	return cluster;
}

void PlannerX::callbackGetCloudClusters(const lidar_tracker::CloudClusterArrayConstPtr& msg)
{
//	timespec timerTemp;
//	UtilityHNS::UtilityH::GetTickCount(timerTemp);
	m_nOriginalObstacles = msg->clusters.size();
	RosHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(m_CurrentPos, m_LocalPlanner.m_CarInfo, *msg, m_DetectedClusters);
	if(m_bEnableTracking)
	{
		m_ObstacleTracking.DoOneStep(m_CurrentPos, m_DetectedClusters);
		m_DetectedClusters = m_ObstacleTracking.m_DetectedObjects;
	}
	//std::cout << "Calculating Contour Time : " <<UtilityHNS::UtilityH::GetTimeDiffNow(timerTemp) << ", For Objectis: " << m_DetectedClusters.size() <<  std::endl;
	bNewClusters = true;
}

void PlannerX::callbackGetBoundingBoxes(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg)
{
//	std::cout << " Number of Detected Boxes =" << msg->boxes.size() << std::endl;
//	RosHelpers::ConvertFromAutowareBoundingBoxObstaclesToPlannerH(*msg, m_DetectedBoxes);
//	bNewBoxes = true;
}

void PlannerX::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.linear.x;
	m_VehicleState.steer = msg->twist.angular.z;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
	//m_VehicleState.steer = atan(m_State.m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
//	if(msg->vector.z == 0x00)
//		m_VehicleState.shift = AW_SHIFT_POS_BB;
//	else if(msg->vector.z == 0x10)
//		m_VehicleState.shift = AW_SHIFT_POS_DD;
//	else if(msg->vector.z == 0x20)
//		m_VehicleState.shift = AW_SHIFT_POS_NN;
//	else if(msg->vector.z == 0x40)
//		m_VehicleState.shift = AW_SHIFT_POS_RR;

	//std::cout << "PlannerX: Read Status Twist_cmd ("<< m_VehicleState.speed << ", " << m_VehicleState.steer<<")" << std::endl;
}

void PlannerX::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.twist.linear.x;
	m_VehicleState.steer = atan(m_LocalPlanner.m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
//	if(msg->vector.z == 0x00)
//		m_VehicleState.shift = AW_SHIFT_POS_BB;
//	else if(msg->vector.z == 0x10)
//		m_VehicleState.shift = AW_SHIFT_POS_DD;
//	else if(msg->vector.z == 0x20)
//		m_VehicleState.shift = AW_SHIFT_POS_NN;
//	else if(msg->vector.z == 0x40)
//		m_VehicleState.shift = AW_SHIFT_POS_RR;

	std::cout << "PlannerX: Read Odometry ("<< m_VehicleState.speed << ", " << m_VehicleState.steer<<")" << std::endl;
}

void PlannerX::callbackGetEmergencyStop(const std_msgs::Int8& msg)
{
	std::cout << "Received Emergency Stop : " << msg.data << std::endl;
	bNewEmergency  = true;
	m_bEmergencyStop = msg.data;
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

void PlannerX::callbackGetOutsideControl(const std_msgs::Int8& msg)
{
	std::cout << "Received Outside Control : " << msg.data << std::endl;
	bNewOutsideControl = true;

	if(m_bEnableOutsideControl && m_CurrentBehavior.state == PlannerHNS::INITIAL_STATE)
		m_bOutsideControl  = msg.data;
	else if(m_bEnableOutsideControl && m_CurrentBehavior.state == PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE)
		m_bGreenLight = 1;
}

void PlannerX::callbackGetAStarPath(const waypoint_follower::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		m_AStarPath.clear();
		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			for(unsigned int j = 0 ; j < msg->lanes.at(i).waypoints.size(); j++)
			{
				PlannerHNS::WayPoint wp(msg->lanes.at(i).waypoints.at(j).pose.pose.position.x,
						msg->lanes.at(i).waypoints.at(j).pose.pose.position.y,
						msg->lanes.at(i).waypoints.at(j).pose.pose.position.z,
						tf::getYaw(msg->lanes.at(i).waypoints.at(j).pose.pose.orientation));
				wp.v = msg->lanes.at(i).waypoints.at(j).twist.twist.linear.x;
				//wp.bDir = msg->lanes.at(i).waypoints.at(j).dtlane.dir;
				m_AStarPath.push_back(wp);
			}
		}
		bNewAStarPath = true;
		m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bRePlan = true;
	}
}

void PlannerX::callbackGetWayPlannerPath(const waypoint_follower::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		m_WayPlannerPaths.clear();
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
				//wp.bDir = msg->lanes.at(i).waypoints.at(j).dtlane.dir;

				PlannerHNS::Lane* pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMapDirectionBased(wp, m_Map, 1);
				if(!pLane && !pPrevValid)
				{
					ROS_ERROR("Map inconsistency between Global Path add Lal Planer, Can't identify current lane.");
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

				path.push_back(wp);
			}
			m_WayPlannerPaths.push_back(path);
		}

		bWayPlannerPath = true;
		m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bRePlan = true;
		//m_goals.at(m_iCurrentGoal) = m_WayPlannerPaths.at(0).at(m_WayPlannerPaths.at(0).size()-1);
		m_CurrentGoal = m_WayPlannerPaths.at(0).at(m_WayPlannerPaths.at(0).size()-1);
		m_LocalPlanner.m_TotalPath = m_WayPlannerPaths;
	}
}

void PlannerX::PlannerMainLoop()
{
	ros::Rate loop_rate(100);
	std::vector<PlannerHNS::DetectedObject> obj_list;

	while (ros::ok())
	{
		timespec iterationTime;
		UtilityHNS::UtilityH::GetTickCount(iterationTime);

		ros::spinOnce();

		if(m_bKmlMap && !bKmlMapLoaded)
		{
			bKmlMapLoaded = true;
			PlannerHNS::MappingHelpers::LoadKML(m_KmlMapPath, m_Map);
			//sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	10,		&PlannerX::callbackGetWayPlannerPath, 	this);
		}
		else if(m_AwMap.bDtLanes && m_AwMap.bLanes && m_AwMap.bPoints && !m_bKmlMap)
		 {
			timespec timerTemp;
			UtilityHNS::UtilityH::GetTickCount(timerTemp);
			 m_AwMap.bDtLanes = m_AwMap.bLanes = m_AwMap.bPoints = false;
			 RosHelpers::UpdateRoadMap(m_AwMap,m_Map);
			 std::cout << "Converting Vector Map Time : " <<UtilityHNS::UtilityH::GetTimeDiffNow(timerTemp) << std::endl;
			 //sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	10,		&PlannerX::callbackGetWayPlannerPath, 	this);
		 }

		if(bInitPos && m_LocalPlanner.m_TotalPath.size()>0)
		//if(bInitPos)
		{
			bool bMakeNewPlan = false;
			m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = m_bOutsideControl;

			double drift = hypot(m_LocalPlanner.state.pos.y-m_CurrentPos.pos.y, m_LocalPlanner.state .pos.x-m_CurrentPos.pos.x);
			if(drift > 10)
				bMakeNewPlan = true;

			m_LocalPlanner.state = m_CurrentPos;

//			int currIndexToal = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_State.m_TotalPath, m_State.state);
//			if(bMakeNewPlan == false && m_CurrentBehavior.state == PlannerHNS::STOPPING_STATE && (m_iCurrentGoal+1) < m_goals.size())
//			{
//				if(m_State.m_TotalPath.size() > 0 && currIndexToal > m_State.m_TotalPath.size() - 8)
//				{
//					m_iCurrentGoal = m_iCurrentGoal + 1;
//					bMakeNewPlan = true;
//				}
//			}


			obj_list = m_DetectedClusters;


//			PlannerHNS::WayPoint goal_wp;
//			if(m_iCurrentGoal+1 < m_goals.size())
//				goal_wp = m_goals.at(m_iCurrentGoal);

			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			m_CurrentBehavior = m_LocalPlanner.DoOneStep(dt, m_VehicleState, obj_list, m_CurrentGoal.pos, m_Map, m_bEmergencyStop, m_bGreenLight, true);

			if(m_CurrentBehavior.state != m_PrevBehavior.state)
			{
				std::cout << m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->ToString(m_CurrentBehavior.state) << ", Speed : " << m_CurrentBehavior.maxVelocity << std::endl;
				m_PrevBehavior = m_CurrentBehavior;
			}
			//std::cout << "Planning Time = " << dt << std::endl;

			geometry_msgs::Twist t;
			geometry_msgs::TwistStamped behavior;
			t.linear.x = m_CurrentBehavior.followDistance;
			t.linear.y = m_CurrentBehavior.stopDistance;
			t.linear.z = (int)m_CurrentBehavior.indicator;

			t.angular.x = m_CurrentBehavior.followVelocity;
			t.angular.y = m_CurrentBehavior.maxVelocity;
			t.angular.z = (int)m_CurrentBehavior.state;

			behavior.twist = t;
			behavior.header.stamp = ros::Time::now();

			pub_BehaviorState.publish(behavior);

			visualization_msgs::MarkerArray detectedPolygons;
			RosHelpers::ConvertFromPlannerObstaclesToAutoware(m_CurrentPos, obj_list, detectedPolygons);
			pub_DetectedPolygonsRviz.publish(detectedPolygons);

		}
		else
		{
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
			sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&PlannerX::callbackGetWayPlannerPath, 	this);
		}

		if(m_CurrentBehavior.bNewPlan)
		{
			waypoint_follower::lane current_trajectory;
			RosHelpers::ConvertFromPlannerHToAutowarePathFormat(m_LocalPlanner.m_Path, current_trajectory);
			pub_LocalPath.publish(current_trajectory);

			visualization_msgs::MarkerArray all_rollOuts;
			RosHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(m_LocalPlanner.m_Path, m_LocalPlanner.m_RollOuts.at(0), all_rollOuts);
			pub_LocalTrajectoriesRviz.publish(all_rollOuts);
		}

		loop_rate.sleep();

		double onePassTime = UtilityHNS::UtilityH::GetTimeDiffNow(iterationTime);
		if(onePassTime > 0.1)
			std::cout << "Slow Iteration Time = " << onePassTime << " , for Obstacles : (" << m_DetectedClusters.size() << ", " << m_nOriginalObstacles << ")" <<  std::endl;
	}
}

}
