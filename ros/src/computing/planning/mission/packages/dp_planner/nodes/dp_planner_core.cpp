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
#include "MatrixOperations.h"

namespace PlannerXNS
{

PlannerX::PlannerX()
{

	clock_gettime(0, &m_Timer);
	m_counter = 0;
	m_frequency = 0;
	m_bSignal = ROBOT_SIGNAL;

	m_nTrackObjects = 0;
	m_nContourPoints = 0;
	m_nOriginalPoints = 0;
	m_TrackingTime = 0;
	bInitPos = false;
	bNewCurrentPos = false;
	bNewClusters = false;
	bNewBoxes = false;
	bVehicleState = false;
	bNewEmergency = false;
	m_bEmergencyStop = 0;
	bNewTrafficLigh = false;
	m_bGreenLight = false; UtilityHNS::UtilityH::GetTickCount(m_TrafficLightTimer);
	bNewOutsideControl = false;
	m_bOutsideControl = 0;
	bNewAStarPath = false;
	UtilityHNS::UtilityH::GetTickCount(m_AStartPlanningTimer);
	bWayPlannerPath = false;
	bKmlMapLoaded = false;
	m_bEnableTracking = true;
	m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE = 6.0;
	m_ObstacleTracking.m_MAX_TRACKS_AFTER_LOSING = 5;
	m_ObstacleTracking.m_DT = 0.12;
	m_ObstacleTracking.m_bUseCenterOnly = true;

	bool bSimulation = false;
	nh.getParam("/dp_planner/enableSimulation", bSimulation);
	if(bSimulation)
		m_bSignal = SIMULATION_SIGNAL;
	else
		m_bSignal = ROBOT_SIGNAL;

	int iSource = 0;
	nh.getParam("/dp_planner/mapSource", iSource);
	if(iSource == 0)
		m_MapSource = MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapSource = MAP_FOLDER;
	else if(iSource == 2)
		m_MapSource = MAP_KML_FILE;

	nh.getParam("/dp_planner/mapFileName", m_KmlMapPath);

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
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::Marker>("behavior_state", 1);


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

	if(m_MapSource == MAP_AUTOWARE)
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
}

PlannerX::~PlannerX()
{
	UtilityHNS::DataRW::WriteLogData(UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName, "MainLog",
			"time,Behavior,Tracked_Objects_Num, Cluster_Points_Num, Contour_Points_Num, Tracking_Time, Calc_Cost_Time, Behavior_Gen_Time, Roll_Out_Gen_Time, RollOuts_Num, Full_Block, idx_Central_traj, idx_safe_traj, id_stop_sign, id_traffic_light, Min_Stop_Distance, Velocity, follow_distance, follow_velocity, X, Y, Z, heading,"
			, m_LogData);
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
}

void PlannerX::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
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
	//Add Simulated Obstacle polygon
	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);
	double width = ((double)(rand()%10)/10.0) * 1.5 + 0.25;
	double length = ((double)(rand()%10)/10.0) * 0.5 + 0.25;

	geometry_msgs::PointStamped point;
	point.point.x = msg->point.x+m_OriginPos.position.x;
	point.point.y = msg->point.y+m_OriginPos.position.y;
	point.point.z = msg->point.z+m_OriginPos.position.z;

	lidar_tracker::CloudClusterArray clusters_array;
	clusters_array.clusters.push_back(GenerateSimulatedObstacleCluster(width, length, 1.0, 150, point));
	m_OriginalClusters.clear();
	int nNum1, nNum2;
	RosHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(m_CurrentPos, m_LocalPlanner.m_CarInfo, clusters_array, m_OriginalClusters, nNum1, nNum2);
	m_TrackedClusters = m_OriginalClusters;
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
	timespec timerTemp;
	UtilityHNS::UtilityH::GetTickCount(timerTemp);

	m_OriginalClusters.clear();
	RosHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(m_CurrentPos, m_LocalPlanner.m_CarInfo, *msg, m_OriginalClusters, m_nOriginalPoints, m_nContourPoints);
	if(m_bEnableTracking)
	{
		m_ObstacleTracking.DoOneStep(m_CurrentPos, m_OriginalClusters);
		m_TrackedClusters = m_ObstacleTracking.m_DetectedObjects;
	}
	else
		m_TrackedClusters = m_OriginalClusters;

	m_nTrackObjects = m_TrackedClusters.size();
	m_TrackingTime = UtilityHNS::UtilityH::GetTimeDiffNow(timerTemp);
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
	// If steering is in angular velocity
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
	//std::cout << "Received Emergency Stop : " << msg.data << std::endl;
	bNewEmergency  = true;
	m_bEmergencyStop = msg.data;
}

void PlannerX::callbackGetTrafficLight(const std_msgs::Int8& msg)
{
	std::cout << "Received Traffic Light : " << msg.data << std::endl;
	bNewTrafficLigh = true;
	if(msg.data == 2)
		m_bGreenLight = true;
	else
		m_bGreenLight = false;
}

void PlannerX::callbackGetOutsideControl(const std_msgs::Int8& msg)
{
	std::cout << "Received Outside Control : " << msg.data << std::endl;
	bNewOutsideControl = true;

	if(m_bEnableOutsideControl && m_CurrentBehavior.state == PlannerHNS::INITIAL_STATE)
		m_bOutsideControl  = msg.data;
	else if(m_bEnableOutsideControl && m_CurrentBehavior.state == PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE)
		m_bGreenLight = true;
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
				wp.laneId = msg->lanes.at(i).waypoints.at(j).twist.twist.linear.y;
				wp.stopLineID = msg->lanes.at(i).waypoints.at(j).twist.twist.linear.z;
				wp.LeftLaneId = msg->lanes.at(i).waypoints.at(j).twist.twist.angular.x;
				wp.RightLaneId = msg->lanes.at(i).waypoints.at(j).twist.twist.angular.y;
				if(msg->lanes.at(i).waypoints.at(j).twist.twist.angular.z == 0)
					wp.bDir = PlannerHNS::FORWARD_DIR;
				else if(msg->lanes.at(i).waypoints.at(j).twist.twist.angular.z == 1)
					wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
				else if(msg->lanes.at(i).waypoints.at(j).twist.twist.angular.z == 2)
					wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;

				PlannerHNS::Lane* pLane = 0;
				pLane = PlannerHNS::MappingHelpers::GetLaneById(wp.laneId, m_Map);
				if(!pLane)
				{
					pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMapDirectionBased(wp, m_Map, 1);

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
				}
				else
					wp.pLane = pLane;

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

	timespec trackingTimer;
	UtilityHNS::UtilityH::GetTickCount(trackingTimer);
	PlannerHNS::WayPoint prevState, state_change;

	while (ros::ok())
	{
		timespec iterationTime;
		UtilityHNS::UtilityH::GetTickCount(iterationTime);

		ros::spinOnce();

		if(m_MapSource == MAP_KML_FILE && !bKmlMapLoaded)
		{
			bKmlMapLoaded = true;
			PlannerHNS::MappingHelpers::LoadKML(m_KmlMapPath, m_Map);
			//sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	10,		&PlannerX::callbackGetWayPlannerPath, 	this);
		}
		else if(m_MapSource == MAP_FOLDER && !bKmlMapLoaded)
		{
			bKmlMapLoaded = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_KmlMapPath, m_Map, true);
		}
		else if(m_MapSource == MAP_AUTOWARE)
		{
			 if(m_AwMap.bDtLanes && m_AwMap.bLanes && m_AwMap.bPoints)
			 {
				timespec timerTemp;
				UtilityHNS::UtilityH::GetTickCount(timerTemp);
				 m_AwMap.bDtLanes = m_AwMap.bLanes = m_AwMap.bPoints = false;
				 RosHelpers::UpdateRoadMap(m_AwMap,m_Map);
				 std::cout << "Converting Vector Map Time : " <<UtilityHNS::UtilityH::GetTimeDiffNow(timerTemp) << std::endl;
				 //sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	10,		&PlannerX::callbackGetWayPlannerPath, 	this);
			 }
		}

		if(bInitPos && m_LocalPlanner.m_TotalPath.size()>0)
		{
//			bool bMakeNewPlan = false;
//			double drift = hypot(m_LocalPlanner.state.pos.y-m_CurrentPos.pos.y, m_LocalPlanner.state .pos.x-m_CurrentPos.pos.x);
//			if(drift > 10)
//				bMakeNewPlan = true;

			m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = m_bOutsideControl;
			m_LocalPlanner.state = m_CurrentPos;

			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			m_CurrentBehavior = m_LocalPlanner.DoOneStep(dt, m_VehicleState, m_TrackedClusters, m_CurrentGoal.pos, m_Map, m_bEmergencyStop, m_bGreenLight, true);

			visualization_msgs::Marker behavior_rviz;

			int iDirection = 0;
			if(m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory > m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
				iDirection = 1;
			else if(m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory < m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
				iDirection = -1;

			RosHelpers::VisualizeBehaviorState(m_CurrentPos, m_CurrentBehavior, m_bGreenLight, iDirection, behavior_rviz);

			pub_BehaviorStateRviz.publish(behavior_rviz);

			if(m_CurrentBehavior.state != m_PrevBehavior.state)
			{
				//std::cout << m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->ToString(m_CurrentBehavior.state) << ", Speed : " << m_CurrentBehavior.maxVelocity << std::endl;
				m_PrevBehavior = m_CurrentBehavior;
			}


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
			RosHelpers::ConvertFromPlannerObstaclesToAutoware(m_CurrentPos, m_TrackedClusters, detectedPolygons);
			pub_DetectedPolygonsRviz.publish(detectedPolygons);

			timespec log_t;
			UtilityHNS::UtilityH::GetTickCount(log_t);
			std::ostringstream dataLine;
			dataLine << UtilityHNS::UtilityH::GetLongTime(log_t) << "," << m_CurrentBehavior.state << ","<< RosHelpers::GetBehaviorNameFromCode(m_CurrentBehavior.state) << "," <<
					m_nTrackObjects << "," << m_nOriginalPoints << "," << m_nContourPoints << "," << m_TrackingTime << "," <<
					m_LocalPlanner.m_CostCalculationTime << "," << m_LocalPlanner.m_BehaviorGenTime << "," << m_LocalPlanner.m_RollOutsGenerationTime << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bFullyBlock << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->currentStopSignID << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->currentTrafficLightID << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->minStoppingDistance << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->currentVelocity << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->distanceToNext << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->velocityOfNext << "," <<
					m_LocalPlanner.state.pos.x << "," << m_LocalPlanner.state.pos.y << "," << m_LocalPlanner.state.pos.z << "," << m_LocalPlanner.state.pos.a << ",";


			m_LogData.push_back(dataLine.str());

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

			std::ostringstream str_out;
			str_out << UtilityHNS::UtilityH::GetHomeDirectory();
			str_out << UtilityHNS::DataRW::LoggingMainfolderName;
			str_out << "LocalPath_";
			PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_LocalPlanner.m_Path);

			visualization_msgs::MarkerArray all_rollOuts;
			RosHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(m_LocalPlanner.m_Path, m_LocalPlanner.m_RollOuts.at(0), all_rollOuts);
			pub_LocalTrajectoriesRviz.publish(all_rollOuts);
		}

		//Traffic Light Simulation Part
		if(m_bGreenLight && UtilityHNS::UtilityH::GetTimeDiffNow(m_TrafficLightTimer) > 8.5)
		{
			m_bGreenLight = false;
			UtilityHNS::UtilityH::GetTickCount(m_TrafficLightTimer);
		}
		else if(!m_bGreenLight && UtilityHNS::UtilityH::GetTimeDiffNow(m_TrafficLightTimer) > 8.5)
		{
			m_bGreenLight = true;
			UtilityHNS::UtilityH::GetTickCount(m_TrafficLightTimer);
		}

		loop_rate.sleep();

		//double onePassTime = UtilityHNS::UtilityH::GetTimeDiffNow(iterationTime);
//		if(onePassTime > 0.1)
//			std::cout << "Slow Iteration Time = " << onePassTime << " , for Obstacles : (" << m_TrackedClusters.size() << ", " << m_OriginalClusters.size() << ")" <<  std::endl;
	}
}

}
