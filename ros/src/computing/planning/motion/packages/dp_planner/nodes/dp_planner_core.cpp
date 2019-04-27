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
#include "dp_planner_core.h"

#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "op_utility/UtilityH.h"
#include "op_planner/MatrixOperations.h"

namespace PlannerXNS
{

PlannerX::PlannerX()
{

	clock_gettime(0, &m_Timer);
	m_counter = 0;
	m_frequency = 0;

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
	m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE = 2.0;
	m_ObstacleTracking.m_dt = 0.12;
	m_ObstacleTracking.m_bUseCenterOnly = true;

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
	ROSHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();


	std::string topic_prefix;

	pub_LocalPath = nh.advertise<autoware_msgs::Lane>(topic_prefix + "/final_waypoints", 100,true);
	pub_LocalBasePath = nh.advertise<autoware_msgs::Lane>(topic_prefix + "/base_waypoints", 100,true);
	pub_ClosestIndex = nh.advertise<std_msgs::Int32>(topic_prefix + "/closest_waypoint", 100,true);

	pub_BehaviorState = nh.advertise<geometry_msgs::TwistStamped>("current_behavior", 1);
	pub_GlobalPlanNodes = nh.advertise<geometry_msgs::PoseArray>("global_plan_nodes", 1);
	pub_StartPoint = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("GlobalStartpose", 1);
	pub_GoalPoint = nh.advertise<geometry_msgs::PoseStamped>("GlobalGoalPose", 1);
	pub_AStarStartPoint = nh.advertise<geometry_msgs::PoseStamped>("global_plan_start", 1);
	pub_AStarGoalPoint = nh.advertise<geometry_msgs::PoseStamped>("global_plan_goal", 1);

	pub_DetectedPolygonsRviz = nh.advertise<visualization_msgs::MarkerArray>("detected_polygons", 1, true);
	pub_TrackedObstaclesRviz = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("dp_planner_tracked_boxes", 1);
	pub_LocalTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories", 1);
	
	pub_TestLineRviz	= nh.advertise<visualization_msgs::MarkerArray>("testing_line", 1);
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::Marker>("behavior_state", 1);
	pub_SafetyBorderRviz  = nh.advertise<visualization_msgs::Marker>("safety_border", 1);
	pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("simu_points_cluster",1);
	pub_SimuBoxPose	  = nh.advertise<geometry_msgs::PoseArray>("sim_box_pose_ego", 100);


	sub_initialpose 	= nh.subscribe("/initialpose", 				1,		&PlannerX::callbackGetInitPose, 		this);
	sub_current_pose 	= nh.subscribe("/current_pose", 			1,		&PlannerX::callbackGetCurrentPose, 		this);
	sub_cluster_cloud 	= nh.subscribe("/cloud_clusters",			1,		&PlannerX::callbackGetCloudClusters, 	this);
	sub_bounding_boxs  	= nh.subscribe("/bounding_boxes",			1,		&PlannerX::callbackGetBoundingBoxes, 	this);
	sub_WayPlannerPaths = nh.subscribe("/realtime_cost_map",		1,		&PlannerX::callbackGetCostMap, 	this);

	/**
	 * @todo This works only in simulation (Autoware or ff_Waypoint_follower), twist_cmd should be changed, consult team
	 */
	int bVelSource = 1;
	nh.getParam("/dp_planner/enableOdometryStatus", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom 			= nh.subscribe("/odom", 					100,	&PlannerX::callbackGetRobotOdom, 	this);
	else if(bVelSource == 1)
		sub_current_velocity 	= nh.subscribe("/current_velocity",		100,	&PlannerX::callbackGetVehicleStatus, 	this);
	else if(bVelSource == 2)
		sub_can_info 			= nh.subscribe("/can_info",		100,	&PlannerX::callbackGetCANInfo, 	this);



	sub_EmergencyStop 	= nh.subscribe("/emergency_stop_signal", 	100,	&PlannerX::callbackGetEmergencyStop, 	this);
	sub_TrafficLight 	= nh.subscribe("/traffic_signal_info", 		10,		&PlannerX::callbackGetTrafficLight, 	this);

	if(m_bEnableOutsideControl)
		sub_OutsideControl 	= nh.subscribe("/usb_controller_r_signal", 	10,		&PlannerX::callbackGetOutsideControl, 	this);
	else
		m_bOutsideControl = 1;

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
}

PlannerX::~PlannerX()
{
#ifdef OPENPLANNER_ENABLE_LOGS
	UtilityHNS::DataRW::WriteLogData(UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::StatesLogFolderName, "MainLog",
			"time,dt,Behavior State,behavior,num_Tracked_Objects,num_Cluster_Points,num_Contour_Points,t_Tracking,t_Calc_Cost, t_Behavior_Gen, t_Roll_Out_Gen, num_RollOuts, Full_Block, idx_Central_traj, iTrajectory, Stop Sign, Traffic Light, Min_Stop_Distance, follow_distance, follow_velocity, Velocity, Steering, X, Y, Z, heading,"
			, m_LogData);
#endif


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

	nh.getParam("/dp_planner/enableSwerving", params.enableSwerving);
	if(params.enableSwerving)
		params.enableFollowing = true;
	else
		nh.getParam("/dp_planner/enableFollowing", params.enableFollowing);

	nh.getParam("/dp_planner/enableHeadingSmoothing", params.enableHeadingSmoothing);
	nh.getParam("/dp_planner/enableTrafficLightBehavior", params.enableTrafficLightBehavior);
	nh.getParam("/dp_planner/enableStopSignBehavior", params.enableStopSignBehavior);

	nh.getParam("/dp_planner/maxVelocity", params.maxSpeed);
	nh.getParam("/dp_planner/minVelocity", params.minSpeed);
	nh.getParam("/dp_planner/maxLocalPlanDistance", params.microPlanDistance);
	nh.getParam("/dp_planner/samplingTipMargin", params.carTipMargin);
	nh.getParam("/dp_planner/samplingOutMargin", params.rollInMargin);
	nh.getParam("/dp_planner/samplingSpeedFactor", params.rollInSpeedFactor);

	nh.getParam("/dp_planner/pathDensity", params.pathDensity);
	nh.getParam("/dp_planner/rollOutDensity", params.rollOutDensity);
	if(params.enableSwerving)
		nh.getParam("/dp_planner/rollOutsNumber", params.rollOutNumber);
	else
		params.rollOutNumber = 0;

	nh.getParam("/dp_planner/horizonDistance", params.horizonDistance);
	nh.getParam("/dp_planner/minFollowingDistance", params.minFollowingDistance);
	nh.getParam("/dp_planner/minDistanceToAvoid", params.minDistanceToAvoid);
	nh.getParam("/dp_planner/maxDistanceToAvoid", params.maxDistanceToAvoid);
	nh.getParam("/dp_planner/speedProfileFactor", params.speedProfileFactor);

	nh.getParam("/dp_planner/horizontalSafetyDistance", params.horizontalSafetyDistancel);
	nh.getParam("/dp_planner/verticalSafetyDistance", params.verticalSafetyDistance);

	nh.getParam("/dp_planner/enableLaneChange", params.enableLaneChange);
	nh.getParam("/dp_planner/enabTrajectoryVelocities", params.enabTrajectoryVelocities);

	nh.getParam("/dp_planner/enableObjectTracking", m_bEnableTracking);
	nh.getParam("/dp_planner/enableOutsideControl", m_bEnableOutsideControl);


	PlannerHNS::ControllerParams controlParams;
	controlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01);
	controlParams.Velocity_Gain = PlannerHNS::PID_CONST(0.1, 0.005, 0.1);
	nh.getParam("/dp_planner/steeringDelay", controlParams.SteeringDelay);
	nh.getParam("/dp_planner/minPursuiteDistance", controlParams.minPursuiteDistance );

	PlannerHNS::CAR_BASIC_INFO vehicleInfo;

	nh.getParam("/dp_planner/width", vehicleInfo.width);
	nh.getParam("/dp_planner/length", vehicleInfo.length);
	nh.getParam("/dp_planner/wheelBaseLength", vehicleInfo.wheel_base);
	nh.getParam("/dp_planner/turningRadius", vehicleInfo.turning_radius);
	nh.getParam("/dp_planner/maxSteerAngle", vehicleInfo.max_steer_angle);
	vehicleInfo.max_speed_forward = params.maxSpeed;
	vehicleInfo.min_speed_forward = params.minSpeed;

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

void PlannerX::callbackGetCostMap(const nav_msgs::OccupancyGrid& msg)
{
}

void PlannerX::callbackGetRvizPoint(const geometry_msgs::PointStampedConstPtr& msg)
{
	//Add Simulated Obstacle polygon
	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);
	double width = SIMU_OBSTACLE_WIDTH;//((double)(rand()%10)/10.0) * 1.5 + 0.25;
	double length = SIMU_OBSTACLE_LENGTH;//((double)(rand()%10)/10.0) * 0.5 + 0.25;
	double height = SIMU_OBSTACLE_HEIGHT;

	geometry_msgs::PointStamped point;
	point.point.x = msg->point.x+m_OriginPos.position.x;
	point.point.y = msg->point.y+m_OriginPos.position.y;
	point.point.z = msg->point.z+m_OriginPos.position.z;

	autoware_msgs::CloudClusterArray clusters_array;
	clusters_array.clusters.push_back(GenerateSimulatedObstacleCluster(width, length, height, 50, point));
	m_OriginalClusters.clear();
	int nNum1, nNum2;
	ROSHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(m_CurrentPos, m_LocalPlanner.m_CarInfo, clusters_array, m_OriginalClusters, nNum1, nNum2);
	m_TrackedClusters = m_OriginalClusters;

	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	pcl::fromROSMsg(clusters_array.clusters.at(0).cloud, point_cloud);
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(point_cloud, cloud_msg);
	cloud_msg.header.frame_id = "map";
	pub_cluster_cloud.publish(cloud_msg);

	if(m_TrackedClusters.size()>0)
	{
		jsk_recognition_msgs::BoundingBoxArray boxes_array;
		boxes_array.header.frame_id = "map";
		boxes_array.header.stamp  = ros::Time();
		jsk_recognition_msgs::BoundingBox box;
		box.header.frame_id = "map";
		box.header.stamp = ros::Time();
		box.pose.position.x = m_TrackedClusters.at(0).center.pos.x;
		box.pose.position.y = m_TrackedClusters.at(0).center.pos.y;
		box.pose.position.z = m_TrackedClusters.at(0).center.pos.z;

		box.value = 0.9;

		//box.pose.orientation = detectedPolygons.markers.at(0)
		box.dimensions.x = SIMU_OBSTACLE_WIDTH;
		box.dimensions.y = SIMU_OBSTACLE_LENGTH;
		box.dimensions.z = SIMU_OBSTACLE_HEIGHT;
		boxes_array.boxes.push_back(box);

		pub_TrackedObstaclesRviz.publish(boxes_array);
	}
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

autoware_msgs::CloudCluster PlannerX::GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::PointStamped& centerPose)
{
	autoware_msgs::CloudCluster cluster;
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

void PlannerX::callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg)
{
	timespec timerTemp;
	UtilityHNS::UtilityH::GetTickCount(timerTemp);

	m_OriginalClusters.clear();
	ROSHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(m_CurrentPos, m_LocalPlanner.m_CarInfo, *msg, m_OriginalClusters, m_nOriginalPoints, m_nContourPoints);
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
//	ROSHelpers::ConvertFromAutowareBoundingBoxObstaclesToPlannerH(*msg, m_DetectedBoxes);
//	bNewBoxes = true;
}

void PlannerX::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.linear.x;

	if(msg->twist.linear.x != 0)
		m_VehicleState.steer = atan(m_LocalPlanner.m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);

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

void PlannerX::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleState.speed = msg->speed/3.6;
	m_VehicleState.steer = msg->angle * m_LocalPlanner.m_CarInfo.max_steer_angle / m_LocalPlanner.m_CarInfo.max_steer_value;
	std::cout << "Can Info, Speed: "<< m_VehicleState.speed << ", Steering: " << m_VehicleState.steer  << std::endl;
}

void PlannerX::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleState.speed = msg->twist.twist.linear.x;
	m_VehicleState.steer += atan(m_LocalPlanner.m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);

	UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
//	if(msg->vector.z == 0x00)
//		m_VehicleState.shift = AW_SHIFT_POS_BB;
//	else if(msg->vector.z == 0x10)
//		m_VehicleState.shift = AW_SHIFT_POS_DD;
//	else if(msg->vector.z == 0x20)
//		m_VehicleState.shift = AW_SHIFT_POS_NN;
//	else if(msg->vector.z == 0x40)
//		m_VehicleState.shift = AW_SHIFT_POS_RR;

	//std::cout << "PlannerX: Read Odometry ("<< m_VehicleState.speed << ", " << m_VehicleState.steer<<")" << std::endl;
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
	m_bOutsideControl  = msg.data;
}

void PlannerX::callbackGetAStarPath(const autoware_msgs::LaneArrayConstPtr& msg)
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

void PlannerX::callbackGetWayPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		m_WayPlannerPaths.clear();
		bool bOldGlobalPath = m_LocalPlanner.m_TotalOriginalPath.size() == msg->lanes.size();
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
				wp.LeftPointId = msg->lanes.at(i).waypoints.at(j).twist.twist.angular.y;
				wp.RightPointId = msg->lanes.at(i).waypoints.at(j).twist.twist.angular.z;

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

			PlannerHNS::PlanningHelpers::CalcAngleAndCost(path);

			m_WayPlannerPaths.push_back(path);

			if(bOldGlobalPath)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(path, m_LocalPlanner.m_TotalOriginalPath.at(i));
			}
		}


		if(!bOldGlobalPath)
		{
			bWayPlannerPath = true;
			m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
			m_LocalPlanner.m_TotalOriginalPath = m_WayPlannerPaths;

			cout << "Global Lanes Size = " << msg->lanes.size() <<", Conv Size= " << m_WayPlannerPaths.size() << ", First Lane Size: " << m_WayPlannerPaths.at(0).size() << endl;
		}
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
				 ROSHelpers::UpdateRoadMap(m_AwMap,m_Map);
				 std::cout << "Converting Vector Map Time : " <<UtilityHNS::UtilityH::GetTimeDiffNow(timerTemp) << std::endl;
				 //sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	10,		&PlannerX::callbackGetWayPlannerPath, 	this);
			 }
		}

		int iDirection = 0;
		if(bInitPos && m_LocalPlanner.m_TotalOriginalPath.size()>0)
		{
			m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = m_bOutsideControl;
			m_LocalPlanner.state = m_CurrentPos;

			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			std::vector<PlannerHNS::TrafficLight> trafficLight;
			m_CurrentBehavior = m_LocalPlanner.DoOneStep(dt, m_VehicleState, m_TrackedClusters, 1, m_Map, m_bEmergencyStop, trafficLight, true);

			visualization_msgs::Marker behavior_rviz;

			if(m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory > m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
				iDirection = 1;
			else if(m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory < m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
				iDirection = -1;

			ROSHelpers::VisualizeBehaviorState(m_CurrentPos, m_CurrentBehavior, m_bGreenLight, iDirection, behavior_rviz);

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
			ROSHelpers::ConvertFromPlannerObstaclesToAutoware(m_CurrentPos, m_TrackedClusters, detectedPolygons);
			pub_DetectedPolygonsRviz.publish(detectedPolygons);

			visualization_msgs::Marker safety_box;
			ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(m_LocalPlanner.m_TrajectoryCostsCalculatotor.m_SafetyBorder.points, safety_box);
			pub_SafetyBorderRviz.publish(safety_box);

			geometry_msgs::PoseArray sim_data;
			geometry_msgs::Pose p_id, p_pose, p_box;


			sim_data.header.frame_id = "map";
			sim_data.header.stamp = ros::Time();

			p_id.position.x = 0;

			p_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(m_LocalPlanner.state.pos.a));
			p_pose.position.x = m_LocalPlanner.state.pos.x;
			p_pose.position.y = m_LocalPlanner.state.pos.y;
			p_pose.position.z = m_LocalPlanner.state.pos.z;



			p_box.position.x = m_LocalPlanner.m_CarInfo.width;
			p_box.position.y = m_LocalPlanner.m_CarInfo.length;
			p_box.position.z = 2.2;

			sim_data.poses.push_back(p_id);
			sim_data.poses.push_back(p_pose);
			sim_data.poses.push_back(p_box);

			pub_SimuBoxPose.publish(sim_data);

			timespec log_t;
			UtilityHNS::UtilityH::GetTickCount(log_t);
			std::ostringstream dataLine;
			std::ostringstream dataLineToOut;
			dataLine << UtilityHNS::UtilityH::GetLongTime(log_t) <<"," << dt << "," << m_CurrentBehavior.state << ","<< ROSHelpers::GetBehaviorNameFromCode(m_CurrentBehavior.state) << "," <<
					m_nTrackObjects << "," << m_nOriginalPoints << "," << m_nContourPoints << "," << m_TrackingTime << "," <<
					m_LocalPlanner.m_CostCalculationTime << "," << m_LocalPlanner.m_BehaviorGenTime << "," << m_LocalPlanner.m_RollOutsGenerationTime << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->m_pParams->rollOutNumber << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bFullyBlock << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory << "," <<
					m_LocalPlanner.m_iSafeTrajectory << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->currentStopSignID << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->currentTrafficLightID << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->minStoppingDistance << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->distanceToNext << "," <<
					m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->velocityOfNext << "," <<
					m_VehicleState.speed << "," <<
					m_VehicleState.steer << "," <<
					m_LocalPlanner.state.pos.x << "," << m_LocalPlanner.state.pos.y << "," << m_LocalPlanner.state.pos.z << "," << UtilityHNS::UtilityH::SplitPositiveAngle(m_LocalPlanner.state.pos.a)+M_PI << ",";
			m_LogData.push_back(dataLine.str());

//			dataLineToOut << ROSHelpers::GetBehaviorNameFromCode(m_CurrentBehavior.state) << ","
//					<< m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->bFullyBlock << ","
//					<< m_LocalPlanner.m_iSafeTrajectory << ","
//					<< m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->minStoppingDistance << ","
//					<< m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->distanceToNext << ","
//					<< m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->velocityOfNext << ","
//					<< m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->currentStopSignID << ","
//					<< m_LocalPlanner.m_pCurrentBehaviorState->GetCalcParams()->currentTrafficLightID << ",";
//
//			cout << dataLineToOut.str() << endl;


		}
		else
		{
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
			sub_WayPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&PlannerX::callbackGetWayPlannerPath, 	this);
		}


		autoware_msgs::Lane current_trajectory;
		std_msgs::Int32 closest_waypoint;
		PlannerHNS::RelativeInfo info;
		PlannerHNS::PlanningHelpers::GetRelativeInfo(m_LocalPlanner.m_Path, m_LocalPlanner.state, info);
		ROSHelpers::ConvertFromPlannerHToAutowarePathFormat(m_LocalPlanner.m_Path, info.iBack, current_trajectory);
		closest_waypoint.data = 1;
		pub_ClosestIndex.publish(closest_waypoint);
		pub_LocalBasePath.publish(current_trajectory);
		pub_LocalPath.publish(current_trajectory);
		visualization_msgs::MarkerArray all_rollOuts;

	
		ROSHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(m_LocalPlanner.m_Path, m_LocalPlanner.m_RollOuts, m_LocalPlanner, all_rollOuts);
		pub_LocalTrajectoriesRviz.publish(all_rollOuts);

		if(m_CurrentBehavior.bNewPlan)
		{
			std::ostringstream str_out;
			str_out << UtilityHNS::UtilityH::GetHomeDirectory();
			str_out << UtilityHNS::DataRW::LoggingMainfolderName;
			str_out << UtilityHNS::DataRW::PathLogFolderName;
			str_out << "LocalPath_";
			PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_LocalPlanner.m_Path);
		}


		//Traffic Light Simulation Part
		if(m_bGreenLight && UtilityHNS::UtilityH::GetTimeDiffNow(m_TrafficLightTimer) > 5)
		{
			m_bGreenLight = false;
			UtilityHNS::UtilityH::GetTickCount(m_TrafficLightTimer);
		}
		else if(!m_bGreenLight && UtilityHNS::UtilityH::GetTimeDiffNow(m_TrafficLightTimer) > 10.0)
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
