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

#include "op_car_simulator_core.h"

#include "op_utility/UtilityH.h"
#include "math.h"
#include "op_planner/MatrixOperations.h"
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/transforms.h>
#include "op_ros_helpers/PolygonGenerator.h"
#include "op_ros_helpers/op_RosHelpers.h"


namespace CarSimulatorNS
{

#define REPLANNING_DISTANCE 7.5

OpenPlannerCarSimulator::OpenPlannerCarSimulator()
{
	m_bMap = false;
	bPredictedObjects = false;
	m_bStepByStep = false;
	m_bGoNextStep = false;
	bUseWheelController = false;

	ReadParamFromLaunchFile(m_CarInfo, m_ControlParams);

	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);

	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	m_PredControl.Init(m_ControlParams, m_CarInfo, false, false);

	m_LocalPlanner = new PlannerHNS::SimuDecisionMaker();
	m_LocalPlanner->Init(m_ControlParams, m_PlanningParams, m_CarInfo);
	m_LocalPlanner->m_SimulationSteeringDelayFactor = m_ControlParams.SimulationSteeringDelay;

	//For rviz visualization
	std::ostringstream str_s1, str_s2, str_s3, str_s4, str_s5, str_s6, str_s7, str_s8, str_s9;
	str_s1 << "curr_simu_pose_";
	str_s1 << m_SimParams.id;

	str_s2 << "sim_beh_txt_" << m_SimParams.id;

	str_s5 << "sim_box_pose_";
	str_s5 << m_SimParams.id;

	str_s3 << "sim_velocity_";
	str_s3 << m_SimParams.id;

	str_s4 << "safety_border_";
	str_s4 << m_SimParams.id;

	str_s6 << "simu_local_trajectory_";
	str_s6 << m_SimParams.id;

	str_s7 << "simu_internal_info";
	str_s7 << m_SimParams.id;

	str_s8 << "simu_car_path_beh_";
	str_s8 << m_SimParams.id;

	pub_CurrPoseRviz				= nh.advertise<visualization_msgs::Marker>(str_s1.str() , 100);
	pub_SimuBoxPose					= nh.advertise<geometry_msgs::PoseArray>(str_s5.str(), 100);
	pub_SafetyBorderRviz  			= nh.advertise<visualization_msgs::Marker>(str_s4.str(), 1);
	pub_LocalTrajectoriesRviz   	= nh.advertise<visualization_msgs::MarkerArray>(str_s6.str(), 1);
	pub_BehaviorStateRviz			= nh.advertise<visualization_msgs::Marker>(str_s2.str(), 1);
	pub_PointerBehaviorStateRviz	= nh.advertise<visualization_msgs::Marker>(str_s2.str(), 1);
	pub_InternalInfoRviz			= nh.advertise<visualization_msgs::MarkerArray>(str_s7.str(), 1);

	pub_CurrentLocalPath 			= nh.advertise<autoware_msgs::lane>(str_s8.str(), 1);

	sub_joystick = nh.subscribe("/joy", 		1, &OpenPlannerCarSimulator::callbackGetJoyStickInfo, 		this);
	sub_StepSignal = nh.subscribe("/simu_step_signal", 		1, &OpenPlannerCarSimulator::callbackGetStepForwardSignals, 		this);

	// define subscribers.
	if(m_SimParams.bRvizPositions)
	{
		bInitPos = false;
		sub_initialpose 		= nh.subscribe("/initialpose", 		1, &OpenPlannerCarSimulator::callbackGetInitPose, 		this);
		bGoalPos = false;
		sub_goalpose 		= nh.subscribe("/move_base_simple/goal", 1, &OpenPlannerCarSimulator::callbackGetGoalPose, 		this);
	}
	else
	{
		PlannerHNS::WayPoint start_p, goal_p;
		int nRecords = LoadSimulationData(start_p, goal_p);
		if(nRecords > 0)
		{
			m_SimParams.startPose.pos = start_p.pos;
			m_SimParams.goalPose.pos = goal_p.pos;
		}
		else
		{
			ROS_ERROR("Can't Read Start and Goal information from log file for OpenPlanner simulated car !");
			return;
		}

	}

	if(m_PlanningParams.enableFollowing)
		sub_predicted_objects 			= nh.subscribe("/tracked_objects", 	1, &OpenPlannerCarSimulator::callbackGetPredictedObjects, 		this);

	if(m_PlanningParams.enableTrafficLightBehavior)
		sub_TrafficLightSignals		= nh.subscribe("/roi_signal", 		10,	&OpenPlannerCarSimulator::callbackGetTrafficLightSignals, 	this);

	std::ostringstream vel_frame_id;
	vel_frame_id << "velodyne_" << m_SimParams.id;
	m_VelodyneFrameID = vel_frame_id.str();
	std::ostringstream base_frame_id;
	base_frame_id << "base_link_" << m_SimParams.id;
	m_BaseLinkFrameID = base_frame_id.str();

	if(m_bSimulatedVelodyne)
	{
		std::ostringstream velodyne_special_points_raw;
		velodyne_special_points_raw << "points_raw_" << m_SimParams.id;

		pub_SimulatedVelodyne   = nh.advertise<const sensor_msgs::PointCloud2>(velodyne_special_points_raw.str(), 1);
		sub_cloud_clusters 		= nh.subscribe("/cloud_clusters", 1, &OpenPlannerCarSimulator::callbackGetCloudClusters, this);
	}

	//Mapping Section
	sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &OpenPlannerCarSimulator::callbackGetVMLanes,  this);
	sub_points = nh.subscribe("/vector_map_info/point", 1, &OpenPlannerCarSimulator::callbackGetVMPoints,  this);
	sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &OpenPlannerCarSimulator::callbackGetVMdtLanes,  this);
	sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &OpenPlannerCarSimulator::callbackGetVMIntersections,  this);
	sup_area = nh.subscribe("/vector_map_info/area", 1, &OpenPlannerCarSimulator::callbackGetVMAreas,  this);
	sub_lines = nh.subscribe("/vector_map_info/line", 1, &OpenPlannerCarSimulator::callbackGetVMLines,  this);
	sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &OpenPlannerCarSimulator::callbackGetVMStopLines,  this);
	sub_signals = nh.subscribe("/vector_map_info/signal", 1, &OpenPlannerCarSimulator::callbackGetVMSignal,  this);
	sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &OpenPlannerCarSimulator::callbackGetVMVectors,  this);
	sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &OpenPlannerCarSimulator::callbackGetVMCurbs,  this);
	sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &OpenPlannerCarSimulator::callbackGetVMRoadEdges,  this);
	sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &OpenPlannerCarSimulator::callbackGetVMWayAreas,  this);
	sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &OpenPlannerCarSimulator::callbackGetVMCrossWalks,  this);
	sub_nodes = nh.subscribe("/vector_map_info/node", 1, &OpenPlannerCarSimulator::callbackGetVMNodes,  this);

	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
	std::cout << "OpenPlannerCarSimulator initialized successfully " << std::endl;
}

void OpenPlannerCarSimulator::ReadParamFromLaunchFile(PlannerHNS::CAR_BASIC_INFO& m_CarInfo, PlannerHNS::ControllerParams& m_ControlParams)
{
	ros::NodeHandle _nh("~");


	_nh.getParam("id" 		, m_SimParams.strID);
	_nh.getParam("id" 		, m_SimParams.id);
	_nh.getParam("enableRvizPoseSelect" 	, m_SimParams.bRvizPositions);
	_nh.getParam("enableLooper" 			, m_SimParams.bLooper);

	_nh.getParam("meshPath" 				, m_SimParams.meshPath);
	_nh.getParam("baseColorR" 			, m_SimParams.modelColor.r);
	_nh.getParam("baseColorG" 			, m_SimParams.modelColor.g);
	_nh.getParam("baseColorB" 			, m_SimParams.modelColor.b);
	m_SimParams.modelColor.a = 0.9;
	_nh.getParam("logFolder" 			, m_SimParams.logPath);


	_nh.getParam("maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("maxVelocity", m_CarInfo.max_speed_forward );
	_nh.getParam("minVelocity", m_CarInfo.min_speed_forward );
	_nh.getParam("maxLocalPlanDistance", m_PlanningParams.microPlanDistance);
	_nh.getParam("samplingTipMargin", m_PlanningParams.carTipMargin);
	_nh.getParam("samplingOutMargin", m_PlanningParams.rollInMargin);
	_nh.getParam("samplingSpeedFactor", m_PlanningParams.rollInSpeedFactor);
	_nh.getParam("enableHeadingSmoothing", m_PlanningParams.enableHeadingSmoothing);

	_nh.getParam("pathDensity", m_PlanningParams.pathDensity);
	_nh.getParam("rollOutDensity", m_PlanningParams.rollOutDensity);
	_nh.getParam("enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("enableFollowing", m_PlanningParams.enableFollowing);

	if(m_PlanningParams.enableSwerving)
		_nh.getParam("rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;

	_nh.getParam("horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);
	_nh.getParam("enableLaneChange", m_PlanningParams.enableLaneChange);

	_nh.getParam("width", 			m_CarInfo.width );
	_nh.getParam("length", 		m_CarInfo.length );
	_nh.getParam("wheelBaseLength", m_CarInfo.wheel_base );
	_nh.getParam("turningRadius", m_CarInfo.turning_radius );
	_nh.getParam("maxSteerAngle", m_CarInfo.max_steer_angle );

	_nh.getParam("steeringDelay", m_ControlParams.SteeringDelay );
	_nh.getParam("minPursuiteDistance", m_ControlParams.minPursuiteDistance );
	_nh.getParam("maxAcceleration", m_CarInfo.max_acceleration );
	_nh.getParam("maxDeceleration", m_CarInfo.max_deceleration );
	_nh.getParam("enableStepByStepSignal", m_bStepByStep );
	_nh.getParam("enableSimulatedVelodyne", m_bSimulatedVelodyne );
	_nh.getParam("enableUsingJoyStick", bUseWheelController );

	//_nh.getParam("enableCurbObstacles", m_bEnableCurbObstacles);
	int iSource = 0;
	_nh.getParam("mapSource" 			, iSource);
	if(iSource == 0)
		m_SimParams.mapSource = MAP_AUTOWARE;
	else if(iSource == 1)
		m_SimParams.mapSource = MAP_FOLDER;
	else if(iSource == 2)
		m_SimParams.mapSource = MAP_KML_FILE;

	_nh.getParam("mapFileName" 		, m_SimParams.KmlMapPath);

	//m_SimParams.KmlMapPath = "/media/hatem/8ac0c5d5-8793-4b98-8728-55f8d67ec0f4/data/ToyotaCity2/map/vector_map/";
	m_PlanningParams.additionalBrakingDistance = 5;
	m_PlanningParams.stopSignStopTime = 10;


	m_ControlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01); // for 3 m/s
	m_ControlParams.Velocity_Gain = PlannerHNS::PID_CONST(0.1, 0.005, 0.1);
}

OpenPlannerCarSimulator::~OpenPlannerCarSimulator()
{
	if(bInitPos && bGoalPos)
		SaveSimulationData();
}

void OpenPlannerCarSimulator::callbackGetJoyStickInfo(const sensor_msgs::JoyConstPtr& msg)
{
//	if(msg->buttons[BUTTON_INDEX] == START_BUTTON_VALUE)
//		std::cout << "Start Button Value: " << 1 << std::endl;
//	else
//		std::cout << "Start Button Value: " << 0 << std::endl;
//				cout << "Steering Axis Value: " << -pAxis[STEERING_AXIS] << endl;
//				cout << "Acceleration Axis Value: " << 1 - pAxis[ACCELERATION_AXIS] << endl;
//				cout << "Braking Axis Value: " << 1 - pAxis[BRAKE_AXIS] << endl;
	m_JoyDesiredStatus.steer = msg->axes[STEERING_AXIS];
	double acceleration = (1 + msg->axes[ACCELERATION_AXIS])/2.0;
	double braking =  (1 + msg->axes[BRAKE_AXIS])/2.0;


	m_JoyDesiredStatus.shift = PlannerHNS::SHIFT_POS_DD;
	m_JoyDesiredStatus.speed = m_CarInfo.max_speed_forward * acceleration;
	m_JoyDesiredStatus.steer = m_JoyDesiredStatus.steer*m_CarInfo.max_steer_angle;

	std::cout << "Steering " << m_JoyDesiredStatus.steer << ", Speed: " <<  m_JoyDesiredStatus.speed <<", acceleration " << acceleration<< ", Braking " << braking <<", MaxSpeed: " << m_CarInfo.max_speed_forward << std::endl;
}

void OpenPlannerCarSimulator::callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg)
{
	sensor_msgs::PointCloud2 others_cloud;
	for(unsigned int i=0; i < msg->clusters.size(); i++)
	{
		pcl::concatenatePointCloud(others_cloud,msg->clusters.at(i).cloud, others_cloud);
	}

	if(others_cloud.data.size() <= 0)
		return;

	try
	{
		tf::StampedTransform transform;
		m_Listener.lookupTransform(m_VelodyneFrameID, "map",ros::Time(0), transform);
		sensor_msgs::PointCloud2 others_cloud_transformed;
		pcl_ros::transformPointCloud(m_VelodyneFrameID, transform, others_cloud, others_cloud_transformed);


		others_cloud_transformed.header.frame_id = m_VelodyneFrameID;
		others_cloud_transformed.header.stamp = ros::Time();
		pub_SimulatedVelodyne.publish(others_cloud_transformed);
		//std::cout << "Successful Transformation ! " << std::endl;
	}
	catch (tf::TransformException& ex)
	{
		//ROS_ERROR("Transformation Failed %s", ex.what());
	}
}

void OpenPlannerCarSimulator::callbackGetStepForwardSignals(const geometry_msgs::TwistStampedConstPtr& msg)
{
	if(msg->twist.linear.x == 1)
		m_bGoNextStep = true;
	else
		m_bGoNextStep = false;
}

void OpenPlannerCarSimulator::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	if(!bInitPos)
	{
		ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

		geometry_msgs::Pose p;
		p.position.x  = msg->pose.pose.position.x + m_OriginPos.position.x;
		p.position.y  = msg->pose.pose.position.y + m_OriginPos.position.y;
		p.position.z  = msg->pose.pose.position.z + m_OriginPos.position.z;
		p.orientation = msg->pose.pose.orientation;
		m_SimParams.startPose =  PlannerHNS::WayPoint(p.position.x, p.position.y, p.position.z , tf::getYaw(p.orientation));

//		if(m_SimParams.bRandomGoal)
//		{
//			SaveSimulationData();
//			InitializeSimuCar(m_SimParams.startPose);
//		}

		bInitPos = true;
	}
}

void OpenPlannerCarSimulator::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	if(!bGoalPos)
	{
		PlannerHNS::WayPoint wp;
		wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y,
				msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
		m_SimParams.goalPose = wp;
		ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

//		SaveSimulationData();
		InitializeSimuCar(m_SimParams.startPose);

		bGoalPos = true;
	}

}

void OpenPlannerCarSimulator::InitializeSimuCar(PlannerHNS::WayPoint start_pose)
{
	m_LocalPlanner->ReInitializePlanner(start_pose);
	std::cout << std::endl << "LocalPlannerInit: ID " << m_SimParams.strID << " , Pose = ( "  << start_pose.pos.ToString() << ")" << std::endl;
}

void OpenPlannerCarSimulator::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
	static tf::TransformListener listener;

	while (1)
	{
		try
		{
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

void OpenPlannerCarSimulator::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	m_PredictedObjects.clear();
	bPredictedObjects = true;

	PlannerHNS::DetectedObject obj;

	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		if(msg->objects.at(i).id != m_SimParams.id)
		{
			PlannerHNS::RosHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
			m_PredictedObjects.push_back(obj);
		}
//		else
//		{
//			std::cout << " Simulated Car avoid detecting itself, ID=" << msg->objects.at(i).id <<  std::endl;
//		}
	}
}

void OpenPlannerCarSimulator::displayFollowingInfo(const std::vector<PlannerHNS::GPSPoint>& safety_rect, PlannerHNS::WayPoint& curr_pose)
{
  static visualization_msgs::Marker m1;
  m1.header.frame_id = "map";
  m1.header.stamp = ros::Time();
  m1.ns = "curr_simu_pose";
  m1.type = visualization_msgs::Marker::MESH_RESOURCE;
  m1.mesh_resource = m_SimParams.meshPath;
  m1.mesh_use_embedded_materials = true;
  m1.action = visualization_msgs::Marker::ADD;

  PlannerHNS::WayPoint pose_center = PlannerHNS::PlanningHelpers::GetRealCenter(curr_pose, m_CarInfo.wheel_base);

  m1.pose.position.x = pose_center.pos.x;
  m1.pose.position.y = pose_center.pos.y;
  m1.pose.position.z = pose_center.pos.z;

  m1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(curr_pose.pos.a));
  m1.color = m_SimParams.modelColor;
  m1.scale.x = 1.0*m_CarInfo.length/4.2;
  m1.scale.y = 1.0*m_CarInfo.width/1.85;
  m1.scale.z = 1.0;

  pub_CurrPoseRviz.publish(m1);

  visualization_msgs::Marker lane_waypoint_marker;
  	lane_waypoint_marker.header.frame_id = "map";
  	lane_waypoint_marker.header.stamp = ros::Time();
  	lane_waypoint_marker.ns = "safety_simu_box";
  	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  	lane_waypoint_marker.scale.x = 0.2;
  	lane_waypoint_marker.scale.y = 0.2;
  	//lane_waypoint_marker.scale.z = 0.1;
  	lane_waypoint_marker.frame_locked = false;
  	lane_waypoint_marker.color.r = 0.0;
  	lane_waypoint_marker.color.g = 1.0;
  	lane_waypoint_marker.color.b = 0.0;
  	lane_waypoint_marker.color.a = 0.6;

  	for(unsigned int i=0; i < safety_rect.size(); i ++)
  	{
  		geometry_msgs::Point p1;
  		p1.x = safety_rect.at(i).x;
		p1.y = safety_rect.at(i).y;
		p1.z = safety_rect.at(i).z;
		lane_waypoint_marker.points.push_back(p1);
  	}
  	pub_SafetyBorderRviz.publish(lane_waypoint_marker);

}

void OpenPlannerCarSimulator::visualizePath(const std::vector<PlannerHNS::WayPoint>& path)
{
	visualization_msgs::MarkerArray markerArray;

	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	std::ostringstream str_sn;
	str_sn << "simu_car_path_";
	str_sn << m_SimParams.id;
	lane_waypoint_marker.ns = str_sn.str();
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;


	std_msgs::ColorRGBA roll_color, curr_color;
	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = 1;
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	lane_waypoint_marker.color.a = 0.5;
	lane_waypoint_marker.color = m_SimParams.modelColor;
	lane_waypoint_marker.frame_locked = false;

	int count = 0;
	for (unsigned int i = 0; i < path.size(); i++)
	{
		geometry_msgs::Point point;
		point.x = path.at(i).pos.x;
		point.y = path.at(i).pos.y;
		point.z = path.at(i).pos.z;

		lane_waypoint_marker.points.push_back(point);
		count++;
	}

	markerArray.markers.push_back(lane_waypoint_marker);

	pub_LocalTrajectoriesRviz.publish(markerArray);
}

void OpenPlannerCarSimulator::callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg)
{
//	std::cout << "Received Traffic Light Signals : " << msg.Signals.size() << std::endl;
//	m_CurrTrafficLight.clear();
//	bNewLightSignal = true;
//	for(unsigned int i = 0 ; i < msg.Signals.size() ; i++)
//	{
//		PlannerHNS::TrafficLight tl;
//		tl.id = msg.Signals.at(i).signalId;
//		if(msg.Signals.at(i).type == 1)
//			tl.lightState = PlannerHNS::GREEN_LIGHT;
//		else
//			tl.lightState = PlannerHNS::RED_LIGHT;
//
//		m_CurrTrafficLight.push_back(tl);
//	}

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
}

void OpenPlannerCarSimulator::visualizeBehaviors()
{
	visualization_msgs::Marker behaviorMarker;
	visualization_msgs::Marker pointerMarker;

	behaviorMarker.header.frame_id = "map";
	behaviorMarker.header.stamp = ros::Time();
	std::ostringstream str_sn;
	str_sn << "sim_behavior_" << m_SimParams.id;
	behaviorMarker.ns = str_sn.str();
	behaviorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	behaviorMarker.scale.z = 1.0;
	behaviorMarker.scale.x = 1.0;
	behaviorMarker.scale.y = 1.0;
	behaviorMarker.color.a = 1.0;
	behaviorMarker.frame_locked = false;

	pointerMarker.header.frame_id = "map";
	pointerMarker.header.stamp = ros::Time();
	std::ostringstream pointer_str_sn;
	pointer_str_sn << "sim_behavior_pointer" << m_SimParams.id;
	pointerMarker.ns = pointer_str_sn.str();
	pointerMarker.type = visualization_msgs::Marker::LINE_STRIP;
	pointerMarker.scale.z = 0.2;
	pointerMarker.scale.x = 0.2;
	pointerMarker.scale.y = 0.2;
	pointerMarker.color.a = 1.0;
	pointerMarker.frame_locked = false;
	pointerMarker.color.r = 1.0;
	pointerMarker.color.g = 0.0;
	pointerMarker.color.b = 1.0;

	if(m_LocalPlanner->m_pCurrentBehaviorState->GetCalcParams()->bTrafficIsRed)
	{
		behaviorMarker.color.r = 1.0;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 0.0;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.0;
	}
	else
	{
		behaviorMarker.color.r = 0.0;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 1.0;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.0;
	}

	geometry_msgs::Point point_center, point_link;

	point_center.x = m_LocalPlanner->state.pos.x;
	point_center.y = m_LocalPlanner->state.pos.y;
	point_center.z = m_LocalPlanner->state.pos.z+2.5;

	point_link.x = m_LocalPlanner->state.pos.x+5.5*cos(m_LocalPlanner->state.pos.a+M_PI_2);
	point_link.y = m_LocalPlanner->state.pos.y+5.5*sin(m_LocalPlanner->state.pos.a+M_PI_2);
	point_link.z = m_LocalPlanner->state.pos.z+2.5;

	behaviorMarker.pose.position = point_link;
	pointerMarker.points.push_back(point_link);
	pointerMarker.points.push_back(point_center);

	pointerMarker.id = 1;
	behaviorMarker.id = 1;

	std::string str = "Un ";
	switch(m_LocalPlanner->m_pCurrentBehaviorState->m_Behavior)
	{
	case PlannerHNS::INITIAL_STATE:
		str = "In ";
		break;
	case PlannerHNS::WAITING_STATE:
		str = "Wa ";
		break;
	case PlannerHNS::FORWARD_STATE:
		str = "Fo ";
		break;
	case PlannerHNS::STOPPING_STATE:
		str = "St ";
		break;
	case PlannerHNS::FINISH_STATE:
		str = "En ";
		break;
	case PlannerHNS::FOLLOW_STATE:
		str = "Fl ";
		break;
	case PlannerHNS::OBSTACLE_AVOIDANCE_STATE:
		str = "Sw ";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_STOP_STATE:
		str = "LS ";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE:
		str = "LW ";
		break;
	case PlannerHNS::STOP_SIGN_STOP_STATE:
		str = "SS ";
		break;
	case PlannerHNS::STOP_SIGN_WAIT_STATE:
		str = "SW ";
		break;
	default:
		str = "Un ";
		break;
	}

	std::ostringstream str_out, beh_out;

	if(m_CurrBehavior.indicator == PlannerHNS::INDICATOR_RIGHT)
		beh_out << "Right" ;
	else if(m_CurrBehavior.indicator == PlannerHNS::INDICATOR_LEFT)
		beh_out << "Left" ;
	else
		beh_out << "Forward" ;


	str_out << str;
	str_out << m_SimParams.id;
	double two_prec_speed = (m_LocalPlanner->m_CurrentVelocity*3.6*100)/100.0;
	std::ostringstream speed_out;
	speed_out.precision(3);
	speed_out << two_prec_speed;
	string speed_str = speed_out.str();
	if(speed_str.size() <= 1 )
		speed_str = "0.0";

	str_out << "(" << speed_str << ")"  << "(" << beh_out.str() << ")";
	behaviorMarker.text = str_out.str();

	visualization_msgs::MarkerArray markerArray;
	PlannerHNS::RosHelpers::GetIndicatorArrows(m_LocalPlanner->state, m_CarInfo.width, m_CarInfo.length, m_CurrBehavior.indicator, m_SimParams.id, markerArray);

	markerArray.markers.push_back(behaviorMarker);
	markerArray.markers.push_back(pointerMarker);

	pub_InternalInfoRviz.publish(markerArray);

//	pub_BehaviorStateRviz.publish(behaviorMarker);
//	pub_PointerBehaviorStateRviz.publish(pointerMarker);
}

void OpenPlannerCarSimulator::SaveSimulationData()
{
	std::vector<std::string> simulationDataPoints;
	std::ostringstream startStr, goalStr;
	startStr << m_SimParams.startPose.pos.x << "," << m_SimParams.startPose.pos.y << "," << m_SimParams.startPose.pos.z << "," << m_SimParams.startPose.pos.a << ","<< m_SimParams.startPose.cost << "," << m_CarInfo.max_speed_forward << ",";
	goalStr << m_SimParams.goalPose.pos.x << "," << m_SimParams.goalPose.pos.y << "," << m_SimParams.goalPose.pos.z << "," << m_SimParams.goalPose.pos.a << "," << 0 << "," << 0 << ",";

	simulationDataPoints.push_back(startStr.str());
	simulationDataPoints.push_back(goalStr.str());

	std::string header = "X,Y,Z,A,C,V,";

	ostringstream fileName;
	fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName;
	fileName << "SimuCar_";
	fileName << m_SimParams.id;
	fileName << ".csv";

	std::ofstream f(fileName.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < simulationDataPoints.size(); i++)
			f << simulationDataPoints.at(i) << "\r\n";
	}

	f.close();
}

int OpenPlannerCarSimulator::LoadSimulationData(PlannerHNS::WayPoint& start_p, PlannerHNS::WayPoint& goal_p)
{
	ostringstream fileName;
	fileName << "SimuCar_";
	fileName << m_SimParams.id;
	fileName << ".csv";

	string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName + fileName.str();
	UtilityHNS::SimulationFileReader sfr(simuDataFileName);
	UtilityHNS::SimulationFileReader::SimulationData data;

	int nData = sfr.ReadAllData(data);
	if(nData == 0)
		return 0;

	start_p = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
	goal_p = PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a);

	bInitPos = true;
	bGoalPos = true;

	start_p.v = data.startPoint.v;
	start_p.cost = data.startPoint.c;

	std::cout << "Loading from simulation File " << start_p.pos.ToString() << std::endl;
	return nData;
}

void OpenPlannerCarSimulator::PublishSpecialTF(const PlannerHNS::WayPoint& pose)
{
	static tf::TransformBroadcaster map_base_link_broadcaster;
	geometry_msgs::TransformStamped base_link_trans;
	base_link_trans.header.stamp = ros::Time::now();
	base_link_trans.header.frame_id = "map";
	base_link_trans.child_frame_id = m_BaseLinkFrameID;
	base_link_trans.transform.translation.x = pose.pos.x;
	base_link_trans.transform.translation.y = pose.pos.y;
	base_link_trans.transform.translation.z = pose.pos.z;
	base_link_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(pose.pos.a));

	// send the transform
	map_base_link_broadcaster.sendTransform(base_link_trans);

	static tf::TransformBroadcaster velodyne_base_link_broadcaster;
	geometry_msgs::TransformStamped vel_trans;
	vel_trans.header.stamp = ros::Time::now();
	vel_trans.header.frame_id = m_BaseLinkFrameID;
	vel_trans.child_frame_id = m_VelodyneFrameID;
	vel_trans.transform.translation.x = 0;
	vel_trans.transform.translation.y = 0;
	vel_trans.transform.translation.z = 0;
	vel_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

	// send the transform
	velodyne_base_link_broadcaster.sendTransform(vel_trans);
}

void OpenPlannerCarSimulator::MainLoop()
{

	ros::Rate loop_rate(50);

	PlannerHNS::VehicleState  currStatus;
	PlannerHNS::VehicleState  desiredStatus;

	while (ros::ok())
	{
		ros::spinOnce();

		bool bMakeNewPlan = false;

		if(m_SimParams.mapSource == MAP_KML_FILE && !m_bMap)
		{
			m_bMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_SimParams.KmlMapPath, m_Map);
			InitializeSimuCar(m_SimParams.startPose);
		}
		else if (m_SimParams.mapSource == MAP_FOLDER && !m_bMap)
		{
			m_bMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_SimParams.KmlMapPath, m_Map, true);
			InitializeSimuCar(m_SimParams.startPose);
		}
		else if (m_SimParams.mapSource == PlannerHNS::MAP_AUTOWARE && !m_bMap)
		{
			std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

			if(m_MapRaw.GetVersion()==2)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
						m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true);

				if(m_Map.roadSegments.size() > 0)
				{
					m_bMap = true;
					InitializeSimuCar(m_SimParams.startPose);
					std::cout << " ******* Map V2 Is Loaded successfully from the Behavior Selector !! " << std::endl;
				}
			}
			else if(m_MapRaw.GetVersion()==1)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true);

				if(m_Map.roadSegments.size() > 0)
				{
					m_bMap = true;
					InitializeSimuCar(m_SimParams.startPose);
					std::cout << " ******* Map V1 Is Loaded successfully from the Behavior Selector !! " << std::endl;
				}
			}
		}

		if(m_bMap && bInitPos && bGoalPos)
		{
			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			//Global Planning Step
			if(m_GlobalPaths.size() > 0 && m_GlobalPaths.at(0).size() > 3)
			{
				PlannerHNS::RelativeInfo info;
				bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GlobalPaths, m_LocalPlanner->state, 0.75, info);
				if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < (int)m_GlobalPaths.size() && info.iFront > 0 && info.iFront < (int)m_GlobalPaths.at(info.iGlobalPath).size())
				{
					PlannerHNS::WayPoint wp_end = m_GlobalPaths.at(info.iGlobalPath).at(m_GlobalPaths.at(info.iGlobalPath).size()-1);
					PlannerHNS::WayPoint wp_first = m_GlobalPaths.at(info.iGlobalPath).at(info.iFront);
					double remaining_distance =   hypot(wp_end.pos.y - wp_first.pos.y, wp_end.pos.x - wp_first.pos.x) + info.to_front_distance;

					if(remaining_distance <= REPLANNING_DISTANCE)
					{
						cout << "Remaining Distance : " << remaining_distance << endl;

						bMakeNewPlan = true;
						if(m_SimParams.bLooper)
						{
							delete m_LocalPlanner;
							m_LocalPlanner = 0;
							m_LocalPlanner = new  PlannerHNS::SimuDecisionMaker();
							m_LocalPlanner->Init(m_ControlParams, m_PlanningParams, m_CarInfo);
							m_LocalPlanner->m_SimulationSteeringDelayFactor = m_ControlParams.SimulationSteeringDelay;
							InitializeSimuCar(m_SimParams.startPose);
						}
					}
				}
			}
			else
				bMakeNewPlan = true;

			if(bMakeNewPlan)
			{
				std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths;
				vector<int> globalPathIds;
//				if(m_SimParams.bRandomGoal)
//					m_GlobalPlanner.PlanUsingDPRandom(m_LocalPlanner->state, PLANNING_DISTANCE, m_Map, generatedTotalPaths);
//				else
					m_GlobalPlanner.PlanUsingDP(m_LocalPlanner->state, m_SimParams.goalPose, 100000, false, globalPathIds, m_Map, generatedTotalPaths);

				for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
				{
					PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_PlanningParams.pathDensity);
					PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.4, 0.25);
					PlannerHNS::PlanningHelpers::GenerateRecommendedSpeed(generatedTotalPaths.at(i), m_CarInfo.max_speed_forward, m_PlanningParams.speedProfileFactor);
					generatedTotalPaths.at(i).at(generatedTotalPaths.at(i).size()-1).v = 0;
				}

				m_GlobalPaths = generatedTotalPaths;
				m_LocalPlanner->SetNewGlobalPath(m_GlobalPaths);
			}

			if(bNewLightSignal)
			{
				m_PrevTrafficLight = m_CurrTrafficLight;
				bNewLightSignal = false;
			}

			if(!m_bStepByStep)
			{
				/**
				 *  Local Planning
				 */
				m_CurrBehavior = m_LocalPlanner->DoOneStep(dt, currStatus, 1, m_CurrTrafficLight, m_PredictedObjects, false);

				/**
				 * Localization, Odometry Simulation and Update
				 */
				if(!bUseWheelController)
					currStatus = m_LocalPlanner->LocalizeStep(dt, desiredStatus);
				else
					currStatus = m_LocalPlanner->LocalizeStep(dt, m_JoyDesiredStatus);

				/**
				 * Control, Path Following
				 */
				if(!bUseWheelController)
					desiredStatus = m_PredControl.DoOneStep(dt, m_CurrBehavior, m_LocalPlanner->m_Path, m_LocalPlanner->state, currStatus, m_CurrBehavior.bNewPlan);

			}
			else
			{
				if(m_bGoNextStep)
				{
					m_bGoNextStep = false;
					dt = 0.02;
					/**
					 *  Local Planning
					 */
					m_CurrBehavior = m_LocalPlanner->DoOneStep(dt, currStatus, 1, m_CurrTrafficLight, m_PredictedObjects, false);

					/**
					 * Localization, Odometry Simulation and Update
					 */
					currStatus = m_LocalPlanner->LocalizeStep(dt, desiredStatus);

					/**
					 * Control, Path Following
					 */
					desiredStatus = m_PredControl.DoOneStep(dt, m_CurrBehavior, m_LocalPlanner->m_Path, m_LocalPlanner->state, currStatus, m_CurrBehavior.bNewPlan);
				}
			}
			displayFollowingInfo(m_LocalPlanner->m_TrajectoryCostsCalculator.m_SafetyBorder.points, m_LocalPlanner->state);
			visualizePath(m_LocalPlanner->m_Path);
			visualizeBehaviors();


			geometry_msgs::PoseArray sim_data;
			geometry_msgs::Pose p_id, p_pose, p_box, p_indicator;


			sim_data.header.frame_id = "map";
			sim_data.header.stamp = ros::Time().now();

			p_id.position.x = m_SimParams.id;
			p_id.position.y = currStatus.speed; // send actual calculated velocity after sensing delay
			p_id.position.z = currStatus.steer; // send actual calculated steering after sensing delay

			PlannerHNS::WayPoint pose_center = PlannerHNS::PlanningHelpers::GetRealCenter(m_LocalPlanner->state, m_CarInfo.wheel_base);

			p_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(pose_center.pos.a));
			p_pose.position.x = pose_center.pos.x;
			p_pose.position.y = pose_center.pos.y;
			p_pose.position.z = pose_center.pos.z;

			p_box.position.x = m_CarInfo.width;
			p_box.position.y = m_CarInfo.length;
			p_box.position.z = 2.0;

			p_indicator.orientation.w = m_CurrBehavior.indicator;

			sim_data.poses.push_back(p_id);
			sim_data.poses.push_back(p_pose);
			sim_data.poses.push_back(p_box);
			sim_data.poses.push_back(p_indicator);

			pub_SimuBoxPose.publish(sim_data);

			//if(m_bSimulatedVelodyne)
				PublishSpecialTF(pose_center);

			if(m_CurrBehavior.bNewPlan && m_SimParams.bEnableLogs)
			{
				std::ostringstream str_out;
				str_out << m_SimParams.logPath;
				str_out << "LocalPath_";
				PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(),  m_LocalPlanner->m_Path);
			}

			if(m_SimParams.bLooper && m_CurrBehavior.state == PlannerHNS::FINISH_STATE)
			{
				m_GlobalPaths.clear();
				delete m_LocalPlanner;
				m_LocalPlanner = 0;
				m_LocalPlanner = new  PlannerHNS::SimuDecisionMaker();
				m_LocalPlanner->Init(m_ControlParams, m_PlanningParams, m_CarInfo);
				m_LocalPlanner->m_SimulationSteeringDelayFactor = m_ControlParams.SimulationSteeringDelay;
				InitializeSimuCar(m_SimParams.startPose);
			}

			if(m_SimParams.bEnableLogs)
			{
				autoware_msgs::lane lane;
				PlannerHNS::RosHelpers::ConvertFromLocalLaneToAutowareLane(m_LocalPlanner->m_Path, lane);
				lane.lane_id = m_SimParams.id;
				lane.lane_index = (int)m_CurrBehavior.state;
				lane.header.stamp = sim_data.header.stamp;
				pub_CurrentLocalPath.publish(lane);
			}
		}

		loop_rate.sleep();
	}
}

//Mapping Section

void OpenPlannerCarSimulator::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void OpenPlannerCarSimulator::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

}
