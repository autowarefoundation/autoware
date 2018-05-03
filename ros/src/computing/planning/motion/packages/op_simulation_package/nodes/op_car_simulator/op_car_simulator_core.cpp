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
#include "../include/op_car_simulator_core.h"

#include "UtilityH.h"
#include "math.h"
#include "MatrixOperations.h"
#include <geometry_msgs/PoseArray.h>
#include "PolygonGenerator.h"
#include "op_RosHelpers.h"

namespace CarSimulatorNS
{

#define REPLANNING_DISTANCE 3

OpenPlannerCarSimulator::OpenPlannerCarSimulator()
{

	m_bMap = false;
	bPredictedObjects = false;

	ReadParamFromLaunchFile(m_CarInfo, m_ControlParams);

	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);

	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	m_PredControl.Init(m_ControlParams, m_CarInfo, false, false);

	m_LocalPlanner = new PlannerHNS::LocalPlannerH();
	m_LocalPlanner->Init(m_ControlParams, m_PlanningParams, m_CarInfo);
	m_LocalPlanner->m_SimulationSteeringDelayFactor = m_ControlParams.SimulationSteeringDelay;

	//For rviz visualization
	std::ostringstream str_s1, str_s2, str_s3, str_s4, str_s5, str_s6, str_s7;
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



	pub_CurrPoseRviz				= nh.advertise<visualization_msgs::Marker>(str_s1.str() , 100);
	pub_SimuBoxPose					= nh.advertise<geometry_msgs::PoseArray>(str_s5.str(), 100);
	pub_SafetyBorderRviz  			= nh.advertise<visualization_msgs::Marker>(str_s4.str(), 1);
	pub_LocalTrajectoriesRviz   	= nh.advertise<visualization_msgs::MarkerArray>(str_s6.str(), 1);
	pub_BehaviorStateRviz			= nh.advertise<visualization_msgs::Marker>(str_s2.str(), 1);
	pub_PointerBehaviorStateRviz	= nh.advertise<visualization_msgs::Marker>(str_s2.str(), 1);


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
			m_CarInfo.max_speed_forward = start_p.v;
		}
		else
		{
			ROS_ERROR("Can't Read Start and Goal information from log file for OpenPlanner simulated car !");
			return;
		}

	}

	if(m_PlanningParams.enableFollowing)
		sub_predicted_objects 			= nh.subscribe("/predicted_objects", 	1, &OpenPlannerCarSimulator::callbackGetPredictedObjects, 		this);

	if(m_PlanningParams.enableTrafficLightBehavior)
		sub_TrafficLightSignals		= nh.subscribe("/roi_signal", 		10,	&OpenPlannerCarSimulator::callbackGetTrafficLightSignals, 	this);

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

	//_nh.getParam("enableCurbObstacles", m_bEnableCurbObstacles);
	int iSource = 0;
	_nh.getParam("mapSource" 			, iSource);
	if(iSource == 0)
		m_SimParams.mapSource = MAP_FOLDER;
	else if(iSource == 1)
		m_SimParams.mapSource = MAP_KML_FILE;

	_nh.getParam("mapFileName" 		, m_SimParams.KmlMapPath);

	//m_SimParams.KmlMapPath = "/media/hatem/8ac0c5d5-8793-4b98-8728-55f8d67ec0f4/data/ToyotaCity2/map/vector_map/";
	m_PlanningParams.additionalBrakingDistance = 0;


	m_ControlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01); // for 3 m/s
	m_ControlParams.Velocity_Gain = PlannerHNS::PID_CONST(0.1, 0.005, 0.1);
}

OpenPlannerCarSimulator::~OpenPlannerCarSimulator()
{
	if(bInitPos && bGoalPos)
		SaveSimulationData();
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
	//cout << endl << "LocalPlannerInit: ID " << m_SimParams.strID << " , Pose = ( "  << start_pose.pos.ToString() << ")" << endl;
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
		PlannerHNS::RosHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
		m_PredictedObjects.push_back(obj);
	}
}

PlannerHNS::WayPoint OpenPlannerCarSimulator::GetRealCenter(const PlannerHNS::WayPoint& currState)
{
	PlannerHNS::WayPoint pose_center = currState;
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	PlannerHNS::Mat3 rotationMatInv(currState.pos.a);
	PlannerHNS::Mat3 translationMatInv(currState.pos.x, currState.pos.y);

	pose_center.pos = translationMat*pose_center.pos;
	pose_center.pos = rotationMat*pose_center.pos;

	pose_center.pos.x += m_CarInfo.wheel_base/3.0;

	pose_center.pos = rotationMatInv*pose_center.pos;
	pose_center.pos = translationMatInv*pose_center.pos;

	return pose_center;
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

  PlannerHNS::WayPoint pose_center = GetRealCenter(curr_pose);

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

	std::ostringstream str_out;
	str_out << str;
	str_out << m_SimParams.id;
	double two_prec_speed = (m_LocalPlanner->m_CurrentVelocity*3.6*100)/100.0;
	std::ostringstream speed_out;
	speed_out.precision(3);
	speed_out << two_prec_speed;
	string speed_str = speed_out.str();
	if(speed_str.size() <= 1 )
		speed_str = "0.0";

	str_out << "(" << speed_str << ")" ;
	behaviorMarker.text = str_out.str();

	pub_BehaviorStateRviz.publish(behaviorMarker);
	pub_PointerBehaviorStateRviz.publish(pointerMarker);
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
	return nData;
}

void OpenPlannerCarSimulator::MainLoop()
{

	ros::Rate loop_rate(50);

	PlannerHNS::BehaviorState currBehavior;
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

		if(m_bMap && bInitPos && bGoalPos)
		{
			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			//Global Planning Step
			if(m_LocalPlanner->m_TotalOriginalPath.size() > 0 && m_LocalPlanner->m_TotalOriginalPath.at(0).size() > 3)
			{
				PlannerHNS::RelativeInfo info;
				bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_LocalPlanner->m_TotalOriginalPath, m_LocalPlanner->state, 0.75, info);
				if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < (int)m_LocalPlanner->m_TotalOriginalPath.size() && info.iFront > 0 && info.iFront < (int)m_LocalPlanner->m_TotalOriginalPath.at(info.iGlobalPath).size())
				{
					PlannerHNS::WayPoint wp_end = m_LocalPlanner->m_TotalOriginalPath.at(info.iGlobalPath).at(m_LocalPlanner->m_TotalOriginalPath.at(info.iGlobalPath).size()-1);
					PlannerHNS::WayPoint wp_first = m_LocalPlanner->m_TotalOriginalPath.at(info.iGlobalPath).at(info.iFront);
					double remaining_distance =   hypot(wp_end.pos.y - wp_first.pos.y, wp_end.pos.x - wp_first.pos.x) + info.to_front_distance;

					if(remaining_distance <= REPLANNING_DISTANCE)
					{

						cout << "Remaining Distance : " << remaining_distance << endl;
						bMakeNewPlan = true;
						if(m_SimParams.bLooper)
						{
							delete m_LocalPlanner;
							m_LocalPlanner = 0;
							m_LocalPlanner = new  PlannerHNS::LocalPlannerH();
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
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
				}

				m_LocalPlanner->m_TotalOriginalPath = generatedTotalPaths;
				m_LocalPlanner->m_pCurrentBehaviorState->GetCalcParams()->bNewGlobalPath = true;
			}

			if(bNewLightSignal)
			{
				m_PrevTrafficLight = m_CurrTrafficLight;
				bNewLightSignal = false;
			}

			/**
			 *  Local Planning
			 */
			currBehavior = m_LocalPlanner->DoOneStep(dt, currStatus, m_PredictedObjects, 1, m_Map, 0, m_PrevTrafficLight, true);

			/**
			 * Localization, Odometry Simulation and Update
			 */
			m_LocalPlanner->SetSimulatedTargetOdometryReadings(desiredStatus.speed, desiredStatus.steer, desiredStatus.shift);
			m_LocalPlanner->UpdateState(desiredStatus, false);
			m_LocalPlanner->LocalizeMe(dt);
			currStatus.shift = desiredStatus.shift;
			currStatus.steer = m_LocalPlanner->m_CurrentSteering;
			currStatus.speed = m_LocalPlanner->m_CurrentVelocity;


			/**
			 * Control, Path Following
			 */
			desiredStatus = m_PredControl.DoOneStep(dt, currBehavior, m_LocalPlanner->m_Path, m_LocalPlanner->state, currStatus, currBehavior.bNewPlan);


			displayFollowingInfo(m_LocalPlanner->m_TrajectoryCostsCalculatotor.m_SafetyBorder.points, m_LocalPlanner->state);
			visualizePath(m_LocalPlanner->m_Path);
			visualizeBehaviors();


			geometry_msgs::PoseArray sim_data;
			geometry_msgs::Pose p_id, p_pose, p_box;


			sim_data.header.frame_id = "map";
			sim_data.header.stamp = ros::Time();

			p_id.position.x = m_SimParams.id;
			p_id.position.y = currStatus.speed; // send actual calculated velocity after sensing delay
			p_id.position.z = currStatus.steer; // send actual calculated steering after sensing delay

			PlannerHNS::WayPoint pose_center = GetRealCenter(m_LocalPlanner->state);

			p_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(pose_center.pos.a));
			p_pose.position.x = pose_center.pos.x;
			p_pose.position.y = pose_center.pos.y;
			p_pose.position.z = pose_center.pos.z;

			p_box.position.x = m_CarInfo.width;
			p_box.position.y = m_CarInfo.length;
			p_box.position.z = 2.0;

			sim_data.poses.push_back(p_id);
			sim_data.poses.push_back(p_pose);
			sim_data.poses.push_back(p_box);

			pub_SimuBoxPose.publish(sim_data);

			if(currBehavior.bNewPlan && m_SimParams.bEnableLogs)
			{
				std::ostringstream str_out;
				str_out << m_SimParams.logPath;
				str_out << "LocalPath_";
				PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(),  m_LocalPlanner->m_Path);
			}

			if(m_SimParams.bLooper && currBehavior.state == PlannerHNS::FINISH_STATE)
			{
				InitializeSimuCar(m_SimParams.startPose);
			}
		}

		loop_rate.sleep();
	}
}

}
