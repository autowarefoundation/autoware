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
#include "ff_waypoint_follower_core.h"
#include "autoware_msgs/LaneArray.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "op_utility/UtilityH.h"
#include "math.h"

using namespace std;

namespace FFSteerControlNS
{

FFSteerControl::FFSteerControl()
{
	clock_gettime(0, &m_Timer);

	int iSignal = 0;
	nh.getParam("/ff_waypoint_follower/signal", iSignal);
	if(iSignal == 0)
		m_CmdParams.statusSource = FFSteerControlNS::SIMULATION_STATUS;
	else if(iSignal == 1)
		m_CmdParams.statusSource = FFSteerControlNS::CONTROL_BOX_STATUS;
	else if(iSignal == 2)
		m_CmdParams.statusSource = FFSteerControlNS::ROBOT_STATUS;

	nh.getParam("/ff_waypoint_follower/bEnableLogs", m_CmdParams.bEnableLogs);
	nh.getParam("/ff_waypoint_follower/bCalibrationMode", m_CmdParams.bCalibration);
	if(m_CmdParams.bCalibration)
		m_CmdParams.bEnableLogs = true;

	string steerModeStr;
	nh.getParam("/ff_waypoint_follower/steerMode", steerModeStr );
	if(steerModeStr.compare("angle") == 0)
		m_CmdParams.bAngleMode = true;
	else if(steerModeStr.compare("torque") == 0)
		m_CmdParams.bAngleMode = false;

	string driveModeStr;
	nh.getParam("/ff_waypoint_follower/driveMode", driveModeStr );
	if(driveModeStr.compare("velocity") == 0)
		m_CmdParams.bVelocityMode = true;
	else if(driveModeStr.compare("stroke") == 0)
		m_CmdParams.bVelocityMode = false;

	string mapRecStr;
	nh.getParam("/ff_waypoint_follower/mapRecorder", m_CmdParams.iMapping );

	nh.getParam("/ff_waypoint_follower/mapDistance", m_CmdParams.recordDistance );

	nh.getParam("/ff_waypoint_follower/mapDensity", m_CmdParams.recordDensity );

	cout << "Initialize Controller .. " << ", "
			<< iSignal << "," << steerModeStr << ", "
			<< driveModeStr << ", " << m_CmdParams.iMapping << ", "
			<< m_CmdParams.recordDistance << ", " << m_CmdParams.recordDensity << endl;

	ReadParamFromLaunchFile(m_CarInfo, m_ControlParams);

	m_PredControl.Init(m_ControlParams, m_CarInfo, m_CmdParams.bEnableLogs, m_CmdParams.bCalibration);

	m_State.Init(m_ControlParams, m_PlanningParams, m_CarInfo);
	m_State.m_SimulationSteeringDelayFactor = 0.2;
	cout << "Current Steering Delay Factor = " << m_State.m_SimulationSteeringDelayFactor << endl;

	m_counter = 0;
	m_frequency = 0;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bNewTrajectory = false;
	bNewVelocity = false;
	bNewBehaviorState = false;
	bInitPos = false;
	m_bOutsideControl = 0;


#ifdef ENABLE_ZMP_LIBRARY_LINK
	m_pComm = 0;
	if(m_CmdParams.statusSource == CONTROL_BOX_STATUS)
	{
		m_pComm = new HevComm();
		m_pComm->InitializeComm(m_CarInfo);
		m_pComm->StartComm();
	}
#endif


	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);
	ROS_INFO("Origin : x=%f, y=%f, z=%f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());

	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();


	pub_VelocityAutoware 		= nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 100);
	pub_StatusAutoware 			= nh.advertise<std_msgs::Bool>("wf_stat", 100);

	//For rviz visualization
	pub_CurrPoseRviz			= nh.advertise<visualization_msgs::Marker>("curr_simu_pose", 100);
	pub_FollowPointRviz			= nh.advertise<visualization_msgs::Marker>("follow_pose", 100);

	pub_SimuPose				= nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 100);
	pub_SimuVelocity			= nh.advertise<geometry_msgs::TwistStamped>("sim_velocity", 100);

	pub_VehicleCommand			= nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 100);
	pub_ControlBoxOdom			= nh.advertise<nav_msgs::Odometry>("ControlBoxOdom", 100);
	pub_VelocityRviz 			= nh.advertise<std_msgs::Float32>("linear_velocity_viz", 10);

	// define subscribers.
	sub_initialpose 		= nh.subscribe("/initialpose", 		100, &FFSteerControl::callbackGetInitPose, 			this);

  	if(m_CmdParams.statusSource != SIMULATION_STATUS)
  		sub_current_pose 	= nh.subscribe("/current_pose", 		100, &FFSteerControl::callbackGetCurrentPose, 		this);

  	if(m_CmdParams.statusSource == ROBOT_STATUS)
		sub_robot_odom			= nh.subscribe("/odom",				100, &FFSteerControl::callbackGetRobotOdom, 		this);
  	else
  		sub_current_velocity= nh.subscribe("/current_velocity", 	100, &FFSteerControl::callbackGetCurrentVelocity, 		this);

  	sub_behavior_state 		= nh.subscribe("/current_behavior",	10,  &FFSteerControl::callbackGetBehaviorState, 	this);
  	sub_current_trajectory 	= nh.subscribe("/final_waypoints", 	10,	&FFSteerControl::callbackGetCurrentTrajectory, this);



  	sub_OutsideControl 	= nh.subscribe("/usb_controller_r_signal", 	10,		&FFSteerControl::callbackGetOutsideControl, 	this);

  	//sub_autoware_odom 		= nh.subscribe("/twist_odom", 		10,	&FFSteerControl::callbackGetAutowareOdom, this);


	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

	std::cout << "ff_waypoint_follower initialized successfully " << std::endl;

}

void FFSteerControl::ReadParamFromLaunchFile(PlannerHNS::CAR_BASIC_INFO& m_CarInfo,
		PlannerHNS::ControllerParams& m_ControlParams)
{
	nh.getParam("/ff_waypoint_follower/width", 			m_CarInfo.width );
	nh.getParam("/ff_waypoint_follower/length", 		m_CarInfo.length );
	nh.getParam("/ff_waypoint_follower/wheelBaseLength", m_CarInfo.wheel_base );
	nh.getParam("/ff_waypoint_follower/turningRadius", m_CarInfo.turning_radius );
	nh.getParam("/ff_waypoint_follower/maxSteerAngle", m_CarInfo.max_steer_angle );

	nh.getParam("/ff_waypoint_follower/maxSteerValue", m_CarInfo.max_steer_value );
	nh.getParam("/ff_waypoint_follower/minSteerValue", m_CarInfo.min_steer_value );
	nh.getParam("/ff_waypoint_follower/maxVelocity", m_CarInfo.max_speed_forward );
	nh.getParam("/ff_waypoint_follower/minVelocity", m_CarInfo.min_speed_forward );
	nh.getParam("/ff_waypoint_follower/minVelocity", m_CarInfo.max_speed_backword );

	nh.getParam("/ff_waypoint_follower/steeringDelay", m_ControlParams.SteeringDelay );
	nh.getParam("/ff_waypoint_follower/minPursuiteDistance", m_ControlParams.minPursuiteDistance );
	nh.getParam("/ff_waypoint_follower/followDistance", m_ControlParams.FollowDistance );
	nh.getParam("/ff_waypoint_follower/lowpassSteerCutoff", m_ControlParams.LowpassSteerCutoff );

	nh.getParam("/ff_waypoint_follower/steerGainKP", m_ControlParams.Steering_Gain.kP );
	nh.getParam("/ff_waypoint_follower/steerGainKI", m_ControlParams.Steering_Gain.kI );
	nh.getParam("/ff_waypoint_follower/steerGainKD", m_ControlParams.Steering_Gain.kD );

	nh.getParam("/ff_waypoint_follower/velocityGainKP", m_ControlParams.Velocity_Gain.kP );
	nh.getParam("/ff_waypoint_follower/velocityGainKI", m_ControlParams.Velocity_Gain.kI );
	nh.getParam("/ff_waypoint_follower/velocityGainKD", m_ControlParams.Velocity_Gain.kD );
	nh.getParam("/ff_waypoint_follower/maxAcceleration", m_CarInfo.max_acceleration );
	nh.getParam("/ff_waypoint_follower/maxDeceleration", m_CarInfo.max_deceleration );

	m_PlanningParams.maxSpeed = m_CarInfo.max_speed_forward;
	m_PlanningParams.minSpeed = m_CarInfo.min_speed_forward;



}

FFSteerControl::~FFSteerControl()
{
#ifdef ENABLE_ZMP_LIBRARY_LINK
	if(m_pComm)
		delete m_pComm;
#endif
}

void FFSteerControl::callbackGetOutsideControl(const std_msgs::Int8& msg)
{
	//std::cout << "Received Outside Control : " << msg.data << std::endl;

	m_bOutsideControl  = msg.data;
}

void FFSteerControl::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, m_frequency);

	geometry_msgs::Pose p;
	p.position.x  = msg->pose.pose.position.x + m_OriginPos.position.x;
	p.position.y  = msg->pose.pose.position.y + m_OriginPos.position.y;
	p.position.z  = msg->pose.pose.position.z + m_OriginPos.position.z;
	p.orientation = msg->pose.pose.orientation;

	m_InitPos =  PlannerHNS::WayPoint(p.position.x, p.position.y, p.position.z , tf::getYaw(p.orientation));
	m_State.FirstLocalizeMe(m_InitPos);
	m_CurrentPos = m_InitPos;
	bInitPos = true;
}

void FFSteerControl::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_counter++;
	double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer);
	if(dt >= 1.0)
	{
		m_frequency = m_counter;
		m_counter = 0;
		clock_gettime(0, &m_Timer);
	}

	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));

	bNewCurrentPos = true;
}

void FFSteerControl::callbackGetCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_CurrVehicleStatus.shift = PlannerHNS::SHIFT_POS_DD;
	m_CurrVehicleStatus.speed = msg->twist.linear.x;
	if(msg->twist.linear.x != 0)
		m_CurrVehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);

	UtilityHNS::UtilityH::GetTickCount(m_CurrVehicleStatus.tStamp);
}

void FFSteerControl::callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg )
{
	m_CurrentBehavior = ConvertBehaviorStateFromAutowareToPlannerH(msg);
	bNewBehaviorState = true;
}

void FFSteerControl::callbackGetCurrentTrajectory(const autoware_msgs::laneConstPtr &msg)
{
	m_State.m_Path.clear();
	for(unsigned int i = 0 ; i < msg->waypoints.size(); i++)
	{
		PlannerHNS::WayPoint wp(msg->waypoints.at(i).pose.pose.position.x,
				msg->waypoints.at(i).pose.pose.position.y, msg->waypoints.at(i).pose.pose.position.z,
				tf::getYaw(msg->waypoints.at(i).pose.pose.orientation));
		wp.v = msg->waypoints.at(i).twist.twist.linear.x;

		if(msg->waypoints.at(i).twist.twist.linear.z == 0)
			wp.bDir = PlannerHNS::FORWARD_DIR;
		else if(msg->waypoints.at(i).twist.twist.linear.z == 1)
			wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
		else if(msg->waypoints.at(i).twist.twist.linear.z == 2)
			wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;
		else if(msg->waypoints.at(i).twist.twist.linear.z == 3)
			wp.bDir = PlannerHNS::BACKWARD_DIR;
		else if(msg->waypoints.at(i).twist.twist.linear.z == 4)
			wp.bDir = PlannerHNS::BACKWARD_LEFT_DIR;
		else if(msg->waypoints.at(i).twist.twist.linear.z == 5)
			wp.bDir = PlannerHNS::BACKWARD_RIGHT_DIR;
		else if(msg->waypoints.at(i).twist.twist.linear.z == 6)
			wp.bDir = PlannerHNS::STANDSTILL_DIR;
		else
			wp.bDir = PlannerHNS::STANDSTILL_DIR;

		m_State.m_Path.push_back(wp);
	}

//	cout << "### Current Trajectory CallBaclk -> " << m_State.m_Path.size() << endl;

	bNewTrajectory = true;
}

//void FFSteerControl::callbackGetAutowareOdom(const geometry_msgs::TwistStampedConstPtr &msg)
//{
//}

void FFSteerControl::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(m_CmdParams.statusSource == ROBOT_STATUS)
	{
//		PlannerHNS::WayPoint odoPose = PlannerHNS::WayPoint(msg->pose.pose.position.x,
//				msg->pose.pose.position.y,msg->pose.pose.position.z , tf::getYaw(msg->pose.pose.orientation));

		m_CurrVehicleStatus.shift = PlannerHNS::SHIFT_POS_DD;
		m_CurrVehicleStatus.speed = msg->twist.twist.linear.x;
		if(msg->twist.twist.linear.x!=0)
			m_CurrVehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
		UtilityHNS::UtilityH::GetTickCount(m_CurrVehicleStatus.tStamp);

//		std::cout << "###### Current Status From Robot Odometry -> (" <<  m_CurrVehicleStatus.speed << ", " << m_CurrVehicleStatus.steer << ")"  << std::endl;
	}
}

void FFSteerControl::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
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

void FFSteerControl::displayFollowingInfo(PlannerHNS::WayPoint& curr_pose, PlannerHNS::WayPoint& perp_pose, PlannerHNS::WayPoint& follow_pose)
{
  visualization_msgs::Marker m1,m2,m3;
  m1.header.frame_id = "map";
  m1.header.stamp = ros::Time();
  m1.ns = "curr_simu_pose";
  m1.type = visualization_msgs::Marker::ARROW;
  m1.action = visualization_msgs::Marker::ADD;
  m1.pose.position.x = curr_pose.pos.x;
  m1.pose.position.y = curr_pose.pos.y;
  m1.pose.position.z = curr_pose.pos.z;
  m1.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(curr_pose.pos.a));
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  m1.color = green;
  m1.scale.x = 1.8;
  m1.scale.y = 0.5;
  m1.scale.z = 0.5;
  m1.frame_locked = true;
  pub_CurrPoseRviz.publish(m1);

  m3.header.frame_id = "map";
  m3.header.stamp = ros::Time();
  m3.ns = "follow_pose";
  m3.type = visualization_msgs::Marker::SPHERE;
  m3.action = visualization_msgs::Marker::ADD;
  m3.pose.position.x = follow_pose.pos.x;
  m3.pose.position.y = follow_pose.pos.y;
  m3.pose.position.z = follow_pose.pos.z;
  m3.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(follow_pose.pos.a));
  std_msgs::ColorRGBA red;
  red.a = 1.0;
  red.b = 0.0;
  red.r = 1.0;
  red.g = 0.0;
  m3.color = red;
  m3.scale.x = 0.7;
  m3.scale.y = 0.7;
  m3.scale.z = 0.7;
  m3.frame_locked = true;
  pub_FollowPointRviz.publish(m3);
}

PlannerHNS::BehaviorState FFSteerControl::ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg)
{
	PlannerHNS::BehaviorState behavior;
	behavior.followDistance 	= msg->twist.linear.x;
	behavior.stopDistance 		= msg->twist.linear.y;
	behavior.followVelocity 	= msg->twist.angular.x;
	behavior.maxVelocity 		= msg->twist.angular.y;


	if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT;
	else if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT;
	else if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH;
	else if(msg->twist.linear.z == PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE)
		behavior.indicator = PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE;

	if(msg->twist.angular.z == PlannerHNS::INITIAL_STATE)
		behavior.state = PlannerHNS::INITIAL_STATE;
	else if(msg->twist.angular.z == PlannerHNS::WAITING_STATE)
		behavior.state = PlannerHNS::WAITING_STATE;
	else if(msg->twist.angular.z == PlannerHNS::FORWARD_STATE)
		behavior.state = PlannerHNS::FORWARD_STATE;
	else if(msg->twist.angular.z == PlannerHNS::STOPPING_STATE)
		behavior.state = PlannerHNS::STOPPING_STATE;
	else if(msg->twist.angular.z == PlannerHNS::EMERGENCY_STATE)
		behavior.state = PlannerHNS::EMERGENCY_STATE;
	else if(msg->twist.angular.z == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
		behavior.state = PlannerHNS::TRAFFIC_LIGHT_STOP_STATE;
	else if(msg->twist.angular.z == PlannerHNS::STOP_SIGN_STOP_STATE)
		behavior.state = PlannerHNS::STOP_SIGN_STOP_STATE;
	else if(msg->twist.angular.z == PlannerHNS::STOP_SIGN_WAIT_STATE)
		behavior.state = PlannerHNS::STOP_SIGN_WAIT_STATE;
	else if(msg->twist.angular.z == PlannerHNS::FOLLOW_STATE)
		behavior.state = PlannerHNS::FOLLOW_STATE;
	else if(msg->twist.angular.z == PlannerHNS::LANE_CHANGE_STATE)
		behavior.state = PlannerHNS::LANE_CHANGE_STATE;
	else if(msg->twist.angular.z == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
		behavior.state = PlannerHNS::OBSTACLE_AVOIDANCE_STATE;
	else if(msg->twist.angular.z == PlannerHNS::FINISH_STATE)
		behavior.state = PlannerHNS::FINISH_STATE;


	return behavior;

}

void FFSteerControl::PlannerMainLoop()
{

	ros::Rate loop_rate(100);
#ifdef ENABLE_ZMP_LIBRARY_LINK
	if(m_pComm)
		m_pComm->GoLive(true);
#endif

	vector<PlannerHNS::WayPoint> path;
	PlannerHNS::WayPoint p2;
	double totalDistance = 0;

	while (ros::ok())
	{
		ros::spinOnce();

		PlannerHNS::BehaviorState currMessage = m_CurrentBehavior;
		double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
		UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

		if(currMessage.state != PlannerHNS::INITIAL_STATE &&  (bInitPos || bNewCurrentPos))
		{
			/**
			 * Localization and Status Reading Part
			 * -----------------------------------------------------------------------------------
			 */
			if(m_CmdParams.statusSource == CONTROL_BOX_STATUS)
			{
				//Read StateData From Control Box
#ifdef ENABLE_ZMP_LIBRARY_LINK
				if(m_pComm && m_pComm->IsAuto())
				{
					m_CurrVehicleStatus.steer = m_pComm->GetCurrentSteerAngle();
					m_CurrVehicleStatus.speed = m_pComm->GetCurrentSpeed();
					m_CurrVehicleStatus.shift = m_pComm->GetCurrentShift();

					//Send status over message to planner
					nav_msgs::Odometry control_box_status;
					control_box_status.header.stamp = ros::Time::now();
					control_box_status.twist.twist.angular.z = m_CurrVehicleStatus.steer;
					control_box_status.twist.twist.linear.x = m_CurrVehicleStatus.speed;
					control_box_status.twist.twist.linear.z = (int)m_CurrVehicleStatus.shift;

					control_box_status.pose.pose.position.x = m_CurrentPos.pos.x;
					control_box_status.pose.pose.position.y = m_CurrentPos.pos.y;
					control_box_status.pose.pose.position.z = m_CurrentPos.pos.z;
					control_box_status.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_CurrentPos.pos.a));

					pub_ControlBoxOdom.publish(control_box_status);

					cout << "Read Live Car Info .. " << endl;
				}
				else
				{
					cout << ">>> Error, Disconnected from Car Control Box !" << endl;
				}
#endif
			}
			else if(m_CmdParams.statusSource == SIMULATION_STATUS)
			{
				//m_CurrVehicleStatus = m_PrevStepTargetStatus;
				//m_State.SimulateOdoPosition(dt, m_CurrVehicleStatus);

				m_State.SimulateOdoPosition(dt, m_PrevStepTargetStatus);
				m_CurrVehicleStatus.steer = m_State.m_CurrentSteering;
				m_CurrVehicleStatus.speed = m_State.m_CurrentVelocity;
				m_CurrVehicleStatus.shift = m_PrevStepTargetStatus.shift;
				m_CurrentPos = m_State.state;

				geometry_msgs::TwistStamped vehicle_status;
				vehicle_status.header.stamp = ros::Time::now();
				vehicle_status.twist.linear.x = m_CurrVehicleStatus.speed;
				vehicle_status.twist.angular.z = ( vehicle_status.twist.linear.x / m_CarInfo.wheel_base ) * tan(m_CurrVehicleStatus.steer);

				vehicle_status.twist.linear.z = (int)m_CurrVehicleStatus.shift;
				pub_SimuVelocity.publish(vehicle_status);

				geometry_msgs::PoseStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.pose.position.x = m_CurrentPos.pos.x;
				pose.pose.position.y = m_CurrentPos.pos.y;
				pose.pose.position.z = m_CurrentPos.pos.z;
				pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_CurrentPos.pos.a));
				//cout << "Send Simulated Position "<< m_CurrentPos.pos.ToString() << endl;

				pub_SimuPose.publish(pose);

				static tf::TransformBroadcaster odom_broadcaster;
				geometry_msgs::TransformStamped odom_trans;
				odom_trans.header.stamp = ros::Time::now();
				odom_trans.header.frame_id = "map";
				odom_trans.child_frame_id = "base_link";

				odom_trans.transform.translation.x = pose.pose.position.x;
				odom_trans.transform.translation.y = pose.pose.position.y;
				odom_trans.transform.translation.z = pose.pose.position.z;
				odom_trans.transform.rotation = pose.pose.orientation;

				// send the transform
				odom_broadcaster.sendTransform(odom_trans);
			}
			else if(m_CmdParams.statusSource == ROBOT_STATUS)
			{
//				geometry_msgs::TwistStamped vehicle_status;
//				vehicle_status.header.stamp = ros::Time::now();
//				vehicle_status.twist.angular.z = m_CurrVehicleStatus.steer;
//				vehicle_status.twist.linear.x = m_CurrVehicleStatus.speed;
//				vehicle_status.twist.linear.z = (int)m_CurrVehicleStatus.shift;
//				pub_VehicleStatus.publish(vehicle_status);
			}
			//----------------------------------------------------------------------------------------------//


			/**
			 * Path Following Part
			 * -----------------------------------------------------------------------------------------------
			 */

			bool bNewPath = false;
			if(PlannerHNS::PlanningHelpers::CompareTrajectories(m_FollowingTrajectory , m_State.m_Path) == false && m_State.m_Path.size()>0)
			{
				m_FollowingTrajectory = m_State.m_Path;
				bNewPath = true;
//				cout << "Path is Updated in the controller .. " << m_State.m_Path.size() << endl;
			}

			//PlannerHNS::ControllerParams c_params = m_ControlParams;
			//c_params.SteeringDelay = m_ControlParams.SteeringDelay / (1.0- UtilityHNS::UtilityH::GetMomentumScaleFactor(m_CurrVehicleStatus.speed));
			//m_PredControl.Init(c_params, m_CarInfo);
			m_PrevStepTargetStatus = m_PredControl.DoOneStep(dt, currMessage, m_FollowingTrajectory, m_CurrentPos, m_CurrVehicleStatus, bNewPath);
			//m_PrevStepTargetStatus.speed = 3.0;
			m_State.state.pos.z = m_PerpPoint.pos.z;
			m_FollowPoint  = m_PredControl.m_FollowMePoint;
			m_PerpPoint    = m_PredControl.m_PerpendicularPoint;

//			cout << "Target Status (" <<m_PrevStepTargetStatus.steer << ", " << m_PrevStepTargetStatus.speed
//					<< ", " << m_PrevStepTargetStatus.shift << ")" << endl;

			//----------------------------------------------------------------------------------------------//



			if(m_CmdParams.statusSource == CONTROL_BOX_STATUS) //send directly to ZMP control box
			{
#ifdef ENABLE_ZMP_LIBRARY_LINK
				if(m_pComm && m_pComm->IsAuto())
				{
					m_pComm->SetNormalizedSteeringAngle(m_PrevStepTargetStatus.steer);
					m_pComm->SetNormalizedSpeed(m_PrevStepTargetStatus.speed);
					m_pComm->SetShift(m_PrevStepTargetStatus.shift);
					cout << "Sending Data to Control Box (" <<m_PrevStepTargetStatus.steer << ", " << m_PrevStepTargetStatus.speed
							<< ", " << m_PrevStepTargetStatus.shift << ")" << endl;
				}
				else
				{
					cout << ">>> Error, Disconnected from Car Control Box !" << endl;
				}
#endif
			}
			else if(m_CmdParams.statusSource == ROBOT_STATUS)
			{
				cout << "Send Data To Robot : Max Speed=" << m_CarInfo.max_speed_forward << ", actual = " <<  m_PrevStepTargetStatus.speed << endl;
				geometry_msgs::Twist t;
				geometry_msgs::TwistStamped twist;
				t.linear.x = m_PrevStepTargetStatus.speed;

				if(t.linear.x > m_CarInfo.max_speed_forward)
					t.linear.x = m_CarInfo.max_speed_forward;

				t.angular.z = m_PrevStepTargetStatus.steer;

				if(t.angular.z > m_CarInfo.max_steer_angle)
					t.angular.z = m_CarInfo.max_steer_angle;
				else if(t.angular.z < -m_CarInfo.max_steer_angle)
					t.angular.z = -m_CarInfo.max_steer_angle;

				twist.twist = t;
				twist.header.stamp = ros::Time::now();

				pub_VehicleCommand.publish(twist);
			}
			else if(m_CmdParams.statusSource == SIMULATION_STATUS)
			{
			}

			displayFollowingInfo(m_CurrentPos, m_PerpPoint, m_FollowPoint);

			std_msgs::Float32 vel_rviz;
			vel_rviz.data = m_CurrVehicleStatus.speed;
			pub_VelocityRviz.publish(vel_rviz);

		}

		 if (m_CmdParams.iMapping == 1 && bNewCurrentPos == true)
		 {
			 bNewCurrentPos = false;
			double _d = hypot(m_CurrentPos.pos.y - p2.pos.y, m_CurrentPos.pos.x - p2.pos.x);
			if(_d > m_CmdParams.recordDensity)
			{
				p2 = m_CurrentPos;

				if(path.size() > 0)
					totalDistance += _d;

				m_CurrentPos.pos.lat = m_CurrentPos.pos.x;
				m_CurrentPos.pos.lon = m_CurrentPos.pos.y;
				m_CurrentPos.pos.alt = m_CurrentPos.pos.z;
				m_CurrentPos.pos.dir = m_CurrentPos.pos.a;

				m_CurrentPos.laneId = 1;
				m_CurrentPos.id = path.size()+1;
				if(path.size() > 0)
				{
					path.at(path.size()-1).toIds.push_back(m_CurrentPos.id);
					m_CurrentPos.fromIds.clear();
					m_CurrentPos.fromIds.push_back(path.at(path.size()-1).id);
				}

				path.push_back(m_CurrentPos);
				std::cout << "Record One Point To Path: " <<  m_CurrentPos.pos.ToString() << std::endl;
			}

//			if(totalDistance > m_CmdParams.recordDistance || m_bOutsideControl != 0)
//			{
//				PlannerHNS::RoadNetwork roadMap;
//				PlannerHNS::RoadSegment segment;
//
//				segment.id = 1;
//
//				PlannerHNS::Lane lane;
//				lane.id = 1;
//				lane.num = 0;
//				lane.roadId = 1;
//				lane.points = path;
//
//				segment.Lanes.push_back(lane);
//				roadMap.roadSegments.push_back(segment);
//
//				ostringstream fileName;
//				fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName;
//				fileName << UtilityHNS:: UtilityH::GetFilePrefixHourMinuteSeconds();
//				fileName << "_RoadNetwork.kml";
//				string kml_templateFilePath = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::KmlMapsFolderName+"PlannerX_MapTemplate.kml";
//
//				//PlannerHNS::MappingHelpers::WriteKML(fileName.str(),kml_templateFilePath , roadMap);
//
//										//string kml_fileToSave =UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmltargetFile;
//					//PlannerHNS::MappingHelpers::WriteKML(kml_fileToSave, kml_templateFilePath, m_RoadMap);
//
//				std::cout << " Mapped Saved Successfuly ... " << std::endl;
//				break;
//			}
		 }

		loop_rate.sleep();
	}
}

}
