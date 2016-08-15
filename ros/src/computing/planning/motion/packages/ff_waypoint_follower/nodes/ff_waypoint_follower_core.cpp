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
#include "waypoint_follower/LaneArray.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "geo_pos_conv.hh"
#include "UtilityH.h"


namespace FFSteerControlNS
{



FFSteerControl::FFSteerControl(const ControlCommandParams& params)
{
	clock_gettime(0, &m_Timer);

	SimulationNS::CAR_BASIC_INFO carInfo;
	m_State.Init(1.9, 4.2, carInfo);

	m_CmdParams = params;

	m_pComm = new HevComm();
	m_pComm->InitializeComm(carInfo);
	m_pComm->StartComm();

	m_counter = 0;
	m_frequency = 0;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bNewTrajectory = false;
	bNewVelocity = false;
	bNewBehaviorState = false;
	bInitPos = false;


	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);
	//ROS_INFO("Origin : x=%f, y=%f, z=%f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());

	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();


	//m_PositionPublisher = nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 10);
	//m_PathPublisherRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_mark", 10, true);
	m_velocity_publisher 		= nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 100);
	m_stat_pub 					= nh.advertise<std_msgs::Bool>("wf_stat", 100);
	m_target_pub				= nh.advertise<visualization_msgs::Marker>("next_target_mark", 100);
	m_simulated_pos_pub 		= nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 100);
  	m_simulated_velocity_pub 	= nh.advertise<geometry_msgs::Vector3Stamped>("sim_velocity", 100);
  	m_current_vehicle_status	= nh.advertise<geometry_msgs::Vector3Stamped>("vehicle_status", 100);


	// define subscribers.
	sub_current_pose 		= nh.subscribe("/current_pose", 				100,
			&FFSteerControl::callbackFromCurrentPose, 		this);
	sub_current_trajectory 	= nh.subscribe("final_waypoints", 				100,
			&FFSteerControl::callbackFromCurrentTrajectory, this);
	sub_twist_velocity		= nh.subscribe("current_velocity", 				100,
			&FFSteerControl::callbackFromVector3Stamped, 	this);
	initialpose_subscriber 	= nh.subscribe("initialpose", 					100,
			&FFSteerControl::callbackSimuInitPose, 			this);
	sub_behavior_state 	= nh.subscribe("current_behavior",		10,
			&FFSteerControl::callbackFromBehaviorState, 	this);


	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);


}

FFSteerControl::~FFSteerControl()
{
//	if(m_pComm)
//		delete m_pComm;
}

void FFSteerControl::callbackFromVector3Stamped(const geometry_msgs::Vector3StampedConstPtr &msg)
{
	if(m_pComm && m_pComm->IsAuto())
	{
		m_CurrentVelocity = msg->vector.x;
		cout << "### Current Velocity CallBaclk -> " << m_CurrentVelocity << endl;
		bNewVelocity = true;
	}
}

void FFSteerControl::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // write procedure for current pose

	//ROS_INFO("Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, m_frequency);

	if(m_pComm && m_pComm->IsAuto())
	{
		m_counter++;
		double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer);
		if(dt >= 1.0)
		{
			m_frequency = m_counter;
			m_counter = 0;
			clock_gettime(0, &m_Timer);
		}

		geometry_msgs::Pose p = msg->pose;
		p.position.x  = msg->pose.position.x + m_OriginPos.position.x;
		p.position.y  = msg->pose.position.y + m_OriginPos.position.y;
		p.position.z  = msg->pose.position.z + m_OriginPos.position.z;
		p.orientation = msg->pose.orientation;

		m_CurrentPos = PlannerHNS::WayPoint(p.position.x, p.position.y
				, p.position.z , tf::getYaw(p.orientation));

		cout << "### Current Pose CallBaclk -> " << m_CurrentPos.pos.ToString() << endl;

//	double distance = hypot(m_CurrentPos.position.y-p.position.y, m_CurrentPos.position.x-p.position.x);
//	m_VehicleState.speed = distance/dt;
//	if(m_VehicleState.speed>0.2 || m_VehicleState.shift == AW_SHIFT_POS_DD )
//		m_VehicleState.shift = AW_SHIFT_POS_DD;
//	else if(m_VehicleState.speed<-0.2)
//		m_VehicleState.shift = AW_SHIFT_POS_RR;
//	else
//		m_VehicleState.shift = AW_SHIFT_POS_NN;

		bNewCurrentPos = true;
	}

}

void FFSteerControl::callbackSimuInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{

	//ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, m_frequency);


		geometry_msgs::Pose p;
		p.position.x  = msg->pose.pose.position.x + m_OriginPos.position.x;
		p.position.y  = msg->pose.pose.position.y + m_OriginPos.position.y;
		p.position.z  = msg->pose.pose.position.z + m_OriginPos.position.z;
		p.orientation = msg->pose.pose.orientation;

		m_InitPos =  PlannerHNS::WayPoint(p.position.x, p.position.y, p.position.z
				, tf::getYaw(p.orientation));
		m_State.FirstLocalizeMe(m_InitPos);
		m_CurrentPos = m_InitPos;
		bInitPos = true;



}

void FFSteerControl::callbackFromCurrentTrajectory(const waypoint_follower::laneConstPtr &msg)
{
	m_State.m_Path.clear();
	for(unsigned int i = 0 ; i < msg->waypoints.size(); i++)
	{
		PlannerHNS::WayPoint wp(msg->waypoints.at(i).pose.pose.position.x,
				msg->waypoints.at(i).pose.pose.position.y, msg->waypoints.at(i).pose.pose.position.z,
				tf::getYaw(msg->waypoints.at(i).pose.pose.orientation));
		wp.v = msg->waypoints.at(i).twist.twist.linear.x;
		m_State.m_Path.push_back(wp);
	}

	cout << "### Current Trajectory CallBaclk -> " << m_State.m_Path.size() << endl;

	bNewTrajectory = true;
  // ROS_INFO_STREAM("waypoint subscribed");
}

void FFSteerControl::callbackFromBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg )
{
	m_CurrentBehavior = ConvertBehaviorStateFromAutowareToPlannerH(msg);
	bNewBehaviorState = true;
}

void FFSteerControl::PlannerMainLoop()
{

	ros::Rate loop_rate(100);
	if(m_pComm)
		m_pComm->GoLive(true);

	while (ros::ok())
	{
		ros::spinOnce();

		PlannerHNS::BehaviorState currMessage = m_CurrentBehavior;

		if(currMessage.state != PlannerHNS::INITIAL_STATE &&  (bInitPos || bNewCurrentPos))
		{

			PlannerHNS::VehicleState currState, targetState;
			//double targetAccStroke = 0, targetBrakeStroke = 0, targetSteerTorque = 0;
			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			if(m_CmdParams.bTest)
			{
				currState.speed = m_CmdParams.targetVelocity;
				currState.steer = m_CmdParams.targetSteer;
				if(currState.speed < 0.15)
					currState.shift = PlannerHNS::SHIFT_POS_NN;
				else
					currState.shift = PlannerHNS::SHIFT_POS_DD;

				cout << m_CmdParams.targetVelocity << ", " << m_CmdParams.targetSteer << ", " << endl;
			}
			else if(m_CmdParams.bVehicleConnect)
			{
				//Read StateData From Control Box
				if(m_pComm && m_pComm->IsAuto())
				{
					currState.steer = m_pComm->GetCurrentSteerAngle();
					currState.speed = m_pComm->GetCurrentSpeed();
					currState.shift = m_pComm->GetCurrentShift();
					cout << "Read Live Car Info .. " << endl;
				}
				else
				{
					currState = m_PrevStepTargetStatus;
					cout << "Read Simulated Car Info .. " << endl;

				}

				//Send status over message to planner
				geometry_msgs::Vector3Stamped curr_status;
				curr_status.header.stamp = ros::Time::now();
				curr_status.vector.x = currState.steer;
				curr_status.vector.y = currState.speed;
				curr_status.vector.z = (int)currState.shift;
				m_current_vehicle_status.publish(curr_status);
			}

			if(m_pComm && !m_pComm->IsAuto())
			{
				m_State.SetSimulatedTargetOdometryReadings(currState.speed, currState.steer, currState.shift);
				m_State.UpdateState(false);
				m_State.LocalizeMe(dt);
				m_CurrentPos = m_State.state;
			}

			if(!m_CmdParams.bTest)
			{
				bool bNewPath = false;
				if(PlannerHNS::PlanningHelpers::CompareTrajectories(m_FollowingTrajectory , m_State.m_Path) == false && m_State.m_Path.size()>0)
				{
					m_FollowingTrajectory = m_State.m_Path;
					bNewPath = true;
					cout << "Path is Updated in the controller .. " << m_State.m_Path.size() << endl;
				}

				targetState = m_PredControl.DoOneStep(dt, currMessage, m_FollowingTrajectory, m_CurrentPos, currState, bNewPath);

				m_FollowPoint  = m_PredControl.m_FollowMePoint;
				m_PerpPoint    = m_PredControl.m_PerpendicularPoint;

				cout << "Target Status (" <<targetState.steer << ", " << targetState.speed
						<< ", " << targetState.shift << ")" << endl;
			}
			else
			{
				targetState = currState;
			}


			if(m_CmdParams.bVehicleConnect) //send directly to ZMP control box
			{
				m_PrevStepTargetStatus = targetState;

				if(m_pComm && m_pComm->IsAuto())
				{
					m_pComm->SetNormalizedSteeringAngle(targetState.steer);
					m_pComm->SetNormalizedSpeed(targetState.speed);
					m_pComm->SetShift(targetState.shift);
					cout << "Sending Data to Control Box (" <<targetState.steer << ", " << targetState.speed
							<< ", " << targetState.shift << ")" << endl;
				}
			}
			else //send to autoware
			{
				cout << "Send Data To Autoware" << endl;
				geometry_msgs::Twist t;
				geometry_msgs::TwistStamped twist;
				std_msgs::Bool wf_stat;
				t.linear.x = targetState.speed;
				t.angular.z = targetState.steer;
				wf_stat.data = true;
				twist.twist = t;
				twist.header.stamp = ros::Time::now();

				m_velocity_publisher.publish(twist);
				m_stat_pub.publish(wf_stat);
			}

			geometry_msgs::Vector3Stamped vel;
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = ros::Time::now();
			vel.header.stamp = ros::Time::now();
			vel.vector.x = targetState.speed;
			pose.pose.position.x = m_CurrentPos.pos.x;
			pose.pose.position.y = m_CurrentPos.pos.y;
			pose.pose.position.z = m_CurrentPos.pos.z;
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_CurrentPos.pos.a));
			cout << "## Position Before Sending "<< m_CurrentPos.pos.ToString() << endl;
			m_simulated_pos_pub.publish(pose);
			m_simulated_velocity_pub.publish(vel);
			displayNextTarget(m_CurrentPos);
		}
		else
		{
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
		}

		loop_rate.sleep();
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
			//ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

void FFSteerControl::displayNextTarget(PlannerHNS::WayPoint target)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "next_target_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = target.pos.x;
  marker.pose.position.y = target.pos.y;
  marker.pose.position.z = target.pos.z;
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  marker.color = green;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.frame_locked = true;
  m_target_pub.publish(marker);
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


}
