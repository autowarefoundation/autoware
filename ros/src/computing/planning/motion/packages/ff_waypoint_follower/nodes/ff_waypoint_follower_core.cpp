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
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "geo_pos_conv.hh"
#include "UtilityH.h"
#include "math.h"


namespace FFSteerControlNS
{
#define kmlTemplateFile "/home/hatem/workspace/Data/templates/PlannerX_MapTemplate.kml"

FFSteerControl::FFSteerControl(const ControlCommandParams& params)
{
	clock_gettime(0, &m_Timer);

	ReadParamFromLaunchFile(m_CarInfo, m_ControlParams);

	m_PredControl.Init(m_ControlParams, m_CarInfo);
	m_State.Init(m_CarInfo);

	m_CmdParams = params;

	m_pComm = 0;
	m_counter = 0;
	m_frequency = 0;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bNewTrajectory = false;
	bNewVelocity = false;
	bNewBehaviorState = false;
	bInitPos = false;


	if(m_CmdParams.statusSource == CONTROL_BOX_STATUS)
	{
		m_pComm = new HevComm();
		m_pComm->InitializeComm(m_CarInfo);
		m_pComm->StartComm();
	}

	if(m_CmdParams.statusSource == AUTOWARE_STATUS)
	{
		tf::StampedTransform transform;
		GetTransformFromTF("map", "world", transform);
		ROS_INFO("Origin : x=%f, y=%f, z=%f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());

		m_OriginPos.position.x  = transform.getOrigin().x();
		m_OriginPos.position.y  = transform.getOrigin().y();
		m_OriginPos.position.z  = transform.getOrigin().z();
	}

	//m_PositionPublisher = nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 10);
	//m_PathPublisherRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_mark", 10, true);
	m_velocity_publisher 		= nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 100);
	m_stat_pub 					= nh.advertise<std_msgs::Bool>("wf_stat", 100);

	m_curr_pos_pub				= nh.advertise<visualization_msgs::Marker>("curr_simu_pose", 100);
	m_perp_pos_pub				= nh.advertise<visualization_msgs::Marker>("perp_pose", 100);
	m_follow_pos_pub			= nh.advertise<visualization_msgs::Marker>("follow_pose", 100);

	m_simulated_pos_pub 		= nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 100);
	m_autoware_pos_pub			= nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 100);
  	//m_simulated_velocity_pub 	= nh.advertise<geometry_msgs::Vector3Stamped>("sim_velocity", 100);
  	m_current_vehicle_status	= nh.advertise<geometry_msgs::Vector3Stamped>("ff_vehicle_status", 100);
  	m_segway_rpm_cmd			= nh.advertise<geometry_msgs::Twist>("joy_action", 100);



	// define subscribers.
	sub_current_pose 		= nh.subscribe("/current_pose", 				100,
			&FFSteerControl::callbackFromCurrentPose, 		this);
	sub_current_trajectory 	= nh.subscribe("final_waypoints", 				100,
			&FFSteerControl::callbackFromCurrentTrajectory, this);
//	sub_twist_velocity		= nh.subscribe("current_velocity", 				100,
//			&FFSteerControl::callbackFromVector3Stamped, 	this);
	initialpose_subscriber 	= nh.subscribe("initialpose", 					100,
			&FFSteerControl::callbackSimuInitPose, 			this);
	sub_behavior_state 	= nh.subscribe("current_behavior",					10,
			&FFSteerControl::callbackFromBehaviorState, 	this);
	sub_segway_rpm_odom		= nh.subscribe("/segway_rmp_node/odom",			100,
			&FFSteerControl::callbackFromSegwayRPM, 		this);


	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

	std::cout << "ff_waypoint_follower initialized successfully " << std::endl;

}

void FFSteerControl::ReadParamFromLaunchFile(SimulationNS::CAR_BASIC_INFO& m_CarInfo,
		  SimulationNS::ControllerParams& m_ControlParams)
{
	nh.getParam("/ff_waypoint_follower/width", 			m_CarInfo.width );
	nh.getParam("/ff_waypoint_follower/length", 		m_CarInfo.length );
	nh.getParam("/ff_waypoint_follower/wheelBaseLength", m_CarInfo.wheel_base );
	nh.getParam("/ff_waypoint_follower/turningRadius", m_CarInfo.turning_radius );
	nh.getParam("/ff_waypoint_follower/maxSteerAngle", m_CarInfo.max_steer_angle );

	nh.getParam("/ff_waypoint_follower/maxSteerValue", m_CarInfo.max_steer_value );
	nh.getParam("/ff_waypoint_follower/minSteerValue", m_CarInfo.min_steer_value );
	nh.getParam("/ff_waypoint_follower/maxVelocity", m_CarInfo.max_speed_forward );
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

}

FFSteerControl::~FFSteerControl()
{
	if(m_pComm)
		delete m_pComm;
}

//void FFSteerControl::callbackFromVector3Stamped(const geometry_msgs::Vector3StampedConstPtr &msg)
//{
//	if(m_pComm && m_pComm->IsAuto() && m_CmdParams.iLocalizationSource == 0)
//	{
//		m_CurrentVelocity = msg->vector.x;
//		cout << "### Current Velocity CallBaclk -> " << m_CurrentVelocity << endl;
//		bNewVelocity = true;
//	}
//}

void FFSteerControl::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // write procedure for current pose

	//ROS_INFO("Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, m_frequency);

	if(m_CmdParams.statusSource != SIMULATION_STATUS )
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

		//cout << "### Current Pose CallBaclk -> " << m_CurrentPos.pos.ToString() << endl;

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
	cout << "New Behavior: " << m_CurrentBehavior.maxVelocity << ", " << m_CurrentBehavior.state << endl;
	bNewBehaviorState = true;
}

void FFSteerControl::callbackFromSegwayRPM(const nav_msgs::OdometryConstPtr& msg)
{
	if(m_CmdParams.statusSource == SEGWAY_STATUS) //segway odometry
	{
		m_CurrentSegwayPos = PlannerHNS::WayPoint(msg->pose.pose.position.x,
				msg->pose.pose.position.y,msg->pose.pose.position.z , tf::getYaw(msg->pose.pose.orientation));

		bNewCurrentSegwayPos = true;

		m_CurrVehicleStatus.shift = PlannerHNS::SHIFT_POS_DD;
		m_CurrVehicleStatus.speed = msg->twist.twist.linear.x;
		m_CurrVehicleStatus.steer = msg->twist.twist.angular.z;

//		m_segway_status.vector.x = msg->twist.twist.angular.z;
//		m_segway_status.vector.y = msg->twist.twist.linear.x;
//		m_segway_status.vector.z = (int)PlannerHNS::SHIFT_POS_DD;

		//std::cout << "### Current Status From Segway Odometry -> (" <<  m_CurrVehicleStatus.speed << ", " << m_CurrVehicleStatus.steer << ")"  << std::endl;
	}
//	else
//	{
//		PlannerHNS::WayPoint p(msg->pose.pose.position.x,
//						msg->pose.pose.position.y,msg->pose.pose.position.z , tf::getYaw(msg->pose.pose.orientation));
//		std::cout << "### Current Pose From Segway Odometry (Ignored) -> " << p.pos.ToString() << std::endl;
//	}
}

void FFSteerControl::PlannerMainLoop()
{

	ros::Rate loop_rate(100);
	if(m_pComm)
		m_pComm->GoLive(true);

	vector<PlannerHNS::WayPoint> path;
	PlannerHNS::WayPoint p2;
	double totalDistance = 0;

	while (ros::ok())
	{
		ros::spinOnce();

		PlannerHNS::BehaviorState currMessage = m_CurrentBehavior;

		if(currMessage.state != PlannerHNS::INITIAL_STATE &&  (bInitPos || bNewCurrentPos))
		{
			double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

			/**
			 * Localization and Status Reading Part
			 * -----------------------------------------------------------------------------------
			 */
			if(m_CmdParams.statusSource == CONTROL_BOX_STATUS)
			{
				//Read StateData From Control Box
				if(m_pComm && m_pComm->IsAuto())
				{
					m_CurrVehicleStatus.steer = m_pComm->GetCurrentSteerAngle();
					m_CurrVehicleStatus.speed = m_pComm->GetCurrentSpeed();
					m_CurrVehicleStatus.shift = m_pComm->GetCurrentShift();

					//Send status over message to planner
					geometry_msgs::Vector3Stamped vehicle_status;
					vehicle_status.header.stamp = ros::Time::now();
					vehicle_status.vector.x = m_CurrVehicleStatus.steer;
					vehicle_status.vector.y = m_CurrVehicleStatus.speed;
					vehicle_status.vector.z = (int)m_CurrVehicleStatus.shift;
					m_current_vehicle_status.publish(vehicle_status);

					cout << "Read Live Car Info .. " << endl;
				}
				else
				{
					cout << ">>> Error, Disconnected from Car Control Box !" << endl;
				}
			}
			else if(m_CmdParams.statusSource == SIMULATION_STATUS)
			{
				m_CurrVehicleStatus = m_PrevStepTargetStatus;
				m_State.SetSimulatedTargetOdometryReadings(m_CurrVehicleStatus.speed, m_CurrVehicleStatus.steer, m_CurrVehicleStatus.shift);
				m_State.UpdateState(false);
				m_State.LocalizeMe(dt);
				m_CurrentPos = m_State.state;

				geometry_msgs::Vector3Stamped vehicle_status;
				vehicle_status.header.stamp = ros::Time::now();
				vehicle_status.vector.x = m_CurrVehicleStatus.steer;
				vehicle_status.vector.y = m_CurrVehicleStatus.speed;
				vehicle_status.vector.z = (int)m_CurrVehicleStatus.shift;
				m_current_vehicle_status.publish(vehicle_status);

				geometry_msgs::PoseStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.pose.position.x = m_CurrentPos.pos.x;
				pose.pose.position.y = m_CurrentPos.pos.y;
				pose.pose.position.z = m_CurrentPos.pos.z;
				pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_CurrentPos.pos.a));
				cout << "Send Simulated Position "<< m_CurrentPos.pos.ToString() << endl;
				m_simulated_pos_pub.publish(pose);
			}
			else if(m_CmdParams.statusSource == AUTOWARE_STATUS)
			{
				m_CurrVehicleStatus = m_PrevStepTargetStatus;
				m_State.SetSimulatedTargetOdometryReadings(m_CurrVehicleStatus.speed, m_CurrVehicleStatus.steer, m_CurrVehicleStatus.shift);
				m_State.UpdateState(false);
				m_State.LocalizeMe(dt);
				m_CurrentPos = m_State.state;

				geometry_msgs::Vector3Stamped vehicle_status;
				vehicle_status.header.stamp = ros::Time::now();
				vehicle_status.vector.x = m_CurrVehicleStatus.steer;
				vehicle_status.vector.y = m_CurrVehicleStatus.speed;
				vehicle_status.vector.z = (int)m_CurrVehicleStatus.shift;
				m_current_vehicle_status.publish(vehicle_status);

				geometry_msgs::PoseStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.pose.position.x = m_CurrentPos.pos.x;
				pose.pose.position.y = m_CurrentPos.pos.y;
				pose.pose.position.z = m_CurrentPos.pos.z;
				pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_CurrentPos.pos.a));
				cout << "Send Simulated Position "<< m_CurrentPos.pos.ToString() << endl;
				m_autoware_pos_pub.publish(pose);
			}
			else if(m_CmdParams.statusSource == SEGWAY_STATUS)
			{
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
				cout << "Path is Updated in the controller .. " << m_State.m_Path.size() << endl;
			}

			m_PrevStepTargetStatus = m_PredControl.DoOneStep(dt, currMessage, m_FollowingTrajectory, m_CurrentPos, m_CurrVehicleStatus, bNewPath);
			//m_PrevStepTargetStatus.speed = 3.0;
			m_State.state.pos.z = m_PerpPoint.pos.z;
			m_FollowPoint  = m_PredControl.m_FollowMePoint;
			m_PerpPoint    = m_PredControl.m_PerpendicularPoint;

			cout << "Target Status (" <<m_PrevStepTargetStatus.steer << ", " << m_PrevStepTargetStatus.speed
					<< ", " << m_PrevStepTargetStatus.shift << ")" << endl;

			//----------------------------------------------------------------------------------------------//



			if(m_CmdParams.statusSource == CONTROL_BOX_STATUS) //send directly to ZMP control box
			{
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
			}
			else if (m_CmdParams.statusSource == AUTOWARE_STATUS)//send to autoware
			{
				cout << "Send Data To Autoware" << endl;
				geometry_msgs::Twist t;
				geometry_msgs::TwistStamped twist;
				std_msgs::Bool wf_stat;
				t.linear.x = m_PrevStepTargetStatus.speed;
				t.angular.z = m_PrevStepTargetStatus.steer;
				wf_stat.data = true;
				twist.twist = t;
				twist.header.stamp = ros::Time::now();

				m_velocity_publisher.publish(twist);
				m_stat_pub.publish(wf_stat);
			}
			else if(m_CmdParams.statusSource == SEGWAY_STATUS)
			{
				cout << "Send Data To Segway" << endl;
				geometry_msgs::Twist t;
				t.linear.x = m_PrevStepTargetStatus.speed;
				if(t.linear.x > 0.5)
					t.linear.x = 0.5;
				t.angular.z = m_PrevStepTargetStatus.steer;
				m_segway_rpm_cmd.publish(t);
			}
			else if(m_CmdParams.statusSource == SIMULATION_STATUS)
			{
			}

			displayFollowingInfo(m_CurrentPos, m_PerpPoint, m_FollowPoint);
		}
		else
		{
			UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
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

			if(totalDistance > m_CmdParams.recordDistance)
			{
				PlannerHNS::RoadNetwork roadMap;
				PlannerHNS::RoadSegment segment;

				segment.id = 1;


				PlannerHNS::Lane lane;
				lane.id = 1;
				lane.num = 0;
				lane.roadId = 1;
				lane.points = path;

				segment.Lanes.push_back(lane);
				roadMap.roadSegments.push_back(segment);

				ostringstream fileName;
				fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName;
				fileName << UtilityHNS:: UtilityH::GetFilePrefixHourMinuteSeconds();
				fileName << "_RoadNetwork.kml";
				string kml_templateFilePath = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::KmlMapsFolderName+"PlannerX_MapTemplate.kml";

				PlannerHNS::MappingHelpers::WriteKML(fileName.str(),kml_templateFilePath , roadMap);

										//string kml_fileToSave =UtilityH::GetHomeDirectory()+DataRW::LoggingMainfolderName + DataRW::KmlMapsFolderName+kmltargetFile;
					//PlannerHNS::MappingHelpers::WriteKML(kml_fileToSave, kml_templateFilePath, m_RoadMap);

				std::cout << " Mapped Saved Successfuly ... " << std::endl;
				break;
			}
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
  m_curr_pos_pub.publish(m1);

  m2.header.frame_id = "map";
  m2.header.stamp = ros::Time();
  m2.ns = "perp_pose";
  m2.type = visualization_msgs::Marker::SPHERE;
  m2.action = visualization_msgs::Marker::ADD;
  m2.pose.position.x = perp_pose.pos.x;
  m2.pose.position.y = perp_pose.pos.y;
  m2.pose.position.z = perp_pose.pos.z;
  m2.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(perp_pose.pos.a));
  std_msgs::ColorRGBA blue;
  blue.a = 1.0;
  blue.b = 1.0;
  blue.r = 0.0;
  blue.g = 0.0;
  m2.color = blue;
  m2.scale.x = 0.75;
  m2.scale.y = 0.75;
  m2.scale.z = 0.75;
  m2.frame_locked = true;
  //m_perp_pos_pub.publish(m2);

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
  m_follow_pos_pub.publish(m3);
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
