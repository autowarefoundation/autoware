/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "op_data_logger_core.h"
#include "op_utility/UtilityH.h"
#include "math.h"
#include "op_planner/MatrixOperations.h"
#include "op_ros_helpers/op_ROSHelpers.h"


namespace DataLoggerNS
{

OpenPlannerDataLogger::OpenPlannerDataLogger()
{
	bMap = false;
	m_iSimuCarsNumber = 5;

	std::string first_str, second_str;
	ros::NodeHandle _nh("~");

	int iSource = 0;
	_nh.getParam("mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 1)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("mapFileName" , m_MapPath);

	UtilityHNS::UtilityH::GetTickCount(m_Timer);

	//Subscription for the Ego vehicle !
	sub_predicted_objects = nh.subscribe("/predicted_objects", 1, &OpenPlannerDataLogger::callbackGetPredictedObjects, this);

	sub_behavior_state 		= nh.subscribe("/current_behavior",	10,  &OpenPlannerDataLogger::callbackGetBehaviorState, 	this);

	//Subscriptions for the simulated cars
	VehicleDataContainer vc;
	vc.id = 0;
	m_SimulatedVehicle.push_back(vc);
	for(int i=1; i <= m_iSimuCarsNumber; i++)
	{
		std::ostringstream str_path_beh, str_pose;
		str_path_beh << "simu_car_path_beh_" << i;

		ros::Subscriber _sub_path_beh;
		_sub_path_beh =  nh.subscribe(str_path_beh.str(), 1, &OpenPlannerDataLogger::callbackGetSimuCarsPathAndState, this);

		sub_simu_paths.push_back(_sub_path_beh);

		str_pose << "sim_box_pose_" << i;
		ros::Subscriber _sub;
		_sub =  nh.subscribe(str_pose.str(), 1, &OpenPlannerDataLogger::callbackGetSimuPose, this);
		sub_objs.push_back(_sub);

		vc.id = i;
		m_SimulatedVehicle.push_back(vc);
		m_LogData.push_back(std::vector<std::string>());
	}

	std::cout << "OpenPlannerDataLogger initialized successfully " << std::endl;
}

OpenPlannerDataLogger::~OpenPlannerDataLogger()
{
	for(int i=0; i < m_iSimuCarsNumber; i++)
	{
		ostringstream car_name;
		car_name << "sim_car_no_" << i+1;
		car_name << "_";
		UtilityHNS::DataRW::WriteLogData(UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::PredictionFolderName,
				car_name.str(),
				"time_diff,distance_diff, heading_diff, velocity_diff, rms, state_diff," , m_LogData.at(i));
	}
}

void OpenPlannerDataLogger::callbackGetSimuPose(const geometry_msgs::PoseArray& msg)
{

	for(unsigned int i=0; i < m_SimulatedVehicle.size(); i++)
	{
		if(msg.poses.size() > 3 )
		{
			if(m_SimulatedVehicle.at(i).id == msg.poses.at(0).position.x)
			{
				//std::cout << "Receive SimuPOse Data ... " << msg.poses.at(0).position.x << ", " << msg.header.stamp <<  std::endl;

				m_SimulatedVehicle.at(i).pose.v = msg.poses.at(0).position.y;
				m_SimulatedVehicle.at(i).pose.pos.x = msg.poses.at(1).position.x;
				m_SimulatedVehicle.at(i).pose.pos.y = msg.poses.at(1).position.y;
				m_SimulatedVehicle.at(i).pose.pos.z = msg.poses.at(1).position.z;
				m_SimulatedVehicle.at(i).pose.pos.a = tf::getYaw(msg.poses.at(1).orientation);

				if(msg.poses.at(3).orientation.w == 0)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_LEFT;
				else if(msg.poses.at(3).orientation.w == 1)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_RIGHT;
				else if(msg.poses.at(3).orientation.w == 2)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_BOTH;
				else if(msg.poses.at(3).orientation.w == 3)
					m_SimulatedVehicle.at(i).beh.indicator = PlannerHNS::INDICATOR_NONE;

				m_SimulatedVehicle.at(i).pose_time = msg.header.stamp;

				break;
			}
		}
	}
}

void OpenPlannerDataLogger::callbackGetSimuCarsPathAndState(const autoware_msgs::LaneConstPtr& msg )
{
	//std::cout << "Receive Lane Data ... " << std::endl;

	for(unsigned int i=0; i < m_SimulatedVehicle.size(); i++)
	{
		if(m_SimulatedVehicle.at(i).id == msg->lane_id)
		{
			m_SimulatedVehicle.at(i).path.clear();
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(*msg, m_SimulatedVehicle.at(i).path);
			m_SimulatedVehicle.at(i).beh.state = GetStateFromNumber(msg->lane_index);
			m_SimulatedVehicle.at(i).path_time = msg->header.stamp;
			break;
		}
	}
}

//Functions related to Ego Vehicle Data

void OpenPlannerDataLogger::callbackGetPredictedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	m_PredictedObjects.clear();
	PlannerHNS::DetectedObject obj;

	//std::cout << "Receive Prediction Data From Ego Vehicle... " << msg->header.stamp <<  std::endl;
	m_pred_time = msg->header.stamp;

	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		if(msg->objects.at(i).id > 0)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);


			obj.behavior_state = GetBehStateFromNumber(msg->objects.at(i).behavior_state);
			int i_best_trajectory = -1;
			double max_cost = 0;
			for(unsigned int k=0; k < obj.predTrajectories.size(); k++)
			{
				if(obj.predTrajectories.at(k).size() > 0)
				{
					//std::cout << "Fins Max Traj Cost .............. " << obj.predTrajectories.at(k).at(0).collisionCost  << std::endl;

					if(obj.predTrajectories.at(k).at(0).collisionCost == 1)
					{

						max_cost = obj.predTrajectories.at(k).at(0).collisionCost;
						i_best_trajectory = k;
					}
				}
			}

			if(i_best_trajectory >= 0 &&  i_best_trajectory < obj.predTrajectories.size())
			{
				std::vector<PlannerHNS::WayPoint> pred_traj = obj.predTrajectories.at(i_best_trajectory);
				obj.predTrajectories.clear();
				obj.predTrajectories.push_back(pred_traj);
			}
			else
				obj.predTrajectories.clear();

			m_PredictedObjects.push_back(obj);
		}
	}
}

void OpenPlannerDataLogger::callbackGetBehaviorState(const geometry_msgs::TwistStampedConstPtr& msg )
{
	m_CurrentBehavior = ConvertBehaviorStateFromAutowareToPlannerH(msg);
//	std::cout << "Receive Behavior Data From Ego Vehicle... " << msg->header.stamp <<  std::endl;
}

PlannerHNS::BehaviorState OpenPlannerDataLogger::ConvertBehaviorStateFromAutowareToPlannerH(const geometry_msgs::TwistStampedConstPtr& msg)
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

	behavior.state = GetStateFromNumber(msg->twist.angular.z);


	return behavior;

}

PlannerHNS::STATE_TYPE OpenPlannerDataLogger::GetStateFromNumber(const int& iBehState)
{
	PlannerHNS::STATE_TYPE _state;
	if(iBehState == PlannerHNS::INITIAL_STATE)
		_state = PlannerHNS::INITIAL_STATE;
	else if(iBehState == PlannerHNS::WAITING_STATE)
		_state = PlannerHNS::WAITING_STATE;
	else if(iBehState == PlannerHNS::FORWARD_STATE)
		_state = PlannerHNS::FORWARD_STATE;
	else if(iBehState == PlannerHNS::STOPPING_STATE)
		_state = PlannerHNS::STOPPING_STATE;
	else if(iBehState == PlannerHNS::EMERGENCY_STATE)
		_state = PlannerHNS::EMERGENCY_STATE;
	else if(iBehState == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
		_state = PlannerHNS::TRAFFIC_LIGHT_STOP_STATE;
	else if(iBehState == PlannerHNS::STOP_SIGN_STOP_STATE)
		_state = PlannerHNS::STOP_SIGN_STOP_STATE;
	else if(iBehState == PlannerHNS::STOP_SIGN_WAIT_STATE)
		_state = PlannerHNS::STOP_SIGN_WAIT_STATE;
	else if(iBehState == PlannerHNS::FOLLOW_STATE)
		_state = PlannerHNS::FOLLOW_STATE;
	else if(iBehState == PlannerHNS::LANE_CHANGE_STATE)
		_state = PlannerHNS::LANE_CHANGE_STATE;
	else if(iBehState == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
		_state = PlannerHNS::OBSTACLE_AVOIDANCE_STATE;
	else if(iBehState == PlannerHNS::FINISH_STATE)
		_state = PlannerHNS::FINISH_STATE;

	return _state;
}

PlannerHNS::BEH_STATE_TYPE OpenPlannerDataLogger::GetBehStateFromNumber(const int& iBehState)
{
	if(iBehState == 0)
		return PlannerHNS::BEH_FORWARD_STATE;
	else if(iBehState == 1)
		return PlannerHNS::BEH_STOPPING_STATE;
	else if(iBehState == 2)
		return PlannerHNS::BEH_BRANCH_LEFT_STATE;
	else if(iBehState == 3)
		return PlannerHNS::BEH_BRANCH_RIGHT_STATE;
	else if(iBehState == 4)
		return PlannerHNS::BEH_YIELDING_STATE;
	else if(iBehState == 5)
		return PlannerHNS::BEH_ACCELERATING_STATE;
	else if(iBehState == 6)
		return PlannerHNS::BEH_SLOWDOWN_STATE;
	else
		return PlannerHNS::BEH_FORWARD_STATE;
}

void OpenPlannerDataLogger::CompareAndLog(VehicleDataContainer& ground_truth, PlannerHNS::DetectedObject& predicted)
{
	//difference between actual and prediction
	//time diff
	//distance , orientation, velocity  diff
	//RMS diff (make both same density first) , -1 if no prediction
	//behavior state matching (1 if match , zero if not)

	double t_diff = fabs(ground_truth.path_time.toSec() - m_pred_time.toSec());
	double d_diff = hypot(ground_truth.pose.pos.y - predicted.center.pos.y, ground_truth.pose.pos.x - predicted.center.pos.x);
	double o_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(ground_truth.pose.pos.a, predicted.center.pos.a);
	double v_diff = fabs(ground_truth.pose.v - predicted.center.v);
	double rms = -1;
	int beh_state_diff = -1;

	if(predicted.predTrajectories.size() > 0)
	{
		rms = CalculateRMS(ground_truth.path, predicted.predTrajectories.at(0));
	}

	if(ground_truth.beh.state == predicted.behavior_state)
		beh_state_diff = 1;
	else
		beh_state_diff = 0;

	beh_state_diff = predicted.behavior_state;


	std::ostringstream dataLine;
	dataLine << t_diff << "," << d_diff << "," <<  o_diff << "," << v_diff << "," << rms << "," << beh_state_diff << ",";
	m_LogData.at(ground_truth.id -1).push_back(dataLine.str());

	//std::cout << "Predicted Behavior: " << predicted.behavior_state << std::endl;
}

double OpenPlannerDataLogger::CalculateRMS(std::vector<PlannerHNS::WayPoint>& path1, std::vector<PlannerHNS::WayPoint>& path2)
{
	if(path1.size() == 0 || path2.size() == 0) return -1;

	//PlannerHNS::PlanningHelpers::FixPathDensity(path1, 0.1);
	//PlannerHNS::PlanningHelpers::FixPathDensity(path2, 0.1);

	int min_size = path1.size();
	if(path2.size() < min_size)
		min_size = path2.size();

	double rms_sum = 0;
	PlannerHNS::RelativeInfo info;
	for(unsigned int i=0; i < min_size - 1 ; i++)
	{
		PlannerHNS::PlanningHelpers::GetRelativeInfo(path1, path2.at(i), info);
		rms_sum += fabs(info.perp_distance);
		//rms_sum += hypot(path1.at(i).pos.y - path2.at(i).pos.y, path1.at(i).pos.x - path2.at(i).pos.x);
	}

	return rms_sum / (double)min_size;
}

void OpenPlannerDataLogger::MainLoop()
{

	ros::Rate loop_rate(50);

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

		ros::Time t;

		for(unsigned int i=0; i < m_SimulatedVehicle.size(); i++)
		{
			if(m_SimulatedVehicle.at(i).pose_time != t && m_SimulatedVehicle.at(i).path_time != t)
			{
				for(unsigned int j = 0; j < m_PredictedObjects.size(); j++)
				{
					//std::cout << "CarID: " <<  m_SimulatedVehicle.at(i).id << ", PredCarID: " << m_PredictedObjects.at(j).id << std::endl;
					if(m_SimulatedVehicle.at(i).id == m_PredictedObjects.at(j).id)
					{
						CompareAndLog(m_SimulatedVehicle.at(i), m_PredictedObjects.at(j));
					}
				}

				//std::cout << "CarID: " <<  m_SimulatedVehicle.at(i).id << ", PoseTime: " << m_SimulatedVehicle.at(i).pose_time.toSec() << ", PredTime: " << m_pred_time.toSec() << std::endl;
				//std::cout << std::endl;
			}
		}

		loop_rate.sleep();
	}
}

}
