#include <ros/ros.h>

// lib
#include <state.hpp>
#include <state_context.hpp>

#include <decision_maker_node.hpp>
#include <autoware_msgs/lamp_cmd.h>

namespace decision_maker
{

void DecisionMakerNode::setupStateCallback(void)
{
  ctx->getStateObject(state_machine::DRIVE_STR_LEFT_STATE)
      ->setUpdateFunc(std::bind(&DecisionMakerNode::updateStateSTR, this, 1));
  ctx->getStateObject(state_machine::DRIVE_STR_RIGHT_STATE)
      ->setUpdateFunc(std::bind(&DecisionMakerNode::updateStateSTR, this, 2));
  ctx->getStateObject(state_machine::DRIVE_STR_STRAIGHT_STATE)
      ->setUpdateFunc(std::bind(&DecisionMakerNode::updateStateSTR, this, 0));
  ctx->getStateObject(state_machine::DRIVE_ACC_STOP_STATE)
       ->setUpdateFunc(std::bind(&DecisionMakerNode::updateStateStop, this, 1));
  ctx->getStateObject(state_machine::DRIVE_ACC_STOP_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateStop, this, 1));
  
  ctx->getStateObject(state_machine::DRIVE_ACC_KEEP_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateKeep, this, 1));
  ctx->getStateObject(state_machine::DRIVE_ACC_ACCELERATION_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateAcc, this, 1));
  ctx->getStateObject(state_machine::DRIVE_ACC_DECELERATION_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateAcc, this, -1));
}


void DecisionMakerNode::updateLaneWaypointsArray(void)
{
	current_stopped_lane_array_ = current_controlled_lane_array_;
	for(auto &lane : current_stopped_lane_array_.lanes)
	{

		for(auto &wp: lane.waypoints){
			wp.twist.twist.linear.x = 0.0;
			wp.wpstate.stopline_state = 0;
		}
	}
}

void DecisionMakerNode::publishControlledLaneArray(void)
{
  Pubs["lane_waypoints_array"].publish(current_controlled_lane_array_);
}
void DecisionMakerNode::publishStoppedLaneArray(void)
{
  updateLaneWaypointsArray();
  Pubs["lane_waypoints_array"].publish(current_stopped_lane_array_);
}

void DecisionMakerNode::changeVelocityBasedLane(void)
{
	current_controlled_lane_array_ = current_based_lane_array_;
}

void DecisionMakerNode::changeVelocityLane(int dir)
{
	for (auto &lane : current_controlled_lane_array_.lanes)
	{
		autoware_msgs::lane temp_lane = lane;
		for (size_t wpi = 1; wpi < lane.waypoints.size(); wpi++)
		{
			double v0 = temp_lane.waypoints.at(wpi - 1).twist.twist.linear.x;
			double v = temp_lane.waypoints.at(wpi).twist.twist.linear.x;

			amathutils::point p0(temp_lane.waypoints.at(wpi).pose.pose.position.x,
					temp_lane.waypoints.at(wpi).pose.pose.position.y,
					temp_lane.waypoints.at(wpi).pose.pose.position.z);
			amathutils::point p1(temp_lane.waypoints.at(wpi - 1).pose.pose.position.x,
					temp_lane.waypoints.at(wpi - 1).pose.pose.position.y,
					temp_lane.waypoints.at(wpi - 1).pose.pose.position.z);

			double distance = amathutils::find_distance(&p0, &p1);
			double _weight = distance * 0.05 * dir;
			lane.waypoints.at(wpi).twist.twist.linear.x =
				lane.waypoints.at(wpi).twist.twist.linear.x + lane.waypoints.at(wpi).twist.twist.linear.x * _weight;
		}
	}
}

void DecisionMakerNode::changedStateKeep(int status)
{
	changeVelocityBasedLane();
	publishControlledLaneArray();
}

void DecisionMakerNode::changedStateAcc(int status)
{
	changeVelocityLane(status);
	publishControlledLaneArray();
}
void DecisionMakerNode::updateStateStop(int status)
{
	static ros::Timer stopping_timer;
	static bool timerflag = false;
	if(current_velocity_ == 0.0 && !timerflag){
		stopping_timer = nh_.createTimer(ros::Duration(1), [&](const ros::TimerEvent&){ctx->setCurrentState(state_machine::DRIVE_ACC_KEEP_STATE); ROS_INFO("Change state to keep from stop\n");timerflag=false; }, this, true);
		timerflag = true;
	}

}

void DecisionMakerNode::changedStateStop(int status)
{
	publishStoppedLaneArray();
}

void DecisionMakerNode::updateStateSTR(int status)
{
  autoware_msgs::lamp_cmd lamp_msg;

  switch(status){
	  case LAMP_LEFT:
		  lamp_msg.l = LAMP_ON;
		  lamp_msg.r = LAMP_OFF;
		  break;
	  case LAMP_RIGHT:
		  lamp_msg.l = LAMP_OFF;
		  lamp_msg.r = LAMP_ON;
		  break;
	  case LAMP_HAZARD:
		  lamp_msg.l = LAMP_ON;
		  lamp_msg.r = LAMP_ON;
		  break;
	  case LAMP_EMPTY:
	  default:
		  lamp_msg.l = LAMP_OFF;
		  lamp_msg.r = LAMP_OFF;
		  break;
  }
  Pubs["lamp_cmd"].publish(lamp_msg);
}
}
