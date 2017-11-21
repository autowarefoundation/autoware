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
  ctx->getStateObject(state_machine::DRIVE_ACC_STOPLINE_STATE)
       ->setUpdateFunc(std::bind(&DecisionMakerNode::updateStateStop, this, 1));
  ctx->getStateObject(state_machine::DRIVE_ACC_STOPLINE_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateStop, this, 1));
  ctx->getStateObject(state_machine::DRIVE_ACC_STOP_STATE)
       ->setUpdateFunc(std::bind(&DecisionMakerNode::updateStateStop, this, 0));
  ctx->getStateObject(state_machine::DRIVE_ACC_STOP_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateStop, this, 0));
  ctx->getStateObject(state_machine::DRIVE_ACC_KEEP_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateKeep, this, 1));
  ctx->getStateObject(state_machine::DRIVE_ACC_ACCELERATION_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateAcc, this, 1));
  ctx->getStateObject(state_machine::DRIVE_ACC_DECELERATION_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateAcc, this, -1));
 
  ctx->getStateObject(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE)
       ->setChangedFunc(std::bind(&DecisionMakerNode::changedStateObstacleAvoid, this, -1));
}

void DecisionMakerNode::createShiftLane(void)
{
	autoware_msgs::lane shift_lane;
	if(!current_shifted_lane_array_.lanes.empty()){
		shift_lane = current_shifted_lane_array_.lanes.at(0);
		size_t idx = 0;
		for(auto &wp : shift_lane.waypoints){
			double current_angle = getPoseAngle(wp.pose.pose);
			wp.pose.pose.position.x -= param_shift_width_ * cos(current_angle + M_PI / 2);
			wp.pose.pose.position.y -= param_shift_width_ * sin(current_angle + M_PI / 2);
			wp.change_flag = current_based_lane_array.lanes.at(0).waypoints.at(idx++).change_flag;

		}
		auto it = current_shifted_lane_array_.lanes.insert(current_shifted_lane_array_.lanes.begin() + 1, shift_lane);

		for(auto &wp : current_shifted_lane_array_.lanes.at(0).waypoints){
			wp.change_flag = 1;
		}
	}

}

void DecisionMakerNode::updateLaneWaypointsArray(void)
{
	//current_stopped_lane_array_ = current_controlled_lane_array_;
	current_stopped_lane_array_ = current_controlled_lane_array_;
#if 0
	size_t idx	= current_finalwaypoints_.waypoints.at(0).gid;
	for(auto &lane : current_stopped_lane_array_.lanes)
	{
		for(auto &wp: lane.waypoints){
			fprintf(stderr,"%d:%d\n",idx, wp.gid);
			if(idx - 5 <= wp.gid &&  wp.gid  < idx){
				wp.twist.twist.linear.x /= 5 - (idx-wp.gid);
			}else{
				wp.twist.twist.linear.x = 0;
			}
			wp.wpstate.stopline_state = 0;
		}
	}
#else

	for(auto &lane : current_stopped_lane_array_.lanes)
	{
		for(auto &wp: lane.waypoints){
			wp.twist.twist.linear.x = 0.0;
			wp.wpstate.stopline_state = 0;
		}
	}
#endif
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
	current_controlled_lane_array_ = current_shifted_lane_array_;
	//current_controlled_lane_array_ = current_based_lane_array_;
}

void DecisionMakerNode::changeVelocityLane(int dir)
{
	for (auto &lane : current_controlled_lane_array_.lanes)
	{
		autoware_msgs::lane temp_lane = lane;
		for (size_t wpi = 1; wpi < lane.waypoints.size(); wpi++)
		{
			amathutils::point p0(temp_lane.waypoints.at(wpi).pose.pose.position.x,
					temp_lane.waypoints.at(wpi).pose.pose.position.y,
					temp_lane.waypoints.at(wpi).pose.pose.position.z);
			amathutils::point p1(temp_lane.waypoints.at(wpi - 1).pose.pose.position.x,
					temp_lane.waypoints.at(wpi - 1).pose.pose.position.y,
					temp_lane.waypoints.at(wpi - 1).pose.pose.position.z);

			double distance = amathutils::find_distance(&p0, &p1);
			double _weight = distance * 0.05 * dir;
			lane.waypoints.at(wpi).twist.twist.linear.x =
				lane.waypoints.at(wpi).twist.twist.linear.x + (lane.waypoints.at(wpi).twist.twist.linear.x * _weight);
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
	
	static ros::Timer stopping_timer;
	static bool timerflag = false;
	stopping_timer = nh_.createTimer(ros::Duration(1), [&](const ros::TimerEvent&){ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE); ROS_INFO("Change state to null from obstacle avoid\n"); }, this, true);
}
void DecisionMakerNode::updateStateStop(int status)
{
	static ros::Timer stopping_timer;
	static bool timerflag = false;
	if(status){
		if(current_velocity_ == 0.0 && !timerflag){
			stopping_timer = nh_.createTimer(ros::Duration(1), [&](const ros::TimerEvent&){ctx->setCurrentState(state_machine::DRIVE_ACC_KEEP_STATE); ROS_INFO("Change state to keep from stop\n");timerflag=false; }, this, true);
			timerflag = true;
		}
	}
}
void DecisionMakerNode::changedStateObstacleAvoid(int status)
{
	createShiftLane();
	publishControlledLaneArray();
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
