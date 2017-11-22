#include <ros/ros.h>

// lib
#include <state.hpp>
#include <state_context.hpp>

#include <autoware_msgs/lamp_cmd.h>
#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::setupStateCallback(void)
{
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_STR_LEFT_STATE, std::bind(&DecisionMakerNode::updateStateSTR, this, 1));
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_STR_RIGHT_STATE, std::bind(&DecisionMakerNode::updateStateSTR, this, 2));
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_STR_STRAIGHT_STATE, std::bind(&DecisionMakerNode::updateStateSTR, this, 0));
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_ACC_STOPLINE_STATE, std::bind(&DecisionMakerNode::updateStateStop, this, 1));
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_ACC_STOP_STATE, std::bind(&DecisionMakerNode::updateStateStop, this, 0));

  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_STOPLINE_STATE,
                      std::bind(&DecisionMakerNode::callbackInStateStop, this, 1));
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_STOP_STATE, std::bind(&DecisionMakerNode::callbackInStateStop, this, 0));
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_KEEP_STATE, std::bind(&DecisionMakerNode::callbackInStateKeep, this, 1));
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_ACCELERATION_STATE,
                      std::bind(&DecisionMakerNode::callbackInStateAcc, this, 1));
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_DECELERATION_STATE,
                      std::bind(&DecisionMakerNode::callbackInStateAcc, this, -1));

  ctx->setCallbackUpdateFunc(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE,
                      std::bind(&DecisionMakerNode::updateStateObstacleAvoid, this, -1));
  ctx->setCallbackInFunc(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE,
                      std::bind(&DecisionMakerNode::callbackInStateObstacleAvoid, this, -1));
  ctx->setCallbackOutFunc(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE,
                      std::bind(&DecisionMakerNode::callbackOutStateObstacleAvoid, this, 1));
}

void DecisionMakerNode::createShiftLane(void)
{
  if(!created_shift_lane_flag_)
  {
	  created_shift_lane_flag_ = true;
	  autoware_msgs::lane shift_lane;
	  current_shifted_lane_array_ = current_based_lane_array_;
	  if (!current_shifted_lane_array_.lanes.empty())
	  {
		  shift_lane = current_shifted_lane_array_.lanes.at(0);
		  size_t idx = 0;
		  for (auto &wp : shift_lane.waypoints)
		  {
			  double current_angle = getPoseAngle(wp.pose.pose);
			  wp.pose.pose.position.x -= param_shift_width_ * cos(current_angle + M_PI / 2);
			  wp.pose.pose.position.y -= param_shift_width_ * sin(current_angle + M_PI / 2);
			  wp.change_flag = current_based_lane_array_.lanes.at(0).waypoints.at(idx++).change_flag;
		  }
		  auto it = current_shifted_lane_array_.lanes.insert(current_shifted_lane_array_.lanes.begin() + 1, shift_lane);

		  for (auto &wp : current_shifted_lane_array_.lanes.at(0).waypoints)
		  {
			  wp.change_flag = 1;
		  }
	  }
  }
}

void DecisionMakerNode::changeShiftLane(void){
	try{
		for(size_t idx = 0; idx < current_shifted_lane_array_.lanes.size(); idx+=2){
			for(size_t wp_idx; wp_idx < current_shifted_lane_array_.lanes.at(idx).waypoints.size(); wp_idx++){
				current_shifted_lane_array_.lanes.at(idx).waypoints.at(wp_idx).change_flag = current_shifted_lane_array_.lanes.at(idx+1).waypoints.at(wp_idx).change_flag;
				current_shifted_lane_array_.lanes.at(idx+1).waypoints.at(wp_idx).change_flag = 2;
			}
		}
	}
	catch(std::out_of_range){
		fprintf(stderr,"out\n");
	}
}

void DecisionMakerNode::removeShiftLane(void){
	current_shifted_lane_array_ = current_based_lane_array_;
	created_shift_lane_flag_ = false;
#if 0
	if(created_shift_lane_flag_){
		if(!current_shifted_lane_array_.lanes.size()>=2){
			for(auto it = begin(current_shifted_lane_array_.lanes)+1; it!=end(current_shifted_lane_array_.lanes); it+=2)
				current_shifted_lane_array_.lanes.erase(it);
		}
		created_shift_lane_flag_ = false;
	}
#endif
}

void DecisionMakerNode::updateLaneWaypointsArray(void)
{
  // current_stopped_lane_array_ = current_controlled_lane_array_;
  current_stopped_lane_array_ = current_controlled_lane_array_;

  for (auto &lane : current_stopped_lane_array_.lanes)
  {
    for (auto &wp : lane.waypoints)
    {
      wp.twist.twist.linear.x = 0.0;
      wp.wpstate.stopline_state = 0;
    }
  }
}

void DecisionMakerNode::publishControlledLaneArray(void)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  Pubs["lane_waypoints_array"].publish(current_controlled_lane_array_);
}
void DecisionMakerNode::publishStoppedLaneArray(void)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  updateLaneWaypointsArray();
  Pubs["lane_waypoints_array"].publish(current_stopped_lane_array_);
}

void DecisionMakerNode::changeVelocityBasedLane(void)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  current_controlled_lane_array_ = current_shifted_lane_array_;
}

void DecisionMakerNode::changeVelocityLane(int dir)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
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

void DecisionMakerNode::callbackInStateKeep(int status)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  changeVelocityBasedLane();
  publishControlledLaneArray();
}

void DecisionMakerNode::callbackInStateAcc(int status)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  changeVelocityLane(status);
  publishControlledLaneArray();
}
void DecisionMakerNode::updateStateStop(int status)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  static ros::Timer stopping_timer;
  static bool timerflag = false;
  if (status)
  {
    if (current_velocity_ == 0.0 && !timerflag)
    {
      stopping_timer = nh_.createTimer(ros::Duration(1),
                                       [&](const ros::TimerEvent &) {
                                         ctx->setCurrentState(state_machine::DRIVE_ACC_KEEP_STATE);
                                         ROS_INFO("Change state to keep from stop\n");
                                         timerflag = false;
                                       },
                                       this, true);
      timerflag = true;
    }
  }
}

void DecisionMakerNode::updateStateObstacleAvoid(int status)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  static ros::Timer avoidance_timer;
  static bool stopped_flag = false;

  if(current_velocity_ == 0.0){
	stopped_flag = true;
  }
}

void DecisionMakerNode::callbackOutStateObstacleAvoid(int status)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
	static ros::Timer avoidance_timer_2;
	ROS_INFO("End of Avoidance\n");
	changeShiftLane();
	changeVelocityBasedLane();
	publishControlledLaneArray();

	ros::Rate loop_rate(1);
	//blocking
	
	
	do{
		ros::spinOnce();
		loop_rate.sleep();
	}
	while(!ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE) &&
				       	!ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE) && ros::ok());
	do{
		ros::spinOnce();
		loop_rate.sleep();
	}
	while((ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE) ||
				       	ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE) ||
				       	ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE) )&& ros::ok() );

	removeShiftLane();
	changeVelocityBasedLane();
	publishControlledLaneArray();
	return;
}

void DecisionMakerNode::callbackInStateObstacleAvoid(int status)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  ctx->setCurrentState(state_machine::DRIVE_ACC_STOPLINE_STATE);
  createShiftLane();
  changeVelocityBasedLane();
}
void DecisionMakerNode::callbackInStateStop(int status)
{
	fprintf(stderr,"[%s]:%d:\n",__func__,__LINE__);
  publishStoppedLaneArray();
}

void DecisionMakerNode::updateStateSTR(int status)
{
  autoware_msgs::lamp_cmd lamp_msg;

  switch (status)
  {
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
