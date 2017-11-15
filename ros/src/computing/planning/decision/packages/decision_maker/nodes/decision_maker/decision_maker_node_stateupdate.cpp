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
       ->setUpdateFunc(std::bind(&DecisionMakerNode::updateStateStop, this, 1));
}

void DecisionMakerNode::updateStateStop(int status){


}

void DecisionMakerNode::updateStateSTR(int status)
{
  ROS_INFO("[%s]:%d\n", __func__, status);
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
