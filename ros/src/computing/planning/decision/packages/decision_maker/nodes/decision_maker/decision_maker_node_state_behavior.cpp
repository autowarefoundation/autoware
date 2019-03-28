#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::updateStoppingState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateBehaviorEmergencyState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_HAZARD);
}
void DecisionMakerNode::exitBehaviorEmergencyState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_CLEAR);
}

void DecisionMakerNode::updateMovingState(cstring_t& state_name, int status)
{
  if (isVehicleOnLaneArea())
  {
    tryNextState("on_lane_area");
  }
  else
  {
    tryNextState("on_free_area");
  }
}

uint8_t DecisionMakerNode::getSteeringStateFromWaypoint(void)
{
  static const double distance_to_target = param_num_of_steer_behind_;
  static const size_t ignore_idx = 0;

  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;
  uint8_t state = 0;

  if (ignore_idx > current_status_.finalwaypoints.waypoints.size())
  {
    return 0;
  }

  for (unsigned int idx = 0; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);

    state = current_status_.finalwaypoints.waypoints.at(idx).wpstate.steering_state;

    if (state && (state != autoware_msgs::WaypointState::STR_STRAIGHT || distance >= distance_to_target))
    {
      break;
    }
    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }
  return state;
}
uint8_t DecisionMakerNode::getEventStateFromWaypoint(void)
{
  static const double distance_to_target = param_num_of_steer_behind_;
  static const size_t ignore_idx = 0;

  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;
  uint8_t state = 0;

  if (ignore_idx > current_status_.finalwaypoints.waypoints.size())
  {
    return 0;
  }

  for (unsigned int idx = 0; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);

    state = current_status_.finalwaypoints.waypoints.at(idx).wpstate.event_state;

    if (state || distance >= distance_to_target)
    {
      break;
    }
    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }
  return state;
}

void DecisionMakerNode::updateFreeAreaState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateLaneAreaState(cstring_t& state_name, int status)
{
  switch (getEventStateFromWaypoint())
  {
    case autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP:
      tryNextState("on_bus_stop");
      break;
    default:
      tryNextState("on_cruise");
      break;
  }
}

void DecisionMakerNode::updateCruiseState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_back_state_waypoint"))
  {
    tryNextState("on_back");
    return;
  }

  if (current_status_.change_flag == enumToInteger<E_ChangeFlags>(E_ChangeFlags::LEFT))
  {
    tryNextState("lane_change_left");
  }
  else if (current_status_.change_flag == enumToInteger<E_ChangeFlags>(E_ChangeFlags::RIGHT))
  {
    tryNextState("lane_change_right");
  }
  else
  {
    switch (getSteeringStateFromWaypoint())
    {
      case autoware_msgs::WaypointState::STR_LEFT:
        tryNextState("on_left_turn");
        break;
      case autoware_msgs::WaypointState::STR_RIGHT:
        tryNextState("on_right_turn");
        break;
      case autoware_msgs::WaypointState::STR_STRAIGHT:
        tryNextState("on_straight");
        break;
      default:
        break;
    }
  }

}

void DecisionMakerNode::entryTurnState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateLeftTurnState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_LEFT);
}
void DecisionMakerNode::updateRightTurnState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_RIGHT);
}
void DecisionMakerNode::updateStraightState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_CLEAR);
}
void DecisionMakerNode::updateBackState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_CLEAR);
}

void DecisionMakerNode::entryLaneChangeState(cstring_t& state_name, int status)
{
  tryNextState("check_target_lane");
}
void DecisionMakerNode::updateLeftLaneChangeState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_LEFT);
}
void DecisionMakerNode::updateCheckLeftLaneState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateChangeToLeftState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateRightLaneChangeState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_RIGHT);
}
void DecisionMakerNode::updateCheckRightLaneState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateChangeToRightState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateBusStopState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updatePullInState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_LEFT);
}
void DecisionMakerNode::updatePullOutState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_RIGHT);
}

void DecisionMakerNode::updateParkingState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_HAZARD);
}

}
