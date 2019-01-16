#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::updateWaitDriveReadyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateWaitEngageState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateMotionEmergencyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::entryDriveState(cstring_t& state_name, int status)
{
  std::pair<uint8_t, int> get_stopsign = getStopSignStateFromWaypoint();
  if (get_stopsign.first != 0)
  {
    current_status_.found_stopsign_idx = get_stopsign.second;
    tryNextState("found_stopline");
    return;
  }
  else if (isEventFlagTrue("entry_stop_state"))
  {
    tryNextState("found_obstacle_in_stopped_area");
    return;
  }

  tryNextState("clear");
}

void DecisionMakerNode::updateDriveState(cstring_t& state_name, int status)
{
  if (isArrivedGoal())
  {
    tryNextState("arrived_goal");
    return;
  }

  if (current_status_.closest_waypoint == -1)
  {
    publishOperatorHelpMessage("The vehicle passed last waypoint or waypoint does not exist near the vehicle.");
    tryNextState("mission_aborted");
    return;
  }

  if (current_status_.finalwaypoints.waypoints.empty())
  {
    ROS_WARN("\"/final_waypoints.\" is not contain waypoints");
    return;
  }
}

std::pair<uint8_t, int> DecisionMakerNode::getStopSignStateFromWaypoint(void)
{
  static const size_t ignore_idx = 0;
  static const double mu = 0.7;  // dry ground/ asphalt/ normal tire
  static const double g = 9.80665;
  static const double margin = 5;
  static const double reaction_time = 0.3 + margin;  // system delay(sec)
  static const size_t reset_count = stopline_reset_count_;
  const double velocity = amathutils::kmph2mps(current_status_.velocity);

  const double free_running_distance = reaction_time * velocity;
  const double braking_distance = velocity * velocity / (2 * g * mu);
  const double distance_to_target = (free_running_distance + braking_distance) * 2 /* safety margin*/;

  std::pair<uint8_t, int> ret(0, -1);

  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;
  uint8_t state = 0;

  if (ignore_idx > current_status_.finalwaypoints.waypoints.size() ||
      3 > current_status_.finalwaypoints.waypoints.size())
  {
    return ret;
  }

  // reset previous stop
  if (current_status_.finalwaypoints.waypoints.at(1).gid > current_status_.prev_stopped_wpidx ||
      (unsigned int)(current_status_.prev_stopped_wpidx - current_status_.finalwaypoints.waypoints.at(1).gid) >
          reset_count)
  {
    current_status_.prev_stopped_wpidx = -1;
  }

  for (unsigned int idx = 0; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);
    state = current_status_.finalwaypoints.waypoints.at(idx).wpstate.stop_state;

    if (state)
    {
      if (current_status_.prev_stopped_wpidx != current_status_.finalwaypoints.waypoints.at(idx).gid)
      {
        ret.first = state;
        ret.second = current_status_.finalwaypoints.waypoints.at(idx).gid;
        break;
      }
    }

    if (distance > distance_to_target)
    {
      break;
    }

    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }
  return ret;
}

void DecisionMakerNode::entryGoState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateGoState(cstring_t& state_name, int status)
{
  std::pair<uint8_t, int> get_stopsign = getStopSignStateFromWaypoint();
  if (get_stopsign.first != 0)
  {
    current_status_.found_stopsign_idx = get_stopsign.second;
    tryNextState("found_stopline");
  }

}

void DecisionMakerNode::updateWaitState(cstring_t& state_name, int status)
{
  publishStoplineWaypointIdx(current_status_.closest_waypoint + 1);
}

void DecisionMakerNode::entryStopState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateStopState(cstring_t& state_name, int status)
{
  publishStoplineWaypointIdx(current_status_.closest_waypoint + 1);
}
void DecisionMakerNode::exitStopState(cstring_t& state_name, int status)
{
  std::pair<uint8_t, int> get_stopsign = getStopSignStateFromWaypoint();
  if (get_stopsign.first != 0)
  {
    current_status_.found_stopsign_idx = get_stopsign.second;
  }
  else
  {
    current_status_.found_stopsign_idx = -1;
    publishStoplineWaypointIdx(current_status_.found_stopsign_idx);
  }
}

void DecisionMakerNode::updateStoplineState(cstring_t& state_name, int status)
{
  std::pair<uint8_t, int> get_stopsign = getStopSignStateFromWaypoint();
  if (get_stopsign.first != 0)
  {
    current_status_.found_stopsign_idx = get_stopsign.second;
  }

  publishStoplineWaypointIdx(current_status_.found_stopsign_idx);
  /* wait for clearing risk*/

  static bool timerflag = false;
  static ros::Timer stopping_timer;

  if (current_status_.velocity == 0.0 && !timerflag)
  {
    stopping_timer = nh_.createTimer(ros::Duration(0.5),
                                     [&](const ros::TimerEvent&) {
                                       timerflag = false;
                                       current_status_.prev_stopped_wpidx = current_status_.found_stopsign_idx;
                                       tryNextState("clear");
                                       /*if found risk,
                                        * tryNextState("found_risk");*/
                                     },
                                     this, true);
    timerflag = true;
  }
}

}
