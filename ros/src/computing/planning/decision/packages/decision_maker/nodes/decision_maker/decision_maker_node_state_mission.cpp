#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::entryWaitVehicleReadyState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateWaitVehicleReadyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::entryWaitOrderState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("Please load mission order (waypoints).");
  setEventFlag("received_based_lane_waypoint", false);
  if (!isSubscriberRegistered("lane_waypoints_array"))
  {
    Subs["lane_waypoints_array"] =
        nh_.subscribe(TPNAME_BASED_LANE_WAYPOINTS_ARRAY, 100, &DecisionMakerNode::callbackFromLaneWaypoint, this);
  }
}

void DecisionMakerNode::updateWaitOrderState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_based_lane_waypoint"))
  {
    setEventFlag("received_based_lane_waypoint", false);
    tryNextState("received_mission_order");
  }
}
void DecisionMakerNode::exitWaitOrderState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::entryMissionCheckState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("Received mission, checking now...");
  setEventFlag("received_back_state_waypoint", false);

  int gid = 0;
  for (auto& lane : current_status_.based_lane_array.lanes)
  {
    int lid = 0;
    for (auto& wp : lane.waypoints)
    {
      wp.wpstate.aid = 0;
      wp.wpstate.steering_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.accel_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.stop_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.lanechange_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.event_state = 0;
      wp.gid = gid++;
      wp.lid = lid++;
      if (!isEventFlagTrue("received_back_state_waypoint") && wp.twist.twist.linear.x < 0.0)
      {
        setEventFlag("received_back_state_waypoint", true);
        publishOperatorHelpMessage("Received back waypoint.");
      }
    }
  }

  // waypoint-state set and insert interpolation waypoint for stopline
  setWaypointState(current_status_.based_lane_array);

  // indexing
  gid = 0;
  for (auto& lane : current_status_.based_lane_array.lanes)
  {
    int lid = 0;
    for (auto& wp : lane.waypoints)
    {
      wp.gid = gid++;
      wp.lid = lid++;
    }
  }

  current_status_.using_lane_array = current_status_.based_lane_array;
  Pubs["lane_waypoints_array"].publish(current_status_.using_lane_array);
  if (!isSubscriberRegistered("final_waypoints"))
  {
    Subs["final_waypoints"] =
        nh_.subscribe("final_waypoints", 100, &DecisionMakerNode::callbackFromFinalWaypoint, this);
  }
}
void DecisionMakerNode::updateMissionCheckState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_finalwaypoints") && current_status_.closest_waypoint != -1)
  {
    tryNextState("mission_is_compatible");
  }
}

void DecisionMakerNode::entryMissionAbortedState(cstring_t& state_name, int status)
{
  tryNextState("operation_end");
}
void DecisionMakerNode::updateMissionAbortedState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    sleep(1);
    tryNextState("goto_wait_order");
    return;
  }
}

void DecisionMakerNode::entryDriveReadyState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("Please order to engage");
}

void DecisionMakerNode::updateDriveReadyState(cstring_t& state_name, int status)
{
  if (!use_fms_ && auto_engage_)
  {
    tryNextState("engage");
  }
}

void DecisionMakerNode::entryDrivingState(cstring_t& state_name, int status)
{
  setEventFlag("received_based_lane_waypoint", false);

  if (isEventFlagTrue("emergency_flag"))
  {
    tryNextState("mission_aborted");
    return;
  }

  tryNextState("operation_start");
}
void DecisionMakerNode::updateDrivingState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("emergency_flag"))
  {
    tryNextState("mission_aborted");
  }

  if (!use_fms_ && auto_mission_change_ && isEventFlagTrue("received_based_lane_waypoint"))
  {
    tryNextState("request_mission_change");
  }
}
void DecisionMakerNode::exitDrivingState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::entryDrivingMissionChangeState(cstring_t& state_name, int status)
{
  if (!auto_mission_change_)
    setEventFlag("received_based_lane_waypoint", false);
}

void DecisionMakerNode::updateDrivingMissionChangeState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_based_lane_waypoint"))
  {
    setEventFlag("received_based_lane_waypoint", false);
    if (!drivingMissionCheck())
    {
      publishOperatorHelpMessage("Failed to change the mission.");
      tryNextState("mission_is_conflicting");
      return;
    }
    else
    {
      publishOperatorHelpMessage("Mission change succeeded.");
      tryNextState("mission_is_compatible");
      return;
    }
  }
}

void DecisionMakerNode::updateMissionChangeSucceededState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    sleep(1);
    tryNextState("return_to_driving");
  }
}
void DecisionMakerNode::updateMissionChangeFailedState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    sleep(1);
    tryNextState("return_to_driving");
  }
}

void DecisionMakerNode::entryMissionCompleteState(cstring_t& state_name, int status)
{
  tryNextState("operation_end");
}
void DecisionMakerNode::updateMissionCompleteState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    if (auto_mission_reload_)
    {
      publishOperatorHelpMessage("Reload mission.");
      tryNextState("re_enter_mission");
      return;
    }
    else
    {
      sleep(1);
      tryNextState("goto_wait_order");
      return;
    }
  }
}
}
