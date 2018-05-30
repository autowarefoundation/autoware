#include <ros/ros.h>

#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <autoware_msgs/lamp_cmd.h>
#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::setupStateCallback(void)
{
  // steering state. these state's update function change lamp
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_STR_STRAIGHT_STATE,
                             std::bind(&DecisionMakerNode::updateStateSTR, this, 0));
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_STR_LEFT_STATE,
                             std::bind(&DecisionMakerNode::updateStateSTR, this, 1));
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_STR_RIGHT_STATE,
                             std::bind(&DecisionMakerNode::updateStateSTR, this, 2));

  // stopline stop state
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_ACC_STOPLINE_STATE,
                             std::bind(&DecisionMakerNode::updateStateStop, this, 1));
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_STOPLINE_STATE,
                         std::bind(&DecisionMakerNode::callbackInStateStop, this, 1));
  ctx->setCallbackOutFunc(state_machine::DRIVE_ACC_STOPLINE_STATE,
                          std::bind(&DecisionMakerNode::callbackOutStateStop, this, 1));

  // stopping state
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_ACC_STOP_STATE,
                             std::bind(&DecisionMakerNode::updateStateStop, this, 0));
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_STOP_STATE,
                         std::bind(&DecisionMakerNode::callbackInStateStop, this, 0));

  ctx->setCallbackInFunc(state_machine::DRIVE_BEHAVIOR_STOPLINE_PLAN_STATE,
                         std::bind(&DecisionMakerNode::StoplinePlanIn, this, 1));
  ctx->setCallbackOutFunc(state_machine::DRIVE_BEHAVIOR_STOPLINE_PLAN_STATE,
                          std::bind(&DecisionMakerNode::StoplinePlanOut, this, 1));

  // speed keep(original speed) state
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_KEEP_STATE,
                         std::bind(&DecisionMakerNode::callbackInStateKeep, this, 1));

  // crawl
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_CRAWL_STATE,
                         std::bind(&DecisionMakerNode::callbackInStateAcc, this, 0));

  // acceleration
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_ACCELERATION_STATE,
                         std::bind(&DecisionMakerNode::callbackInStateAcc, this, 1));

  // deceleration
  ctx->setCallbackInFunc(state_machine::DRIVE_ACC_DECELERATION_STATE,
                         std::bind(&DecisionMakerNode::callbackInStateAcc, this, -1));

  // LaneChange
  ctx->setCallbackOutFunc(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE,
                          std::bind(&DecisionMakerNode::callbackOutStateLaneChange, this, 1));
  ctx->setCallbackOutFunc(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE,
                          std::bind(&DecisionMakerNode::callbackOutStateLaneChange, this, 1));

  // obstacle avoidance
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE,
                             std::bind(&DecisionMakerNode::updateStateObstacleAvoid, this, -1));
  ctx->setCallbackInFunc(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE,
                         std::bind(&DecisionMakerNode::callbackInStateObstacleAvoid, this, -1));
  ctx->setCallbackOutFunc(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE,
                          std::bind(&DecisionMakerNode::callbackOutStateObstacleAvoid, this, 1));

  // trraficlight
  ctx->setCallbackInFunc(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE, [&]() {
    ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE);
    publishLightColor((int)state_machine::E_RED);
  });
  ctx->setCallbackInFunc(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE, [&]() {
    ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE);
    publishLightColor((int)state_machine::E_GREEN);
  });

#if 0
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE,
                             std::bind(&DecisionMakerNode::publishLightColor, this, (int)state_machine::E_RED));
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE,
                             std::bind(&DecisionMakerNode::publishLightColor, this, (int)state_machine::E_GREEN));
  ctx->setCallbackInFunc(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE,
                         [&]() { ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE); });
  ctx->setCallbackUpdateFunc(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE,
                             std::bind(&DecisionMakerNode::publishLightColor, this, (int)state_machine::E_GREEN));
#endif
}

void DecisionMakerNode::callbackOutStateLaneChange(int status)
{
  if (ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_ACCEPT_LANECHANGE_STATE))
  {
    ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_ACCEPT_LANECHANGE_STATE);
  }
}

void DecisionMakerNode::publishLightColor(int status)
{
  autoware_msgs::traffic_light msg;
  msg.traffic_light = status;
  Pubs["light_color"].publish(msg);
}

void DecisionMakerNode::publishStoplineWaypointIdx(int wp_idx)
{
  std_msgs::Int32 msg;
  msg.data = wp_idx;
  Pubs["state/stopline_wpidx"].publish(msg);
}

#define SHIFTED_LANE_FLAG -99999
void DecisionMakerNode::createShiftLane(void)
{
  bool isRightShift = param_shift_width_ >= 0;

  autoware_msgs::LaneArray shift_lanes = current_shifted_lane_array_ = current_based_lane_array_;
  if (!current_shifted_lane_array_.lanes.empty())
  {
    size_t lane_idx = 0;
    for (auto& lane : shift_lanes.lanes)
    {
      lane.increment = SHIFTED_LANE_FLAG;
      size_t wp_idx = 0;
      for (auto& wp : lane.waypoints)
      {
        double angle = getPoseAngle(wp.pose.pose);
        wp.pose.pose.position.x -= param_shift_width_ * cos(angle + M_PI / 2);
        wp.pose.pose.position.y -= param_shift_width_ * sin(angle + M_PI / 2);
        wp.change_flag = current_based_lane_array_.lanes.at(lane_idx).waypoints.at(wp_idx++).change_flag;
      }
      lane_idx++;
    }
  }
  int insert_lane_idx_offset = isRightShift ? 1 : 0;
  auto it_shift = begin(shift_lanes.lanes);
  try
  {
    for (auto it = begin(current_shifted_lane_array_.lanes);
         it != end(current_shifted_lane_array_.lanes) && it_shift != end(shift_lanes.lanes);)
    {
      for (auto& wp : it->waypoints)
      {
        wp.change_flag = isRightShift ? 1 : 2;
      }
      it = current_shifted_lane_array_.lanes.insert(it + insert_lane_idx_offset, *(it_shift++));
      it++;
      if (!isRightShift)
      {
        it++;
      }
    }
  }
  catch (std::length_error)
  {
  }
}

void DecisionMakerNode::changeShiftLane(void)
{
  auto based_it = begin(current_shifted_lane_array_.lanes);

  for (auto& lane : current_shifted_lane_array_.lanes)
  {
    if (lane.increment == SHIFTED_LANE_FLAG)
    {
      for (auto& wp : lane.waypoints)
      {
        wp.change_flag = param_shift_width_ >= 0 ? 2 : 1;
      }
    }
    else
    {
      auto based_wp_it = begin(based_it++->waypoints);
      for (auto& wp : lane.waypoints)
      {
        wp.change_flag = based_wp_it++->change_flag;
      }
    }
  }
}

void DecisionMakerNode::removeShiftLane(void)
{
  current_shifted_lane_array_ = current_based_lane_array_;
}

void DecisionMakerNode::updateLaneWaypointsArray(void)
{
  current_stopped_lane_array_ = current_controlled_lane_array_;

  for (auto& lane : current_stopped_lane_array_.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      wp.twist.twist.linear.x = 0.0;
      wp.wpstate.stopline_state = 0;
    }
  }
  for (auto& lane : current_shifted_lane_array_.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      // if stopped at stopline, to delete flags already used.
      if (CurrentStoplineTarget_.gid - 2 <= wp.gid && wp.gid <= CurrentStoplineTarget_.gid + 2)
      {
        wp.wpstate.stopline_state = 0;
      }
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
  current_controlled_lane_array_ = current_shifted_lane_array_;
}

void DecisionMakerNode::setAllStoplineStop(void)
{
  std::vector<StopLine> stoplines = g_vmap.findByFilter([&](const StopLine& stopline) { return true; });

  for (auto& lane : current_shifted_lane_array_.lanes)
  {
    for (size_t wp_idx = 0; wp_idx < lane.waypoints.size() - 1; wp_idx++)
    {
      for (auto& stopline : stoplines)
      {
        geometry_msgs::Point bp =
            to_geoPoint(g_vmap.findByKey(Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).bpid)));
        geometry_msgs::Point fp =
            to_geoPoint(g_vmap.findByKey(Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).fpid)));

        if (amathutils::isIntersectLine(lane.waypoints.at(wp_idx).pose.pose.position.x,
                                        lane.waypoints.at(wp_idx).pose.pose.position.y,
                                        lane.waypoints.at(wp_idx + 1).pose.pose.position.x,
                                        lane.waypoints.at(wp_idx + 1).pose.pose.position.y, bp.x, bp.y, fp.x, fp.y))
        {
          geometry_msgs::Point center_point;
          center_point.x = (bp.x * 2 + fp.x) / 3;
          center_point.y = (bp.y * 2 + fp.y) / 3;
          if (amathutils::isPointLeftFromLine(
                  center_point.x, center_point.y, lane.waypoints.at(wp_idx).pose.pose.position.x,
                  lane.waypoints.at(wp_idx).pose.pose.position.y, lane.waypoints.at(wp_idx + 1).pose.pose.position.x,
                  lane.waypoints.at(wp_idx + 1).pose.pose.position.y))
          {
            amathutils::point* a = new amathutils::point();
            amathutils::point* b = new amathutils::point();
            a->x = center_point.x;
            a->y = center_point.y;
            b->x = lane.waypoints.at(wp_idx).pose.pose.position.x;
            b->y = lane.waypoints.at(wp_idx).pose.pose.position.y;
            if (amathutils::find_distance(a, b) <= 4)  //
              lane.waypoints.at(wp_idx).wpstate.stopline_state = 1;
          }
        }
      }
    }
  }
}

void DecisionMakerNode::StoplinePlanIn(int status)
{
  setAllStoplineStop();
  changeVelocityBasedLane();
  publishControlledLaneArray();
}
void DecisionMakerNode::StoplinePlanOut(int status)
{
  current_shifted_lane_array_ = current_based_lane_array_;
  changeVelocityBasedLane();
  publishControlledLaneArray();
}

void DecisionMakerNode::changeVelocityLane(int dir)
{
  if (dir != 0)
  {
    for (auto& lane : current_controlled_lane_array_.lanes)
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
        double _rate = 0.2;                       // accelerated/decelerated rate
        double _weight = distance * _rate * dir;  //
        lane.waypoints.at(wpi).twist.twist.linear.x =
            lane.waypoints.at(wpi).twist.twist.linear.x + (lane.waypoints.at(wpi).twist.twist.linear.x * _weight);
      }
    }
  }
  else
  {
    for (auto& lane : current_controlled_lane_array_.lanes)
    {
      for (auto& wp : lane.waypoints)
      {
        wp.twist.twist.linear.x = amathutils::kmph2mps(param_crawl_velocity_);
      }
    }
  }
}

void DecisionMakerNode::callbackInStateKeep(int status)
{
  changeVelocityBasedLane();
  publishControlledLaneArray();
}

void DecisionMakerNode::callbackInStateAcc(int status)
{
  changeVelocityLane(status);
  publishControlledLaneArray();
}

// for stopping state(stopline/stop)
void DecisionMakerNode::updateStateStop(int status)
{
  static bool timerflag;
  static ros::Timer stopping_timer;

  if (status)
  {
    if (current_velocity_ == 0.0 && !foundOtherVehicleForIntersectionStop_ && !timerflag)
    {
      stopping_timer = nh_.createTimer(ros::Duration(1),
                                       [&](const ros::TimerEvent&) {
                                         ctx->setCurrentState(state_machine::DRIVE_ACC_KEEP_STATE);
                                         ROS_INFO("Change state to [KEEP] from [STOP]\n");
                                         timerflag = false;
                                       },
                                       this, true);
      timerflag = true;
    }
    else
    {
      if (foundOtherVehicleForIntersectionStop_ && timerflag)
      {
        stopping_timer.stop();
        timerflag = false;
      }
      publishStoplineWaypointIdx(closest_stopline_waypoint_);
    }
  }
}
void DecisionMakerNode::callbackInStateStop(int status)
{
  if (!status) /*not a stopline*/
  {
    publishStoppedLaneArray();
  }
  else /* handling stopline */
  {
    publishStoplineWaypointIdx(closest_stopline_waypoint_);
  }
}
void DecisionMakerNode::callbackOutStateStop(int status)
{
  if (status)
  {
    closest_stopline_waypoint_ = -1;
    publishStoplineWaypointIdx(closest_stopline_waypoint_);
  }
}

void DecisionMakerNode::updateStateObstacleAvoid(int status)
{
}

void DecisionMakerNode::callbackOutStateObstacleAvoid(int status)
{
  changeShiftLane();
  changeVelocityBasedLane();
  publishControlledLaneArray();
  ros::Rate loop_rate(1);

  // wait for the start of lane change to the original lane
  if (created_shift_lane_flag_)
  {
    do
    {
      ros::spinOnce();
      loop_rate.sleep();
    } while (!ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE) &&
             !ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE) && ros::ok());
    // wait for the end of lane change
    do
    {
      ros::spinOnce();
      loop_rate.sleep();
    } while ((ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE) ||
              ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE) ||
              ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE)) &&
             ros::ok());
    removeShiftLane();
    created_shift_lane_flag_ = false;
  }
  changeVelocityBasedLane();  // rebased controlled lane
  publishControlledLaneArray();
  return;
}

void DecisionMakerNode::callbackInStateObstacleAvoid(int status)
{
  // this state is temporary implementation.
  // It means this state is desirable to use a way which enables the avoidance
  // plannner such as astar, state lattice.

  // if car shoud stop before avoidance,
  if (!created_shift_lane_flag_)
  {
    created_shift_lane_flag_ = true;
    ctx->setCurrentState(state_machine::DRIVE_ACC_STOPLINE_STATE);
    createShiftLane();
  }

  changeVelocityBasedLane();
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
