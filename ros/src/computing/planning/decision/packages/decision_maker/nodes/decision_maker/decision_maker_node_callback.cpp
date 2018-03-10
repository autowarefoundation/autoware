#include <stdio.h>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <autoware_msgs/lane.h>
#include <autoware_msgs/traffic_light.h>

#include <cross_road_area.hpp>
#include <decision_maker_node.hpp>
#include <state.hpp>
#include <state_context.hpp>

namespace decision_maker
{
void DecisionMakerNode::callbackFromCurrentPose(const geometry_msgs::PoseStamped &msg)
{
  geometry_msgs::PoseStamped _pose = current_pose_ = msg;
  bool initLocalizationFlag = ctx->isCurrentState(state_machine::INITIAL_LOCATEVEHICLE_STATE);
  if (initLocalizationFlag &&
      isLocalizationConvergence(_pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z,
                                _pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z))
  {
    ROS_INFO("Localization was convergence");
  }
}

bool DecisionMakerNode::handleStateCmd(const uint64_t _state_num)
{
  bool _ret;
  ctx->setEnableForceSetState(true);
  if (!ctx->isCurrentState(_state_num))
  {
    _ret = ctx->setCurrentState((state_machine::StateFlags)_state_num);
    if(_state_num == state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE 
		    || _state_num == state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE){
	    isManualLight = true;
    }
  }
  else
  {
    _ret = ctx->disableCurrentState((state_machine::StateFlags)_state_num);
    if(_state_num == state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE 
		    || _state_num == state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE){
	    isManualLight = false;
    }
  }
  ctx->setEnableForceSetState(false);


  return _ret;
}

void DecisionMakerNode::callbackFromSimPose(const geometry_msgs::PoseStamped &msg)
{
  ROS_INFO("Received system is going to simulation mode");
  handleStateCmd(state_machine::DRIVE_STATE);
  Subs["sim_pose"].shutdown();
}

void DecisionMakerNode::callbackFromStateCmd(const std_msgs::Int32 &msg)
{
  ROS_INFO("Received forcing state changing request: %llx", 1ULL << (uint64_t)msg.data);
  handleStateCmd((uint64_t)1ULL << (uint64_t)msg.data);
}

void DecisionMakerNode::callbackFromLaneChangeFlag(const std_msgs::Int32 &msg)
{
  if (msg.data == enumToInteger<E_ChangeFlags>(E_ChangeFlags::LEFT) && ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_ACCEPT_LANECHANGE_STATE))
  {
    ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE);
    ctx->setCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE);
  }
  else if (msg.data == enumToInteger<E_ChangeFlags>(E_ChangeFlags::RIGHT) && ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_ACCEPT_LANECHANGE_STATE))
  {
    ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE);
    ctx->setCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE);
  }
  else
  {
    ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE);
    ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE);
  }
}

void DecisionMakerNode::callbackFromConfig(const autoware_msgs::ConfigDecisionMaker &msg)
{
  ROS_INFO("Param setted by Runtime Manager");
  enableDisplayMarker = msg.enable_display_marker;
  ctx->setEnableForceSetState(msg.enable_force_state_change);

  param_target_waypoint_ = msg.target_waypoint;
  param_stopline_target_waypoint_ = msg.stopline_target_waypoint;
  param_shift_width_ = msg.shift_width;
}

void DecisionMakerNode::callbackFromLightColor(const ros::MessageEvent<autoware_msgs::traffic_light const> &event)
{    
  const autoware_msgs::traffic_light *light = event.getMessage().get();
//  const ros::M_string &header = event.getConnectionHeader();
//  std::string topic = header.at("topic"); 
  
  if(!isManualLight){// && topic.find("manage") == std::string::npos){
	  current_traffic_light_ = light->traffic_light;
	  if (current_traffic_light_ == state_machine::E_RED || current_traffic_light_ == state_machine::E_YELLOW)
	  {
		  ctx->setCurrentState(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE);
		  ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE);
	  }
	  else
	  {
		  ctx->setCurrentState(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE);
		  ctx->disableCurrentState(state_machine::DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE);
	  }
  }
}

void DecisionMakerNode::callbackFromPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (ctx->setCurrentState(state_machine::INITIAL_LOCATEVEHICLE_STATE))
    Subs["points_raw"].shutdown();
}

void DecisionMakerNode::insertPointWithinCrossRoad(const std::vector<CrossRoadArea> &_intersects,
                                                   autoware_msgs::LaneArray &lane_array)
{
  for (auto &lane : lane_array.lanes)
  {
    for (auto &wp : lane.waypoints)
    {
      geometry_msgs::Point pp;
      pp.x = wp.pose.pose.position.x;
      pp.y = wp.pose.pose.position.y;
      pp.z = wp.pose.pose.position.z;

      for (auto &area : intersects)
      {
        if (CrossRoadArea::isInsideArea(&area, pp))
        {
          // area's
          if (area.insideLanes.empty() || wp.gid != area.insideLanes.back().waypoints.back().gid + 1)
          {
            autoware_msgs::lane nlane;
            area.insideLanes.push_back(nlane);
          }
          area.insideLanes.back().waypoints.push_back(wp);
          area.insideWaypoint_points.push_back(pp);  // geometry_msgs::point
          // area.insideLanes.Waypoints.push_back(wp);//autoware_msgs::waypoint
          // lane's wp
          wp.wpstate.aid = area.area_id;
        }
      }
    }
  }
}

void DecisionMakerNode::setWaypointState(autoware_msgs::LaneArray &lane_array)
{
  insertPointWithinCrossRoad(intersects, lane_array);
  // STR
  for (auto &area : intersects)
  {
    for (auto &laneinArea : area.insideLanes)
    {
      // To straight/left/right recognition by using angle
      // between first-waypoint and end-waypoint in intersection area.
      int angle_deg = ((int)std::floor(calcIntersectWayAngle(laneinArea)));  // normalized
      int steering_state;

      if (angle_deg <= ANGLE_LEFT)
        steering_state = autoware_msgs::WaypointState::STR_LEFT;
      else if (angle_deg >= ANGLE_RIGHT)
        steering_state = autoware_msgs::WaypointState::STR_RIGHT;
      else
        steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;

      for (auto &wp_lane : laneinArea.waypoints)
        for (auto &lane : lane_array.lanes)
          for (auto &wp : lane.waypoints)
            if (wp.gid == wp_lane.gid && wp.wpstate.aid == area.area_id)
            {
              wp.wpstate.steering_state = steering_state;
            }
    }
  }
  // STOP
  std::vector<StopLine> stoplines = g_vmap.findByFilter([&](const StopLine &stopline) {
    return (g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type == (int)vector_map_msgs::RoadSign::TYPE_STOP);
  });

  for (auto &lane : lane_array.lanes)
  {
    for (size_t wp_idx = 0; wp_idx < lane.waypoints.size() - 1; wp_idx++)
    {
      for (auto &stopline : stoplines)
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
            lane.waypoints.at(wp_idx).wpstate.stopline_state = 1;
            // lane.waypoints.at(wp_idx + 1).wpstate.stopline_state = 1;
          }
        }
      }
    }
  }
}

// for based waypoint
void DecisionMakerNode::callbackFromLaneWaypoint(const autoware_msgs::LaneArray &msg)
{
  ROS_INFO("[%s]:LoadedWaypointLaneArray\n", __func__);
  current_based_lane_array_ = msg;  // cached based path
  // indexing
  int gid = 0;
  for (auto &lane : current_based_lane_array_.lanes)
  {
    int lid = 0;
    for (auto &wp : lane.waypoints)
    {
      wp.gid = gid++;
      wp.lid = lid++;
      wp.wpstate.aid = 0;
      wp.wpstate.steering_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.accel_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.stopline_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.lanechange_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.event_state = 0;
    }
  }
  setWaypointState(current_based_lane_array_);
  current_controlled_lane_array_ = current_shifted_lane_array_ = current_based_lane_array_;  // controlled path

  publishControlledLaneArray();
  updateLaneWaypointsArray();
}

state_machine::StateFlags getStateFlags(uint8_t msg_state)
{
  if (msg_state == (uint8_t)autoware_msgs::WaypointState::STR_LEFT)
    return state_machine::DRIVE_STR_LEFT_STATE;
  else if (msg_state == (uint8_t)autoware_msgs::WaypointState::STR_RIGHT)
    return state_machine::DRIVE_STR_RIGHT_STATE;
  else
    return state_machine::DRIVE_STR_STRAIGHT_STATE;
}

void DecisionMakerNode::callbackFromFinalWaypoint(const autoware_msgs::lane &msg)
{
  if (!hasvMap())
  {
    std::cerr << "Not found vmap subscribe" << std::endl;
    return;
  }

  if (!ctx->isCurrentState(state_machine::DRIVE_STATE))
  {
    std::cerr << "State is not DRIVE_STATE[" << ctx->getCurrentStateName() << "]" << std::endl;
    return;
  }
  // cached
  current_finalwaypoints_ = msg;

  size_t idx = current_finalwaypoints_.waypoints.size() - 1 > param_stopline_target_waypoint_ ?
                   param_stopline_target_waypoint_ :
                   current_finalwaypoints_.waypoints.size() - 1;
  
  if (current_finalwaypoints_.waypoints.at(idx).wpstate.stopline_state)
	  ctx->setCurrentState(state_machine::DRIVE_ACC_STOPLINE_STATE);
  // steering
  idx = current_finalwaypoints_.waypoints.size() - 1 > param_target_waypoint_ ?
            param_target_waypoint_ :
            current_finalwaypoints_.waypoints.size() - 1;
  if (ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE))
  {
	  ctx->setCurrentState(state_machine::DRIVE_STR_LEFT_STATE);
  }
  if (ctx->isCurrentState(state_machine::DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE))
  {
	  ctx->setCurrentState(state_machine::DRIVE_STR_RIGHT_STATE);
  }
  else
  {
	  ctx->setCurrentState(getStateFlags(current_finalwaypoints_.waypoints.at(idx).wpstate.steering_state));
  }

  // for publish plan of velocity
  publishToVelocityArray();
}
void DecisionMakerNode::callbackFromTwistCmd(const geometry_msgs::TwistStamped &msg)
{
  static bool Twistflag = false;

  if (Twistflag)
    ctx->handleTwistCmd(false);
  else
    Twistflag = true;
}

void DecisionMakerNode::callbackFromClosestWaypoint(const std_msgs::Int32 &msg)
{
  closest_waypoint_ = msg.data;
}

void DecisionMakerNode::callbackFromVectorMapArea(const vector_map_msgs::AreaArray &msg)
{
  initVectorMap();
}
void DecisionMakerNode::callbackFromVectorMapPoint(const vector_map_msgs::PointArray &msg)
{
  initVectorMap();
}
void DecisionMakerNode::callbackFromVectorMapLine(const vector_map_msgs::LineArray &msg)
{
  initVectorMap();
}
void DecisionMakerNode::callbackFromVectorMapCrossRoad(const vector_map_msgs::CrossRoadArray &msg)
{
  initVectorMap();
}

void DecisionMakerNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStamped &msg)
{
  current_velocity_ = amathutils::mps2kmph(msg.twist.linear.x);
}
}
