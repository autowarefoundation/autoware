#include <mutex>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tf/transform_listener.h>

// lib
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <decision_maker_node.hpp>
//#include <vector_map/vector_map.h>

#include <autoware_msgs/lamp_cmd.h>
#include <autoware_msgs/lane.h>
#include <autoware_msgs/state.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace decision_maker
{
void DecisionMakerNode::initROS(int argc, char **argv)
{
  ctx->setCurrentState(state_machine::INITIAL_STATE);

  // status subscriber
  Subs["sim_pose"] = nh_.subscribe("sim_pose", 20, &DecisionMakerNode::callbackFromSimPose, this);
  Subs["current_pose"] = nh_.subscribe("current_pose", 20, &DecisionMakerNode::callbackFromCurrentPose, this);
  Subs["current_velocity"] =
      nh_.subscribe("current_velocity", 20, &DecisionMakerNode::callbackFromCurrentVelocity, this);
  Subs["light_color"] = nh_.subscribe("light_color", 10, &DecisionMakerNode::callbackFromLightColor, this);
  Subs["points_raw"] = nh_.subscribe("filtered_points", 1, &DecisionMakerNode::callbackFromPointsRaw, this);
  Subs["final_waypoints"] = nh_.subscribe("final_waypoints", 100, &DecisionMakerNode::callbackFromFinalWaypoint, this);
  Subs["twist_cmd"] = nh_.subscribe("twist_cmd", 10, &DecisionMakerNode::callbackFromTwistCmd, this);
  Subs["change_flag"] = nh_.subscribe("change_flag", 1, &DecisionMakerNode::callbackFromLaneChangeFlag, this);
  Subs["state_cmd"] = nh_.subscribe("state_cmd", 1, &DecisionMakerNode::callbackFromStateCmd, this);
  Subs["closest_waypoint"] =
      nh_.subscribe("closest_waypoint", 1, &DecisionMakerNode::callbackFromClosestWaypoint, this);
  Subs["cloud_clusters"] =
      nh_.subscribe("cloud_clusters", 1, &DecisionMakerNode::callbackFromObjectDetector, this);

  // Config subscriber
  Subs["config/decision_maker"] =
      nh_.subscribe("/config/decision_maker", 3, &DecisionMakerNode::callbackFromConfig, this);


  // pub
  //
  Pubs["state/stopline_wpidx"] = nh_.advertise<std_msgs::Int32>("/state/stopline_wpidx", 1, true);

  // for controlling other planner
  Pubs["state"] = nh_.advertise<std_msgs::String>("state", 1);
  Pubs["lane_waypoints_array"] = nh_.advertise<autoware_msgs::LaneArray>(TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY, 10, true);
  Pubs["states"] = nh_.advertise<autoware_msgs::state>("/decisionmaker/states", 1, true);
  Pubs["light_color"] = nh_.advertise<autoware_msgs::traffic_light>("/light_color_managed", 1);

  // for controlling vehicle
  Pubs["lamp_cmd"] = nh_.advertise<autoware_msgs::lamp_cmd>("/lamp_cmd", 1);

  // for visualize status
  Pubs["state_overlay"] = nh_.advertise<jsk_rviz_plugins::OverlayText>("/state/overlay_text", 1);
  Pubs["crossroad_marker"] = nh_.advertise<visualization_msgs::MarkerArray>("/state/cross_road_marker", 1);
  Pubs["crossroad_inside_marker"] = nh_.advertise<visualization_msgs::Marker>("/state/cross_inside_marker", 1);
  Pubs["crossroad_bbox"] = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/state/crossroad_bbox", 10);
  Pubs["detection_area"] = nh_.advertise<visualization_msgs::Marker>("/state/detection_area",1);
  Pubs["stopline_target"] = nh_.advertise<visualization_msgs::Marker>("/state/stopline_target",1);

  // for debug
  Pubs["target_velocity_array"] = nh_.advertise<std_msgs::Float64MultiArray>("/target_velocity_array", 1);
  Pubs["state_local_diffdistance"] = nh_.advertise<std_msgs::Float64>("/state/val_diff_distance", 1);
  Pubs["exectime"] = nh_.advertise<std_msgs::Float64>("/state/exectime", 1);

  // message setup
  state_text_msg.width = 400;
  state_text_msg.height = 500;
  state_text_msg.top = 10;
  state_text_msg.left = 10;
  state_text_msg.bg_color.r = 0;
  state_text_msg.bg_color.g = 0;
  state_text_msg.bg_color.b = 0;
  state_text_msg.bg_color.a = 0.8;

  state_text_msg.line_width = 2;
  state_text_msg.text_size = 18;
  state_text_msg.font = "DejaVu Sans Mono";
  state_text_msg.fg_color.r = 0.1;
  state_text_msg.fg_color.g = 1.0;
  state_text_msg.fg_color.b = 0.94;
  state_text_msg.fg_color.a = 0.8;
  state_text_msg.text = "UNDEFINED";

  // initial publishing state message
  update_msgs();

  // setup a callback for state update();
  setupStateCallback();

  g_vmap.subscribe(nh_,
                   Category::POINT | Category::LINE | Category::VECTOR | Category::AREA |
                       Category::POLE |  // basic class
                       Category::DTLANE | Category::STOP_LINE | Category::ROAD_SIGN | Category::CROSS_ROAD);
  initVectorMap();

  {
    if (enableDisplayMarker)
      displayMarker();
  }

  ROS_INFO("Initialized OUT\n");
  ctx->setCurrentState(state_machine::INITIAL_LOCATEVEHICLE_STATE);
  
  Subs["lane_waypoints_array"] =
      nh_.subscribe(TPNAME_BASED_LANE_WAYPOINTS_ARRAY, 100, &DecisionMakerNode::callbackFromLaneWaypoint, this);
}

void DecisionMakerNode::initVectorMap(void)
{
  int _index = 0;
  // if(vector_map_init)
  //      return;
  std::vector<CrossRoad> crossroads = g_vmap.findByFilter([](const CrossRoad &crossroad) { return true; });
  if (crossroads.empty())
  {
    ROS_INFO("crossroads have not found\n");
    return;
  }

  vector_map_init = true;  // loaded flag
  for (const auto &cross_road : crossroads)
  {
    geometry_msgs::Point _prev_point;
    Area area = g_vmap.findByKey(Key<Area>(cross_road.aid));
    CrossRoadArea carea;
    carea.id = _index++;
    carea.area_id = area.aid;

    double x_avg = 0.0, x_min = 0.0, x_max = 0.0;
    double y_avg = 0.0, y_min = 0.0, y_max = 0.0;
    double z = 0.0;
    int points_count = 0;

    std::vector<Line> lines =
        g_vmap.findByFilter([&area](const Line &line) { return area.slid <= line.lid && line.lid <= area.elid; });
    for (const auto &line : lines)
    {
      std::vector<Point> points =
          g_vmap.findByFilter([&line](const Point &point) { return line.bpid == point.pid;});
      for (const auto &point : points)
      {
        geometry_msgs::Point _point;
        _point.x = point.ly;
        _point.y = point.bx;
        _point.z = point.h;

        if (_prev_point.x == _point.x && _prev_point.y == _point.y)
          continue;

        _prev_point = _point;
        points_count++;
        carea.points.push_back(_point);

        // calc a centroid point and about intersects size
        x_avg += _point.x;
        y_avg += _point.y;
        x_min = (x_min == 0.0) ? _point.x : std::min(_point.x, x_min);
        x_max = (x_max == 0.0) ? _point.x : std::max(_point.x, x_max);
        y_min = (y_min == 0.0) ? _point.y : std::min(_point.y, y_min);
        y_max = (y_max == 0.0) ? _point.y : std::max(_point.y, y_max);
        z = _point.z;
      }  // points iter
    }    // line iter
    carea.bbox.pose.position.x = x_avg / (double)points_count * 1.5/* expanding rate */;
    carea.bbox.pose.position.y = y_avg / (double)points_count * 1.5;
    carea.bbox.pose.position.z = z;
    carea.bbox.dimensions.x = x_max - x_min;
    carea.bbox.dimensions.y = y_max - y_min;
    carea.bbox.dimensions.z = 2;
    carea.bbox.label = 1;
    intersects.push_back(carea);
  }
}
}
