#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tf/transform_listener.h>

// lib
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <decision_maker_node.hpp>

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
void DecisionMakerNode::update_pubsub(void)
{
  // if state machine require to re-subscribe topic,
  // this function will re-definition subscriber.
}

int DecisionMakerNode::createCrossRoadAreaMarker(visualization_msgs::Marker &crossroad_marker, double scale)
{
  crossroad_marker.header.frame_id = "/map";
  crossroad_marker.header.stamp = ros::Time();
  crossroad_marker.id = 1;
  crossroad_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crossroad_marker.action = visualization_msgs::Marker::ADD;
  crossroad_marker.ns = "crossroad";

  crossroad_marker.scale.x = scale;
  crossroad_marker.scale.y = scale;
  crossroad_marker.scale.z = 0.5;
  crossroad_marker.color.a = 0.15;
  crossroad_marker.color.r = 1.0;
  crossroad_marker.color.g = 0.0;
  crossroad_marker.color.b = 0.0;
  crossroad_marker.frame_locked = true;
  crossroad_marker.lifetime = ros::Duration(0.3);

  return 0;
}

void DecisionMakerNode::displayMarker(void)
{
  // vector_map init
  // parse vectormap
  jsk_recognition_msgs::BoundingBoxArray bbox_array;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crossroad_marker;
  visualization_msgs::Marker stopline_target_marker;
  visualization_msgs::Marker inside_marker;
  visualization_msgs::Marker inside_line_marker;
  visualization_msgs::Marker detection_area_marker;

  double scale = 3.0;
  createCrossRoadAreaMarker(crossroad_marker, scale);
  
  detection_area_marker = crossroad_marker;
  detection_area_marker.header.frame_id = param_baselink_tf_;
  detection_area_marker.scale.x = (detectionArea_.x1 - detectionArea_.x2) * param_detection_area_rate_;
  detection_area_marker.scale.y = (detectionArea_.y1 - detectionArea_.y2) * param_detection_area_rate_;
  detection_area_marker.color.r = foundOtherVehicleForIntersectionStop_?0.0:1.0;
  detection_area_marker.pose.position.x =  detection_area_marker.scale.x/2;
  detection_area_marker.color.g = 1;
  detection_area_marker.color.b = 0.3;
  detection_area_marker.color.a = 0.3;
  detection_area_marker.type = visualization_msgs::Marker::CUBE;
  detection_area_marker.lifetime=ros::Duration();

  stopline_target_marker = crossroad_marker;
  stopline_target_marker.type = visualization_msgs::Marker::SPHERE;
  stopline_target_marker.scale.x = 1;
  stopline_target_marker.scale.y = 1;
  stopline_target_marker.scale.z = 1;
  stopline_target_marker.color.a = 1;
  stopline_target_marker.color.r = 1.0;
  stopline_target_marker.color.g = 0.0;
  stopline_target_marker.color.b = 0.0;
  stopline_target_marker.ns = "stopline_target";
  stopline_target_marker.lifetime = ros::Duration();
  stopline_target_marker.pose.position.x = CurrentStoplineTarget_.pose.pose.position.x;
  stopline_target_marker.pose.position.y = CurrentStoplineTarget_.pose.pose.position.y;
  stopline_target_marker.pose.position.z = CurrentStoplineTarget_.pose.pose.position.z;
  
  inside_marker = crossroad_marker;
  inside_marker.scale.x = scale / 3;
  inside_marker.scale.y = scale / 3;
  inside_marker.scale.z = 0.4;
  inside_marker.scale.z = 0.5;
  inside_marker.color.a = 0.5;
  inside_marker.color.r = 1.0;
  inside_marker.color.g = 0.0;
  inside_marker.color.b = 0.0;
  inside_marker.ns = "inside";
  inside_marker.lifetime = ros::Duration();
   

  inside_marker = crossroad_marker;
  inside_marker.scale.x = scale / 3;
  inside_marker.scale.y = scale / 3;
  inside_marker.scale.z = 0.4;
  inside_marker.scale.z = 0.5;
  inside_marker.color.a = 0.5;
  inside_marker.color.r = 1.0;
  inside_marker.color.g = 0.0;
  inside_marker.color.b = 0.0;
  inside_marker.ns = "inside";
  inside_marker.lifetime = ros::Duration();

  bbox_array.header = crossroad_marker.header;
  inside_marker.points.clear();

  inside_line_marker = inside_marker;
  inside_line_marker.type = visualization_msgs::Marker::LINE_STRIP;

  for (auto &area : intersects)
  {
    area.bbox.header = crossroad_marker.header;
    bbox_array.boxes.push_back(area.bbox);
    for (const auto &p : area.insideWaypoint_points)
    {
      inside_marker.points.push_back(p);
    }

    for (const auto &lane : area.insideLanes)
    {
      inside_line_marker.points.clear();
      int id = inside_line_marker.id;
      inside_line_marker.id += 1;
      inside_marker.scale.x = scale / 3;
      inside_marker.scale.y = scale / 3;
      inside_line_marker.color.r = std::fmod(0.12345 * (id), 1.0);
      inside_line_marker.color.g = std::fmod(0.32345 * (5 - (id % 5)), 1.0);
      inside_line_marker.color.b = std::fmod(0.52345 * (10 - (id % 10)), 1.0);
      for (const auto &wp : lane.waypoints)
      {
        inside_line_marker.points.push_back(wp.pose.pose.position);
      }
      marker_array.markers.push_back(inside_line_marker);
    }
  }
  inside_line_marker.scale.x = 0.2;  // 0.3;
  inside_line_marker.scale.y = 0.2;  // 0.3;
  inside_line_marker.color.r = 0;
  inside_line_marker.color.g = 1;
  inside_line_marker.color.b = 0.3;
  inside_line_marker.color.a = 1;
  inside_line_marker.ns = "shiftline";
  for (const auto &lane : current_controlled_lane_array_.lanes)
  {
    inside_line_marker.points.clear();
    for (size_t idx = 0; idx < lane.waypoints.size(); idx++)
    {
      inside_line_marker.id += 1;

      geometry_msgs::Pose shift_p = lane.waypoints.at(idx).pose.pose;

      double current_angle = getPoseAngle(shift_p);

      shift_p.position.x -= param_shift_width_ * cos(current_angle + M_PI / 2);
      shift_p.position.y -= param_shift_width_ * sin(current_angle + M_PI / 2);

      inside_line_marker.points.push_back(shift_p.position);
    }
    marker_array.markers.push_back(inside_line_marker);
  }


  Pubs["detection_area"].publish(detection_area_marker);
  Pubs["crossroad_bbox"].publish(bbox_array);
  Pubs["crossroad_marker"].publish(marker_array);
  Pubs["stopline_target"].publish(stopline_target_marker);
  bbox_array.boxes.clear();
  // Pubs["crossroad_inside_marker"].publish(inside_marker);
  Pubs["crossroad_inside_marker"].publish(inside_line_marker);
}

void DecisionMakerNode::update_msgs(void)
{
  if (ctx)
  {
    static std::string prevStateName;
    CurrentStateName = ctx->getCurrentStateName();

    if (prevStateName != CurrentStateName)
    {
      prevStateName = CurrentStateName;
      update_pubsub();
    }

    autoware_msgs::state state_msg;
    state_msg.main_state = ctx->getCurrentStateName((uint8_t)state_machine::StateKinds::MAIN_STATE);
    state_msg.acc_state = ctx->getCurrentStateName((uint8_t)state_machine::StateKinds::ACC_STATE);
    state_msg.str_state = ctx->getCurrentStateName((uint8_t)state_machine::StateKinds::STR_STATE);
    state_msg.behavior_state = ctx->getCurrentStateName((uint8_t)state_machine::StateKinds::BEHAVIOR_STATE);

    state_string_msg.data = CurrentStateName;
    // state_text_msg.text = createStateMessageText();
    state_text_msg.text = state_msg.main_state + "\n" + state_msg.acc_state + "\n" + state_msg.str_state + "\n" +
                          state_msg.behavior_state + "\n";

    Pubs["states"].publish(state_msg);
    // Pubs["state"].publish(state_string_msg);
    Pubs["state_overlay"].publish(state_text_msg);
  }
  else
    std::cerr << "ctx is not found " << std::endl;
}

std::string DecisionMakerNode::createStateMessageText()
{
  return ctx->createStateMessageText();
}

void DecisionMakerNode::publishToVelocityArray()
{
  int count = 0;
  std_msgs::Float64MultiArray msg;

  for (const auto &i : current_finalwaypoints_.waypoints)
  {
    msg.data.push_back(amathutils::mps2kmph(i.twist.twist.linear.x));
    if (++count >= 10)
      break;
  }
  Pubs["target_velocity_array"].publish(msg);
}
}
