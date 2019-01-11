#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::publishLampCmd(const E_Lamp& status)
{
  autoware_msgs::LampCmd lamp_msg;

  switch (status)
  {
    case E_Lamp::LAMP_LEFT:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_OFF;
      break;
    case E_Lamp::LAMP_RIGHT:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_HAZARD:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_EMPTY:
    default:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_OFF;
      break;
  }
  Pubs["lamp_cmd"].publish(lamp_msg);
}

jsk_rviz_plugins::OverlayText createOverlayText(cstring_t& data, const int column)
{
  jsk_rviz_plugins::OverlayText ret;

  // message setup
  ret.width = 500;
  ret.height = 500;
  ret.top = 10 + (column * 500);
  ret.left = 10;
  ret.bg_color.r = 0;
  ret.bg_color.g = 0;
  ret.bg_color.b = 0;
  ret.bg_color.a = 0.8;

  ret.line_width = 2;
  ret.text_size = 9;
  ret.font = "DejaVu Sans Mono";
  ret.fg_color.r = 1.0;
  ret.fg_color.g = 1.0;
  ret.fg_color.b = 0.5;
  ret.fg_color.a = 0.9;

  ret.text = data;

  return ret;
}

void DecisionMakerNode::publishOperatorHelpMessage(cstring_t& message)
{
  static std::vector<std::string> msg_log;
  static const size_t log_size = 10;

  msg_log.push_back(message);

  if (msg_log.size() >= log_size)
  {
    msg_log.erase(msg_log.begin());
  }

  std::string joined_msg;
  for (const auto& i : msg_log)
  {
    joined_msg += "> " + i + "\n";
  }
  Pubs["operator_help_text"].publish(createOverlayText(joined_msg, 0));
}

void DecisionMakerNode::update_pubsub(void)
{
}

int DecisionMakerNode::createCrossRoadAreaMarker(visualization_msgs::Marker& crossroad_marker, double scale)
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

void DecisionMakerNode::update_msgs(void)
{
#if 1
  if (ctx_vehicle && ctx_mission && ctx_behavior && ctx_motion)
  {
    static std_msgs::String state_msg;
    state_msg.data = ctx_vehicle->getStateText() + "\n" + ctx_mission->getStateText() + "\n" + ctx_behavior->getStateText() + "\n" + ctx_motion->getStateText();
    Pubs["state"].publish(state_msg);
    Pubs["state_overlay"].publish(createOverlayText(state_msg.data, 1));

    static std_msgs::String transition_msg;
    transition_msg.data = ctx_vehicle->getAvailableTransition() + "\n" + ctx_mission->getAvailableTransition() + "\n" +
                          ctx_behavior->getAvailableTransition() + "\n" + ctx_motion->getAvailableTransition();

    Pubs["available_transition"].publish(transition_msg);
  }
  else
  {
    std::cerr << "ctx is not found " << std::endl;
  }
#endif
}

void DecisionMakerNode::publishLightColor(int status)
{
  autoware_msgs::TrafficLight msg;
  msg.traffic_light = status;
  Pubs["light_color"].publish(msg);
}

void DecisionMakerNode::publishStoplineWaypointIdx(int wp_idx)
{
  std_msgs::Int32 msg;
  msg.data = wp_idx;
  Pubs["state/stopline_wpidx"].publish(msg);
}

void DecisionMakerNode::publishToVelocityArray()
{
  int count = 0;
  std_msgs::Float64MultiArray msg;

  for (const auto& i : current_status_.finalwaypoints.waypoints)
  {
    msg.data.push_back(amathutils::mps2kmph(i.twist.twist.linear.x));
    if (++count >= 10)
      break;
  }
  Pubs["target_velocity_array"].publish(msg);
}
}
