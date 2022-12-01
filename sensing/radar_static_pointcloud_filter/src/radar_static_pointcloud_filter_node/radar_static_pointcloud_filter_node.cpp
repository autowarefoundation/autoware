// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "radar_static_pointcloud_filter/radar_static_pointcloud_filter_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });
  // Not found
  if (itr == params.cend()) {
    return false;
  }
  value = itr->template get_value<T>();
  return true;
}

geometry_msgs::msg::Vector3 getVelocity(const radar_msgs::msg::RadarReturn & radar)
{
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(radar.doppler_velocity * std::cos(radar.azimuth))
    .y(radar.doppler_velocity * std::sin(radar.azimuth))
    .z(0.0);
}

geometry_msgs::msg::Vector3 getTransformedVelocity(
  const geometry_msgs::msg::Vector3 velocity,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  geometry_msgs::msg::Vector3Stamped velocity_stamped{};
  velocity_stamped.vector = velocity;
  geometry_msgs::msg::Vector3Stamped transformed_velocity_stamped{};
  tf2::doTransform(velocity_stamped, transformed_velocity_stamped, *transform);
  return velocity_stamped.vector;
}

geometry_msgs::msg::Vector3 compensateEgoVehicleTwist(
  const radar_msgs::msg::RadarReturn & radar,
  const geometry_msgs::msg::TwistWithCovariance & ego_vehicle_twist_with_covariance,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  const geometry_msgs::msg::Vector3 radar_velocity = getVelocity(radar);
  const geometry_msgs::msg::Vector3 v_r = getTransformedVelocity(radar_velocity, transform);

  const auto v_e = ego_vehicle_twist_with_covariance.twist.linear;
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(v_r.x + v_e.x)
    .y(v_r.y + v_e.y)
    .z(v_r.z + v_e.z);
}
}  // namespace

namespace radar_static_pointcloud_filter
{
using nav_msgs::msg::Odometry;
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

RadarStaticPointcloudFilterNode::RadarStaticPointcloudFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("radar_static_pointcloud_filter", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarStaticPointcloudFilterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.doppler_velocity_sd = declare_parameter<double>("doppler_velocity_sd", 2.0);

  // Subscriber
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  sub_radar_.subscribe(this, "~/input/radar", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_odometry_.subscribe(this, "~/input/odometry", rclcpp::QoS{1}.get_rmw_qos_profile());

  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(10), sub_radar_, sub_odometry_);
  sync_ptr_->registerCallback(std::bind(&RadarStaticPointcloudFilterNode::onData, this, _1, _2));

  // Publisher
  pub_static_radar_ = create_publisher<RadarScan>("~/output/static_radar_scan", 1);
  pub_dynamic_radar_ = create_publisher<RadarScan>("~/output/dynamic_radar_scan", 1);
}

rcl_interfaces::msg::SetParametersResult RadarStaticPointcloudFilterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    auto & p = node_param_;
    update_param(params, "doppler_velocity_sd", p.doppler_velocity_sd);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

void RadarStaticPointcloudFilterNode::onData(
  const RadarScan::ConstSharedPtr radar_msg, const Odometry::ConstSharedPtr odom_msg)
{
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform;

  try {
    transform = transform_listener_->getTransform(
      odom_msg->header.frame_id, radar_msg->header.frame_id, odom_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.2));
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform");
    return;
  }

  RadarScan static_radar{};
  RadarScan dynamic_radar{};
  static_radar.header = radar_msg->header;
  dynamic_radar.header = radar_msg->header;

  for (const auto & radar_return : radar_msg->returns) {
    if (isStaticPointcloud(radar_return, odom_msg, transform)) {
      static_radar.returns.emplace_back(radar_return);
    } else {
      dynamic_radar.returns.emplace_back(radar_return);
    }
  }
  pub_static_radar_->publish(static_radar);
  pub_dynamic_radar_->publish(dynamic_radar);
}

bool RadarStaticPointcloudFilterNode::isStaticPointcloud(
  const RadarReturn & radar_return, const Odometry::ConstSharedPtr & odom_msg,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  geometry_msgs::msg::Vector3 compensated_velocity =
    compensateEgoVehicleTwist(radar_return, odom_msg->twist, transform);

  return (-node_param_.doppler_velocity_sd < compensated_velocity.x) &&
         (compensated_velocity.x < node_param_.doppler_velocity_sd);
}

}  // namespace radar_static_pointcloud_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_static_pointcloud_filter::RadarStaticPointcloudFilterNode)
