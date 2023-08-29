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

#include "radar_scan_to_pointcloud2/radar_scan_to_pointcloud2_node.hpp"

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

pcl::PointXYZI getPointXYZI(const radar_msgs::msg::RadarReturn & radar, float intensity)
{
  pcl::PointXYZI point;
  const float r_xy = radar.range * std::cos(radar.elevation);
  point.x = r_xy * std::cos(radar.azimuth);
  point.y = r_xy * std::sin(radar.azimuth);
  point.z = radar.range * std::sin(radar.elevation);
  point.intensity = intensity;
  return point;
}

pcl::PointCloud<pcl::PointXYZI> toAmplitudePCL(const radar_msgs::msg::RadarScan & radar_scan)
{
  pcl::PointCloud<pcl::PointXYZI> pcl;
  for (const auto & radar : radar_scan.returns) {
    pcl.push_back(getPointXYZI(radar, radar.amplitude));
  }
  return pcl;
}

sensor_msgs::msg::PointCloud2 toAmplitudePointcloud2(const radar_msgs::msg::RadarScan & radar_scan)
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  auto pcl_pointcloud = toAmplitudePCL(radar_scan);
  pcl::toROSMsg(pcl_pointcloud, pointcloud_msg);
  pointcloud_msg.header = radar_scan.header;
  return pointcloud_msg;
}

pcl::PointCloud<pcl::PointXYZI> toDopplerPCL(const radar_msgs::msg::RadarScan & radar_scan)
{
  pcl::PointCloud<pcl::PointXYZI> pcl;
  for (const auto & radar : radar_scan.returns) {
    pcl.push_back(getPointXYZI(radar, radar.doppler_velocity));
  }
  return pcl;
}

sensor_msgs::msg::PointCloud2 toDopplerPointcloud2(const radar_msgs::msg::RadarScan & radar_scan)
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  auto pcl_pointcloud = toDopplerPCL(radar_scan);
  pcl::toROSMsg(pcl_pointcloud, pointcloud_msg);
  pointcloud_msg.header = radar_scan.header;
  return pointcloud_msg;
}
}  // namespace

namespace radar_scan_to_pointcloud2
{
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;
using sensor_msgs::msg::PointCloud2;

RadarScanToPointcloud2Node::RadarScanToPointcloud2Node(const rclcpp::NodeOptions & node_options)
: Node("radar_scan_to_pointcloud2", node_options)
{
  using std::placeholders::_1;

  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarScanToPointcloud2Node::onSetParam, this, _1));

  // Node Parameter
  node_param_.publish_amplitude_pointcloud =
    declare_parameter<bool>("publish_amplitude_pointcloud");
  node_param_.publish_doppler_pointcloud = declare_parameter<bool>("publish_doppler_pointcloud");

  // Subscriber
  sub_radar_ = create_subscription<RadarScan>(
    "~/input/radar", rclcpp::QoS{1}, std::bind(&RadarScanToPointcloud2Node::onData, this, _1));

  // Publisher
  pub_amplitude_pointcloud_ = create_publisher<PointCloud2>("~/output/amplitude_pointcloud", 1);
  pub_doppler_pointcloud_ = create_publisher<PointCloud2>("~/output/doppler_pointcloud", 1);
}

rcl_interfaces::msg::SetParametersResult RadarScanToPointcloud2Node::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    auto & p = node_param_;
    update_param(params, "publish_amplitude_pointcloud", p.publish_amplitude_pointcloud);
    update_param(params, "publish_doppler_pointcloud", p.publish_doppler_pointcloud);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

void RadarScanToPointcloud2Node::onData(const RadarScan::ConstSharedPtr radar_msg)
{
  if (node_param_.publish_amplitude_pointcloud) {
    amplitude_pointcloud = toAmplitudePointcloud2(*radar_msg);
    pub_amplitude_pointcloud_->publish(amplitude_pointcloud);
  }
  if (node_param_.publish_doppler_pointcloud) {
    doppler_pointcloud = toDopplerPointcloud2(*radar_msg);
    pub_doppler_pointcloud_->publish(doppler_pointcloud);
  }
}

}  // namespace radar_scan_to_pointcloud2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_scan_to_pointcloud2::RadarScanToPointcloud2Node)
