// Copyright 2023 Tier IV, Inc.
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

#include "traffic_light_occlusion_predictor/nodelet.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace traffic_light
{

TrafficLightOcclusionPredictorNodelet::TrafficLightOcclusionPredictorNodelet(
  const rclcpp::NodeOptions & node_options)
: Node("traffic_light_occlusion_predictor_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // subscribers
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightOcclusionPredictorNodelet::mapCallback, this, _1));

  // publishers
  signal_pub_ =
    create_publisher<tier4_perception_msgs::msg::TrafficSignalArray>("~/output/traffic_signals", 1);

  // configuration parameters
  config_.azimuth_occlusion_resolution_deg =
    declare_parameter<double>("azimuth_occlusion_resolution_deg", 0.15);
  config_.elevation_occlusion_resolution_deg =
    declare_parameter<double>("elevation_occlusion_resolution_deg", 0.08);
  config_.max_valid_pt_dist = declare_parameter<double>("max_valid_pt_dist", 50.0);
  config_.max_image_cloud_delay = declare_parameter<double>("max_image_cloud_delay", 1.0);
  config_.max_wait_t = declare_parameter<double>("max_wait_t", 0.02);
  config_.max_occlusion_ratio = declare_parameter<int>("max_occlusion_ratio", 50);

  cloud_occlusion_predictor_ = std::make_shared<CloudOcclusionPredictor>(
    this, config_.max_valid_pt_dist, config_.azimuth_occlusion_resolution_deg,
    config_.elevation_occlusion_resolution_deg);

  const std::vector<std::string> topics{
    "~/input/car/traffic_signals", "~/input/rois", "~/input/camera_info", "~/input/cloud"};
  const std::vector<rclcpp::QoS> qos(topics.size(), rclcpp::SensorDataQoS());
  synchronizer_ = std::make_shared<SynchronizerType>(
    this, topics, qos,
    std::bind(
      &TrafficLightOcclusionPredictorNodelet::syncCallback, this, _1, _2, _3, _4,
      tier4_perception_msgs::msg::TrafficLightRoi::CAR_TRAFFIC_LIGHT),
    config_.max_image_cloud_delay, config_.max_wait_t);

  const std::vector<std::string> topics_ped{
    "~/input/pedestrian/traffic_signals", "~/input/rois", "~/input/camera_info", "~/input/cloud"};
  const std::vector<rclcpp::QoS> qos_ped(topics_ped.size(), rclcpp::SensorDataQoS());
  synchronizer_ped_ = std::make_shared<SynchronizerType>(
    this, topics_ped, qos_ped,
    std::bind(
      &TrafficLightOcclusionPredictorNodelet::syncCallback, this, _1, _2, _3, _4,
      tier4_perception_msgs::msg::TrafficLightRoi::PEDESTRIAN_TRAFFIC_LIGHT),
    config_.max_image_cloud_delay, config_.max_wait_t);

  subscribed_.resize(2, false);
}

void TrafficLightOcclusionPredictorNodelet::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg)
{
  traffic_light_position_map_.clear();
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (!lsp.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      lanelet::ConstLineString3d string3d = static_cast<lanelet::ConstLineString3d>(lsp);
      traffic_light_position_map_[lsp.id()] = traffic_light_utils::getTrafficLightCenter(string3d);
    }
  }
}

void TrafficLightOcclusionPredictorNodelet::syncCallback(
  const tier4_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr in_signal_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_cam_info_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_cloud_msg,
  const uint8_t traffic_light_type)
{
  std::vector<int> occlusion_ratios;
  if (in_cloud_msg == nullptr || in_cam_info_msg == nullptr || in_roi_msg == nullptr) {
    occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
  } else {
    tier4_perception_msgs::msg::TrafficLightRoiArray selected_roi_msg;
    selected_roi_msg.rois.reserve(in_roi_msg->rois.size());
    for (size_t i = 0; i < in_roi_msg->rois.size(); ++i) {
      if (in_roi_msg->rois.at(i).traffic_light_type == traffic_light_type) {
        selected_roi_msg.rois.push_back(in_roi_msg->rois.at(i));
      }
    }

    tier4_perception_msgs::msg::TrafficSignalArray out_msg = *in_signal_msg;

    if (selected_roi_msg.rois.size() != in_signal_msg->signals.size()) {
      occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
    } else {
      auto selected_roi_msg_ptr =
        std::make_shared<const tier4_perception_msgs::msg::TrafficLightRoiArray>(selected_roi_msg);
      cloud_occlusion_predictor_->predict(
        in_cam_info_msg, selected_roi_msg_ptr, in_cloud_msg, tf_buffer_,
        traffic_light_position_map_, occlusion_ratios);
    }
  }

  size_t predicted_num = out_msg_.signals.size();

  for (size_t i = 0; i < occlusion_ratios.size(); i++) {
    out_msg_.signals.push_back(in_signal_msg->signals.at(i));

    if (occlusion_ratios[i] >= config_.max_occlusion_ratio) {
      traffic_light_utils::setSignalUnknown(out_msg_.signals.at(predicted_num + i), 0.0);
    }
  }
  subscribed_.at(traffic_light_type) = true;

  if (std::all_of(subscribed_.begin(), subscribed_.end(), [](bool v) { return v; })) {
    auto pub_msg = std::make_unique<tier4_perception_msgs::msg::TrafficSignalArray>(out_msg_);
    pub_msg->header = in_signal_msg->header;
    signal_pub_->publish(std::move(pub_msg));
    out_msg_.signals.clear();
    std::fill(subscribed_.begin(), subscribed_.end(), false);
  }
}
}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightOcclusionPredictorNodelet)
