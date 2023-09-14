// Copyright 2023 TIER IV, Inc.
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

#include "traffic_light_multi_camera_fusion/node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace
{

bool isUnknown(const tier4_perception_msgs::msg::TrafficSignal & signal)
{
  return signal.elements.size() == 1 &&
         signal.elements[0].color == tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN &&
         signal.elements[0].shape == tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
}

/**
 * @brief Currently the visible score only considers the truncation.
 * If the detection roi is very close to the image boundary, it would be considered as truncated.
 *
 * @param record    fusion record
 * @return 0 if traffic light is truncated, otherwise 1
 */
int calVisibleScore(const traffic_light::FusionRecord & record)
{
  const uint32_t boundary = 5;
  uint32_t x1 = record.roi.roi.x_offset;
  uint32_t x2 = record.roi.roi.x_offset + record.roi.roi.width;
  uint32_t y1 = record.roi.roi.y_offset;
  uint32_t y2 = record.roi.roi.y_offset + record.roi.roi.height;
  if (
    x1 <= boundary || (record.cam_info.width - x2) <= boundary || y1 <= boundary ||
    (record.cam_info.height - y2) <= boundary) {
    return 0;
  } else {
    return 1;
  }
}

int compareRecord(const traffic_light::FusionRecord & r1, const traffic_light::FusionRecord & r2)
{
  /*
  if both records are from the same sensor but different stamp, trust the latest one
  */
  double t1 = rclcpp::Time(r1.header.stamp).seconds();
  double t2 = rclcpp::Time(r2.header.stamp).seconds();
  const double dt_thres = 1e-3;
  if (r1.header.frame_id == r2.header.frame_id && std::abs(t1 - t2) >= dt_thres) {
    return t1 < t2 ? -1 : 1;
  }
  bool r1_is_unknown = isUnknown(r1.signal);
  bool r2_is_unknown = isUnknown(r2.signal);
  /*
  if both are unknown, they are of the same priority
  */
  if (r1_is_unknown && r2_is_unknown) {
    return 0;
  } else if (r1_is_unknown ^ r2_is_unknown) {
    /*
    if either is unknown, the unknown is of lower priority
    */
    return r1_is_unknown ? -1 : 1;
  }
  int visible_score_1 = calVisibleScore(r1);
  int visible_score_2 = calVisibleScore(r2);
  if (visible_score_1 == visible_score_2) {
    int area_1 = r1.roi.roi.width * r1.roi.roi.height;
    int area_2 = r2.roi.roi.width * r2.roi.roi.height;
    if (area_1 < area_2) {
      return -1;
    } else {
      return static_cast<int>(area_1 > area_2);
    }
  } else {
    return visible_score_1 < visible_score_2 ? -1 : 1;
  }
}

template <class K, class V>
V at_or(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  return map.count(key) ? map.at(key) : value;
}

autoware_perception_msgs::msg::TrafficSignalElement convert(
  const tier4_perception_msgs::msg::TrafficLightElement & input)
{
  typedef tier4_perception_msgs::msg::TrafficLightElement OldElem;
  typedef autoware_perception_msgs::msg::TrafficSignalElement NewElem;
  static const std::unordered_map<OldElem::_color_type, NewElem::_color_type> color_map(
    {{OldElem::RED, NewElem::RED},
     {OldElem::AMBER, NewElem::AMBER},
     {OldElem::GREEN, NewElem::GREEN},
     {OldElem::WHITE, NewElem::WHITE}});
  static const std::unordered_map<OldElem::_shape_type, NewElem::_shape_type> shape_map(
    {{OldElem::CIRCLE, NewElem::CIRCLE},
     {OldElem::LEFT_ARROW, NewElem::LEFT_ARROW},
     {OldElem::RIGHT_ARROW, NewElem::RIGHT_ARROW},
     {OldElem::UP_ARROW, NewElem::UP_ARROW},
     {OldElem::UP_LEFT_ARROW, NewElem::UP_LEFT_ARROW},
     {OldElem::UP_RIGHT_ARROW, NewElem::UP_RIGHT_ARROW},
     {OldElem::DOWN_ARROW, NewElem::DOWN_ARROW},
     {OldElem::DOWN_LEFT_ARROW, NewElem::DOWN_LEFT_ARROW},
     {OldElem::DOWN_RIGHT_ARROW, NewElem::DOWN_RIGHT_ARROW},
     {OldElem::CROSS, NewElem::CROSS}});
  static const std::unordered_map<OldElem::_status_type, NewElem::_status_type> status_map(
    {{OldElem::SOLID_OFF, NewElem::SOLID_OFF},
     {OldElem::SOLID_ON, NewElem::SOLID_ON},
     {OldElem::FLASHING, NewElem::FLASHING}});
  // clang-format on

  NewElem output;
  output.color = at_or(color_map, input.color, NewElem::UNKNOWN);
  output.shape = at_or(shape_map, input.shape, NewElem::UNKNOWN);
  output.status = at_or(status_map, input.status, NewElem::UNKNOWN);
  output.confidence = input.confidence;
  return output;
}

}  // namespace

namespace traffic_light
{

MultiCameraFusion::MultiCameraFusion(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_multi_camera_fusion", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  std::vector<std::string> camera_namespaces =
    this->declare_parameter("camera_namespaces", std::vector<std::string>{});
  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync", false);
  message_lifespan_ = this->declare_parameter<double>("message_lifespan", 0.09);
  for (const std::string & camera_ns : camera_namespaces) {
    std::string signal_topic = camera_ns + "/classification/traffic_signals";
    std::string roi_topic = camera_ns + "/detection/rois";
    std::string cam_info_topic = camera_ns + "/camera_info";
    roi_subs_.emplace_back(
      new mf::Subscriber<RoiArrayType>(this, roi_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    signal_subs_.emplace_back(new mf::Subscriber<SignalArrayType>(
      this, signal_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    cam_info_subs_.emplace_back(new mf::Subscriber<CamInfoType>(
      this, cam_info_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile()));
    if (is_approximate_sync_ == false) {
      exact_sync_subs_.emplace_back(new ExactSync(
        ExactSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      exact_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    } else {
      approximate_sync_subs_.emplace_back(new ApproximateSync(
        ApproximateSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      approximate_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    }
  }

  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MultiCameraFusion::mapCallback, this, _1));
  signal_pub_ = create_publisher<NewSignalArrayType>("~/output/traffic_signals", rclcpp::QoS{1});
}

void MultiCameraFusion::trafficSignalRoiCallback(
  const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
  const SignalArrayType::ConstSharedPtr signal_msg)
{
  rclcpp::Time stamp(roi_msg->header.stamp);
  /*
  Insert the received record array to the table.
  Attention should be payed that this record array might not have the newest timestamp
  */
  record_arr_set_.insert(
    FusionRecordArr{cam_info_msg->header, *cam_info_msg, *roi_msg, *signal_msg});

  std::map<IdType, FusionRecord> fused_record_map, grouped_record_map;
  multiCameraFusion(fused_record_map);
  groupFusion(fused_record_map, grouped_record_map);

  NewSignalArrayType msg_out;
  convertOutputMsg(grouped_record_map, msg_out);
  msg_out.stamp = cam_info_msg->header.stamp;
  signal_pub_->publish(msg_out);
}

void MultiCameraFusion::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg)
{
  lanelet::LaneletMapPtr lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (const auto & light : lights) {
      traffic_light_id_to_regulatory_ele_id_[light.id()].emplace_back(tl->id());
    }
  }
}

void MultiCameraFusion::convertOutputMsg(
  const std::map<IdType, FusionRecord> & grouped_record_map, NewSignalArrayType & msg_out)
{
  msg_out.signals.clear();
  for (const auto & p : grouped_record_map) {
    IdType reg_ele_id = p.first;
    const SignalType & signal = p.second.signal;
    NewSignalType signal_out;
    signal_out.traffic_signal_id = reg_ele_id;
    for (const auto & ele : signal.elements) {
      signal_out.elements.push_back(convert(ele));
    }
    msg_out.signals.push_back(signal_out);
  }
}

void MultiCameraFusion::multiCameraFusion(std::map<IdType, FusionRecord> & fused_record_map)
{
  fused_record_map.clear();
  /*
  this should not happen. Just in case
  */
  if (record_arr_set_.empty()) {
    RCLCPP_ERROR(get_logger(), "record_arr_set_ is empty! This should not happen");
    return;
  }
  const rclcpp::Time & newest_stamp(record_arr_set_.rbegin()->header.stamp);
  for (auto it = record_arr_set_.begin(); it != record_arr_set_.end();) {
    /*
    remove all old record arrays whose timestamp difference with newest record is larger than
    threshold
    */
    if (
      (newest_stamp - rclcpp::Time(it->header.stamp)) >
      rclcpp::Duration::from_seconds(message_lifespan_)) {
      it = record_arr_set_.erase(it);
    } else {
      /*
      generate fused record result with the saved records
      */
      const FusionRecordArr & record_arr = *it;
      for (size_t i = 0; i < record_arr.rois.rois.size(); i++) {
        const RoiType & roi = record_arr.rois.rois[i];
        auto signal_it = std::find_if(
          record_arr.signals.signals.begin(), record_arr.signals.signals.end(),
          [roi](const SignalType & s1) { return roi.traffic_light_id == s1.traffic_light_id; });
        /*
        failed to find corresponding signal. skip it
        */
        if (signal_it == record_arr.signals.signals.end()) {
          continue;
        }
        FusionRecord record{record_arr.header, record_arr.cam_info, roi, *signal_it};
        /*
        if this traffic light is not detected yet or can be updated by higher priority record,
        update it
        */
        if (
          fused_record_map.find(roi.traffic_light_id) == fused_record_map.end() ||
          ::compareRecord(record, fused_record_map[roi.traffic_light_id]) >= 0) {
          fused_record_map[roi.traffic_light_id] = record;
        }
      }
      it++;
    }
  }
}

void MultiCameraFusion::groupFusion(
  std::map<IdType, FusionRecord> & fused_record_map,
  std::map<IdType, FusionRecord> & grouped_record_map)
{
  grouped_record_map.clear();
  for (auto & p : fused_record_map) {
    IdType roi_id = p.second.roi.traffic_light_id;
    /*
    this should not happen
    */
    if (traffic_light_id_to_regulatory_ele_id_.count(roi_id) == 0) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Found Traffic Light Id = " << roi_id << " which is not defined in Map");
      continue;
    }

    /*
    keep the best record for every regulatory element id
    */
    const auto reg_ele_id_vec =
      traffic_light_id_to_regulatory_ele_id_[p.second.roi.traffic_light_id];
    for (const auto & reg_ele_id : reg_ele_id_vec) {
      if (
        grouped_record_map.count(reg_ele_id) == 0 ||
        ::compareRecord(p.second, grouped_record_map[reg_ele_id]) >= 0) {
        grouped_record_map[reg_ele_id] = p.second;
      }
    }
  }
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::MultiCameraFusion)
