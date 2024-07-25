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

#ifndef TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_NODE_HPP_
#define TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <lanelet2_core/Forward.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <list>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

namespace mf = message_filters;

struct FusionRecord
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo cam_info;
  tier4_perception_msgs::msg::TrafficLightRoi roi;
  tier4_perception_msgs::msg::TrafficLight signal;
};

struct FusionRecordArr
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::CameraInfo cam_info;
  tier4_perception_msgs::msg::TrafficLightRoiArray rois;
  tier4_perception_msgs::msg::TrafficLightArray signals;
};

bool operator<(const FusionRecordArr & r1, const FusionRecordArr & r2)
{
  return rclcpp::Time(r1.header.stamp) < rclcpp::Time(r2.header.stamp);
}

class MultiCameraFusion : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::CameraInfo CamInfoType;
  typedef tier4_perception_msgs::msg::TrafficLightRoi RoiType;
  typedef tier4_perception_msgs::msg::TrafficLight SignalType;
  typedef tier4_perception_msgs::msg::TrafficLightArray SignalArrayType;
  typedef tier4_perception_msgs::msg::TrafficLightRoiArray RoiArrayType;
  typedef tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type IdType;
  typedef autoware_perception_msgs::msg::TrafficLightGroup NewSignalType;
  typedef autoware_perception_msgs::msg::TrafficLightGroupArray NewSignalArrayType;

  typedef std::pair<RoiArrayType, SignalArrayType> RecordArrayType;

  explicit MultiCameraFusion(const rclcpp::NodeOptions & node_options);

private:
  void trafficSignalRoiCallback(
    const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
    const SignalArrayType::ConstSharedPtr signal_msg);

  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg);

  void multiCameraFusion(std::map<IdType, FusionRecord> & fused_record_map);

  void convertOutputMsg(
    const std::map<IdType, FusionRecord> & grouped_record_map, NewSignalArrayType & msg_out);

  void groupFusion(
    std::map<IdType, FusionRecord> & fused_record_map,
    std::map<IdType, FusionRecord> & grouped_record_map);

  typedef mf::sync_policies::ExactTime<CamInfoType, RoiArrayType, SignalArrayType> ExactSyncPolicy;
  typedef mf::Synchronizer<ExactSyncPolicy> ExactSync;
  typedef mf::sync_policies::ApproximateTime<CamInfoType, RoiArrayType, SignalArrayType>
    ApproximateSyncPolicy;
  typedef mf::Synchronizer<ApproximateSyncPolicy> ApproximateSync;

  std::vector<std::unique_ptr<mf::Subscriber<SignalArrayType>>> signal_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<RoiArrayType>>> roi_subs_;
  std::vector<std::unique_ptr<mf::Subscriber<CamInfoType>>> cam_info_subs_;
  std::vector<std::unique_ptr<ExactSync>> exact_sync_subs_;
  std::vector<std::unique_ptr<ApproximateSync>> approximate_sync_subs_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;

  rclcpp::Publisher<NewSignalArrayType>::SharedPtr signal_pub_;
  /*
  the mapping from traffic light id (instance id) to regulatory element id (group id)
  */
  std::map<lanelet::Id, std::vector<lanelet::Id>> traffic_light_id_to_regulatory_ele_id_;
  /*
  save record arrays by increasing timestamp order.
  use multiset in case there are multiple cameras publishing images at exactly the same time
  */
  std::multiset<FusionRecordArr> record_arr_set_;
  bool is_approximate_sync_;
  /*
  for every input message input_m, if the timestamp difference between input_m and the latest
  message is smaller than message_lifespan_, then input_m would be used for the fusion. Otherwise,
  it would be discarded
  */
  double message_lifespan_;
};
}  // namespace autoware::traffic_light
#endif  // TRAFFIC_LIGHT_MULTI_CAMERA_FUSION_NODE_HPP_
