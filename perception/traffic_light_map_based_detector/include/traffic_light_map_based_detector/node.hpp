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

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace traffic_light
{
class MapBasedDetector : public rclcpp::Node
{
public:
  explicit MapBasedDetector(const rclcpp::NodeOptions & node_options);

private:
  struct Config
  {
    double max_vibration_pitch;
    double max_vibration_yaw;
    double max_vibration_height;
    double max_vibration_width;
    double max_vibration_depth;
    double min_timestamp_offset;
    double max_timestamp_offset;
    double timestamp_sample_len;
    double max_detection_range;
    double car_traffic_light_max_angle_range;
    double pedestrian_traffic_light_max_angle_range;
  };

  struct IdLessThan
  {
    bool operator()(
      const lanelet::ConstLineString3d & left, const lanelet::ConstLineString3d & right) const
    {
      return left.id() < right.id();
    }
  };

private:
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  /**
   * @brief publish the rois of traffic lights with angular and distance offset
   *
   */
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
  /**
   * @brief publish the rois of traffic lights with zero angular and distance offset
   *
   */
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr expect_roi_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  using TrafficLightSet = std::set<lanelet::ConstLineString3d, IdLessThan>;

  std::shared_ptr<TrafficLightSet> all_traffic_lights_ptr_;
  std::shared_ptr<TrafficLightSet> route_traffic_lights_ptr_;

  std::set<int64_t> pedestrian_tl_id_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;

  Config config_;
  /**
   * @brief Calculated the transform from map to frame_id at timestamp t
   *
   * @param t           specified timestamp
   * @param frame_id    specified target frame id
   * @param tf          calculated transform
   * @return true       calculation succeed
   * @return false      calculation failed
   */
  bool getTransform(
    const rclcpp::Time & t, const std::string & frame_id, tf2::Transform & tf) const;
  /**
   * @brief callback function for the map message
   *
   * @param input_msg
   */
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg);
  /**
   * @brief callback function for the camera info message. The main process function of the node
   *
   * @param input_msg
   */
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg);
  /**
   * @brief callback function for the route message
   *
   * @param input_msg
   */
  void routeCallback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr input_msg);
  /**
   * @brief Get the Visible Traffic Lights object
   *
   * @param all_traffic_lights      all the traffic lights in the route or in the map
   * @param tf_map2camera_vec           the transformation sequences from map to camera
   * @param pinhole_camera_model    pinhole model calculated from camera_info
   * @param visible_traffic_lights  the visible traffic lights object
   */
  void getVisibleTrafficLights(
    const TrafficLightSet & all_traffic_lights,
    const std::vector<tf2::Transform> & tf_map2camera_vec,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    std::vector<lanelet::ConstLineString3d> & visible_traffic_lights) const;
  /**
   * @brief Get the Traffic Light Roi from one tf
   *
   * @param tf_map2camera         the transformation from map to camera
   * @param pinhole_camera_model  pinhole model calculated from camera_info
   * @param traffic_light         lanelet traffic light object
   * @param config                offset configuration
   * @param roi                   computed result result
   * @return true                 the computation succeed
   * @return false                the computation failed
   */
  bool getTrafficLightRoi(
    const tf2::Transform & tf_map2camera,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    const lanelet::ConstLineString3d traffic_light, const Config & config,
    tier4_perception_msgs::msg::TrafficLightRoi & roi) const;
  /**
   * @brief Calculate one traffic light roi for every tf and return the roi containing all of them
   *
   * @param tf_map2camera_vec     the transformation vector
   * @param pinhole_camera_model  pinhole model calculated from camera_info
   * @param traffic_light         lanelet traffic light object
   * @param config                offset configuration
   * @param roi                   computed result result
   * @return true                 the computation succeed
   * @return false                the computation failed
   */
  bool getTrafficLightRoi(
    const std::vector<tf2::Transform> & tf_map2camera_vec,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    const lanelet::ConstLineString3d traffic_light, const Config & config,
    tier4_perception_msgs::msg::TrafficLightRoi & roi) const;
  /**
   * @brief Publish the traffic lights for visualization
   *
   * @param tf_map2camera           the transformation from map to camera
   * @param cam_info_header         header of the camera_info message
   * @param visible_traffic_lights  the visible traffic light object vector
   * @param pub                     publisher
   */
  void publishVisibleTrafficLights(
    const tf2::Transform & tf_map2camera, const std_msgs::msg::Header & cam_info_header,
    const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_
