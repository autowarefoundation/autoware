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

#include "detected_object_validation/detected_object_filter/object_lanelet_filter.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <object_recognition_utils/object_recognition_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace object_lanelet_filter
{
ObjectLaneletFilterNode::ObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options)
: Node("object_lanelet_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN", false);
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR", false);
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK", false);
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS", false);
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER", false);
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE", false);
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE", false);
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN", false);

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObjectLaneletFilterNode::mapCallback, this, _1));
  object_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectLaneletFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});
}

void ObjectLaneletFilterNode::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_frame_id_ = map_msg->header.frame_id;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
}

void ObjectLaneletFilterNode::objectCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_auto_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  if (!lanelet_map_ptr_) {
    RCLCPP_ERROR(get_logger(), "No vector map received.");
    return;
  }
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *input_msg, lanelet_frame_id_, tf_buffer_, transformed_objects)) {
    RCLCPP_ERROR(get_logger(), "Failed transform to %s.", lanelet_frame_id_.c_str());
    return;
  }

  // calculate convex hull
  const auto convex_hull = getConvexHull(transformed_objects);
  // get intersected lanelets
  lanelet::ConstLanelets intersected_road_lanelets =
    getIntersectedLanelets(convex_hull, road_lanelets_);
  lanelet::ConstLanelets intersected_shoulder_lanelets =
    getIntersectedLanelets(convex_hull, shoulder_lanelets_);

  int index = 0;
  for (const auto & object : transformed_objects.objects) {
    const auto footprint = setFootprint(object);
    const auto & label = object.classification.front().label;
    if (filter_target_.isTarget(label)) {
      Polygon2d polygon;
      for (const auto & point : footprint.points) {
        const geometry_msgs::msg::Point32 point_transformed =
          tier4_autoware_utils::transformPoint(point, object.kinematics.pose_with_covariance.pose);
        polygon.outer().emplace_back(point_transformed.x, point_transformed.y);
      }
      polygon.outer().push_back(polygon.outer().front());
      if (isPolygonOverlapLanelets(polygon, intersected_road_lanelets)) {
        output_object_msg.objects.emplace_back(input_msg->objects.at(index));
      } else if (isPolygonOverlapLanelets(polygon, intersected_shoulder_lanelets)) {
        output_object_msg.objects.emplace_back(input_msg->objects.at(index));
      }
    } else {
      output_object_msg.objects.emplace_back(input_msg->objects.at(index));
    }
    ++index;
  }
  object_pub_->publish(output_object_msg);
}

geometry_msgs::msg::Polygon ObjectLaneletFilterNode::setFootprint(
  const autoware_auto_perception_msgs::msg::DetectedObject & detected_object)
{
  geometry_msgs::msg::Polygon footprint;
  if (detected_object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto object_size = detected_object.shape.dimensions;
    const double x_front = object_size.x / 2.0;
    const double x_rear = -object_size.x / 2.0;
    const double y_left = object_size.y / 2.0;
    const double y_right = -object_size.y / 2.0;

    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_left).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_left).z(0.0));
  } else {
    footprint = detected_object.shape.footprint;
  }
  return footprint;
}

LinearRing2d ObjectLaneletFilterNode::getConvexHull(
  const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects)
{
  MultiPoint2d candidate_points;
  for (const auto & object : detected_objects.objects) {
    const auto & pos = object.kinematics.pose_with_covariance.pose.position;
    const auto footprint = setFootprint(object);
    for (const auto & p : footprint.points) {
      candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
    }
  }

  LinearRing2d convex_hull;
  boost::geometry::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

lanelet::ConstLanelets ObjectLaneletFilterNode::getIntersectedLanelets(
  const LinearRing2d & convex_hull, const lanelet::ConstLanelets & road_lanelets)
{
  lanelet::ConstLanelets intersected_lanelets;

  // WARNING: This implementation currently iterate all lanelets, which could degrade performance
  // when handling large sized map.
  for (const auto & road_lanelet : road_lanelets) {
    if (boost::geometry::intersects(convex_hull, road_lanelet.polygon2d().basicPolygon())) {
      intersected_lanelets.emplace_back(road_lanelet);
    }
  }
  return intersected_lanelets;
}

bool ObjectLaneletFilterNode::isPolygonOverlapLanelets(
  const Polygon2d & polygon, const lanelet::ConstLanelets & intersected_lanelets)
{
  for (const auto & lanelet : intersected_lanelets) {
    if (!boost::geometry::disjoint(polygon, lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

}  // namespace object_lanelet_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_lanelet_filter::ObjectLaneletFilterNode)
