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

#include "detected_object_filter/object_lanelet_filter.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>

boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("object_lanelet_filter"), ex.what());
    return boost::none;
  }
}
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
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

void ObjectLaneletFilterNode::objectCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_auto_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  if (!lanelet_map_ptr_) {
    RCLCPP_ERROR(get_logger(), "No vector map received.");
    return;
  }
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!transformDetectedObjects(*input_msg, "map", tf_buffer_, transformed_objects)) {
    RCLCPP_ERROR(get_logger(), "Failed transform to map.");
    return;
  }

  // calculate convex hull
  const auto convex_hull = getConvexHull(transformed_objects);
  // get intersected lanelets
  lanelet::ConstLanelets intersected_lanelets = getIntersectedLanelets(convex_hull, road_lanelets_);

  int index = 0;
  for (const auto & object : transformed_objects.objects) {
    const auto & footprint = object.shape.footprint;
    const auto & position = object.kinematics.pose_with_covariance.pose.position;
    const auto & label = object.classification.front().label;
    if (
      (label == Label::UNKNOWN && filter_target_.UNKNOWN) ||
      (label == Label::CAR && filter_target_.CAR) ||
      (label == Label::TRUCK && filter_target_.TRUCK) ||
      (label == Label::BUS && filter_target_.BUS) ||
      (label == Label::TRAILER && filter_target_.TRAILER) ||
      (label == Label::MOTORCYCLE && filter_target_.MOTORCYCLE) ||
      (label == Label::BICYCLE && filter_target_.BICYCLE) ||
      (label == Label::PEDESTRIAN && filter_target_.PEDESTRIAN)) {
      Polygon2d polygon;
      for (const auto & point : footprint.points) {
        polygon.outer().emplace_back(point.x + position.x, point.y + position.y);
      }
      polygon.outer().push_back(polygon.outer().front());
      if (isPolygonOverlapLanelets(polygon, intersected_lanelets)) {
        output_object_msg.objects.emplace_back(input_msg->objects.at(index));
      }
    } else {
      output_object_msg.objects.emplace_back(input_msg->objects.at(index));
    }
    ++index;
  }
  object_pub_->publish(output_object_msg);
}

bool ObjectLaneletFilterNode::transformDetectedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_auto_perception_msgs::msg::DetectedObjects & output_msg)
{
  output_msg = input_msg;

  /* transform to world coordinate */
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, object.kinematics.pose_with_covariance.pose);
    }
  }
  return true;
}

LinearRing2d ObjectLaneletFilterNode::getConvexHull(
  const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects)
{
  MultiPoint2d candidate_points;
  for (const auto & object : detected_objects.objects) {
    const auto & pos = object.kinematics.pose_with_covariance.pose.position;
    for (const auto & p : object.shape.footprint.points) {
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
