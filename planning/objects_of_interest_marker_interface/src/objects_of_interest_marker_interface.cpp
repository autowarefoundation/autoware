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

#include "objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/constants.hpp>
#include <tier4_autoware_utils/math/trigonometry.hpp>

namespace objects_of_interest_marker_interface
{
using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::Pose;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

ObjectsOfInterestMarkerInterface::ObjectsOfInterestMarkerInterface(
  rclcpp::Node * node, const std::string & name)
: name_{name}
{
  // Publisher
  pub_marker_ = node->create_publisher<MarkerArray>(topic_namespace_ + "/" + name, 1);
}

void ObjectsOfInterestMarkerInterface::insertObjectData(
  const Pose & pose, const Shape & shape, const ColorName & color_name)
{
  insertObjectDataWithCustomColor(pose, shape, getColor(color_name));
}

void ObjectsOfInterestMarkerInterface::insertObjectDataWithCustomColor(
  const Pose & pose, const Shape & shape, const ColorRGBA & color)
{
  ObjectMarkerData data;
  data.pose = pose;
  data.shape = shape;
  data.color = color;

  obj_marker_data_array_.push_back(data);
}

void ObjectsOfInterestMarkerInterface::publishMarkerArray()
{
  MarkerArray marker_array;
  for (size_t i = 0; i < obj_marker_data_array_.size(); ++i) {
    const auto data = obj_marker_data_array_.at(i);
    const MarkerArray target_marker =
      marker_utils::createTargetMarker(i, data, getName(), getHeightOffset());
    marker_array.markers.insert(
      marker_array.markers.end(), target_marker.markers.begin(), target_marker.markers.end());
  }
  pub_marker_->publish(marker_array);
  obj_marker_data_array_.clear();
}

void ObjectsOfInterestMarkerInterface::setHeightOffset(const double offset)
{
  height_offset_ = offset;
}

ColorRGBA ObjectsOfInterestMarkerInterface::getColor(
  const ColorName & color_name, const float alpha)
{
  switch (color_name) {
    case ColorName::GREEN:
      return coloring::getGreen(alpha);
    case ColorName::AMBER:
      return coloring::getAmber(alpha);
    case ColorName::RED:
      return coloring::getRed(alpha);
    case ColorName::GRAY:
      return coloring::getGray(alpha);
    default:
      return coloring::getGray(alpha);
  }
}

}  // namespace objects_of_interest_marker_interface
