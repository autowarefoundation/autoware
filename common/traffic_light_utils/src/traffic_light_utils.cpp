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

#include "traffic_light_utils/traffic_light_utils.hpp"

namespace traffic_light_utils
{

bool isRoiValid(
  const tier4_perception_msgs::msg::TrafficLightRoi & roi, uint32_t width, uint32_t height)
{
  uint32_t x1 = roi.roi.x_offset;
  uint32_t x2 = roi.roi.x_offset + roi.roi.width;
  uint32_t y1 = roi.roi.y_offset;
  uint32_t y2 = roi.roi.y_offset + roi.roi.height;
  return roi.roi.width > 0 && roi.roi.height > 0 && x1 < width && y1 < height && x2 < width &&
         y2 < height;
}

void setRoiInvalid(tier4_perception_msgs::msg::TrafficLightRoi & roi)
{
  roi.roi.height = roi.roi.width = 0;
}

bool isSignalUnknown(const tier4_perception_msgs::msg::TrafficSignal & signal)
{
  return signal.elements.size() == 1 &&
         signal.elements[0].shape == tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN &&
         signal.elements[0].color == tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
}

void setSignalUnknown(tier4_perception_msgs::msg::TrafficSignal & signal, float confidence)
{
  signal.elements.resize(1);
  signal.elements[0].shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
  signal.elements[0].color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
  // the default value is -1, which means to not set confidence
  if (confidence > 0) {
    signal.elements[0].confidence = confidence;
  }
}

tf2::Vector3 getTrafficLightTopLeft(const lanelet::ConstLineString3d & traffic_light)
{
  const auto & tl_bl = traffic_light.front();
  const double tl_height = traffic_light.attributeOr("height", 0.0);
  return tf2::Vector3(tl_bl.x(), tl_bl.y(), tl_bl.z() + tl_height);
}

tf2::Vector3 getTrafficLightBottomRight(const lanelet::ConstLineString3d & traffic_light)
{
  const auto & tl_bl = traffic_light.back();
  return tf2::Vector3(tl_bl.x(), tl_bl.y(), tl_bl.z());
}

tf2::Vector3 getTrafficLightCenter(const lanelet::ConstLineString3d & traffic_light)
{
  tf2::Vector3 top_left = getTrafficLightTopLeft(traffic_light);
  tf2::Vector3 bottom_right = getTrafficLightBottomRight(traffic_light);
  return (top_left + bottom_right) / 2;
}

}  // namespace traffic_light_utils
