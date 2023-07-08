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

#ifndef TRAFFIC_LIGHT_CLASSIFIER__CLASSIFIER_INTERFACE_HPP_
#define TRAFFIC_LIGHT_CLASSIFIER__CLASSIFIER_INTERFACE_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tier4_perception_msgs/msg/traffic_signal_array.hpp>

#include <vector>

namespace traffic_light
{
class ClassifierInterface
{
public:
  virtual bool getTrafficSignals(
    const std::vector<cv::Mat> & input_image,
    tier4_perception_msgs::msg::TrafficSignalArray & traffic_signals) = 0;
};
}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER__CLASSIFIER_INTERFACE_HPP_
