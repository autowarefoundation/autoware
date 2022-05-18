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

#ifndef IMAGE_PROJECTION_BASED_FUSION__DEBUGGER_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__DEBUGGER_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <boost/circular_buffer.hpp>

#include <memory>
#include <vector>

namespace image_projection_based_fusion
{

using sensor_msgs::msg::RegionOfInterest;

class Debugger
{
public:
  explicit Debugger(
    rclcpp::Node * node_ptr, const std::size_t image_num, const std::size_t image_buffer_size);

  void publishImage(const std::size_t image_id, const rclcpp::Time & stamp);

  void clear();

  std::vector<RegionOfInterest> image_rois_;
  std::vector<RegionOfInterest> obstacle_rois_;
  std::vector<Eigen::Vector2d> obstacle_points_;

private:
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg, const std::size_t image_id);

  rclcpp::Node * node_ptr_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  std::vector<image_transport::Subscriber> image_subs_;
  std::vector<image_transport::Publisher> image_pubs_;
  std::vector<boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr>> image_buffers_;

  std::size_t image_buffer_size_;
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__DEBUGGER_HPP_
