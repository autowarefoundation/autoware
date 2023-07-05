// Copyright 2023 The Autoware Contributors
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

#include "map_height_fitter/map_height_fitter.hpp"

#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>

#include <memory>

using tier4_localization_msgs::srv::PoseWithCovarianceStamped;

class MapHeightFitterNode : public rclcpp::Node
{
public:
  MapHeightFitterNode() : Node("map_height_fitter"), fitter_(this)
  {
    const auto on_service = [this](
                              const PoseWithCovarianceStamped::Request::SharedPtr req,
                              const PoseWithCovarianceStamped::Response::SharedPtr res) {
      const auto & pose = req->pose_with_covariance;
      const auto fitted = fitter_.fit(pose.pose.pose.position, pose.header.frame_id);
      if (fitted) {
        res->pose_with_covariance = req->pose_with_covariance;
        res->pose_with_covariance.pose.pose.position = fitted.value();
        res->success = true;
      } else {
        res->success = false;
      }
    };
    srv_ = create_service<PoseWithCovarianceStamped>("~/service", on_service);
  }

private:
  map_height_fitter::MapHeightFitter fitter_;
  rclcpp::Service<PoseWithCovarianceStamped>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MapHeightFitterNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
