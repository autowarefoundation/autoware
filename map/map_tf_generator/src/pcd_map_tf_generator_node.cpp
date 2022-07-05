// Copyright 2020 Tier IV, Inc.
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

#include "map_tf_generator/uniform_random.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

constexpr size_t N_SAMPLES = 20;

class PcdMapTFGeneratorNode : public rclcpp::Node
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  explicit PcdMapTFGeneratorNode(const rclcpp::NodeOptions & options)
  : Node("pcd_map_tf_generator", options)
  {
    map_frame_ = declare_parameter("map_frame", "map");
    viewer_frame_ = declare_parameter("viewer_frame", "viewer");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&PcdMapTFGeneratorNode::onPointCloud, this, std::placeholders::_1));

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

private:
  std::string map_frame_;
  std::string viewer_frame_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr clouds_ros)
  {
    // fix random seed to produce the same viewer position every time
    // 3939 is just the author's favorite number
    srand(3939);

    PointCloud clouds;
    pcl::fromROSMsg<pcl::PointXYZ>(*clouds_ros, clouds);

    const std::vector<size_t> indices = UniformRandom(clouds.size(), N_SAMPLES);
    double coordinate[3] = {0, 0, 0};
    for (const auto i : indices) {
      coordinate[0] += clouds.points[i].x;
      coordinate[1] += clouds.points[i].y;
      coordinate[2] += clouds.points[i].z;
    }
    coordinate[0] = coordinate[0] / indices.size();
    coordinate[1] = coordinate[1] / indices.size();
    coordinate[2] = coordinate[2] / indices.size();

    geometry_msgs::msg::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = this->now();
    static_transformStamped.header.frame_id = map_frame_;
    static_transformStamped.child_frame_id = viewer_frame_;
    static_transformStamped.transform.translation.x = coordinate[0];
    static_transformStamped.transform.translation.y = coordinate[1];
    static_transformStamped.transform.translation.z = coordinate[2];
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();

    static_broadcaster_->sendTransform(static_transformStamped);

    RCLCPP_INFO_STREAM(
      get_logger(), "broadcast static tf. map_frame:"
                      << map_frame_ << ", viewer_frame:" << viewer_frame_ << ", x:" << coordinate[0]
                      << ", y:" << coordinate[1] << ", z:" << coordinate[2]);
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PcdMapTFGeneratorNode)
