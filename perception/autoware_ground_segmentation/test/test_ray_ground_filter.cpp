// Copyright 2022 The Autoware Contributors
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

#include "../src/ray_ground_filter/node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif
#include <yaml-cpp/yaml.h>
class RayGroundFilterComponentTestSuite : public ::testing::Test
{
protected:
  void SetUp() { rclcpp::init(0, nullptr); }
  void TearDown() { (void)rclcpp::shutdown(); }
};  // sanity_check

class RayGroundFilterComponentTest : public autoware::ground_segmentation::RayGroundFilterComponent
{
public:
  explicit RayGroundFilterComponentTest(const rclcpp::NodeOptions & options)
  : RayGroundFilterComponent(options)
  {
    input_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test_ray_ground_filter/input_cloud", 1);

    output_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test_ray_ground_filter/output_cloud", 1);
  }

  ~RayGroundFilterComponentTest() {}

  void filter(const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
  {
    return RayGroundFilterComponent::filter(input, indices, output);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub_;
};

void convertPCL2PointCloud2(
  const pcl::PointCloud<pcl::PointXYZI> & pcl_cloud, sensor_msgs::msg::PointCloud2 & cloud)
{
  cloud.height = 1;
  cloud.width = pcl_cloud.size();
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(4);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1;
  cloud.data.resize(cloud.row_step * cloud.height);
  for (size_t i = 0; i < pcl_cloud.size(); ++i) {
    memcpy(
      &cloud.data[i * cloud.point_step + cloud.fields[0].offset], &pcl_cloud[i].x, sizeof(float));
    memcpy(
      &cloud.data[i * cloud.point_step + cloud.fields[1].offset], &pcl_cloud[i].y, sizeof(float));
    memcpy(
      &cloud.data[i * cloud.point_step + cloud.fields[2].offset], &pcl_cloud[i].z, sizeof(float));
    memcpy(
      &cloud.data[i * cloud.point_step + cloud.fields[3].offset], &pcl_cloud[i].intensity,
      sizeof(float));
  }
}

TEST_F(RayGroundFilterComponentTestSuite, TestCase1)
{
  const auto share_dir =
    ament_index_cpp::get_package_share_directory("autoware_ground_segmentation");
  const auto config_path = share_dir + "/config/ray_ground_filter.param.yaml";
  // std::cout << "config_path:" << config_path << std::endl;
  YAML::Node config = YAML::LoadFile(config_path);
  auto params = config["/**"]["ros__parameters"];

  double min_x_ = params["min_x"].as<float>();
  double max_x_ = params["max_x"].as<float>();
  double min_y_ = params["min_y"].as<float>();
  double max_y_ = params["max_y"].as<float>();
  bool use_vehicle_footprint_ = params["use_vehicle_footprint"].as<bool>();
  double general_max_slope_ = params["general_max_slope"].as<float>();
  double local_max_slope_ = params["local_max_slope"].as<float>();
  double initial_max_slope_ = params["initial_max_slope"].as<float>();
  double radial_divider_angle_ = params["radial_divider_angle"].as<float>();
  double min_height_threshold_ = params["min_height_threshold"].as<float>();
  double concentric_divider_distance_ = params["concentric_divider_distance"].as<float>();
  double reclass_distance_threshold_ = params["reclass_distance_threshold"].as<float>();
  bool publish_processing_time_detail_ = params["publish_processing_time_detail"].as<bool>();

  const auto pcd_path = share_dir + "/data/test.pcd";
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, cloud);

  sensor_msgs::msg::PointCloud2::SharedPtr origin_input_msg_ptr(new sensor_msgs::msg::PointCloud2);
  convertPCL2PointCloud2(cloud, *origin_input_msg_ptr);
  origin_input_msg_ptr->header.frame_id = "velodyne_top";

  // input cloud frame MUST be base_link
  sensor_msgs::msg::PointCloud2::SharedPtr input_msg_ptr(new sensor_msgs::msg::PointCloud2);

  geometry_msgs::msg::TransformStamped t;
  // t.header.stamp = this->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "velodyne_top";
  t.transform.translation.x = 0.6;
  t.transform.translation.y = 0;
  t.transform.translation.z = 2;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf2::doTransform(*origin_input_msg_ptr, *input_msg_ptr, t);

  rclcpp::NodeOptions node_options;
  std::vector<rclcpp::Parameter> parameters;

  parameters.emplace_back(rclcpp::Parameter("base_frame", "base_link"));
  parameters.emplace_back(rclcpp::Parameter("general_max_slope", general_max_slope_));
  parameters.emplace_back(rclcpp::Parameter("local_max_slope", local_max_slope_));
  parameters.emplace_back(rclcpp::Parameter("initial_max_slope", initial_max_slope_));
  parameters.emplace_back(rclcpp::Parameter("radial_divider_angle", radial_divider_angle_));
  parameters.emplace_back(rclcpp::Parameter("min_height_threshold", min_height_threshold_));
  parameters.emplace_back(
    rclcpp::Parameter("concentric_divider_distance", concentric_divider_distance_));
  parameters.emplace_back(
    rclcpp::Parameter("reclass_distance_threshold", reclass_distance_threshold_));
  parameters.emplace_back(rclcpp::Parameter("min_x", min_x_));
  parameters.emplace_back(rclcpp::Parameter("max_x", max_x_));
  parameters.emplace_back(rclcpp::Parameter("min_y", min_y_));
  parameters.emplace_back(rclcpp::Parameter("max_y", max_y_));
  parameters.emplace_back(rclcpp::Parameter("use_vehicle_footprint", use_vehicle_footprint_));
  parameters.emplace_back(
    rclcpp::Parameter("publish_processing_time_detail", publish_processing_time_detail_));

  node_options.parameter_overrides(parameters);
  auto ray_ground_filter_test = std::make_shared<RayGroundFilterComponentTest>(node_options);
  ray_ground_filter_test->input_pointcloud_pub_->publish(*input_msg_ptr);

  sensor_msgs::msg::PointCloud2 out_cloud;
  ray_ground_filter_test->filter(input_msg_ptr, nullptr, out_cloud);
  ray_ground_filter_test->output_pointcloud_pub_->publish(out_cloud);

  // check out_cloud
  int effect_num = 0;
  int total_num = 0;
  const float min_no_ground_point_z = 0.1;  // z in base_frame
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(out_cloud, "x"), iter_y(out_cloud, "y"),
       iter_z(out_cloud, "z"), iter_intensity(out_cloud, "intensity");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    const float z = *iter_z;
    total_num += 1;
    if (z > min_no_ground_point_z) {
      effect_num += 1;
    }
  }

  const float percent = 1.0 * effect_num / total_num;
  // std::cout << "effect_num=" << effect_num << ",total_num=" << total_num
  //           << ",percentage:" << percent << std::endl;
  EXPECT_GE(percent, 0.9);
}
