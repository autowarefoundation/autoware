// Copyright 2024 TIER IV, Inc.
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

#include "../src/ransac_ground_filter/node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <experimental/random>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
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

void setPointCloud2Fields(sensor_msgs::msg::PointCloud2 & pointcloud)
{
  pointcloud.fields.resize(4);
  pointcloud.fields[0].name = "x";
  pointcloud.fields[1].name = "y";
  pointcloud.fields[2].name = "z";
  pointcloud.fields[3].name = "intensity";
  pointcloud.fields[0].offset = 0;
  pointcloud.fields[1].offset = 4;
  pointcloud.fields[2].offset = 8;
  pointcloud.fields[3].offset = 12;
  pointcloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[0].count = 1;
  pointcloud.fields[1].count = 1;
  pointcloud.fields[2].count = 1;
  pointcloud.fields[3].count = 1;
  pointcloud.height = 1;
  pointcloud.point_step = 16;
  pointcloud.is_bigendian = false;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = "base_link";
  pointcloud.header.stamp.sec = 0;
  pointcloud.header.stamp.nanosec = 0;
}

class RansacGroundFilterTestSuite : public ::testing::Test
{
protected:
  void SetUp() { rclcpp::init(0, nullptr); }
  void TearDown() { (void)rclcpp::shutdown(); }
};

class RansacGroundFilterTest : public autoware::ground_segmentation::RANSACGroundFilterComponent
{
public:
  explicit RansacGroundFilterTest(const rclcpp::NodeOptions & options)
  : RANSACGroundFilterComponent(options)
  {
    input_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test_ransac_ground_filter/input_cloud", 1);

    output_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test_ransac_ground_filter/output_cloud", 1);
  }

  ~RansacGroundFilterTest() {}

  void filter(const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
  {
    return RANSACGroundFilterComponent::filter(input, indices, output);
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

TEST_F(RansacGroundFilterTestSuite, TestCase1)
{
  const auto share_dir =
    ament_index_cpp::get_package_share_directory("autoware_ground_segmentation");
  const auto config_path = share_dir + "/config/ransac_ground_filter.param.yaml";
  YAML::Node config = YAML::LoadFile(config_path);
  auto params = config["/**"]["ros__parameters"];

  std::string base_frame = params["base_frame"].as<std::string>();
  std::string unit_axis = params["unit_axis"].as<std::string>();
  int max_iterations = params["max_iterations"].as<int>();
  int min_trial = params["min_trial"].as<int>();
  int min_points = params["min_points"].as<int>();
  double outlier_threshold = params["outlier_threshold"].as<float>();
  double plane_slope_threshold = params["plane_slope_threshold"].as<float>();
  double voxel_size_x = params["voxel_size_x"].as<float>();
  double voxel_size_y = params["voxel_size_y"].as<float>();
  double voxel_size_z = params["voxel_size_z"].as<float>();
  double height_threshold = params["height_threshold"].as<float>();
  bool debug = params["debug"].as<bool>();
  bool publish_processing_time_detail = params["publish_processing_time_detail"].as<bool>();

  const auto pcd_path = share_dir + "/data/test.pcd";
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, cloud);

  sensor_msgs::msg::PointCloud2::SharedPtr origin_input_msg_ptr(new sensor_msgs::msg::PointCloud2);
  convertPCL2PointCloud2(cloud, *origin_input_msg_ptr);
  origin_input_msg_ptr->header.frame_id = "velodyne_top";

  // input cloud frame MUST be base_link
  sensor_msgs::msg::PointCloud2::SharedPtr input_msg_ptr(new sensor_msgs::msg::PointCloud2);

  geometry_msgs::msg::TransformStamped t;
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
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("base_frame", base_frame);
  parameters.emplace_back("unit_axis", unit_axis);
  parameters.emplace_back("max_iterations", max_iterations);
  parameters.emplace_back("min_trial", min_trial);
  parameters.emplace_back("min_points", min_points);
  parameters.emplace_back("outlier_threshold", outlier_threshold);
  parameters.emplace_back("plane_slope_threshold", plane_slope_threshold);
  parameters.emplace_back("voxel_size_x", voxel_size_x);
  parameters.emplace_back("voxel_size_y", voxel_size_y);
  parameters.emplace_back("voxel_size_z", voxel_size_z);
  parameters.emplace_back("height_threshold", height_threshold);
  parameters.emplace_back("debug", debug);
  parameters.emplace_back("publish_processing_time_detail", publish_processing_time_detail);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(parameters);
  auto ransac_ground_filter_test = std::make_shared<RansacGroundFilterTest>(node_options);
  ransac_ground_filter_test->input_pointcloud_pub_->publish(*input_msg_ptr);

  sensor_msgs::msg::PointCloud2 out_cloud;
  ransac_ground_filter_test->filter(input_msg_ptr, nullptr, out_cloud);
  ransac_ground_filter_test->output_pointcloud_pub_->publish(out_cloud);
  std::cout << "out_cloud.width: " << out_cloud.width << std::endl;
  std::cout << "out_cloud.height: " << out_cloud.height << std::endl;
  std::cout << "out_cloud.row_step: " << out_cloud.row_step << std::endl;
  std::cout << "out_cloud.point_step: " << out_cloud.point_step << std::endl;
  std::cout << "out_cloud.is_dense: " << out_cloud.is_dense << std::endl;
  std::cout << "out_cloud.is_bigendian: " << out_cloud.is_bigendian << std::endl;
  std::cout << "out_cloud.data.size(): " << out_cloud.data.size() << std::endl;

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
  std::cout << "effect_num=" << effect_num << ",total_num=" << total_num
            << ",percentage:" << percent << std::endl;
  EXPECT_GE(percent, 0.8);
}
