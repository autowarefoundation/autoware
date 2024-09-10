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

#include "../src/scan_ground_filter/node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/transform_broadcaster.h"

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
class ScanGroundFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    parse_yaml();

    dummy_node_ = std::make_shared<rclcpp::Node>("ScanGroundFilterTest");
    input_pointcloud_pub_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
      dummy_node_, "/test_scan_ground_filter/input_cloud", 1);

    output_pointcloud_pub_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
      dummy_node_, "/test_scan_ground_filter/output_cloud", 1);

    // no real usages, ScanGroundFilterComponent constructor need these params
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back(rclcpp::Parameter("wheel_radius", 0.39));
    parameters.emplace_back(rclcpp::Parameter("wheel_width", 0.42));
    parameters.emplace_back(rclcpp::Parameter("wheel_base", 2.74));
    parameters.emplace_back(rclcpp::Parameter("wheel_tread", 1.63));
    parameters.emplace_back(rclcpp::Parameter("front_overhang", 1.0));
    parameters.emplace_back(rclcpp::Parameter("rear_overhang", 1.03));
    parameters.emplace_back(rclcpp::Parameter("left_overhang", 0.1));
    parameters.emplace_back(rclcpp::Parameter("right_overhang", 0.1));
    parameters.emplace_back(rclcpp::Parameter("vehicle_height", 2.5));
    parameters.emplace_back(rclcpp::Parameter("max_steer_angle", 0.7));

    parameters.emplace_back(
      rclcpp::Parameter("global_slope_max_angle_deg", global_slope_max_angle_deg_));
    parameters.emplace_back(
      rclcpp::Parameter("local_slope_max_angle_deg", local_slope_max_angle_deg_));
    parameters.emplace_back(
      rclcpp::Parameter("split_points_distance_tolerance", split_points_distance_tolerance_));
    parameters.emplace_back(
      rclcpp::Parameter("use_virtual_ground_point", use_virtual_ground_point_));
    parameters.emplace_back(rclcpp::Parameter("split_height_distance", split_height_distance_));
    parameters.emplace_back(
      rclcpp::Parameter("non_ground_height_threshold", non_ground_height_threshold_));
    parameters.emplace_back(rclcpp::Parameter("grid_size_m", grid_size_m_));
    parameters.emplace_back(rclcpp::Parameter("grid_mode_switch_radius", grid_mode_switch_radius_));
    parameters.emplace_back(rclcpp::Parameter("gnd_grid_buffer_size", gnd_grid_buffer_size_));
    parameters.emplace_back(rclcpp::Parameter("detection_range_z_max", detection_range_z_max_));
    parameters.emplace_back(rclcpp::Parameter("elevation_grid_mode", elevation_grid_mode_));
    parameters.emplace_back(rclcpp::Parameter("low_priority_region_x", low_priority_region_x_));
    parameters.emplace_back(rclcpp::Parameter("center_pcl_shift", center_pcl_shift_));
    parameters.emplace_back(
      rclcpp::Parameter("radial_divider_angle_deg", radial_divider_angle_deg_));
    parameters.emplace_back(
      rclcpp::Parameter("use_recheck_ground_cluster", use_recheck_ground_cluster_));
    parameters.emplace_back(rclcpp::Parameter("use_lowest_point", use_lowest_point_));
    parameters.emplace_back(
      rclcpp::Parameter("publish_processing_time_detail", publish_processing_time_detail_));

    options.parameter_overrides(parameters);

    scan_ground_filter_ =
      std::make_shared<autoware::ground_segmentation::ScanGroundFilterComponent>(options);

    // read pcd to pointcloud
    sensor_msgs::msg::PointCloud2::SharedPtr origin_input_msg_ptr =
      std::make_shared<sensor_msgs::msg::PointCloud2>();
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("autoware_ground_segmentation");
    const auto pcd_path = share_dir + "/data/test.pcd";
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, cloud);
    convertPCL2PointCloud2(cloud, *origin_input_msg_ptr);
    origin_input_msg_ptr->header.frame_id = "velodyne_top";

    // input cloud frame MUST be base_link
    input_msg_ptr_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
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

    tf2::doTransform(*origin_input_msg_ptr, *input_msg_ptr_, t);
  }

  ScanGroundFilterTest() {}

  ~ScanGroundFilterTest() override { rclcpp::shutdown(); }

public:
  std::shared_ptr<autoware::ground_segmentation::ScanGroundFilterComponent> scan_ground_filter_;
  rclcpp::Node::SharedPtr dummy_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub_;

  sensor_msgs::msg::PointCloud2::SharedPtr input_msg_ptr_;

  // wrapper function to test private function filter
  void filter(sensor_msgs::msg::PointCloud2 & out_cloud)
  {
    autoware::pointcloud_preprocessor::TransformInfo transform_info;
    scan_ground_filter_->faster_filter(input_msg_ptr_, nullptr, out_cloud, transform_info);
  }

  void parse_yaml()
  {
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("autoware_ground_segmentation");
    const auto config_path = share_dir + "/config/scan_ground_filter.param.yaml";
    // std::cout << "config_path:" << config_path << std::endl;
    YAML::Node config = YAML::LoadFile(config_path);
    auto params = config["/**"]["ros__parameters"];
    global_slope_max_angle_deg_ = params["global_slope_max_angle_deg"].as<float>();
    local_slope_max_angle_deg_ = params["local_slope_max_angle_deg"].as<float>();
    split_points_distance_tolerance_ = params["split_points_distance_tolerance"].as<float>();
    split_height_distance_ = params["split_height_distance"].as<float>();
    non_ground_height_threshold_ = params["non_ground_height_threshold"].as<float>();
    grid_size_m_ = params["grid_size_m"].as<float>();
    grid_mode_switch_radius_ = params["grid_mode_switch_radius"].as<float>();
    gnd_grid_buffer_size_ = params["gnd_grid_buffer_size"].as<uint16_t>();
    detection_range_z_max_ = params["detection_range_z_max"].as<float>();
    elevation_grid_mode_ = params["elevation_grid_mode"].as<bool>();
    low_priority_region_x_ = params["low_priority_region_x"].as<float>();
    use_virtual_ground_point_ = params["use_virtual_ground_point"].as<bool>();
    center_pcl_shift_ = params["center_pcl_shift"].as<float>();
    radial_divider_angle_deg_ = params["radial_divider_angle_deg"].as<float>();
    use_recheck_ground_cluster_ = params["use_recheck_ground_cluster"].as<bool>();
    use_lowest_point_ = params["use_lowest_point"].as<bool>();
    publish_processing_time_detail_ = params["publish_processing_time_detail"].as<bool>();
  }

  float global_slope_max_angle_deg_ = 0.0;
  float local_slope_max_angle_deg_ = 0.0;
  float split_points_distance_tolerance_ = 0.0;
  float split_height_distance_ = 0.0;
  float non_ground_height_threshold_ = 0.0;
  float grid_size_m_ = 0.0;
  float grid_mode_switch_radius_ = 0.0;
  uint16_t gnd_grid_buffer_size_ = 0;
  float detection_range_z_max_ = 0.0;
  bool elevation_grid_mode_ = false;
  float low_priority_region_x_ = 0.0;
  bool use_virtual_ground_point_;
  float center_pcl_shift_;
  float radial_divider_angle_deg_;
  bool use_recheck_ground_cluster_;
  bool use_lowest_point_;
  bool publish_processing_time_detail_;
};

TEST_F(ScanGroundFilterTest, TestCase1)
{
  input_pointcloud_pub_->publish(*input_msg_ptr_);
  sensor_msgs::msg::PointCloud2 out_cloud;

  filter(out_cloud);
  output_pointcloud_pub_->publish(out_cloud);

  // check out_cloud
  int effect_num = 0;
  int total_num = 0;
  const float min_no_ground_point_z = 0.1;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(out_cloud, "x"), iter_y(out_cloud, "y"),
       iter_z(out_cloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
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
