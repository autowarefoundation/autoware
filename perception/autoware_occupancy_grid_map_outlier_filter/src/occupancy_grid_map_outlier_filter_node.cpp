// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "occupancy_grid_map_outlier_filter_node.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/ros/debug_publisher.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <pcl_ros/transforms.hpp>

#include <boost/optional.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::universe_utils::ScopedTimeTrack;

bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  geometry_msgs::msg::TransformStamped tf_stamped{};
  try {
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("occupancy_grid_map_outlier_filter"), clock, 5000, "%s", ex.what());
    return false;
  }
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

geometry_msgs::msg::PoseStamped getPoseStamped(
  const tf2_ros::Buffer & tf2, const std::string & target_frame_id,
  const std::string & src_frame_id, const rclcpp::Time & time)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  geometry_msgs::msg::TransformStamped tf_stamped{};
  try {
    tf_stamped =
      tf2.lookupTransform(target_frame_id, src_frame_id, time, rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("occupancy_grid_map_outlier_filter"), clock, 5000, "%s", ex.what());
  }
  return autoware::universe_utils::transform2pose(tf_stamped);
}

boost::optional<char> getCost(
  const nav_msgs::msg::OccupancyGrid & map, const double & x, const double & y)
{
  const auto & map_position = map.info.origin.position;
  const auto & map_resolution = map.info.resolution;
  const double map_height_m = map.info.height /* cell size */ * map_resolution;
  const double map_width_m = map.info.width /* cell size */ * map_resolution;
  const double map_min_x = map_position.x;
  const double map_max_x = map_position.x + map_width_m;
  const double map_min_y = map_position.y;
  const double map_max_y = map_position.y + map_height_m;

  if (map_min_x < x && x < map_max_x && map_min_y < y && y < map_max_y) {
    unsigned int map_cell_x{};
    unsigned int map_cell_y{};
    map_cell_x = std::floor((x - map_position.x) / map_resolution);
    map_cell_y = std::floor((y - map_position.y) / map_resolution);
    size_t index = map_cell_y * map.info.width + map_cell_x;
    return map.data.at(index);
  }
  return boost::none;
}

}  // namespace

namespace autoware::occupancy_grid_map_outlier_filter
{
RadiusSearch2dFilter::RadiusSearch2dFilter(rclcpp::Node & node)
{
  search_radius_ = node.declare_parameter<float>("radius_search_2d_filter.search_radius");
  min_points_and_distance_ratio_ =
    node.declare_parameter<float>("radius_search_2d_filter.min_points_and_distance_ratio");
  min_points_ = node.declare_parameter<int>("radius_search_2d_filter.min_points");
  max_points_ = node.declare_parameter<int>("radius_search_2d_filter.max_points");
  max_filter_points_nb_ =
    node.declare_parameter<int>("radius_search_2d_filter.max_filter_points_nb");
  kd_tree_ = pcl::make_shared<pcl::search::KdTree<pcl::PointXY>>(false);
}

void RadiusSearch2dFilter::filter(
  const PointCloud2 & input, const Pose & pose, PointCloud2 & output, PointCloud2 & outlier)
{
  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  int point_step = input.point_step;
  int x_offset = input.fields[pcl::getFieldIndex(input, "x")].offset;
  int y_offset = input.fields[pcl::getFieldIndex(input, "y")].offset;
  xy_cloud->points.resize(input.data.size() / point_step);
  for (size_t i = 0; i < input.data.size() / point_step; ++i) {
    std::memcpy(&xy_cloud->points[i].x, &input.data[i * point_step + x_offset], sizeof(float));
    std::memcpy(&xy_cloud->points[i].y, &input.data[i * point_step + y_offset], sizeof(float));
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_distances(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);
  size_t output_size = 0;
  size_t outlier_size = 0;
  for (size_t i = 0; i < xy_cloud->points.size(); ++i) {
    const float distance =
      std::hypot(xy_cloud->points[i].x - pose.position.x, xy_cloud->points[i].y - pose.position.y);
    const int min_points_threshold = std::min(
      std::max(static_cast<int>(min_points_and_distance_ratio_ / distance + 0.5f), min_points_),
      max_points_);
    const int points_num =
      kd_tree_->radiusSearch(i, search_radius_, k_indices, k_distances, min_points_threshold);

    if (min_points_threshold <= points_num) {
      std::memcpy(&output.data[output_size], &input.data[i * point_step], point_step);
      output_size += point_step;
    } else {
      std::memcpy(&outlier.data[outlier_size], &input.data[i * point_step], point_step);
      outlier_size += point_step;
    }
  }
  output.data.resize(output_size);
  outlier.data.resize(outlier_size);
}

void RadiusSearch2dFilter::filter(
  const PointCloud2 & high_conf_xyz_cloud, const PointCloud2 & low_conf_xyz_cloud,
  const Pose & pose, PointCloud2 & output, PointCloud2 & outlier)
{
  // check the limit points number
  if (low_conf_xyz_cloud.width > max_filter_points_nb_) {
    RCLCPP_WARN(
      rclcpp::get_logger("OccupancyGridMapOutlierFilterComponent"),
      "Skip outlier filter since too much low_confidence pointcloud!");
    return;
  }
  int x_offset = low_conf_xyz_cloud.fields[pcl::getFieldIndex(low_conf_xyz_cloud, "x")].offset;
  int y_offset = low_conf_xyz_cloud.fields[pcl::getFieldIndex(low_conf_xyz_cloud, "y")].offset;
  int point_step = low_conf_xyz_cloud.point_step;
  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  xy_cloud->points.resize(low_conf_xyz_cloud.width + high_conf_xyz_cloud.width);
  for (size_t i = 0; i < low_conf_xyz_cloud.width; ++i) {
    std::memcpy(
      &xy_cloud->points[i].x, &low_conf_xyz_cloud.data[i * point_step + x_offset], sizeof(float));
    std::memcpy(
      &xy_cloud->points[i].y, &low_conf_xyz_cloud.data[i * point_step + y_offset], sizeof(float));
  }

  for (size_t i = low_conf_xyz_cloud.width; i < xy_cloud->points.size(); ++i) {
    size_t high_conf_xyz_cloud_index = i - low_conf_xyz_cloud.width;
    std::memcpy(
      &xy_cloud->points[i].x,
      &high_conf_xyz_cloud.data[high_conf_xyz_cloud_index * point_step + x_offset], sizeof(float));
    std::memcpy(
      &xy_cloud->points[i].y,
      &high_conf_xyz_cloud.data[high_conf_xyz_cloud_index * point_step + y_offset], sizeof(float));
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_distances(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);

  size_t output_size = 0;
  size_t outlier_size = 0;
  for (size_t i = 0; i < low_conf_xyz_cloud.data.size() / low_conf_xyz_cloud.point_step; ++i) {
    const float distance =
      std::hypot(xy_cloud->points[i].x - pose.position.x, xy_cloud->points[i].y - pose.position.y);
    const int min_points_threshold = std::min(
      std::max(static_cast<int>(min_points_and_distance_ratio_ / distance + 0.5f), min_points_),
      max_points_);
    const int points_num =
      kd_tree_->radiusSearch(i, search_radius_, k_indices, k_distances, min_points_threshold);

    if (min_points_threshold <= points_num) {
      std::memcpy(
        &output.data[output_size], &low_conf_xyz_cloud.data[i * low_conf_xyz_cloud.point_step],
        low_conf_xyz_cloud.point_step);
      output_size += low_conf_xyz_cloud.point_step;
    } else {
      std::memcpy(
        &outlier.data[outlier_size], &low_conf_xyz_cloud.data[i * low_conf_xyz_cloud.point_step],
        low_conf_xyz_cloud.point_step);
      outlier_size += low_conf_xyz_cloud.point_step;
    }
  }

  output.data.resize(output_size);
  outlier.data.resize(outlier_size);
}

OccupancyGridMapOutlierFilterComponent::OccupancyGridMapOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Node("OccupancyGridMapOutlierFilter", options)
{
  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "occupancy_grid_map_outlier_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  /* params */
  map_frame_ = declare_parameter<std::string>("map_frame");
  base_link_frame_ = declare_parameter<std::string>("base_link_frame");
  cost_threshold_ = declare_parameter<int>("cost_threshold");
  auto use_radius_search_2d_filter = declare_parameter<bool>("use_radius_search_2d_filter");
  auto enable_debugger = declare_parameter<bool>("enable_debugger");

  /* tf */
  tf2_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_);

  /* Subscriber and publisher */
  pointcloud_sub_.subscribe(this, "~/input/pointcloud", rmw_qos_profile_sensor_data);
  occupancy_grid_map_sub_.subscribe(
    this, "~/input/occupancy_grid_map", rclcpp::QoS{1}.get_rmw_qos_profile());
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(5), occupancy_grid_map_sub_, pointcloud_sub_);
  sync_ptr_->registerCallback(std::bind(
    &OccupancyGridMapOutlierFilterComponent::onOccupancyGridMapAndPointCloud2, this,
    std::placeholders::_1, std::placeholders::_2));
  pointcloud_pub_ = create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS());

  /* Radius search 2d filter */
  if (use_radius_search_2d_filter) {
    radius_search_2d_filter_ptr_ = std::make_shared<RadiusSearch2dFilter>(*this);
  }
  /* debugger */
  if (enable_debugger) {
    debugger_ptr_ = std::make_shared<Debugger>(*this);
  }

  // time keeper
  bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware::universe_utils::TimeKeeper(detailed_processing_time_publisher_);
    time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(time_keeper);
  }
}

void OccupancyGridMapOutlierFilterComponent::splitPointCloudFrontBack(
  const PointCloud2::ConstSharedPtr & input_pc, PointCloud2 & front_pc, PointCloud2 & behind_pc)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  int x_offset = input_pc->fields[pcl::getFieldIndex(*input_pc, "x")].offset;
  int point_step = input_pc->point_step;
  size_t front_count = 0;
  size_t behind_count = 0;

  for (size_t global_offset = 0; global_offset < input_pc->data.size();
       global_offset += point_step) {
    float x;
    std::memcpy(&x, &input_pc->data[global_offset + x_offset], sizeof(float));
    if (x < 0.0) {
      std::memcpy(
        &behind_pc.data[behind_count * point_step], &input_pc->data[global_offset],
        input_pc->point_step);
      behind_count++;
    } else {
      std::memcpy(
        &front_pc.data[front_count * point_step], &input_pc->data[global_offset],
        input_pc->point_step);
      front_count++;
    }
  }
  front_pc.data.resize(front_count * point_step);
  behind_pc.data.resize(behind_count * point_step);
}
void OccupancyGridMapOutlierFilterComponent::onOccupancyGridMapAndPointCloud2(
  const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  stop_watch_ptr_->toc("processing_time", true);
  // Transform to occupancy grid map frame

  PointCloud2 input_behind_pc{};
  PointCloud2 input_front_pc{};
  initializerPointCloud2(*input_pc, input_front_pc);
  initializerPointCloud2(*input_pc, input_behind_pc);
  // Split pointcloud into front and behind of the vehicle to reduce the calculation cost
  splitPointCloudFrontBack(input_pc, input_front_pc, input_behind_pc);
  finalizePointCloud2(*input_pc, input_front_pc);
  finalizePointCloud2(*input_pc, input_behind_pc);

  PointCloud2 ogm_frame_pc{};
  PointCloud2 ogm_frame_input_behind_pc{};
  {  // transform pointclouds
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("transformPointcloud", *time_keeper_);

    if (
      !transformPointcloud(input_front_pc, *tf2_, input_ogm->header.frame_id, ogm_frame_pc) ||
      !transformPointcloud(
        input_behind_pc, *tf2_, input_ogm->header.frame_id, ogm_frame_input_behind_pc)) {
      return;
    }
  }

  // Occupancy grid map based filter
  PointCloud2 high_confidence_pc{};
  PointCloud2 low_confidence_pc{};
  PointCloud2 out_ogm_pc{};
  initializerPointCloud2(ogm_frame_pc, high_confidence_pc);
  initializerPointCloud2(ogm_frame_pc, low_confidence_pc);
  initializerPointCloud2(ogm_frame_pc, out_ogm_pc);
  // split front pointcloud into high and low confidence and out of map pointcloud
  filterByOccupancyGridMap(
    *input_ogm, ogm_frame_pc, high_confidence_pc, low_confidence_pc, out_ogm_pc);
  // Apply Radius search 2d filter for low confidence pointcloud
  PointCloud2 filtered_low_confidence_pc{};
  PointCloud2 outlier_pc{};
  initializerPointCloud2(low_confidence_pc, outlier_pc);
  initializerPointCloud2(low_confidence_pc, filtered_low_confidence_pc);

  if (radius_search_2d_filter_ptr_) {
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("radius_search_2d_filter", *time_keeper_);

    auto pc_frame_pose_stamped = getPoseStamped(
      *tf2_, input_ogm->header.frame_id, input_pc->header.frame_id, input_ogm->header.stamp);
    radius_search_2d_filter_ptr_->filter(
      high_confidence_pc, low_confidence_pc, pc_frame_pose_stamped.pose, filtered_low_confidence_pc,
      outlier_pc);
  } else {
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("low_confidence_pc_filter", *time_keeper_);

    std::memcpy(&outlier_pc.data[0], &low_confidence_pc.data[0], low_confidence_pc.data.size());
    outlier_pc.data.resize(low_confidence_pc.data.size());
  }

  // Concatenate high confidence pointcloud from occupancy grid map and non-outlier pointcloud
  PointCloud2 ogm_frame_filtered_pc{};
  concatPointCloud2(ogm_frame_filtered_pc, high_confidence_pc);
  concatPointCloud2(ogm_frame_filtered_pc, filtered_low_confidence_pc);
  concatPointCloud2(ogm_frame_filtered_pc, out_ogm_pc);
  concatPointCloud2(ogm_frame_filtered_pc, ogm_frame_input_behind_pc);
  finalizePointCloud2(ogm_frame_pc, ogm_frame_filtered_pc);

  auto base_link_frame_filtered_pc_ptr = std::make_unique<PointCloud2>();
  {  // scope for the timekeeper to track the time spent on transformPointcloud
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("transformPointcloud", *time_keeper_);

    ogm_frame_filtered_pc.header = ogm_frame_pc.header;
    if (!transformPointcloud(
          ogm_frame_filtered_pc, *tf2_, base_link_frame_, *base_link_frame_filtered_pc_ptr)) {
      return;
    }
  }

  {  // scope for the timekeeper to track the time spent on publishing pointcloud
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("publish_pointcloud", *time_keeper_);

    pointcloud_pub_->publish(std::move(base_link_frame_filtered_pc_ptr));
  }

  if (debugger_ptr_) {
    finalizePointCloud2(ogm_frame_pc, high_confidence_pc);
    finalizePointCloud2(ogm_frame_pc, filtered_low_confidence_pc);
    finalizePointCloud2(ogm_frame_pc, outlier_pc);
    debugger_ptr_->publishHighConfidence(high_confidence_pc, ogm_frame_pc.header);
    debugger_ptr_->publishLowConfidence(filtered_low_confidence_pc, ogm_frame_pc.header);
    debugger_ptr_->publishOutlier(outlier_pc, ogm_frame_pc.header);
  }

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

void OccupancyGridMapOutlierFilterComponent::initializerPointCloud2(
  const PointCloud2 & input, PointCloud2 & output)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  output.point_step = input.point_step;
  output.data.resize(input.data.size());
}

void OccupancyGridMapOutlierFilterComponent::finalizePointCloud2(
  const PointCloud2 & input, PointCloud2 & output)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  output.header = input.header;
  output.point_step = input.point_step;
  output.fields = input.fields;
  output.height = input.height;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = input.is_dense;
  output.width = output.data.size() / output.point_step / output.height;
  output.row_step = output.data.size() / output.height;
}

void OccupancyGridMapOutlierFilterComponent::concatPointCloud2(
  PointCloud2 & output, const PointCloud2 & input)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  size_t output_size = output.data.size();
  output.data.resize(output.data.size() + input.data.size());
  std::memcpy(&output.data[output_size], &input.data[0], input.data.size());
}
void OccupancyGridMapOutlierFilterComponent::filterByOccupancyGridMap(
  const OccupancyGrid & occupancy_grid_map, const PointCloud2 & pointcloud,
  PointCloud2 & high_confidence, PointCloud2 & low_confidence, PointCloud2 & out_ogm)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  int x_offset = pointcloud.fields[pcl::getFieldIndex(pointcloud, "x")].offset;
  int y_offset = pointcloud.fields[pcl::getFieldIndex(pointcloud, "y")].offset;
  size_t high_confidence_size = 0;
  size_t low_confidence_size = 0;
  size_t out_ogm_size = 0;

  for (size_t global_offset = 0; global_offset < pointcloud.data.size();
       global_offset += pointcloud.point_step) {
    float x;
    float y;
    std::memcpy(&x, &pointcloud.data[global_offset + x_offset], sizeof(float));
    std::memcpy(&y, &pointcloud.data[global_offset + y_offset], sizeof(float));

    const auto cost = getCost(occupancy_grid_map, x, y);
    if (cost) {
      if (cost_threshold_ < *cost) {
        std::memcpy(
          &high_confidence.data[high_confidence_size], &pointcloud.data[global_offset],
          pointcloud.point_step);
        high_confidence_size += pointcloud.point_step;
      } else {
        std::memcpy(
          &low_confidence.data[low_confidence_size], &pointcloud.data[global_offset],
          pointcloud.point_step);
        low_confidence_size += pointcloud.point_step;
      }
    } else {
      std::memcpy(
        &out_ogm.data[out_ogm_size], &pointcloud.data[global_offset], pointcloud.point_step);
      out_ogm_size += pointcloud.point_step;
    }
  }
  high_confidence.data.resize(high_confidence_size);
  low_confidence.data.resize(low_confidence_size);
  out_ogm.data.resize(out_ogm_size);
  finalizePointCloud2(pointcloud, high_confidence);
  finalizePointCloud2(pointcloud, low_confidence);
  finalizePointCloud2(pointcloud, out_ogm);
}

OccupancyGridMapOutlierFilterComponent::Debugger::Debugger(
  OccupancyGridMapOutlierFilterComponent & node)
: node_(node)
{
  outlier_pointcloud_pub_ = node.create_publisher<PointCloud2>(
    "~/output/debug/outlier/pointcloud", rclcpp::SensorDataQoS());
  low_confidence_pointcloud_pub_ = node.create_publisher<PointCloud2>(
    "~/output/debug/low_confidence/pointcloud", rclcpp::SensorDataQoS());
  high_confidence_pointcloud_pub_ = node.create_publisher<PointCloud2>(
    "~/output/debug/high_confidence/pointcloud", rclcpp::SensorDataQoS());
}

void OccupancyGridMapOutlierFilterComponent::Debugger::publishOutlier(
  const PointCloud2 & input, const Header & header)
{
  auto output_ptr = std::make_unique<PointCloud2>();
  transformToBaseLink(input, header, *output_ptr);
  outlier_pointcloud_pub_->publish(std::move(output_ptr));
}
void OccupancyGridMapOutlierFilterComponent::Debugger::publishHighConfidence(
  const PointCloud2 & input, const Header & header)
{
  auto output_ptr = std::make_unique<PointCloud2>();
  transformToBaseLink(input, header, *output_ptr);
  high_confidence_pointcloud_pub_->publish(std::move(output_ptr));
}

void OccupancyGridMapOutlierFilterComponent::Debugger::publishLowConfidence(
  const PointCloud2 & input, const Header & header)
{
  auto output_ptr = std::make_unique<PointCloud2>();
  transformToBaseLink(input, header, *output_ptr);
  low_confidence_pointcloud_pub_->publish(std::move(output_ptr));
}

void OccupancyGridMapOutlierFilterComponent::Debugger::transformToBaseLink(
  const PointCloud2 & ros_input, [[maybe_unused]] const Header & header, PointCloud2 & output)
{
  transformPointcloud(ros_input, *(node_.tf2_), node_.base_link_frame_, output);
}

}  // namespace autoware::occupancy_grid_map_outlier_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent)
