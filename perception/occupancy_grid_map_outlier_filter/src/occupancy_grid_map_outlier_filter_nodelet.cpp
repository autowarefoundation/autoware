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

#include "occupancy_grid_map_outlier_filter/occupancy_grid_map_outlier_filter_nodelet.hpp"

#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

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
  return tier4_autoware_utils::transform2pose(tf_stamped);
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

namespace occupancy_grid_map_outlier_filter
{
RadiusSearch2dFilter::RadiusSearch2dFilter(rclcpp::Node & node)
{
  search_radius_ = node.declare_parameter("radius_search_2d_filter.search_radius", 1.0f);
  min_points_and_distance_ratio_ =
    node.declare_parameter("radius_search_2d_filter.min_points_and_distance_ratio", 400.0f);
  min_points_ = node.declare_parameter("radius_search_2d_filter.min_points", 4);
  max_points_ = node.declare_parameter("radius_search_2d_filter.max_points", 70);
  max_filter_points_nb_ =
    node.declare_parameter("radius_search_2d_filter.max_filter_points_nb", 15000);
  kd_tree_ = pcl::make_shared<pcl::search::KdTree<pcl::PointXY>>(false);
}

void RadiusSearch2dFilter::filter(
  const PclPointCloud & input, const Pose & pose, PclPointCloud & output, PclPointCloud & outlier)
{
  const auto & xyz_cloud = input;
  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  xy_cloud->points.resize(xyz_cloud.points.size());
  for (size_t i = 0; i < xyz_cloud.points.size(); ++i) {
    xy_cloud->points[i].x = xyz_cloud.points[i].x;
    xy_cloud->points[i].y = xyz_cloud.points[i].y;
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_distances(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);
  for (size_t i = 0; i < xy_cloud->points.size(); ++i) {
    const float distance =
      std::hypot(xy_cloud->points[i].x - pose.position.x, xy_cloud->points[i].y - pose.position.y);
    const int min_points_threshold = std::min(
      std::max(static_cast<int>(min_points_and_distance_ratio_ / distance + 0.5f), min_points_),
      max_points_);
    const int points_num =
      kd_tree_->radiusSearch(i, search_radius_, k_indices, k_distances, min_points_threshold);

    if (min_points_threshold <= points_num) {
      output.points.push_back(xyz_cloud.points.at(i));
    } else {
      outlier.points.push_back(xyz_cloud.points.at(i));
    }
  }
}

void RadiusSearch2dFilter::filter(
  const PclPointCloud & high_conf_input, const PclPointCloud & low_conf_input, const Pose & pose,
  PclPointCloud & output, PclPointCloud & outlier)
{
  const auto & high_conf_xyz_cloud = high_conf_input;
  const auto & low_conf_xyz_cloud = low_conf_input;
  // check the limit points number
  if (low_conf_xyz_cloud.points.size() > max_filter_points_nb_) {
    RCLCPP_WARN(
      rclcpp::get_logger("OccupancyGridMapOutlierFilterComponent"),
      "Skip outlier filter since too much low_confidence pointcloud!");
    return;
  }

  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>);
  xy_cloud->points.resize(low_conf_xyz_cloud.points.size() + high_conf_xyz_cloud.points.size());
  for (size_t i = 0; i < low_conf_xyz_cloud.points.size(); ++i) {
    xy_cloud->points[i].x = low_conf_xyz_cloud.points[i].x;
    xy_cloud->points[i].y = low_conf_xyz_cloud.points[i].y;
  }
  for (size_t i = low_conf_xyz_cloud.points.size(); i < xy_cloud->points.size(); ++i) {
    xy_cloud->points[i].x = high_conf_xyz_cloud.points[i - low_conf_xyz_cloud.points.size()].x;
    xy_cloud->points[i].y = high_conf_xyz_cloud.points[i - low_conf_xyz_cloud.points.size()].y;
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_distances(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);
  for (size_t i = 0; i < low_conf_xyz_cloud.points.size(); ++i) {
    const float distance =
      std::hypot(xy_cloud->points[i].x - pose.position.x, xy_cloud->points[i].y - pose.position.y);
    const int min_points_threshold = std::min(
      std::max(static_cast<int>(min_points_and_distance_ratio_ / distance + 0.5f), min_points_),
      max_points_);
    const int points_num =
      kd_tree_->radiusSearch(i, search_radius_, k_indices, k_distances, min_points_threshold);

    if (min_points_threshold <= points_num) {
      output.points.push_back(low_conf_xyz_cloud.points.at(i));
    } else {
      outlier.points.push_back(low_conf_xyz_cloud.points.at(i));
    }
  }
}

OccupancyGridMapOutlierFilterComponent::OccupancyGridMapOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Node("OccupancyGridMapOutlierFilter", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "occupancy_grid_map_outlier_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  /* params */
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");
  cost_threshold_ = declare_parameter("cost_threshold", 45);
  auto use_radius_search_2d_filter = declare_parameter("use_radius_search_2d_filter", true);
  auto enable_debugger = declare_parameter("enable_debugger", false);

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
}

void OccupancyGridMapOutlierFilterComponent::splitPointCloudFrontBack(
  const PointCloud2::ConstSharedPtr & input_pc, PointCloud2 & front_pc, PointCloud2 & behind_pc)
{
  PclPointCloud tmp_behind_pc;
  PclPointCloud tmp_front_pc;
  for (sensor_msgs::PointCloud2ConstIterator<float> x(*input_pc, "x"), y(*input_pc, "y"),
       z(*input_pc, "z");
       x != x.end(); ++x, ++y, ++z) {
    if (*x < 0.0) {
      tmp_behind_pc.push_back(pcl::PointXYZ(*x, *y, *z));
    } else {
      tmp_front_pc.push_back(pcl::PointXYZ(*x, *y, *z));
    }
  }
  pcl::toROSMsg(tmp_front_pc, front_pc);
  pcl::toROSMsg(tmp_behind_pc, behind_pc);
  front_pc.header = input_pc->header;
  behind_pc.header = input_pc->header;
}
void OccupancyGridMapOutlierFilterComponent::onOccupancyGridMapAndPointCloud2(
  const OccupancyGrid::ConstSharedPtr & input_ogm, const PointCloud2::ConstSharedPtr & input_pc)
{
  stop_watch_ptr_->toc("processing_time", true);
  // Transform to occupancy grid map frame
  PointCloud2 ogm_frame_pc{};
  PointCloud2 input_front_pc{};
  PointCloud2 input_behind_pc{};
  PointCloud2 ogm_frame_input_behind_pc{};
  splitPointCloudFrontBack(input_pc, input_front_pc, input_behind_pc);
  if (
    !transformPointcloud(input_front_pc, *tf2_, input_ogm->header.frame_id, ogm_frame_pc) ||
    !transformPointcloud(
      input_behind_pc, *tf2_, input_ogm->header.frame_id, ogm_frame_input_behind_pc)) {
    return;
  }
  // Occupancy grid map based filter
  PclPointCloud high_confidence_pc{};
  PclPointCloud low_confidence_pc{};
  PclPointCloud out_ogm_pc{};
  PclPointCloud ogm_frame_behind_pc;
  pcl::fromROSMsg(ogm_frame_input_behind_pc, ogm_frame_behind_pc);
  filterByOccupancyGridMap(
    *input_ogm, ogm_frame_pc, high_confidence_pc, low_confidence_pc, out_ogm_pc);
  // Apply Radius search 2d filter for low confidence pointcloud
  PclPointCloud filtered_low_confidence_pc{};
  PclPointCloud outlier_pc{};
  if (radius_search_2d_filter_ptr_) {
    auto pc_frame_pose_stamped = getPoseStamped(
      *tf2_, input_ogm->header.frame_id, input_pc->header.frame_id, input_ogm->header.stamp);
    radius_search_2d_filter_ptr_->filter(
      high_confidence_pc, low_confidence_pc, pc_frame_pose_stamped.pose, filtered_low_confidence_pc,
      outlier_pc);
  } else {
    outlier_pc = low_confidence_pc;
  }
  // Concatenate high confidence pointcloud from occupancy grid map and non-outlier pointcloud
  PclPointCloud concat_pc =
    high_confidence_pc + filtered_low_confidence_pc + out_ogm_pc + ogm_frame_behind_pc;
  // Convert to ros msg
  {
    PointCloud2 ogm_frame_filtered_pc{};
    auto base_link_frame_filtered_pc_ptr = std::make_unique<PointCloud2>();
    pcl::toROSMsg(concat_pc, ogm_frame_filtered_pc);
    ogm_frame_filtered_pc.header = ogm_frame_pc.header;
    if (!transformPointcloud(
          ogm_frame_filtered_pc, *tf2_, base_link_frame_, *base_link_frame_filtered_pc_ptr)) {
      return;
    }
    pointcloud_pub_->publish(std::move(base_link_frame_filtered_pc_ptr));
  }
  if (debugger_ptr_) {
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

void OccupancyGridMapOutlierFilterComponent::filterByOccupancyGridMap(
  const OccupancyGrid & occupancy_grid_map, const PointCloud2 & pointcloud,
  PclPointCloud & high_confidence, PclPointCloud & low_confidence, PclPointCloud & out_ogm)
{
  for (sensor_msgs::PointCloud2ConstIterator<float> x(pointcloud, "x"), y(pointcloud, "y"),
       z(pointcloud, "z");
       x != x.end(); ++x, ++y, ++z) {
    const auto cost = getCost(occupancy_grid_map, *x, *y);
    if (cost) {
      if (cost_threshold_ < *cost) {
        high_confidence.push_back(pcl::PointXYZ(*x, *y, *z));
      } else {
        low_confidence.push_back(pcl::PointXYZ(*x, *y, *z));
      }
    } else {
      out_ogm.push_back(pcl::PointXYZ(*x, *y, *z));
    }
  }
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
  const PclPointCloud & input, const Header & header)
{
  auto output_ptr = std::make_unique<PointCloud2>();
  transformToBaseLink(input, header, *output_ptr);
  outlier_pointcloud_pub_->publish(std::move(output_ptr));
}
void OccupancyGridMapOutlierFilterComponent::Debugger::publishHighConfidence(
  const PclPointCloud & input, const Header & header)
{
  auto output_ptr = std::make_unique<PointCloud2>();
  transformToBaseLink(input, header, *output_ptr);
  high_confidence_pointcloud_pub_->publish(std::move(output_ptr));
}

void OccupancyGridMapOutlierFilterComponent::Debugger::publishLowConfidence(
  const PclPointCloud & input, const Header & header)
{
  auto output_ptr = std::make_unique<PointCloud2>();
  transformToBaseLink(input, header, *output_ptr);
  low_confidence_pointcloud_pub_->publish(std::move(output_ptr));
}

void OccupancyGridMapOutlierFilterComponent::Debugger::transformToBaseLink(
  const PclPointCloud & pcl_input, const Header & header, PointCloud2 & output)
{
  PointCloud2 ros_input{};
  pcl::toROSMsg(pcl_input, ros_input);
  ros_input.header = header;
  transformPointcloud(ros_input, *(node_.tf2_), node_.base_link_frame_, output);
}

}  // namespace occupancy_grid_map_outlier_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent)
