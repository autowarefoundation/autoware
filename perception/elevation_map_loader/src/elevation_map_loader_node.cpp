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

#include "elevation_map_loader/elevation_map_loader_node.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/InpaintFilter.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>
#include <rclcpp/logger.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rosbag2_storage_default_plugins/sqlite/sqlite_statement_wrapper.hpp>

ElevationMapLoaderNode::ElevationMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("elevation_map_loader", options)
{
  layer_name_ = this->declare_parameter("map_layer_name", std::string("elevation"));
  param_file_path_ = this->declare_parameter("param_file_path", "path_default");
  map_frame_ = this->declare_parameter("map_frame", "map");
  use_inpaint_ = this->declare_parameter("use_inpaint", true);
  inpaint_radius_ = this->declare_parameter("inpaint_radius", 0.3);
  use_elevation_map_cloud_publisher_ =
    this->declare_parameter("use_elevation_map_cloud_publisher", false);
  elevation_map_directory_ = this->declare_parameter("elevation_map_directory", "path_default");
  const bool use_lane_filter = this->declare_parameter("use_lane_filter", false);
  data_manager_.use_lane_filter_ = use_lane_filter;

  lane_filter_.use_lane_filter_ = use_lane_filter;
  lane_filter_.lane_margin_ = this->declare_parameter("lane_margin", 0.0);

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_elevation_map_ =
    this->create_publisher<grid_map_msgs::msg::GridMap>("output/elevation_map", durable_qos);

  if (use_elevation_map_cloud_publisher_) {
    pub_elevation_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output/elevation_map_cloud", durable_qos);
  }

  using std::placeholders::_1;
  sub_map_hash_ = create_subscription<tier4_external_api_msgs::msg::MapHash>(
    "/api/autoware/get/map/info/hash", durable_qos,
    std::bind(&ElevationMapLoaderNode::onMapHash, this, _1));
  sub_pointcloud_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud_map", durable_qos,
    std::bind(&ElevationMapLoaderNode::onPointcloudMap, this, _1));
  sub_vector_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", durable_qos, std::bind(&ElevationMapLoaderNode::onVectorMap, this, _1));
}

void ElevationMapLoaderNode::publish()
{
  struct stat info;
  if (stat(data_manager_.elevation_map_path_->c_str(), &info) != 0) {
    RCLCPP_INFO(this->get_logger(), "Create elevation map from pointcloud map ");
    createElevationMap();
  } else if (info.st_mode & S_IFDIR) {
    RCLCPP_INFO(
      this->get_logger(), "Load elevation map from: %s",
      data_manager_.elevation_map_path_->c_str());

    // Check if bag can be loaded
    bool is_bag_loaded = false;
    try {
      is_bag_loaded = grid_map::GridMapRosConverter::loadFromBag(
        *data_manager_.elevation_map_path_, "elevation_map", elevation_map_);
    } catch (rosbag2_storage_plugins::SqliteException & e) {
      is_bag_loaded = false;
    }
    if (!is_bag_loaded) {
      // Delete directory including elevation map if bag is broken
      RCLCPP_ERROR(
        this->get_logger(), "Try to loading bag, but bag is broken. Remove %s",
        data_manager_.elevation_map_path_->c_str());
      std::filesystem::remove_all(data_manager_.elevation_map_path_->c_str());
      // Create elevation map from pointcloud map if bag is broken
      RCLCPP_INFO(this->get_logger(), "Create elevation map from pointcloud map ");
      createElevationMap();
    }
  }

  elevation_map_.setFrameId(map_frame_);
  auto msg = grid_map::GridMapRosConverter::toMessage(elevation_map_);
  pub_elevation_map_->publish(std::move(msg));

  if (use_elevation_map_cloud_publisher_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr elevation_map_cloud_ptr =
      createPointcloudFromElevationMap();
    sensor_msgs::msg::PointCloud2 elevation_map_cloud_msg;
    pcl::toROSMsg(*elevation_map_cloud_ptr, elevation_map_cloud_msg);
    pub_elevation_map_cloud_->publish(elevation_map_cloud_msg);
  }
}

void ElevationMapLoaderNode::onMapHash(
  const tier4_external_api_msgs::msg::MapHash::ConstSharedPtr map_hash)
{
  RCLCPP_INFO(this->get_logger(), "subscribe map_hash");
  const auto elevation_map_hash = map_hash->pcd;
  data_manager_.elevation_map_path_ = std::make_unique<std::filesystem::path>(
    std::filesystem::path(elevation_map_directory_) / elevation_map_hash);
  if (data_manager_.isInitialized()) {
    publish();
  }
}

void ElevationMapLoaderNode::onPointcloudMap(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_map)
{
  RCLCPP_INFO(this->get_logger(), "subscribe pointcloud_map");
  {
    pcl::PointCloud<pcl::PointXYZ> map_pcl;
    pcl::fromROSMsg<pcl::PointXYZ>(*pointcloud_map, map_pcl);
    data_manager_.map_pcl_ptr_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);
  }
  if (data_manager_.isInitialized()) {
    publish();
  }
}

void ElevationMapLoaderNode::onVectorMap(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr vector_map)
{
  RCLCPP_INFO(this->get_logger(), "subscribe vector_map");
  data_manager_.lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*vector_map, data_manager_.lanelet_map_ptr_);
  const lanelet::ConstLanelets all_lanelets =
    lanelet::utils::query::laneletLayer(data_manager_.lanelet_map_ptr_);
  lane_filter_.road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  if (data_manager_.isInitialized()) {
    publish();
  }
}

void ElevationMapLoaderNode::createElevationMap()
{
  auto grid_map_logger = rclcpp::get_logger("grid_map_logger");
  grid_map_logger.set_level(rclcpp::Logger::Level::Error);
  {
    pcl::shared_ptr<grid_map::GridMapPclLoader> grid_map_pcl_loader =
      pcl::make_shared<grid_map::GridMapPclLoader>(grid_map_logger);
    grid_map_pcl_loader->loadParameters(param_file_path_);
    grid_map_pcl_loader->setInputCloud(data_manager_.map_pcl_ptr_);
    createElevationMapFromPointcloud(grid_map_pcl_loader);
    elevation_map_ = grid_map_pcl_loader->getGridMap();
  }
  if (use_inpaint_) {
    inpaintElevationMap(inpaint_radius_);
  }
  saveElevationMap();
}

void ElevationMapLoaderNode::createElevationMapFromPointcloud(
  const pcl::shared_ptr<grid_map::GridMapPclLoader> & grid_map_pcl_loader)
{
  const auto start = std::chrono::high_resolution_clock::now();
  grid_map_pcl_loader->preProcessInputCloud();
  grid_map_pcl_loader->initializeGridMapGeometryFromInputCloud();
  grid_map_pcl_loader->addLayerFromInputCloud(layer_name_);
  grid_map::grid_map_pcl::printTimeElapsedToRosInfoStream(
    start, "Finish creating elevation map. Total time: ", this->get_logger());
}

void ElevationMapLoaderNode::inpaintElevationMap(const float radius)
{
  // Convert elevation layer to OpenCV image to fill in holes.
  // Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
  namespace bg = boost::geometry;
  using tier4_autoware_utils::Point2d;

  elevation_map_.add("inpaint_mask", 0.0);

  elevation_map_.setBasicLayers(std::vector<std::string>());
  if (lane_filter_.use_lane_filter_) {
    for (const auto & lanelet : lane_filter_.road_lanelets_) {
      auto lane_polygon = lanelet.polygon2d().basicPolygon();
      grid_map::Polygon polygon;

      if (lane_filter_.lane_margin_ > 0) {
        lanelet::BasicPolygons2d out;
        bg::strategy::buffer::distance_symmetric<double> distance_strategy(
          lane_filter_.lane_margin_);
        bg::strategy::buffer::join_miter join_strategy;
        bg::strategy::buffer::end_flat end_strategy;
        bg::strategy::buffer::point_square point_strategy;
        bg::strategy::buffer::side_straight side_strategy;
        bg::buffer(
          lane_polygon, out, distance_strategy, side_strategy, join_strategy, end_strategy,
          point_strategy);
        lane_polygon = out.front();
      }
      for (const auto & p : lane_polygon) {
        polygon.addVertex(grid_map::Position(p[0], p[1]));
      }
      for (grid_map_utils::PolygonIterator iterator(elevation_map_, polygon); !iterator.isPastEnd();
           ++iterator) {
        if (!elevation_map_.isValid(*iterator, layer_name_)) {
          elevation_map_.at("inpaint_mask", *iterator) = 1.0;
        }
      }
    }
  } else {
    for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator) {
      if (!elevation_map_.isValid(*iterator, layer_name_)) {
        elevation_map_.at("inpaint_mask", *iterator) = 1.0;
      }
    }
  }
  cv::Mat original_image;
  cv::Mat mask;
  cv::Mat filled_image;
  const float min_value = elevation_map_.get(layer_name_).minCoeffOfFinites();
  const float max_value = elevation_map_.get(layer_name_).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<unsigned char, 3>(
    elevation_map_, layer_name_, CV_8UC3, min_value, max_value, original_image);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
    elevation_map_, "inpaint_mask", CV_8UC1, mask);

  const float radius_in_pixels = radius / elevation_map_.getResolution();
  cv::inpaint(original_image, mask, filled_image, radius_in_pixels, cv::INPAINT_NS);

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
    filled_image, layer_name_, elevation_map_, min_value, max_value);
  elevation_map_.erase("inpaint_mask");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ElevationMapLoaderNode::createPointcloudFromElevationMap()
{
  pcl::PointCloud<pcl::PointXYZ> output_cloud;
  output_cloud.header.frame_id = elevation_map_.getFrameId();

  for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator) {
    float z = elevation_map_.at(layer_name_, *iterator);
    if (!std::isnan(z)) {
      grid_map::Position position;
      elevation_map_.getPosition(grid_map::Index(*iterator), position);
      output_cloud.push_back(pcl::PointXYZ(position.x(), position.y(), z));
    }
  }
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  output_cloud.points.resize(output_cloud.width * output_cloud.height);
  output_cloud.is_dense = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr;
  output_cloud_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(output_cloud);
  return output_cloud_ptr;
}

void ElevationMapLoaderNode::saveElevationMap()
{
  const bool saving_successful = grid_map::GridMapRosConverter::saveToBag(
    elevation_map_, *data_manager_.elevation_map_path_, "elevation_map");
  RCLCPP_INFO_STREAM(
    this->get_logger(), "Saving elevation map successful: " << std::boolalpha << saving_successful);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ElevationMapLoaderNode)
