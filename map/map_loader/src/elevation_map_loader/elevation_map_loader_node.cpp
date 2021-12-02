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

#include "map_loader/elevation_map_loader_node.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/InpaintFilter.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <hash_library_vendor/md5.h>
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

namespace
{
bool isPcdFile(const std::string & p)
{
  if (!std::filesystem::is_regular_file(std::filesystem::status(p))) {
    return false;
  }

  const auto ext = p.substr(p.find_last_of(".") + 1);

  if (ext != "pcd" && ext != "PCD") {
    return false;
  }

  return true;
}

std::string getMd5Sum(const std::string & data)
{
  MD5 digest_md5;
  digest_md5.add(data.c_str(), data.size());
  return digest_md5.getHash();
}

// reference https://create.stephan-brumme.com/hash-library/
std::string getMd5Sum(const std::filesystem::path & file_path)
{
  MD5 digest_md5;
  const size_t buffer_size = 1'000'000;
  auto buffer = std::array<char, buffer_size>();
  std::ifstream file(file_path, std::ios::in | std::ios::binary);

  while (file) {
    file.read(buffer.begin(), buffer_size);
    const auto num_bytes_read = static_cast<size_t>(file.gcount());
    digest_md5.add(buffer.begin(), num_bytes_read);
  }

  return digest_md5.getHash();
}

nlohmann::json getPcdMapHashJson(const std::string & file_path)
{
  // Get PCD file paths
  std::vector<std::filesystem::path> pcd_file_paths;
  if (std::filesystem::is_directory(file_path)) {
    for (const auto & file : std::filesystem::directory_iterator(file_path)) {
      if (!isPcdFile(file.path())) {
        continue;
      }
      pcd_file_paths.push_back(file.path());
    }
  } else if (isPcdFile(file_path)) {
    pcd_file_paths.push_back(file_path);
  }

  // Create JSON meta file
  nlohmann::json j;
  for (const auto & file_path : pcd_file_paths) {
    j[file_path.string()] = getMd5Sum(file_path);
  }

  return j;
}
}  // namespace

ElevationMapLoaderNode::ElevationMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("elevation_map_loader", options)
{
  layer_name_ = this->declare_parameter("map_layer_name", std::string("elevation"));
  std::string param_file_path = this->declare_parameter("param_file_path", "path_default");
  map_frame_ = this->declare_parameter("map_frame", "map");
  use_inpaint_ = this->declare_parameter("use_inpaint", true);
  inpaint_radius_ = this->declare_parameter("inpaint_radius", 0.3);
  use_elevation_map_cloud_publisher_ =
    this->declare_parameter("use_elevation_map_cloud_publisher", false);

  lane_filter_.use_lane_filter_ = this->declare_parameter("use_lane_filter", false);
  lane_filter_.lane_margin_ = this->declare_parameter("lane_margin", 0.5);
  lane_filter_.lane_height_diff_thresh_ = this->declare_parameter("lane_height_diff_thresh", 1.0);
  lane_filter_.voxel_size_x_ = declare_parameter("lane_filter_voxel_size_x", 0.04);
  lane_filter_.voxel_size_y_ = declare_parameter("lane_filter_voxel_size_y", 0.04);
  lane_filter_.voxel_size_z_ = declare_parameter("lane_filter_voxel_size_z", 0.04);

  grid_map_pcl_loader_ = pcl::make_shared<grid_map::GridMapPclLoader>(this->get_logger());
  grid_map_pcl_loader_->loadParameters(param_file_path);

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_elevation_map_ =
    this->create_publisher<grid_map_msgs::msg::GridMap>("output/elevation_map", durable_qos);

  if (use_elevation_map_cloud_publisher_) {
    pub_elevation_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output/elevation_map_cloud", durable_qos);
  }

  std::filesystem::path pointcloud_map_path(
    this->declare_parameter("pointcloud_map_path", "path_default"));
  hash_json_ = getPcdMapHashJson(pointcloud_map_path.lexically_normal().string());

  const auto elevation_map_hash = getMd5Sum(hash_json_.dump());
  const std::string elevation_map_directory =
    this->declare_parameter("elevation_map_directory", "path_default");
  elevation_map_path_ = std::filesystem::path(elevation_map_directory) / elevation_map_hash;

  use_elevation_map_file_ = false;
  struct stat info;
  if (stat(elevation_map_path_.c_str(), &info) != 0) {
    RCLCPP_INFO(this->get_logger(), "Create elevation map from pointcloud map ");
    already_sub_vector_map_ = false;
    already_sub_pointcloud_map_ = false;
    using std::placeholders::_1;
    sub_pointcloud_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/pointcloud_map", durable_qos,
      std::bind(&ElevationMapLoaderNode::onPointcloudMap, this, _1));
    sub_vector_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "input/vector_map", durable_qos, std::bind(&ElevationMapLoaderNode::onVectorMap, this, _1));
  } else if (info.st_mode & S_IFDIR) {
    RCLCPP_INFO(this->get_logger(), "Load elevation map from: %s", elevation_map_path_.c_str());
    use_elevation_map_file_ = grid_map::GridMapRosConverter::loadFromBag(
      elevation_map_path_, "elevation_map", elevation_map_);
    publishElevationMap();
  }
}

void ElevationMapLoaderNode::onPointcloudMap(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_map)
{
  RCLCPP_INFO(this->get_logger(), "subscribe pointcloud_map");
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*pointcloud_map, map_pcl);
  map_pcl_ptr_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);

  already_sub_pointcloud_map_ = true;
  if (already_sub_vector_map_) {
    publishElevationMap();
  }
}

void ElevationMapLoaderNode::onVectorMap(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr vector_map)
{
  RCLCPP_INFO(this->get_logger(), "subscribe vector_map");
  already_sub_vector_map_ = false;
  lanelet::LaneletMapPtr lanelet_map_ptr;
  lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*vector_map, lanelet_map_ptr);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  lane_filter_.road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);

  already_sub_vector_map_ = true;
  if (already_sub_pointcloud_map_) {
    publishElevationMap();
  }
}

void ElevationMapLoaderNode::publishElevationMap()
{
  if (!use_elevation_map_file_) {
    if (lane_filter_.use_lane_filter_) {
      // calculate convex hull
      const auto convex_hull = getConvexHull(map_pcl_ptr_);
      // get intersected lanelets
      lanelet::ConstLanelets intersected_lanelets =
        getIntersectedLanelets(convex_hull, lane_filter_.road_lanelets_);
      // filter pointcloud by lanelet
      pcl::PointCloud<pcl::PointXYZ>::Ptr lane_filtered_map_pcl_ptr =
        getLaneFilteredPointCloud(intersected_lanelets, map_pcl_ptr_);
      grid_map_pcl_loader_->setInputCloud(lane_filtered_map_pcl_ptr);
    } else {
      grid_map_pcl_loader_->setInputCloud(map_pcl_ptr_);
    }
    createElevationMapFromPointcloud();
    elevation_map_ = grid_map_pcl_loader_->getGridMap();
    if (use_inpaint_) {
      inpaintElevationMap(inpaint_radius_);
    }
    saveElevationMap();
  }

  elevation_map_.setFrameId(map_frame_);
  auto msg = grid_map::GridMapRosConverter::toMessage(elevation_map_);
  pub_elevation_map_->publish(std::move(msg));
  already_sub_pointcloud_map_ = false;
  already_sub_vector_map_ = false;

  if (use_elevation_map_cloud_publisher_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr elevation_map_cloud_ptr =
      createPointcloudFromElevationMap();
    sensor_msgs::msg::PointCloud2 elevation_map_cloud_msg;
    pcl::toROSMsg(*elevation_map_cloud_ptr, elevation_map_cloud_msg);
    pub_elevation_map_cloud_->publish(elevation_map_cloud_msg);
  }
}

void ElevationMapLoaderNode::createElevationMapFromPointcloud()
{
  const auto start = std::chrono::high_resolution_clock::now();
  grid_map_pcl_loader_->preProcessInputCloud();
  grid_map_pcl_loader_->initializeGridMapGeometryFromInputCloud();
  grid_map_pcl_loader_->addLayerFromInputCloud(layer_name_);
  grid_map::grid_map_pcl::printTimeElapsedToRosInfoStream(
    start, "Finish creating elevation map. Total time: ", this->get_logger());
}

void ElevationMapLoaderNode::inpaintElevationMap(const float radius)
{
  // Convert elevation layer to OpenCV image to fill in holes.
  // Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
  elevation_map_.add("inpaint_mask", 0.0);

  elevation_map_.setBasicLayers(std::vector<std::string>());
  for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator) {
    if (!elevation_map_.isValid(*iterator, layer_name_)) {
      elevation_map_.at("inpaint_mask", *iterator) = 1.0;
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

autoware_utils::LinearRing2d ElevationMapLoaderNode::getConvexHull(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud)
{
  // downsample pointcloud to reduce convex hull calculation cost
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  downsampled_cloud->points.reserve(input_cloud->points.size());
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input_cloud);
  filter.setLeafSize(0.5, 0.5, 100.0);
  filter.filter(*downsampled_cloud);

  autoware_utils::MultiPoint2d candidate_points;
  for (const auto & p : downsampled_cloud->points) {
    candidate_points.emplace_back(p.x, p.y);
  }

  autoware_utils::LinearRing2d convex_hull;
  boost::geometry::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

lanelet::ConstLanelets ElevationMapLoaderNode::getIntersectedLanelets(
  const autoware_utils::LinearRing2d & convex_hull, const lanelet::ConstLanelets & road_lanelets)
{
  lanelet::ConstLanelets intersected_lanelets;
  for (const auto & road_lanelet : road_lanelets) {
    if (boost::geometry::intersects(convex_hull, road_lanelet.polygon2d().basicPolygon())) {
      intersected_lanelets.push_back(road_lanelet);
    }
  }
  return intersected_lanelets;
}

bool ElevationMapLoaderNode::checkPointWithinLanelets(
  const pcl::PointXYZ & point, const lanelet::ConstLanelets & intersected_lanelets)
{
  autoware_utils::Point2d point2d(point.x, point.y);
  for (const auto & lanelet : intersected_lanelets) {
    if (lane_filter_.lane_margin_ > 0) {
      if (
        boost::geometry::distance(point2d, lanelet.polygon2d().basicPolygon()) >
        lane_filter_.lane_margin_) {
        continue;
      }
    } else {
      if (!boost::geometry::within(point2d, lanelet.polygon2d().basicPolygon())) {
        continue;
      }
    }

    if (lane_filter_.lane_height_diff_thresh_ > 0) {
      float distance = calculateDistancePointFromPlane(point, lanelet);
      if (distance < lane_filter_.lane_height_diff_thresh_) {
        return true;
      }
    } else {
      return true;
    }
  }
  return false;
}

float ElevationMapLoaderNode::calculateDistancePointFromPlane(
  const pcl::PointXYZ & point, const lanelet::ConstLanelet & lanelet)
{
  const Eigen::Vector3d point_3d(point.x, point.y, point.z);
  const Eigen::Vector2d point_2d(point.x, point.y);

  const float distance_3d = boost::geometry::distance(point_3d, lanelet.centerline3d());
  const float distance_2d = boost::geometry::distance(point_2d, lanelet.centerline2d());
  const float distance = std::sqrt(distance_3d * distance_3d - distance_2d * distance_2d);

  return distance;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ElevationMapLoaderNode::getLaneFilteredPointCloud(
  const lanelet::ConstLanelets & intersected_lanelets,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  filtered_cloud.header = cloud->header;

  pcl::PointCloud<pcl::PointXYZ>::Ptr centralized_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  centralized_cloud->reserve(cloud->size());

  // The coordinates of the point cloud are too large, resulting in calculation errors,
  // so offset them to the center.
  // https://github.com/PointCloudLibrary/pcl/issues/4895
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  for (const auto & p : cloud->points) {
    centralized_cloud->points.push_back(
      pcl::PointXYZ(p.x - centroid[0], p.y - centroid[1], p.z - centroid[2]));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(lane_filter_.voxel_size_x_, lane_filter_.voxel_size_y_, 100000.0);
  voxel_grid.setInputCloud(centralized_cloud);
  voxel_grid.setSaveLeafLayout(true);
  voxel_grid.filter(*downsampled_cloud);

  std::unordered_map<size_t, pcl::PointCloud<pcl::PointXYZ>> downsampled2original_map;
  for (const auto & p : centralized_cloud->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }
    const size_t index = voxel_grid.getCentroidIndex(p);
    downsampled2original_map[index].points.push_back(p);
  }

  for (auto & point : downsampled_cloud->points) {
    if (checkPointWithinLanelets(
          pcl::PointXYZ(point.x + centroid[0], point.y + centroid[1], point.z + centroid[2]),
          intersected_lanelets)) {
      const size_t index = voxel_grid.getCentroidIndex(point);
      for (auto & original_point : downsampled2original_map[index].points) {
        original_point.x += centroid[0];
        original_point.y += centroid[1];
        original_point.z += centroid[2];
        filtered_cloud.points.push_back(original_point);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr;
  filtered_cloud_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filtered_cloud);
  return filtered_cloud_ptr;
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
  const bool saving_successful =
    grid_map::GridMapRosConverter::saveToBag(elevation_map_, elevation_map_path_, "elevation_map");

  std::ofstream json_file(elevation_map_path_ / "input_pcd.json");
  json_file << hash_json_;
  json_file.close();

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Saving elevation map successful: " << std::boolalpha << saving_successful);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ElevationMapLoaderNode)
