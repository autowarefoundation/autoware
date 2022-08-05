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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, private_node
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    private_node list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    private_node software without specific prior written permission.
 *
 *  private_node SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF private_node SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include "costmap_generator/costmap_generator.hpp"
#include "costmap_generator/object_map_utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <pcl_ros/transforms.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Copied from scenario selector
geometry_msgs::msg::PoseStamped::ConstSharedPtr getCurrentPose(
  const tf2_ros::Buffer & tf_buffer, const rclcpp::Logger & logger)
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger, "%s", ex.what());
    return nullptr;
  }

  geometry_msgs::msg::PoseStamped::SharedPtr p(new geometry_msgs::msg::PoseStamped());
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return geometry_msgs::msg::PoseStamped::ConstSharedPtr(p);
}

// copied from scenario selector
std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  const auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);

  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    current_position, all_parking_lots, linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

// copied from scenario selector
bool isInParkingLot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
    findNearestParkinglot(lanelet_map_ptr, search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

// Convert from Point32 to Point
std::vector<geometry_msgs::msg::Point> poly2vector(const geometry_msgs::msg::Polygon & poly)
{
  std::vector<geometry_msgs::msg::Point> ps;
  for (const auto & p32 : poly.points) {
    geometry_msgs::msg::Point p;
    p.x = p32.x;
    p.y = p32.y;
    p.z = p32.z;
    ps.push_back(p);
  }
  return ps;
}

pcl::PointCloud<pcl::PointXYZ> getTransformedPointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg,
  const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}

}  // namespace

CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & node_options)
: Node("costmap_generator", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Parameters
  costmap_frame_ = this->declare_parameter<std::string>("costmap_frame", "map");
  vehicle_frame_ = this->declare_parameter<std::string>("vehicle_frame", "base_link");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  update_rate_ = this->declare_parameter<double>("update_rate", 10.0);
  activate_by_scenario_ = this->declare_parameter<bool>("activate_by_scenario", true);
  grid_min_value_ = this->declare_parameter<double>("grid_min_value", 0.0);
  grid_max_value_ = this->declare_parameter<double>("grid_max_value", 1.0);
  grid_resolution_ = this->declare_parameter<double>("grid_resolution", 0.2);
  grid_length_x_ = this->declare_parameter<double>("grid_length_x", 50);
  grid_length_y_ = this->declare_parameter<double>("grid_length_y", 30);
  grid_position_x_ = this->declare_parameter<double>("grid_position_x", 20);
  grid_position_y_ = this->declare_parameter<double>("grid_position_y", 0);
  maximum_lidar_height_thres_ = this->declare_parameter<double>("maximum_lidar_height_thres", 0.3);
  minimum_lidar_height_thres_ = this->declare_parameter<double>("minimum_lidar_height_thres", -2.2);
  use_objects_ = this->declare_parameter<bool>("use_objects", true);
  use_points_ = this->declare_parameter<bool>("use_points", true);
  use_wayarea_ = this->declare_parameter<bool>("use_wayarea", true);
  use_parkinglot_ = this->declare_parameter<bool>("use_parkinglot", true);
  expand_polygon_size_ = this->declare_parameter<double>("expand_polygon_size", 1.0);
  size_of_expansion_kernel_ = this->declare_parameter<int>("size_of_expansion_kernel", 9);

  // Wait for first tf
  // We want to do this before creating subscriptions
  while (rclcpp::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "waiting for initial pose...");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }

  // Subscribers
  using std::placeholders::_1;
  sub_objects_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", 1, std::bind(&CostmapGenerator::onObjects, this, _1));
  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/points_no_ground", rclcpp::SensorDataQoS(),
    std::bind(&CostmapGenerator::onPoints, this, _1));
  sub_lanelet_bin_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CostmapGenerator::onLaneletMapBin, this, _1));
  sub_scenario_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
    "~/input/scenario", 1, std::bind(&CostmapGenerator::onScenario, this, _1));

  // Publishers
  pub_costmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/output/grid_map", 1);
  pub_occupancy_grid_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/output/occupancy_grid", 1);

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&CostmapGenerator::onTimer, this));

  // Initialize
  initGridmap();
}

void CostmapGenerator::loadRoadAreasFromLaneletMap(
  const lanelet::LaneletMapPtr lanelet_map,
  std::vector<std::vector<geometry_msgs::msg::Point>> * area_points)
{
  // use all lanelets in map of subtype road to give way area
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  // convert lanelets to polygons and put into area_points array
  for (const auto & ll : road_lanelets) {
    geometry_msgs::msg::Polygon poly;
    lanelet::visualization::lanelet2Polygon(ll, &poly);
    area_points->push_back(poly2vector(poly));
  }
}

void CostmapGenerator::loadParkingAreasFromLaneletMap(
  const lanelet::LaneletMapPtr lanelet_map,
  std::vector<std::vector<geometry_msgs::msg::Point>> * area_points)
{
  // Parking lots
  lanelet::ConstPolygons3d all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map);
  for (const auto & ll_poly : all_parking_lots) {
    geometry_msgs::msg::Polygon poly;
    lanelet::utils::conversion::toGeomMsgPoly(ll_poly, &poly);
    area_points->push_back(poly2vector(poly));
  }

  // Parking spaces
  lanelet::ConstLineStrings3d all_parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(lanelet_map);
  for (const auto & parking_space : all_parking_spaces) {
    lanelet::ConstPolygon3d ll_poly;
    lanelet::utils::lineStringWithWidthToPolygon(parking_space, &ll_poly);

    geometry_msgs::msg::Polygon poly;
    lanelet::utils::conversion::toGeomMsgPoly(ll_poly, &poly);
    area_points->push_back(poly2vector(poly));
  }
}

void CostmapGenerator::onLaneletMapBin(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);

  if (use_wayarea_) {
    loadRoadAreasFromLaneletMap(lanelet_map_, &primitives_points_);
  }

  if (use_parkinglot_) {
    loadParkingAreasFromLaneletMap(lanelet_map_, &primitives_points_);
  }
}

void CostmapGenerator::onObjects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  objects_ = msg;
}

void CostmapGenerator::onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  points_ = msg;
}

void CostmapGenerator::onScenario(const tier4_planning_msgs::msg::Scenario::ConstSharedPtr msg)
{
  scenario_ = msg;
}

void CostmapGenerator::onTimer()
{
  if (!isActive()) {
    return;
  }

  // Get current pose
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(
      costmap_frame_, vehicle_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }

  // Set grid center
  grid_map::Position p;
  p.x() = tf.transform.translation.x;
  p.y() = tf.transform.translation.y;
  costmap_.setPosition(p);

  if ((use_wayarea_ || use_parkinglot_) && lanelet_map_) {
    costmap_[LayerName::primitives] = generatePrimitivesCostmap();
  }

  if (use_objects_ && objects_) {
    costmap_[LayerName::objects] = generateObjectsCostmap(objects_);
  }

  if (use_points_ && points_) {
    costmap_[LayerName::points] = generatePointsCostmap(points_);
  }

  costmap_[LayerName::combined] = generateCombinedCostmap();

  publishCostmap(costmap_);
}

bool CostmapGenerator::isActive()
{
  if (activate_by_scenario_) {
    if (scenario_) {
      const auto & s = scenario_->activating_scenarios;
      if (
        std::find(std::begin(s), std::end(s), tier4_planning_msgs::msg::Scenario::PARKING) !=
        std::end(s)) {
        return true;
      }
    }
    return false;
  } else {
    const auto & current_pose_wrt_map = getCurrentPose(tf_buffer_, this->get_logger());
    return isInParkingLot(lanelet_map_, current_pose_wrt_map->pose);
  }
}

void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(costmap_frame_);
  costmap_.setGeometry(
    grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
    grid_map::Position(grid_position_x_, grid_position_y_));

  costmap_.add(LayerName::points, grid_min_value_);
  costmap_.add(LayerName::objects, grid_min_value_);
  costmap_.add(LayerName::primitives, grid_min_value_);
  costmap_.add(LayerName::combined, grid_min_value_);
}

grid_map::Matrix CostmapGenerator::generatePointsCostmap(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_points)
{
  geometry_msgs::msg::TransformStamped points2costmap;
  try {
    points2costmap =
      tf_buffer_.lookupTransform(costmap_frame_, in_points->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("costmap_generator"), "%s", ex.what());
  }

  const auto transformed_points = getTransformedPointCloud(*in_points, points2costmap.transform);

  grid_map::Matrix points_costmap = points2costmap_.makeCostmapFromPoints(
    maximum_lidar_height_thres_, minimum_lidar_height_thres_, grid_min_value_, grid_max_value_,
    costmap_, LayerName::points, transformed_points);

  return points_costmap;
}

autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr transformObjects(
  const tf2_ros::Buffer & tf_buffer,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr in_objects,
  const std::string & target_frame_id, const std::string & src_frame_id)
{
  auto objects = new autoware_auto_perception_msgs::msg::PredictedObjects();
  *objects = *in_objects;
  objects->header.frame_id = target_frame_id;

  geometry_msgs::msg::TransformStamped objects2costmap;
  try {
    objects2costmap = tf_buffer.lookupTransform(target_frame_id, src_frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("costmap_generator"), "%s", ex.what());
  }

  for (auto & object : objects->objects) {
    geometry_msgs::msg::PoseStamped output_stamped, input_stamped;
    input_stamped.pose = object.kinematics.initial_pose_with_covariance.pose;
    tf2::doTransform(input_stamped, output_stamped, objects2costmap);
    object.kinematics.initial_pose_with_covariance.pose = output_stamped.pose;
  }

  return autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr(objects);
}

grid_map::Matrix CostmapGenerator::generateObjectsCostmap(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr in_objects)
{
  const auto object_frame = in_objects->header.frame_id;
  const auto transformed_objects =
    transformObjects(tf_buffer_, in_objects, costmap_frame_, object_frame);

  grid_map::Matrix objects_costmap = objects2costmap_.makeCostmapFromObjects(
    costmap_, expand_polygon_size_, size_of_expansion_kernel_, transformed_objects);

  return objects_costmap;
}

grid_map::Matrix CostmapGenerator::generatePrimitivesCostmap()
{
  grid_map::GridMap lanelet2_costmap = costmap_;
  if (!primitives_points_.empty()) {
    object_map::FillPolygonAreas(
      lanelet2_costmap, primitives_points_, LayerName::primitives, grid_max_value_, grid_min_value_,
      grid_min_value_, grid_max_value_, costmap_frame_, map_frame_, tf_buffer_);
  }
  return lanelet2_costmap[LayerName::primitives];
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap()
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::combined].setConstant(grid_min_value_);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::points]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::primitives]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::objects]);

  return combined_costmap[LayerName::combined];
}

void CostmapGenerator::publishCostmap(const grid_map::GridMap & costmap)
{
  // Set header
  std_msgs::msg::Header header;
  header.frame_id = costmap_frame_;
  header.stamp = this->now();

  // Publish OccupancyGrid
  nav_msgs::msg::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(
    costmap, LayerName::combined, grid_min_value_, grid_max_value_, out_occupancy_grid);
  out_occupancy_grid.header = header;
  pub_occupancy_grid_->publish(out_occupancy_grid);

  // Publish GridMap
  auto out_gridmap_msg = grid_map::GridMapRosConverter::toMessage(costmap);
  out_gridmap_msg->header = header;
  pub_costmap_->publish(*out_gridmap_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CostmapGenerator)
