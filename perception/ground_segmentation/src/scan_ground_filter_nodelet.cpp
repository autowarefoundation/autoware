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

#include "ground_segmentation/scan_ground_filter_nodelet.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ground_segmentation
{
using pointcloud_preprocessor::get_param;
using tier4_autoware_utils::calcDistance3d;
using tier4_autoware_utils::deg2rad;
using tier4_autoware_utils::normalizeDegree;
using tier4_autoware_utils::normalizeRadian;
using vehicle_info_util::VehicleInfoUtil;

ScanGroundFilterComponent::ScanGroundFilterComponent(const rclcpp::NodeOptions & options)
: Filter("ScanGroundFilter", options)
{
  // set initial parameters
  {
    low_priority_region_x_ = static_cast<float>(declare_parameter("low_priority_region_x", -20.0f));
    detection_range_z_max_ = static_cast<float>(declare_parameter("detection_range_z_max", 2.5f));
    center_pcl_shift_ = static_cast<float>(declare_parameter("center_pcl_shift", 0.0));
    non_ground_height_threshold_ =
      static_cast<float>(declare_parameter("non_ground_height_threshold", 0.20));
    grid_mode_switch_radius_ =
      static_cast<float>(declare_parameter("grid_mode_switch_radius", 20.0));

    grid_size_m_ = static_cast<float>(declare_parameter("grid_size_m", 0.5));
    gnd_grid_buffer_size_ = static_cast<int>(declare_parameter("gnd_grid_buffer_size", 4));
    elevation_grid_mode_ = static_cast<bool>(declare_parameter("elevation_grid_mode", true));
    global_slope_max_angle_rad_ = deg2rad(declare_parameter("global_slope_max_angle_deg", 8.0));
    local_slope_max_angle_rad_ = deg2rad(declare_parameter("local_slope_max_angle_deg", 10.0));
    radial_divider_angle_rad_ = deg2rad(declare_parameter("radial_divider_angle_deg", 1.0));
    split_points_distance_tolerance_ = declare_parameter("split_points_distance_tolerance", 0.2);
    split_height_distance_ = declare_parameter("split_height_distance", 0.2);
    use_virtual_ground_point_ = declare_parameter("use_virtual_ground_point", true);
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    vehicle_info_ = VehicleInfoUtil(*this).getVehicleInfo();

    grid_mode_switch_grid_id_ =
      grid_mode_switch_radius_ / grid_size_m_;  // changing the mode of grid division
    virtual_lidar_z_ = vehicle_info_.vehicle_height_m;
    grid_mode_switch_angle_rad_ = std::atan2(grid_mode_switch_radius_, virtual_lidar_z_);
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ScanGroundFilterComponent::onParameter, this, _1));

  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, "scan_ground_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
}

void ScanGroundFilterComponent::convertPointcloudGridScan(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
  std::vector<PointCloudRefVector> & out_radial_ordered_points)
{
  out_radial_ordered_points.resize(radial_dividers_num_);
  PointRef current_point;
  uint16_t back_steps_num = 1;

  grid_size_rad_ =
    normalizeRadian(std::atan2(grid_mode_switch_radius_ + grid_size_m_, virtual_lidar_z_)) -
    normalizeRadian(std::atan2(grid_mode_switch_radius_, virtual_lidar_z_));
  for (size_t i = 0; i < in_cloud->points.size(); ++i) {
    auto x{
      in_cloud->points[i].x - vehicle_info_.wheel_base_m / 2.0f -
      center_pcl_shift_};  // base on front wheel center
    // auto y{in_cloud->points[i].y};
    auto radius{static_cast<float>(std::hypot(x, in_cloud->points[i].y))};
    auto theta{normalizeRadian(std::atan2(x, in_cloud->points[i].y), 0.0)};

    // divide by vertical angle
    auto gamma{normalizeRadian(std::atan2(radius, virtual_lidar_z_), 0.0f)};
    auto radial_div{
      static_cast<size_t>(std::floor(normalizeDegree(theta / radial_divider_angle_rad_, 0.0)))};
    uint16_t grid_id = 0;
    float curr_grid_size = 0.0f;
    if (radius <= grid_mode_switch_radius_) {
      grid_id = static_cast<uint16_t>(radius / grid_size_m_);
      curr_grid_size = grid_size_m_;
    } else {
      grid_id = grid_mode_switch_grid_id_ + (gamma - grid_mode_switch_angle_rad_) / grid_size_rad_;
      if (grid_id <= grid_mode_switch_grid_id_ + back_steps_num) {
        curr_grid_size = grid_size_m_;
      } else {
        curr_grid_size = std::tan(gamma) - std::tan(gamma - grid_size_rad_);
        curr_grid_size *= virtual_lidar_z_;
      }
    }
    current_point.grid_id = grid_id;
    current_point.grid_size = curr_grid_size;
    current_point.radius = radius;
    current_point.theta = theta;
    current_point.radial_div = radial_div;
    current_point.point_state = PointLabel::INIT;
    current_point.orig_index = i;
    current_point.orig_point = &in_cloud->points[i];

    // radial divisions
    out_radial_ordered_points[radial_div].emplace_back(current_point);
  }

  // sort by distance
  for (size_t i = 0; i < radial_dividers_num_; ++i) {
    std::sort(
      out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
      [](const PointRef & a, const PointRef & b) { return a.radius < b.radius; });
  }
}
void ScanGroundFilterComponent::convertPointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
  std::vector<PointCloudRefVector> & out_radial_ordered_points)
{
  out_radial_ordered_points.resize(radial_dividers_num_);
  PointRef current_point;

  for (size_t i = 0; i < in_cloud->points.size(); ++i) {
    auto radius{static_cast<float>(std::hypot(in_cloud->points[i].x, in_cloud->points[i].y))};
    auto theta{normalizeRadian(std::atan2(in_cloud->points[i].x, in_cloud->points[i].y), 0.0)};
    auto radial_div{
      static_cast<size_t>(std::floor(normalizeDegree(theta / radial_divider_angle_rad_, 0.0)))};

    current_point.radius = radius;
    current_point.theta = theta;
    current_point.radial_div = radial_div;
    current_point.point_state = PointLabel::INIT;
    current_point.orig_index = i;
    current_point.orig_point = &in_cloud->points[i];

    // radial divisions
    out_radial_ordered_points[radial_div].emplace_back(current_point);
  }

  // sort by distance
  for (size_t i = 0; i < radial_dividers_num_; ++i) {
    std::sort(
      out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
      [](const PointRef & a, const PointRef & b) { return a.radius < b.radius; });
  }
}

void ScanGroundFilterComponent::calcVirtualGroundOrigin(pcl::PointXYZ & point)
{
  point.x = vehicle_info_.wheel_base_m;
  point.y = 0;
  point.z = 0;
}

void ScanGroundFilterComponent::initializeFirstGndGrids(
  const float h, const float r, const uint16_t id, std::vector<GridCenter> & gnd_grids)
{
  GridCenter curr_gnd_grid;
  for (int ind_grid = id - 1 - gnd_grid_buffer_size_; ind_grid < id - 1; ind_grid++) {
    float ind_gnd_z = ind_grid - id + 1 + gnd_grid_buffer_size_;
    ind_gnd_z *= h / static_cast<float>(gnd_grid_buffer_size_);

    float ind_gnd_radius = ind_grid - id + 1 + gnd_grid_buffer_size_;
    ind_gnd_radius *= r / static_cast<float>(gnd_grid_buffer_size_);

    curr_gnd_grid.radius = ind_gnd_radius;
    curr_gnd_grid.avg_height = ind_gnd_z;
    curr_gnd_grid.max_height = ind_gnd_z;
    curr_gnd_grid.grid_id = ind_grid;
    gnd_grids.push_back(curr_gnd_grid);
  }
}

void ScanGroundFilterComponent::checkContinuousGndGrid(
  PointRef & p, const std::vector<GridCenter> & gnd_grids_list, PointsCentroid & gnd_cluster)
{
  float next_gnd_z = 0.0f;
  float curr_gnd_slope_rad = 0.0f;
  float gnd_buff_z_mean = 0.0f;
  float gnd_buff_z_max = 0.0f;
  float gnd_buff_radius = 0.0f;

  for (auto it = gnd_grids_list.end() - gnd_grid_buffer_size_ - 1; it < gnd_grids_list.end() - 1;
       it++) {
    gnd_buff_radius += it->radius;
    gnd_buff_z_mean += it->avg_height;
    gnd_buff_z_max += it->max_height;
  }

  gnd_buff_radius /= static_cast<float>(gnd_grid_buffer_size_ - 1);
  gnd_buff_z_max /= static_cast<float>(gnd_grid_buffer_size_ - 1);
  gnd_buff_z_mean /= static_cast<float>(gnd_grid_buffer_size_ - 1);

  float tmp_delta_mean_z = gnd_grids_list.back().avg_height - gnd_buff_z_mean;
  float tmp_delta_radius = gnd_grids_list.back().radius - gnd_buff_radius;

  curr_gnd_slope_rad = std::atan(tmp_delta_mean_z / tmp_delta_radius);
  curr_gnd_slope_rad = curr_gnd_slope_rad < -global_slope_max_angle_rad_
                         ? -global_slope_max_angle_rad_
                         : curr_gnd_slope_rad;
  curr_gnd_slope_rad = curr_gnd_slope_rad > global_slope_max_angle_rad_
                         ? global_slope_max_angle_rad_
                         : curr_gnd_slope_rad;

  next_gnd_z = std::tan(curr_gnd_slope_rad) * (p.radius - gnd_buff_radius) + gnd_buff_z_mean;

  float gnd_z_local_thresh = std::tan(DEG2RAD(5.0)) * (p.radius - gnd_grids_list.back().radius);

  tmp_delta_mean_z = p.orig_point->z - (gnd_grids_list.end() - 2)->avg_height;
  tmp_delta_radius = p.radius - (gnd_grids_list.end() - 2)->radius;
  float local_slope = std::atan(tmp_delta_mean_z / tmp_delta_radius);
  if (
    abs(p.orig_point->z - next_gnd_z) <= non_ground_height_threshold_ + gnd_z_local_thresh ||
    abs(local_slope) <= local_slope_max_angle_rad_) {
    gnd_cluster.addPoint(p.radius, p.orig_point->z);
    p.point_state = PointLabel::GROUND;
  } else if (p.orig_point->z - next_gnd_z > non_ground_height_threshold_ + gnd_z_local_thresh) {
    p.point_state = PointLabel::NON_GROUND;
  }
}
void ScanGroundFilterComponent::checkDiscontinuousGndGrid(
  PointRef & p, const std::vector<GridCenter> & gnd_grids_list, PointsCentroid & gnd_cluster)
{
  float tmp_delta_max_z = p.orig_point->z - gnd_grids_list.back().max_height;
  float tmp_delta_avg_z = p.orig_point->z - gnd_grids_list.back().avg_height;
  float tmp_delta_radius = p.radius - gnd_grids_list.back().radius;
  float local_slope = std::atan(tmp_delta_avg_z / tmp_delta_radius);

  if (
    abs(local_slope) < local_slope_max_angle_rad_ ||
    abs(tmp_delta_avg_z) < non_ground_height_threshold_ ||
    abs(tmp_delta_max_z) < non_ground_height_threshold_) {
    gnd_cluster.addPoint(p.radius, p.orig_point->z);
    p.point_state = PointLabel::GROUND;
  } else if (local_slope > global_slope_max_angle_rad_) {
    p.point_state = PointLabel::NON_GROUND;
  }
}

void ScanGroundFilterComponent::checkBreakGndGrid(
  PointRef & p, const std::vector<GridCenter> & gnd_grids_list, PointsCentroid & gnd_cluster)
{
  float tmp_delta_avg_z = p.orig_point->z - gnd_grids_list.back().avg_height;
  float tmp_delta_radius = p.radius - gnd_grids_list.back().radius;
  float local_slope = std::atan(tmp_delta_avg_z / tmp_delta_radius);
  if (abs(local_slope) < global_slope_max_angle_rad_) {
    gnd_cluster.addPoint(p.radius, p.orig_point->z);
    p.point_state = PointLabel::GROUND;
  } else if (local_slope > global_slope_max_angle_rad_) {
    p.point_state = PointLabel::NON_GROUND;
  }
}
void ScanGroundFilterComponent::classifyPointCloudGridScan(
  std::vector<PointCloudRefVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices)
{
  out_no_ground_indices.indices.clear();
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) {
    PointsCentroid ground_cluster;
    ground_cluster.initialize();
    std::vector<GridCenter> gnd_grids;
    GridCenter curr_gnd_grid;

    // check empty ray
    if (in_radial_ordered_clouds[i].size() == 0) {
      continue;
    }

    // check the first point in ray
    auto * p = &in_radial_ordered_clouds[i][0];
    PointRef * prev_p;
    prev_p = &in_radial_ordered_clouds[i][0];  // for checking the distance to prev point

    bool initialized_first_gnd_grid = false;
    bool prev_list_init = false;

    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) {
      p = &in_radial_ordered_clouds[i][j];
      float global_slope_p = std::atan(p->orig_point->z / p->radius);
      float non_ground_height_threshold_local = non_ground_height_threshold_;
      if (p->orig_point->x < low_priority_region_x_) {
        non_ground_height_threshold_local =
          non_ground_height_threshold_ * abs(p->orig_point->x / low_priority_region_x_);
      }
      // classify first grid's point cloud
      if (
        !initialized_first_gnd_grid && global_slope_p >= global_slope_max_angle_rad_ &&
        p->orig_point->z > non_ground_height_threshold_local) {
        out_no_ground_indices.indices.push_back(p->orig_index);
        p->point_state = PointLabel::NON_GROUND;
        prev_p = p;
        continue;
      }

      if (
        !initialized_first_gnd_grid && abs(global_slope_p) < global_slope_max_angle_rad_ &&
        abs(p->orig_point->z) < non_ground_height_threshold_local) {
        ground_cluster.addPoint(p->radius, p->orig_point->z);
        p->point_state = PointLabel::GROUND;
        initialized_first_gnd_grid = static_cast<bool>(p->grid_id - prev_p->grid_id);
        prev_p = p;
        continue;
      }

      if (!initialized_first_gnd_grid) {
        prev_p = p;
        continue;
      }

      // initialize lists of previous gnd grids
      if (prev_list_init == false && initialized_first_gnd_grid == true) {
        float h = ground_cluster.getAverageHeight();
        float r = ground_cluster.getAverageRadius();
        initializeFirstGndGrids(h, r, p->grid_id, gnd_grids);
        prev_list_init = true;
      }

      if (prev_list_init == false && initialized_first_gnd_grid == false) {
        // assume first gnd grid is zero
        initializeFirstGndGrids(0.0f, p->radius, p->grid_id, gnd_grids);
        prev_list_init = true;
      }

      // move to new grid
      if (p->grid_id > prev_p->grid_id && ground_cluster.getAverageRadius() > 0.0) {
        // check if the prev grid have ground point cloud
        curr_gnd_grid.radius = ground_cluster.getAverageRadius();
        curr_gnd_grid.avg_height = ground_cluster.getAverageHeight();
        curr_gnd_grid.max_height = ground_cluster.getMaxHeight();
        curr_gnd_grid.grid_id = prev_p->grid_id;
        gnd_grids.push_back(curr_gnd_grid);
        ground_cluster.initialize();
      }
      // classify
      if (p->orig_point->z - gnd_grids.back().avg_height > detection_range_z_max_) {
        p->point_state = PointLabel::OUT_OF_RANGE;
        prev_p = p;
        continue;
      }
      float points_xy_distance = std::hypot(
        p->orig_point->x - prev_p->orig_point->x, p->orig_point->y - prev_p->orig_point->y);
      if (
        prev_p->point_state == PointLabel::NON_GROUND &&
        points_xy_distance < split_points_distance_tolerance_ &&
        p->orig_point->z > prev_p->orig_point->z) {
        p->point_state = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(p->orig_index);
        prev_p = p;
        continue;
      }

      if (global_slope_p > global_slope_max_angle_rad_) {
        out_no_ground_indices.indices.push_back(p->orig_index);
        prev_p = p;
        continue;
      }
      // gnd grid is continuous, the last gnd grid is close
      uint16_t next_gnd_grid_id_thresh = (gnd_grids.end() - gnd_grid_buffer_size_)->grid_id +
                                         gnd_grid_buffer_size_ + gnd_grid_continual_thresh_;
      if (
        p->grid_id < next_gnd_grid_id_thresh &&
        p->radius - gnd_grids.back().radius < gnd_grid_continual_thresh_ * p->grid_size) {
        checkContinuousGndGrid(*p, gnd_grids, ground_cluster);

      } else if (
        p->radius - gnd_grids.back().radius < gnd_grid_continual_thresh_ * p->grid_size ||
        p->radius < grid_mode_switch_radius_ * 2.0f) {
        checkDiscontinuousGndGrid(*p, gnd_grids, ground_cluster);
      } else {
        checkBreakGndGrid(*p, gnd_grids, ground_cluster);
      }
      if (p->point_state == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(p->orig_index);
      }
      prev_p = p;
    }
  }
}

void ScanGroundFilterComponent::classifyPointCloud(
  std::vector<PointCloudRefVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices)
{
  out_no_ground_indices.indices.clear();

  const pcl::PointXYZ init_ground_point(0, 0, 0);
  pcl::PointXYZ virtual_ground_point(0, 0, 0);
  calcVirtualGroundOrigin(virtual_ground_point);

  // point classification algorithm
  // sweep through each radial division
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) {
    float prev_gnd_radius = 0.0f;
    float prev_gnd_slope = 0.0f;
    float points_distance = 0.0f;
    PointsCentroid ground_cluster, non_ground_cluster;
    float local_slope = 0.0f;
    PointLabel prev_point_label = PointLabel::INIT;
    pcl::PointXYZ prev_gnd_point(0, 0, 0);
    // loop through each point in the radial div
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) {
      const float global_slope_max_angle = global_slope_max_angle_rad_;
      const float local_slope_max_angle = local_slope_max_angle_rad_;
      auto * p = &in_radial_ordered_clouds[i][j];
      auto * p_prev = &in_radial_ordered_clouds[i][j - 1];

      if (j == 0) {
        bool is_front_side = (p->orig_point->x > virtual_ground_point.x);
        if (use_virtual_ground_point_ && is_front_side) {
          prev_gnd_point = virtual_ground_point;
        } else {
          prev_gnd_point = init_ground_point;
        }
        prev_gnd_radius = std::hypot(prev_gnd_point.x, prev_gnd_point.y);
        prev_gnd_slope = 0.0f;
        ground_cluster.initialize();
        non_ground_cluster.initialize();
        points_distance = calcDistance3d(*p->orig_point, prev_gnd_point);
      } else {
        points_distance = calcDistance3d(*p->orig_point, *p_prev->orig_point);
      }

      float radius_distance_from_gnd = p->radius - prev_gnd_radius;
      float height_from_gnd = p->orig_point->z - prev_gnd_point.z;
      float height_from_obj = p->orig_point->z - non_ground_cluster.getAverageHeight();
      bool calculate_slope = false;
      bool is_point_close_to_prev =
        (points_distance <
         (p->radius * radial_divider_angle_rad_ + split_points_distance_tolerance_));

      float global_slope = std::atan2(p->orig_point->z, p->radius);
      // check points which is far enough from previous point
      if (global_slope > global_slope_max_angle) {
        p->point_state = PointLabel::NON_GROUND;
        calculate_slope = false;
      } else if (
        (prev_point_label == PointLabel::NON_GROUND) &&
        (std::abs(height_from_obj) >= split_height_distance_)) {
        calculate_slope = true;
      } else if (is_point_close_to_prev && std::abs(height_from_gnd) < split_height_distance_) {
        // close to the previous point, set point follow label
        p->point_state = PointLabel::POINT_FOLLOW;
        calculate_slope = false;
      } else {
        calculate_slope = true;
      }
      if (is_point_close_to_prev) {
        height_from_gnd = p->orig_point->z - ground_cluster.getAverageHeight();
        radius_distance_from_gnd = p->radius - ground_cluster.getAverageRadius();
      }
      if (calculate_slope) {
        // far from the previous point
        local_slope = std::atan2(height_from_gnd, radius_distance_from_gnd);
        if (local_slope - prev_gnd_slope > local_slope_max_angle) {
          // the point is outside of the local slope threshold
          p->point_state = PointLabel::NON_GROUND;
        } else {
          p->point_state = PointLabel::GROUND;
        }
      }

      if (p->point_state == PointLabel::GROUND) {
        ground_cluster.initialize();
        non_ground_cluster.initialize();
      }
      if (p->point_state == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(p->orig_index);
      } else if (  // NOLINT
        (prev_point_label == PointLabel::NON_GROUND) &&
        (p->point_state == PointLabel::POINT_FOLLOW)) {
        p->point_state = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(p->orig_index);
      } else if (  // NOLINT
        (prev_point_label == PointLabel::GROUND) && (p->point_state == PointLabel::POINT_FOLLOW)) {
        p->point_state = PointLabel::GROUND;
      } else {
      }

      // update the ground state
      prev_point_label = p->point_state;
      if (p->point_state == PointLabel::GROUND) {
        prev_gnd_radius = p->radius;
        prev_gnd_point = pcl::PointXYZ(p->orig_point->x, p->orig_point->y, p->orig_point->z);
        ground_cluster.addPoint(p->radius, p->orig_point->z);
        prev_gnd_slope = ground_cluster.getAverageSlope();
      }
      // update the non ground state
      if (p->point_state == PointLabel::NON_GROUND) {
        non_ground_cluster.addPoint(p->radius, p->orig_point->z);
      }
    }
  }
}

void ScanGroundFilterComponent::extractObjectPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_object_cloud_ptr)
{
  for (const auto & i : in_indices.indices) {
    out_object_cloud_ptr->points.emplace_back(in_cloud_ptr->points[i]);
  }
}

void ScanGroundFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *current_sensor_cloud_ptr);

  std::vector<PointCloudRefVector> radial_ordered_points;

  pcl::PointIndices no_ground_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  no_ground_cloud_ptr->points.reserve(current_sensor_cloud_ptr->points.size());

  if (elevation_grid_mode_) {
    convertPointcloudGridScan(current_sensor_cloud_ptr, radial_ordered_points);
    classifyPointCloudGridScan(radial_ordered_points, no_ground_indices);
  } else {
    convertPointcloud(current_sensor_cloud_ptr, radial_ordered_points);
    classifyPointCloud(radial_ordered_points, no_ground_indices);
  }

  extractObjectPoints(current_sensor_cloud_ptr, no_ground_indices, no_ground_cloud_ptr);

  auto no_ground_cloud_msg_ptr = std::make_shared<PointCloud2>();
  pcl::toROSMsg(*no_ground_cloud_ptr, *no_ground_cloud_msg_ptr);

  no_ground_cloud_msg_ptr->header = input->header;
  output = *no_ground_cloud_msg_ptr;

  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

rcl_interfaces::msg::SetParametersResult ScanGroundFilterComponent::onParameter(
  const std::vector<rclcpp::Parameter> & p)
{
  double global_slope_max_angle_deg{get_parameter("global_slope_max_angle_deg").as_double()};
  if (get_param(p, "global_slope_max_angle_deg", global_slope_max_angle_deg)) {
    global_slope_max_angle_rad_ = deg2rad(global_slope_max_angle_deg);
    RCLCPP_DEBUG(
      get_logger(), "Setting global_slope_max_angle_rad to: %f.", global_slope_max_angle_rad_);
  }
  double local_slope_max_angle_deg{get_parameter("local_slope_max_angle_deg").as_double()};
  if (get_param(p, "local_slope_max_angle_deg", local_slope_max_angle_deg)) {
    local_slope_max_angle_rad_ = deg2rad(local_slope_max_angle_deg);
    RCLCPP_DEBUG(
      get_logger(), "Setting local_slope_max_angle_rad to: %f.", local_slope_max_angle_rad_);
  }
  double radial_divider_angle_deg{get_parameter("radial_divider_angle_deg").as_double()};
  if (get_param(p, "radial_divider_angle_deg", radial_divider_angle_deg)) {
    radial_divider_angle_rad_ = deg2rad(radial_divider_angle_deg);
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    RCLCPP_DEBUG(
      get_logger(), "Setting radial_divider_angle_rad to: %f.", radial_divider_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting radial_dividers_num to: %zu.", radial_dividers_num_);
  }
  if (get_param(p, "split_points_distance_tolerance", split_points_distance_tolerance_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting split_points_distance_tolerance to: %f.",
      split_points_distance_tolerance_);
  }
  if (get_param(p, "split_height_distance", split_height_distance_)) {
    RCLCPP_DEBUG(get_logger(), "Setting split_height_distance to: %f.", split_height_distance_);
  }
  if (get_param(p, "use_virtual_ground_point", use_virtual_ground_point_)) {
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Setting use_virtual_ground_point to: " << std::boolalpha << use_virtual_ground_point_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace ground_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ground_segmentation::ScanGroundFilterComponent)
