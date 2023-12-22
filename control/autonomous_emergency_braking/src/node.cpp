// Copyright 2022 TIER IV, Inc.
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

#include "autonomous_emergency_braking/node.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>

#include <pcl/filters/voxel_grid.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/within.hpp>
namespace autoware::motion::control::autonomous_emergency_braking
{
using diagnostic_msgs::msg::DiagnosticStatus;
namespace bg = boost::geometry;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

Polygon2d createPolygon(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & next_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double expand_width)
{
  Polygon2d polygon;

  const double longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0 + expand_width;
  const double rear_overhang = vehicle_info.rear_overhang_m;

  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(base_pose, longitudinal_offset, width, 0.0).position);
  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(base_pose, longitudinal_offset, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(base_pose, -rear_overhang, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(base_pose, -rear_overhang, width, 0.0).position);

  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(next_pose, longitudinal_offset, width, 0.0).position);
  appendPointToPolygon(
    polygon,
    tier4_autoware_utils::calcOffsetPose(next_pose, longitudinal_offset, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(next_pose, -rear_overhang, -width, 0.0).position);
  appendPointToPolygon(
    polygon, tier4_autoware_utils::calcOffsetPose(next_pose, -rear_overhang, width, 0.0).position);

  polygon = tier4_autoware_utils::isClockwise(polygon)
              ? polygon
              : tier4_autoware_utils::inverseClockwise(polygon);

  Polygon2d hull_polygon;
  bg::convex_hull(polygon, hull_polygon);

  return hull_polygon;
}

AEB::AEB(const rclcpp::NodeOptions & node_options)
: Node("AEB", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  collision_data_keeper_(this->get_clock())
{
  // Subscribers
  sub_point_cloud_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&AEB::onPointCloud, this, std::placeholders::_1));

  sub_velocity_ = this->create_subscription<VelocityReport>(
    "~/input/velocity", rclcpp::QoS{1}, std::bind(&AEB::onVelocity, this, std::placeholders::_1));

  sub_imu_ = this->create_subscription<Imu>(
    "~/input/imu", rclcpp::QoS{1}, std::bind(&AEB::onImu, this, std::placeholders::_1));

  sub_predicted_traj_ = this->create_subscription<Trajectory>(
    "~/input/predicted_trajectory", rclcpp::QoS{1},
    std::bind(&AEB::onPredictedTrajectory, this, std::placeholders::_1));

  sub_autoware_state_ = this->create_subscription<AutowareState>(
    "/autoware/state", rclcpp::QoS{1},
    std::bind(&AEB::onAutowareState, this, std::placeholders::_1));

  // Publisher
  pub_obstacle_pointcloud_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/obstacle_pointcloud", 1);
  debug_ego_path_publisher_ = this->create_publisher<MarkerArray>("~/debug/markers", 1);

  // Diagnostics
  updater_.setHardwareID("autonomous_emergency_braking");
  updater_.add("aeb_emergency_stop", this, &AEB::onCheckCollision);

  // parameter
  publish_debug_pointcloud_ = declare_parameter<bool>("publish_debug_pointcloud");
  use_predicted_trajectory_ = declare_parameter<bool>("use_predicted_trajectory");
  use_imu_path_ = declare_parameter<bool>("use_imu_path");
  voxel_grid_x_ = declare_parameter<double>("voxel_grid_x");
  voxel_grid_y_ = declare_parameter<double>("voxel_grid_y");
  voxel_grid_z_ = declare_parameter<double>("voxel_grid_z");
  min_generated_path_length_ = declare_parameter<double>("min_generated_path_length");
  expand_width_ = declare_parameter<double>("expand_width");
  longitudinal_offset_ = declare_parameter<double>("longitudinal_offset");
  t_response_ = declare_parameter<double>("t_response");
  a_ego_min_ = declare_parameter<double>("a_ego_min");
  a_obj_min_ = declare_parameter<double>("a_obj_min");
  imu_prediction_time_horizon_ = declare_parameter<double>("imu_prediction_time_horizon");
  imu_prediction_time_interval_ = declare_parameter<double>("imu_prediction_time_interval");
  mpc_prediction_time_horizon_ = declare_parameter<double>("mpc_prediction_time_horizon");
  mpc_prediction_time_interval_ = declare_parameter<double>("mpc_prediction_time_interval");

  const auto collision_keeping_sec = declare_parameter<double>("collision_keeping_sec");
  collision_data_keeper_.setTimeout(collision_keeping_sec);

  // start time
  const double aeb_hz = declare_parameter<double>("aeb_hz");
  const auto period_ns = rclcpp::Rate(aeb_hz).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&AEB::onTimer, this));
}

void AEB::onTimer()
{
  updater_.force_update();
}

void AEB::onVelocity(const VelocityReport::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

void AEB::onImu(const Imu::ConstSharedPtr input_msg)
{
  // transform imu
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "base_link", input_msg->header.frame_id, input_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[AEB] Failed to look up transform from base_link to" << input_msg->header.frame_id);
    return;
  }

  angular_velocity_ptr_ = std::make_shared<Vector3>();
  tf2::doTransform(input_msg->angular_velocity, *angular_velocity_ptr_, transform_stamped);
}

void AEB::onPredictedTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg)
{
  predicted_traj_ptr_ = input_msg;
}

void AEB::onAutowareState(const AutowareState::ConstSharedPtr input_msg)
{
  autoware_state_ = input_msg;
}

void AEB::onPointCloud(const PointCloud2::ConstSharedPtr input_msg)
{
  PointCloud::Ptr pointcloud_ptr(new PointCloud);
  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);

  if (input_msg->header.frame_id != "base_link") {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[AEB]: Input point cloud frame is not base_link and it is " << input_msg->header.frame_id);
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped{};
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link", input_msg->header.frame_id, input_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "[AEB] Failed to look up transform from base_link to" << input_msg->header.frame_id);
      return;
    }

    // transform by using eigen matrix
    PointCloud2 transformed_points{};
    const Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(affine_matrix, *input_msg, transformed_points);
    pcl::fromROSMsg(transformed_points, *pointcloud_ptr);
  }

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);
  filter.setInputCloud(pointcloud_ptr);
  filter.setLeafSize(voxel_grid_x_, voxel_grid_y_, voxel_grid_z_);
  filter.filter(*no_height_filtered_pointcloud_ptr);

  obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
  if (publish_debug_pointcloud_) {
    pub_obstacle_pointcloud_->publish(*obstacle_ros_pointcloud_ptr_);
  }
}

bool AEB::isDataReady()
{
  const auto missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "[AEB] waiting for %s", name);
    return false;
  };

  if (!current_velocity_ptr_) {
    return missing("ego velocity");
  }

  if (!obstacle_ros_pointcloud_ptr_) {
    return missing("object pointcloud");
  }

  if (use_imu_path_ && !angular_velocity_ptr_) {
    return missing("imu");
  }

  if (use_predicted_trajectory_ && !predicted_traj_ptr_) {
    return missing("control predicted trajectory");
  }

  if (!autoware_state_) {
    return missing("autoware_state");
  }

  return true;
}

void AEB::onCheckCollision(DiagnosticStatusWrapper & stat)
{
  MarkerArray debug_markers;
  checkCollision(debug_markers);

  if (!collision_data_keeper_.checkExpired()) {
    const std::string error_msg = "[AEB]: Emergency Brake";
    const auto diag_level = DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
    const auto & data = collision_data_keeper_.get();
    stat.addf("RSS", "%.2f", data.rss);
    stat.addf("Distance", "%.2f", data.distance_to_object);
    addCollisionMarker(data, debug_markers);
  } else {
    const std::string error_msg = "[AEB]: No Collision";
    const auto diag_level = DiagnosticStatus::OK;
    stat.summary(diag_level, error_msg);
  }

  // publish debug markers
  debug_ego_path_publisher_->publish(debug_markers);
}

bool AEB::checkCollision(MarkerArray & debug_markers)
{
  // step1. check data
  if (!isDataReady()) {
    return false;
  }

  // if not driving, disable aeb
  if (autoware_state_->state != AutowareState::DRIVING) {
    return false;
  }

  // step2. create velocity data check if the vehicle stops or not
  const double current_v = current_velocity_ptr_->longitudinal_velocity;
  if (current_v < 0.1) {
    return false;
  }

  // step3. create ego path based on sensor data
  bool has_collision_ego = false;
  if (use_imu_path_) {
    Path ego_path;
    std::vector<Polygon2d> ego_polys;
    const double current_w = angular_velocity_ptr_->z;
    constexpr double color_r = 0.0 / 256.0;
    constexpr double color_g = 148.0 / 256.0;
    constexpr double color_b = 205.0 / 256.0;
    constexpr double color_a = 0.999;
    const auto current_time = get_clock()->now();
    generateEgoPath(current_v, current_w, ego_path, ego_polys);

    std::vector<ObjectData> objects;
    createObjectData(ego_path, ego_polys, current_time, objects);
    has_collision_ego = hasCollision(current_v, ego_path, objects);

    std::string ns = "ego";
    addMarker(
      current_time, ego_path, ego_polys, objects, color_r, color_g, color_b, color_a, ns,
      debug_markers);
  }

  // step4. transform predicted trajectory from control module
  bool has_collision_predicted = false;
  if (use_predicted_trajectory_) {
    Path predicted_path;
    std::vector<Polygon2d> predicted_polys;
    const auto predicted_traj_ptr = predicted_traj_ptr_;
    constexpr double color_r = 0.0;
    constexpr double color_g = 100.0 / 256.0;
    constexpr double color_b = 0.0;
    constexpr double color_a = 0.999;
    const auto current_time = predicted_traj_ptr->header.stamp;
    generateEgoPath(*predicted_traj_ptr, predicted_path, predicted_polys);
    std::vector<ObjectData> objects;
    createObjectData(predicted_path, predicted_polys, current_time, objects);
    has_collision_predicted = hasCollision(current_v, predicted_path, objects);

    std::string ns = "predicted";
    addMarker(
      current_time, predicted_path, predicted_polys, objects, color_r, color_g, color_b, color_a,
      ns, debug_markers);
  }

  return has_collision_ego || has_collision_predicted;
}

bool AEB::hasCollision(
  const double current_v, const Path & ego_path, const std::vector<ObjectData> & objects)
{
  // calculate RSS
  const auto current_p = tier4_autoware_utils::createPoint(
    ego_path[0].position.x, ego_path[0].position.y, ego_path[0].position.z);
  const double & t = t_response_;
  for (const auto & obj : objects) {
    const double & obj_v = obj.velocity;
    const double rss_dist = current_v * t + (current_v * current_v) / (2 * std::fabs(a_ego_min_)) -
                            obj_v * obj_v / (2 * std::fabs(a_obj_min_)) + longitudinal_offset_;

    // check the object is front the ego or not
    if ((obj.position.x - ego_path[0].position.x) > 0) {
      const double dist_ego_to_object =
        motion_utils::calcSignedArcLength(ego_path, current_p, obj.position) -
        vehicle_info_.max_longitudinal_offset_m;
      if (dist_ego_to_object < rss_dist) {
        // collision happens
        ObjectData collision_data = obj;
        collision_data.rss = rss_dist;
        collision_data.distance_to_object = dist_ego_to_object;
        collision_data_keeper_.update(collision_data);
        return true;
      }
    }
  }
  return false;
}

void AEB::generateEgoPath(
  const double curr_v, const double curr_w, Path & path, std::vector<Polygon2d> & polygons)
{
  double curr_x = 0.0;
  double curr_y = 0.0;
  double curr_yaw = 0.0;
  geometry_msgs::msg::Pose ini_pose;
  ini_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
  ini_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
  path.push_back(ini_pose);

  if (curr_v < 0.1) {
    // if current velocity is too small, assume it stops at the same point
    return;
  }

  constexpr double epsilon = 1e-6;
  const double & dt = imu_prediction_time_interval_;
  const double & horizon = imu_prediction_time_horizon_;
  for (double t = 0.0; t < horizon + epsilon; t += dt) {
    curr_x = curr_x + curr_v * std::cos(curr_yaw) * dt;
    curr_y = curr_y + curr_v * std::sin(curr_yaw) * dt;
    curr_yaw = curr_yaw + curr_w * dt;
    geometry_msgs::msg::Pose current_pose;
    current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
    current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
    if (tier4_autoware_utils::calcDistance2d(path.back(), current_pose) < 1e-3) {
      continue;
    }
    path.push_back(current_pose);
  }

  // If path is shorter than minimum path length
  while (motion_utils::calcArcLength(path) < min_generated_path_length_) {
    curr_x = curr_x + curr_v * std::cos(curr_yaw) * dt;
    curr_y = curr_y + curr_v * std::sin(curr_yaw) * dt;
    curr_yaw = curr_yaw + curr_w * dt;
    geometry_msgs::msg::Pose current_pose;
    current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
    current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
    if (tier4_autoware_utils::calcDistance2d(path.back(), current_pose) < 1e-3) {
      continue;
    }
    path.push_back(current_pose);
  }

  // generate ego polygons
  polygons.resize(path.size() - 1);
  for (size_t i = 0; i < path.size() - 1; ++i) {
    polygons.at(i) = createPolygon(path.at(i), path.at(i + 1), vehicle_info_, expand_width_);
  }
}

void AEB::generateEgoPath(
  const Trajectory & predicted_traj, Path & path,
  std::vector<tier4_autoware_utils::Polygon2d> & polygons)
{
  if (predicted_traj.points.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "base_link", predicted_traj.header.frame_id, predicted_traj.header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "[AEB] Failed to look up transform from base_link to map");
    return;
  }

  // create path
  path.resize(predicted_traj.points.size());
  for (size_t i = 0; i < predicted_traj.points.size(); ++i) {
    geometry_msgs::msg::Pose map_pose;
    tf2::doTransform(predicted_traj.points.at(i).pose, map_pose, transform_stamped);
    path.at(i) = map_pose;

    if (i * mpc_prediction_time_interval_ > mpc_prediction_time_horizon_) {
      break;
    }
  }
  // create polygon
  polygons.resize(path.size());
  for (size_t i = 0; i < path.size() - 1; ++i) {
    polygons.at(i) = createPolygon(path.at(i), path.at(i + 1), vehicle_info_, expand_width_);
  }
}

void AEB::createObjectData(
  const Path & ego_path, const std::vector<tier4_autoware_utils::Polygon2d> & ego_polys,
  const rclcpp::Time & stamp, std::vector<ObjectData> & objects)
{
  // check if the predicted path has valid number of points
  if (ego_path.size() < 2 || ego_polys.empty()) {
    return;
  }

  PointCloud::Ptr obstacle_points_ptr(new PointCloud);
  pcl::fromROSMsg(*obstacle_ros_pointcloud_ptr_, *obstacle_points_ptr);
  for (const auto & point : obstacle_points_ptr->points) {
    ObjectData obj;
    obj.stamp = stamp;
    obj.position = tier4_autoware_utils::createPoint(point.x, point.y, point.z);
    obj.velocity = 0.0;
    const Point2d obj_point(point.x, point.y);
    const double lat_dist = motion_utils::calcLateralOffset(ego_path, obj.position);
    if (lat_dist > 5.0) {
      continue;
    }
    for (const auto & ego_poly : ego_polys) {
      if (bg::within(obj_point, ego_poly)) {
        objects.push_back(obj);
        break;
      }
    }
  }
}

void AEB::addMarker(
  const rclcpp::Time & current_time, const Path & path, const std::vector<Polygon2d> & polygons,
  const std::vector<ObjectData> & objects, const double color_r, const double color_g,
  const double color_b, const double color_a, const std::string & ns, MarkerArray & debug_markers)
{
  auto path_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", current_time, ns + "_path", 0L, Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.2, 0.2, 0.2),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  path_marker.points.resize(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    path_marker.points.at(i) = path.at(i).position;
  }
  debug_markers.markers.push_back(path_marker);

  auto polygon_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", current_time, ns + "_polygon", 0, Marker::LINE_LIST,
    tier4_autoware_utils::createMarkerScale(0.03, 0.0, 0.0),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto & poly : polygons) {
    for (size_t dp_idx = 0; dp_idx < poly.outer().size(); ++dp_idx) {
      const auto & boost_cp = poly.outer().at(dp_idx);
      const auto & boost_np = poly.outer().at((dp_idx + 1) % poly.outer().size());
      const auto curr_point = tier4_autoware_utils::createPoint(boost_cp.x(), boost_cp.y(), 0.0);
      const auto next_point = tier4_autoware_utils::createPoint(boost_np.x(), boost_np.y(), 0.0);
      polygon_marker.points.push_back(curr_point);
      polygon_marker.points.push_back(next_point);
    }
  }
  debug_markers.markers.push_back(polygon_marker);

  auto object_data_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", current_time, ns + "_objects", 0, Marker::SPHERE_LIST,
    tier4_autoware_utils::createMarkerScale(0.05, 0.05, 0.05),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto & e : objects) {
    object_data_marker.points.push_back(e.position);
  }
  debug_markers.markers.push_back(object_data_marker);
}

void AEB::addCollisionMarker(const ObjectData & data, MarkerArray & debug_markers)
{
  auto point_marker = tier4_autoware_utils::createDefaultMarker(
    "base_link", data.stamp, "collision_point", 0, Marker::SPHERE,
    tier4_autoware_utils::createMarkerScale(0.3, 0.3, 0.3),
    tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.3));
  point_marker.pose.position = data.position;
  debug_markers.markers.push_back(point_marker);
}

}  // namespace autoware::motion::control::autonomous_emergency_braking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::autonomous_emergency_braking::AEB)
