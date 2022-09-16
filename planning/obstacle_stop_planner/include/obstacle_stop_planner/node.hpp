// Copyright 2019 Autoware Foundation
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

#ifndef OBSTACLE_STOP_PLANNER__NODE_HPP_
#define OBSTACLE_STOP_PLANNER__NODE_HPP_

#include "obstacle_stop_planner/adaptive_cruise_control.hpp"
#include "obstacle_stop_planner/debug_marker.hpp"
#include "obstacle_stop_planner/planner_data.hpp"

#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_planning_msgs/msg/expand_stop_range.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace motion_planning
{

namespace bg = boost::geometry;

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using std_msgs::msg::Header;

using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using tier4_debug_msgs::msg::BoolStamped;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_debug_msgs::msg::Float32Stamped;
using tier4_planning_msgs::msg::ExpandStopRange;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_planning_msgs::msg::VelocityLimitClearCommand;
using vehicle_info_util::VehicleInfo;

using TrajectoryPoints = std::vector<TrajectoryPoint>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_point_cloud_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;

  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr sub_acceleration_;

  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_dynamic_objects_;

  rclcpp::Subscription<ExpandStopRange>::SharedPtr sub_expand_stop_range_;

  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;

  rclcpp::Publisher<DiagnosticStatus>::SharedPtr pub_stop_reason_;

  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;

  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;

  std::unique_ptr<AdaptiveCruiseController> acc_controller_;
  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  boost::optional<StopPoint> latest_stop_point_{boost::none};
  boost::optional<SlowDownSection> latest_slow_down_section_{boost::none};
  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr_{nullptr};
  PredictedObjects::ConstSharedPtr object_ptr_{nullptr};
  rclcpp::Time last_detect_time_collision_point_;
  rclcpp::Time last_detect_time_slowdown_point_;

  Odometry::ConstSharedPtr current_velocity_ptr_{nullptr};
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_ptr_{nullptr};
  bool is_driving_forward_{true};

  bool set_velocity_limit_{false};

  VehicleInfo vehicle_info_;
  NodeParam node_param_;
  StopParam stop_param_;
  SlowDownParam slow_down_param_;

  // mutex for vehicle_info_, stop_param_, current_acc_, obstacle_ros_pointcloud_ptr_
  // NOTE: shared_ptr itself is thread safe so we do not have to care if *ptr is not used
  //   (current_velocity_ptr_)
  std::mutex mutex_;

  void searchObstacle(
    const TrajectoryPoints & decimate_trajectory, TrajectoryPoints & output,
    PlannerData & planner_data, const Header & trajectory_header, const VehicleInfo & vehicle_info,
    const StopParam & stop_param, const PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr);

  void insertVelocity(
    TrajectoryPoints & trajectory, PlannerData & planner_data, const Header & trajectory_header,
    const VehicleInfo & vehicle_info, const double current_acc, const double current_vel,
    const StopParam & stop_param);

  bool searchPointcloudNearTrajectory(
    const TrajectoryPoints & trajectory, const PointCloud2::ConstSharedPtr & input_points_ptr,
    PointCloud::Ptr output_points_ptr, const Header & trajectory_header,
    const VehicleInfo & vehicle_info, const StopParam & stop_param);

  StopPoint createTargetPoint(
    const int idx, const double margin, const TrajectoryPoints & base_trajectory,
    const double dist_remain);

  StopPoint searchInsertPoint(
    const int idx, const TrajectoryPoints & base_trajectory, const double dist_remain,
    const StopParam & stop_param);

  SlowDownSection createSlowDownSection(
    const int idx, const TrajectoryPoints & base_trajectory, const double lateral_deviation,
    const double dist_remain, const double dist_vehicle_to_obstacle,
    const VehicleInfo & vehicle_info, const double current_acc, const double current_vel);

  SlowDownSection createSlowDownSectionFromMargin(
    const int idx, const TrajectoryPoints & base_trajectory, const double forward_margin,
    const double backward_margin, const double velocity);

  void insertSlowDownSection(const SlowDownSection & slow_down_section, TrajectoryPoints & output);

  TrajectoryPoints trimTrajectoryWithIndexFromSelfPose(
    const TrajectoryPoints & input, const Pose & self_pose, size_t & index);

  void setExternalVelocityLimit();

  void resetExternalVelocityLimit(const double current_acc, const double current_vel);

  void publishDebugData(
    const PlannerData & planner_data, const double current_acc, const double current_vel);

  // Callback
  void onTrigger(const Trajectory::ConstSharedPtr input_msg);

  void onOdometry(const Odometry::ConstSharedPtr input_msg);

  void onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr input_msg);

  void onPointCloud(const PointCloud2::ConstSharedPtr input_msg);

  void onDynamicObjects(const PredictedObjects::ConstSharedPtr input_msg);

  void onExpandStopRange(const ExpandStopRange::ConstSharedPtr input_msg);
};
}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__NODE_HPP_
