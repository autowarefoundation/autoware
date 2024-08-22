// Copyright 2021 The Autoware Foundation.
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

#ifndef SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_
#define SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"
#include "simple_planning_simulator/visibility_control.hpp"
#include "tier4_api_utils/tier4_api_utils.hpp"

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/srv/control_mode_command.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tier4_external_api_msgs/srv/initialize_pose.hpp"
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <random>
#include <string>
#include <variant>
#include <vector>

namespace simulation
{
namespace simple_planning_simulator
{

using autoware_control_msgs::msg::Control;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::ControlModeReport;
using autoware_vehicle_msgs::msg::Engage;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::GearReport;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::HazardLightsReport;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using autoware_vehicle_msgs::msg::VelocityReport;
using autoware_vehicle_msgs::srv::ControlModeCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using tier4_external_api_msgs::srv::InitializePose;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;
using tier4_vehicle_msgs::msg::ActuationStatusStamped;

class DeltaTime
{
public:
  DeltaTime() : prev_updated_time_ptr_(nullptr) {}
  double get_dt(const rclcpp::Time & now)
  {
    if (prev_updated_time_ptr_ == nullptr) {
      prev_updated_time_ptr_ = std::make_shared<rclcpp::Time>(now);
      return 0.0;
    }
    const double dt = (now - *prev_updated_time_ptr_).seconds();
    *prev_updated_time_ptr_ = now;
    return dt;
  }

private:
  std::shared_ptr<rclcpp::Time> prev_updated_time_ptr_;
};

class MeasurementNoiseGenerator
{
public:
  MeasurementNoiseGenerator() {}

  std::shared_ptr<std::mt19937> rand_engine_;
  std::shared_ptr<std::normal_distribution<>> pos_dist_;
  std::shared_ptr<std::normal_distribution<>> vel_dist_;
  std::shared_ptr<std::normal_distribution<>> rpy_dist_;
  std::shared_ptr<std::normal_distribution<>> steer_dist_;
};

using InputCommand = std::variant<std::monostate, ActuationCommandStamped, Control>;

class PLANNING_SIMULATOR_PUBLIC SimplePlanningSimulator : public rclcpp::Node
{
public:
  explicit SimplePlanningSimulator(const rclcpp::NodeOptions & options);

private:
  /* ros system */
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_report_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_report_;
  rclcpp::Publisher<TurnIndicatorsReport>::SharedPtr pub_turn_indicators_report_;
  rclcpp::Publisher<HazardLightsReport>::SharedPtr pub_hazard_lights_report_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_current_pose_;
  rclcpp::Publisher<ActuationStatusStamped>::SharedPtr pub_actuation_status_;

  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_cmd_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_manual_gear_cmd_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_cmd_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_cmd_;
  rclcpp::Subscription<Control>::SharedPtr sub_manual_ackermann_cmd_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_init_twist_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Engage>::SharedPtr sub_engage_;

  // todo
  rclcpp::Subscription<Control>::SharedPtr sub_ackermann_cmd_;
  rclcpp::Subscription<ActuationCommandStamped>::SharedPtr sub_actuation_cmd_;

  rclcpp::Service<ControlModeCommand>::SharedPtr srv_mode_req_;

  rclcpp::CallbackGroup::SharedPtr group_api_service_;
  tier4_api_utils::Service<InitializePose>::SharedPtr srv_set_pose_;

  uint32_t timer_sampling_time_ms_;        //!< @brief timer sampling time
  rclcpp::TimerBase::SharedPtr on_timer_;  //!< @brief timer for simulation

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  lanelet::ConstLanelets road_lanelets_;

  /* tf */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /* received & published topics */
  PoseWithCovarianceStamped::ConstSharedPtr initial_pose_{};
  TwistStamped initial_twist_{};
  VelocityReport current_velocity_{};
  Odometry current_odometry_{};
  SteeringReport current_steer_{};
  Control current_ackermann_cmd_{};
  Control current_manual_ackermann_cmd_{};
  GearCommand current_gear_cmd_{};
  GearCommand current_manual_gear_cmd_{};
  TurnIndicatorsCommand::ConstSharedPtr current_turn_indicators_cmd_ptr_{};
  HazardLightsCommand::ConstSharedPtr current_hazard_lights_cmd_ptr_{};
  Trajectory::ConstSharedPtr current_trajectory_ptr_{};
  bool simulate_motion_ = true;  //!< stop vehicle motion simulation if false
  ControlModeReport current_control_mode_{};
  bool enable_road_slope_simulation_ = true;

  // if false, it is expected to be converted and published from actuation_status in other nodes
  // (e.g. raw_vehicle_cmd_converter)
  bool enable_pub_steer_ = true;  //!< @brief flag to publish steering report.

  /* frame_id */
  std::string simulated_frame_id_ = "";  //!< @brief simulated vehicle frame id
  std::string origin_frame_id_ = "";     //!< @brief map frame_id

  /* flags */
  bool is_initialized_ = false;         //!< @brief flag to check the initial position is set
  bool add_measurement_noise_ = false;  //!< @brief flag to add measurement noise

  InputCommand current_input_command_{};

  DeltaTime delta_time_{};  //!< @brief to calculate delta time

  MeasurementNoiseGenerator measurement_noise_{};  //!< @brief for measurement noise

  double x_stddev_ = 0.0;  //!< @brief x standard deviation for dummy covariance in map coordinate
  double y_stddev_ = 0.0;  //!< @brief y standard deviation for dummy covariance in map coordinate

  /* vehicle model */
  enum class VehicleModelType {
    IDEAL_STEER_ACC = 0,
    IDEAL_STEER_ACC_GEARED = 1,
    DELAY_STEER_ACC = 2,
    DELAY_STEER_ACC_GEARED = 3,
    IDEAL_STEER_VEL = 4,
    DELAY_STEER_VEL = 5,
    DELAY_STEER_MAP_ACC_GEARED = 6,
    LEARNED_STEER_VEL = 7,
    DELAY_STEER_ACC_GEARED_WO_FALL_GUARD = 8,
    ACTUATION_CMD = 9
  } vehicle_model_type_;  //!< @brief vehicle model type to decide the model dynamics
  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model pointer

  void set_input(const InputCommand & cmd, const double acc_by_slope);

  /**
   * @brief set input steering, velocity, and acceleration of the vehicle model
   */
  void set_input(const Control & cmd, const double acc_by_slope);

  void set_input(const ActuationCommandStamped & cmd, const double acc_by_slope);

  /**
   * @brief set current_vehicle_state_ with received message
   */
  void on_turn_indicators_cmd(const TurnIndicatorsCommand::ConstSharedPtr msg);

  /**
   * @brief set current_vehicle_state_ with received message
   */
  void on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg);

  /**
   * @brief subscribe lanelet map
   */
  void on_map(const LaneletMapBin::ConstSharedPtr msg);

  /**
   * @brief set initial pose for simulation with received message
   */
  void on_initialpose(const PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief set initial twist for simulation with received message
   */
  void on_initialtwist(const TwistStamped::ConstSharedPtr msg);

  /**
   * @brief set initial pose for simulation with received request
   */
  void on_set_pose(
    const InitializePose::Request::ConstSharedPtr request,
    const InitializePose::Response::SharedPtr response);

  /**
   * @brief subscribe trajectory for deciding self z position.
   */
  void on_trajectory(const Trajectory::ConstSharedPtr msg);

  /**
   * @brief subscribe autoware engage
   */
  void on_engage(const Engage::ConstSharedPtr msg);

  /**
   * @brief ControlModeRequest server
   */
  void on_control_mode_request(
    const ControlModeCommand::Request::ConstSharedPtr request,
    const ControlModeCommand::Response::SharedPtr response);

  /**
   * @brief get z-position from trajectory
   * @param [in] x current x-position
   * @param [in] y current y-position
   * @param [in] prev_odometry odometry calculated in the previous step
   * @return get z-position from trajectory
   */
  double get_z_pose_from_trajectory(const double x, const double y, const Odometry & prev_odometry);

  /**
   * @brief get transform from two frame_ids
   * @param [in] parent_frame parent frame id
   * @param [in] child_frame child frame id
   * @return transform from parent frame to child frame
   */
  TransformStamped get_transform_msg(const std::string parent_frame, const std::string child_frame);

  /**
   * @brief calculate ego pitch angle from trajectory
   * @return ego pitch angle
   */
  double calculate_ego_pitch() const;

  /**
   * @brief timer callback for simulation with loop_rate
   */
  void on_timer();

  /**
   * @brief initialize vehicle_model_ptr
   */
  void initialize_vehicle_model(const std::string & vehicle_model_type_str);

  /**
   * @brief add measurement noise
   * @param [in] odometry odometry to add noise
   * @param [in] vel velocity report to add noise
   * @param [in] steer steering to add noise
   */
  void add_measurement_noise(Odometry & odom, VelocityReport & vel, SteeringReport & steer) const;

  /**
   * @brief set initial state of simulated vehicle
   * @param [in] pose initial position and orientation
   * @param [in] twist initial velocity and angular velocity
   */
  void set_initial_state(const Pose & pose, const Twist & twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void set_initial_state_with_transform(const PoseStamped & pose, const Twist & twist);

  /**
   * @brief publish velocity
   * @param [in] velocity The velocity report to publish
   */
  void publish_velocity(const VelocityReport & velocity);

  /**
   * @brief publish pose and twist
   * @param [in] odometry The odometry to publish
   */
  void publish_odometry(const Odometry & odometry);

  /**
   * @brief publish pose
   * @param [in] odometry The odometry to publish its pose
   */
  void publish_pose(const Odometry & odometry);

  /**
   * @brief publish steering
   * @param [in] steer The steering to publish
   */
  void publish_steering(const SteeringReport & steer);

  /**
   * @brief publish acceleration
   */
  void publish_acceleration();

  /**
   * @brief publish imu
   */
  void publish_imu();

  /**
   * @brief publish control_mode report
   */
  void publish_control_mode_report();

  /**
   * @brief publish gear report
   */
  void publish_gear_report();

  /**
   * @brief publish turn indicators report
   */
  void publish_turn_indicators_report();

  /**
   * @brief publish hazard lights report
   */
  void publish_hazard_lights_report();

  void publish_actuation_status();

  /**
   * @brief publish tf
   * @param [in] state The kinematic state to publish as a TF
   */
  void publish_tf(const Odometry & odometry);
};
}  // namespace simple_planning_simulator
}  // namespace simulation

#endif  // SIMPLE_PLANNING_SIMULATOR__SIMPLE_PLANNING_SIMULATOR_CORE_HPP_
