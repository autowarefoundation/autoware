//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#include "accel_brake_map_calibrator/accel_brake_map_calibrator_node.hpp"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

AccelBrakeMapCalibrator::AccelBrakeMapCalibrator(const rclcpp::NodeOptions & node_options)
: Node("accel_brake_map_calibrator", node_options)
{
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);
  // get parameter
  update_hz_ = this->declare_parameter<double>("update_hz", 10.0);
  covariance_ = this->declare_parameter<double>("initial_covariance", 0.05);
  velocity_min_threshold_ = this->declare_parameter<double>("velocity_min_threshold", 0.1);
  velocity_diff_threshold_ = this->declare_parameter<double>("velocity_diff_threshold", 0.556);
  pedal_diff_threshold_ = this->declare_parameter<double>("pedal_diff_threshold", 0.03);
  max_steer_threshold_ = this->declare_parameter<double>("max_steer_threshold", 0.2);
  max_pitch_threshold_ = this->declare_parameter<double>("max_pitch_threshold", 0.02);
  max_jerk_threshold_ = this->declare_parameter<double>("max_jerk_threshold", 0.7);
  pedal_velocity_thresh_ = this->declare_parameter<double>("pedal_velocity_thresh", 0.15);
  max_accel_ = this->declare_parameter<double>("max_accel", 5.0);
  min_accel_ = this->declare_parameter<double>("min_accel", -5.0);
  pedal_to_accel_delay_ = this->declare_parameter<double>("pedal_to_accel_delay", 0.3);
  max_data_count_ = this->declare_parameter<int>("max_data_count", 200);
  pedal_accel_graph_output_ = this->declare_parameter<bool>("pedal_accel_graph_output", false);
  progress_file_output_ = this->declare_parameter<bool>("progress_file_output", false);
  precision_ = this->declare_parameter<int>("precision", 3);
  const auto get_pitch_method_str =
    this->declare_parameter<std::string>("get_pitch_method", std::string("tf"));
  if (get_pitch_method_str == std::string("tf")) {
    get_pitch_method_ = GET_PITCH_METHOD::TF;
  } else if (get_pitch_method_str == std::string("none")) {
    get_pitch_method_ = GET_PITCH_METHOD::NONE;
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("accel_brake_map_calibrator"),
      "update_method_"
        << " is wrong. (available method: tf, file, none");
    return;
  }
  update_suggest_thresh_ = this->declare_parameter<double>("update_suggest_thresh", 0.7);
  csv_calibrated_map_dir_ =
    this->declare_parameter<std::string>("csv_calibrated_map_dir", std::string(""));
  output_accel_file_ = csv_calibrated_map_dir_ + "/accel_map.csv";
  output_brake_file_ = csv_calibrated_map_dir_ + "/brake_map.csv";
  const std::string update_method_str =
    this->declare_parameter<std::string>("update_method", std::string("update_offset_each_cell"));
  if (update_method_str == std::string("update_offset_each_cell")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_EACH_CELL;
  } else if (update_method_str == std::string("update_offset_total")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_TOTAL;
  } else if (update_method_str == std::string("update_offset_four_cell_around")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND;
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("accel_brake_map_calibrator"),
      "update_method"
        << " is wrong. (available method: update_offset_each_cell, update_offset_total");
    return;
  }
  // initializer

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);

  // Publisher for checkUpdateSuggest
  calibration_status_pub_ = create_publisher<tier4_external_api_msgs::msg::CalibrationStatus>(
    "/accel_brake_map_calibrator/output/calibration_status", durable_qos);

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / update_hz_);
  updater_ptr_->setHardwareID("accel_brake_map_calibrator");
  updater_ptr_->add(
    "accel_brake_map_calibrator", this, &AccelBrakeMapCalibrator::checkUpdateSuggest);

  {
    csv_default_map_dir_ =
      this->declare_parameter<std::string>("csv_default_map_dir", std::string(""));

    std::string csv_path_accel_map = csv_default_map_dir_ + "/accel_map.csv";
    std::string csv_path_brake_map = csv_default_map_dir_ + "/brake_map.csv";
    if (
      !accel_map_.readAccelMapFromCSV(csv_path_accel_map) ||
      !new_accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
      is_default_map_ = false;
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("accel_brake_map_calibrator"),
        "Cannot read accelmap. csv path = " << csv_path_accel_map.c_str() << ". stop calculation.");
      return;
    }
    if (
      !brake_map_.readBrakeMapFromCSV(csv_path_brake_map) ||
      !new_brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
      is_default_map_ = false;
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("accel_brake_map_calibrator"),
        "Cannot read brakemap. csv path = " << csv_path_brake_map.c_str() << ". stop calculation.");
      return;
    }
  }

  std::string output_log_file =
    this->declare_parameter<std::string>("output_log_file", std::string(""));
  output_log_.open(output_log_file);
  addIndexToCSV(&output_log_);

  debug_values_.data.resize(num_debug_values_);

  // input map info
  accel_map_value_ = accel_map_.getAccelMap();
  brake_map_value_ = brake_map_.getBrakeMap();
  accel_vel_index_ = accel_map_.getVelIdx();
  brake_vel_index_ = brake_map_.getVelIdx();
  accel_pedal_index_ = accel_map_.getThrottleIdx();
  brake_pedal_index_ = brake_map_.getBrakeIdx();
  update_accel_map_value_.resize((accel_map_value_.size()));
  update_brake_map_value_.resize((brake_map_value_.size()));
  for (auto & m : update_accel_map_value_) {
    m.resize(accel_map_value_.at(0).size());
  }
  for (auto & m : update_brake_map_value_) {
    m.resize(brake_map_value_.at(0).size());
  }
  map_value_data_.resize(accel_map_value_.size() + brake_map_value_.size() - 1);
  for (auto & m : map_value_data_) {
    m.resize(accel_map_value_.at(0).size());
  }

  std::copy(accel_map_value_.begin(), accel_map_value_.end(), update_accel_map_value_.begin());
  std::copy(brake_map_value_.begin(), brake_map_value_.end(), update_brake_map_value_.begin());

  // publisher
  update_suggest_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/output/update_suggest", durable_qos);
  original_map_occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/accel_brake_map_calibrator/debug/original_occ_map", durable_qos);
  update_map_occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/accel_brake_map_calibrator/debug/update_occ_map", durable_qos);
  data_ave_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/accel_brake_map_calibrator/debug/data_average_occ_map", durable_qos);
  data_std_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/accel_brake_map_calibrator/debug/data_std_dev_occ_map", durable_qos);
  data_count_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/accel_brake_map_calibrator/debug/data_count_occ_map", durable_qos);
  data_count_with_self_pose_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/accel_brake_map_calibrator/debug/data_count_self_pose_occ_map", durable_qos);
  index_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/accel_brake_map_calibrator/debug/occ_index", durable_qos);
  original_map_raw_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/accel_brake_map_calibrator/debug/original_raw_map", durable_qos);
  update_map_raw_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/accel_brake_map_calibrator/output/update_raw_map", durable_qos);
  debug_pub_ = create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
    "/accel_brake_map_calibrator/output/debug_values", durable_qos);
  current_map_error_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/accel_brake_map_calibrator/output/current_map_error", durable_qos);
  updated_map_error_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/accel_brake_map_calibrator/output/updated_map_error", durable_qos);
  map_error_ratio_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/accel_brake_map_calibrator/output/map_error_ratio", durable_qos);

  // subscriber
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  velocity_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "~/input/velocity", queue_size,
    std::bind(&AccelBrakeMapCalibrator::callbackVelocity, this, _1));
  steer_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "~/input/steer", queue_size, std::bind(&AccelBrakeMapCalibrator::callbackSteer, this, _1));
  actuation_status_sub_ = create_subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "~/input/actuation_status", queue_size,
    std::bind(&AccelBrakeMapCalibrator::callbackActuationStatus, this, _1));

  // Service
  update_map_dir_server_ = create_service<tier4_vehicle_msgs::srv::UpdateAccelBrakeMap>(
    "~/input/update_map_dir",
    std::bind(&AccelBrakeMapCalibrator::callbackUpdateMapService, this, _1, _2, _3));

  // timer
  initTimer(1.0 / update_hz_);
  initOutputCSVTimer(30.0);
}

void AccelBrakeMapCalibrator::initOutputCSVTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_output_csv_ = rclcpp::create_timer(
    this, get_clock(), period_ns,
    std::bind(&AccelBrakeMapCalibrator::timerCallbackOutputCSV, this));
}

void AccelBrakeMapCalibrator::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&AccelBrakeMapCalibrator::timerCallback, this));
}

bool AccelBrakeMapCalibrator::getCurrentPitchFromTF(double * pitch)
{
  if (get_pitch_method_ == GET_PITCH_METHOD::NONE) {
    // do not get pitch from tf
    *pitch = 0.0;
    return true;
  }

  // get tf
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform;
  try {
    transform = transform_listener_->getTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    auto & clk = *this->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000,
      "cannot get map to base_link transform. " << ex.what());
    return false;
  }
  double roll, raw_pitch, yaw;
  tf2::getEulerYPR(transform->transform.rotation, roll, raw_pitch, yaw);
  debug_values_.data.at(CURRENT_RAW_PITCH) = raw_pitch;
  *pitch = lowpass(*pitch, raw_pitch, 0.2);
  debug_values_.data.at(CURRENT_PITCH) = *pitch;
  return true;
}

void AccelBrakeMapCalibrator::timerCallback()
{
  update_count_++;
  auto & clk = *this->get_clock();

  RCLCPP_DEBUG_STREAM_THROTTLE(
    rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000,
    "map updating... count: " << update_success_count_ << " / " << update_count_ << "\n\t"
                              << "lack_of_data_count: " << lack_of_data_count_ << "\n\t"
                              << " failed_to_get_pitch_count: " << failed_to_get_pitch_count_
                              << "\n\t"
                              << "too_large_pitch_count: " << too_large_pitch_count_ << "\n\t"
                              << " too_low_speed_count: " << too_low_speed_count_ << "\n\t"
                              << "too_large_steer_count: " << too_large_steer_count_ << "\n\t"
                              << "too_large_jerk_count: " << too_large_jerk_count_ << "\n\t"
                              << "invalid_acc_brake_count: " << invalid_acc_brake_count_ << "\n\t"
                              << "too_large_pedal_spd_count: " << too_large_pedal_spd_count_
                              << "\n\t"
                              << "update_fail_count_: " << update_fail_count_ << "\n");

  /* valid check */

  // data check
  if (
    !twist_ptr_ || !steer_ptr_ || !accel_pedal_ptr_ || !brake_pedal_ptr_ ||
    !delayed_accel_pedal_ptr_ || !delayed_brake_pedal_ptr_) {
    // lack of data
    auto & clk = *this->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000,
      "lack of topics (twist, steer, accel, brake)");
    lack_of_data_count_++;
    return;
  }

  // data check 2
  if (
    isTimeout(twist_ptr_->header.stamp, timeout_sec_) ||
    isTimeout(steer_ptr_->stamp, timeout_sec_) || isTimeout(accel_pedal_ptr_, timeout_sec_) ||
    isTimeout(brake_pedal_ptr_, timeout_sec_)) {
    auto & clk = *this->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000,
      "timeout of topics (twist, steer, accel, brake)");
    lack_of_data_count_++;
    return;
  }

  // data check 3
  if (!getCurrentPitchFromTF(&pitch_)) {
    // cannot get pitch
    auto & clk = *this->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000, "cannot get pitch");
    failed_to_get_pitch_count_++;
    return;
  }

  /* write data to log */
  if (pedal_accel_graph_output_) {
    addLogToCSV(
      &output_log_, rclcpp::Time(twist_ptr_->header.stamp).seconds(), twist_ptr_->twist.linear.x,
      acceleration_, getPitchCompensatedAcceleration(), delayed_accel_pedal_ptr_->data,
      delayed_brake_pedal_ptr_->data, accel_pedal_speed_, brake_pedal_speed_, pitch_,
      steer_ptr_->steering_tire_angle, jerk_, full_original_accel_rmse_, part_original_accel_rmse_,
      new_accel_rmse_);
  }

  /* publish map  & debug_values*/
  publishMap(accel_map_value_, brake_map_value_, "original");
  publishMap(update_accel_map_value_, update_brake_map_value_, "update");
  publishCountMap();
  publishIndex();
  publishUpdateSuggestFlag();
  debug_pub_->publish(debug_values_);
  publishFloat32("current_map_error", part_original_accel_rmse_);
  publishFloat32("updated_map_error", new_accel_rmse_);
  publishFloat32(
    "map_error_ratio",
    part_original_accel_rmse_ != 0.0 ? new_accel_rmse_ / part_original_accel_rmse_ : 1.0);

  // -- processing start --

  /* initialize */
  debug_values_.data.at(SUCCESS_TO_UPDATE) = false;
  update_success_ = false;

  // twist check
  if (twist_ptr_->twist.linear.x < velocity_min_threshold_) {
    // too low speed ( or backward velocity)
    too_low_speed_count_++;
    return;
  }

  // accel / brake map evaluation (do not evaluate when the car stops)
  executeEvaluation();

  // pitch check
  if (std::fabs(pitch_) > max_pitch_threshold_) {
    // too large pitch
    too_large_pitch_count_++;
    return;
  }

  // steer check
  if (std::fabs(steer_ptr_->steering_tire_angle) > max_steer_threshold_) {
    // too large steer
    too_large_steer_count_++;
    return;
  }

  // jerk check
  if (std::fabs(jerk_) > max_jerk_threshold_) {
    // too large jerk
    too_large_jerk_count_++;
    return;
  }

  // pedal check
  if (
    delayed_accel_pedal_ptr_->data > std::numeric_limits<double>::epsilon() &&
    delayed_brake_pedal_ptr_->data > std::numeric_limits<double>::epsilon()) {
    // both (accel/brake) output
    auto & clk = *this->get_clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000,
      "invalid pedal value (Both of accel output and brake output area not zero. )");
    invalid_acc_brake_count_++;
    return;
  }

  // pedal speed check
  if (
    std::fabs(accel_pedal_speed_) > pedal_velocity_thresh_ ||
    std::fabs(brake_pedal_speed_) > pedal_velocity_thresh_) {
    // too large pedal speed
    too_large_pedal_spd_count_++;
    return;
  }

  /* update map */
  if (updateAccelBrakeMap()) {
    update_success_count_++;
    debug_values_.data.at(SUCCESS_TO_UPDATE) = true;
    update_success_ = true;
  } else {
    update_fail_count_++;
  }
}

void AccelBrakeMapCalibrator::timerCallbackOutputCSV()
{
  // write accel/ brake map to file
  const auto ros_time = std::to_string(this->now().seconds());
  writeMapToCSV(accel_vel_index_, accel_pedal_index_, update_accel_map_value_, output_accel_file_);
  writeMapToCSV(brake_vel_index_, brake_pedal_index_, update_brake_map_value_, output_brake_file_);
  if (progress_file_output_) {
    writeMapToCSV(
      accel_vel_index_, accel_pedal_index_, update_accel_map_value_,
      output_accel_file_ + "_" + ros_time);
    writeMapToCSV(
      brake_vel_index_, brake_pedal_index_, update_brake_map_value_,
      output_brake_file_ + "_" + ros_time);
    writeMapToCSV(
      accel_vel_index_, accel_pedal_index_, accel_map_value_, output_accel_file_ + "_original");
    writeMapToCSV(
      brake_vel_index_, brake_pedal_index_, brake_map_value_, output_brake_file_ + "_original");
  }

  // update newest accel / brake map
  // check file existence
  std::ifstream af(output_accel_file_);
  std::ifstream bf(output_brake_file_);
  if (!af.is_open() || !bf.is_open()) {
    RCLCPP_WARN(
      rclcpp::get_logger("accel_brake_map_calibrator"), "Accel map or brake map does not exist");
    return;
  }

  new_accel_map_ = AccelMap();
  if (!new_accel_map_.readAccelMapFromCSV(output_accel_file_)) {
    RCLCPP_WARN(
      rclcpp::get_logger("accel_brake_map_calibrator"), "Cannot read accelmap. csv path = %s. ",
      output_accel_file_.c_str());
  }
  new_brake_map_ = BrakeMap();
  if (!new_brake_map_.readBrakeMapFromCSV(output_brake_file_)) {
    RCLCPP_WARN(
      rclcpp::get_logger("accel_brake_map_calibrator"), "Cannot read brakemap. csv path = %s. ",
      output_brake_file_.c_str());
  }
}

void AccelBrakeMapCalibrator::callbackVelocity(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
{
  // convert velocity-report to twist-stamped
  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header = msg->header;
  twist_msg->twist.linear.x = msg->longitudinal_velocity;
  twist_msg->twist.linear.y = msg->lateral_velocity;
  twist_msg->twist.angular.z = msg->heading_rate;

  if (!twist_vec_.empty()) {
    const auto past_msg = getNearestTimeDataFromVec(twist_msg, dif_twist_time_, twist_vec_);
    const double raw_acceleration = getAccel(past_msg, twist_msg);
    acceleration_ = lowpass(acceleration_, raw_acceleration, 0.25);
    acceleration_time_ = rclcpp::Time(msg->header.stamp).seconds();
    debug_values_.data.at(CURRENT_RAW_ACCEL) = raw_acceleration;
    debug_values_.data.at(CURRENT_ACCEL) = acceleration_;

    // calculate jerk
    if (
      this->now().seconds() - pre_acceleration_time_ > timeout_sec_ ||
      (acceleration_time_ - pre_acceleration_time_) <= std::numeric_limits<double>::epsilon()) {
      auto & clk = *this->get_clock();
      RCLCPP_DEBUG_STREAM_THROTTLE(
        rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000, "cannot calculate jerk");
      // does not update jerk
    } else {
      const double raw_jerk = getJerk();
      // to avoid to get delayed jerk, set high gain
      jerk_ = lowpass(jerk_, raw_jerk, 0.5);
    }
    debug_values_.data.at(CURRENT_JERK) = jerk_;
    pre_acceleration_ = acceleration_;
    pre_acceleration_time_ = acceleration_time_;
  }

  debug_values_.data.at(CURRENT_SPEED) = twist_msg->twist.linear.x;
  twist_ptr_ = twist_msg;
  pushDataToVec(twist_msg, twist_vec_max_size_, &twist_vec_);
}

void AccelBrakeMapCalibrator::callbackSteer(
  const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg)
{
  debug_values_.data.at(CURRENT_STEER) = msg->steering_tire_angle;
  steer_ptr_ = msg;
}

void AccelBrakeMapCalibrator::callbackActuationStatus(
  const tier4_vehicle_msgs::msg::ActuationStatusStamped::ConstSharedPtr msg)
{
  // get accel data
  accel_pedal_ptr_ =
    std::make_shared<DataStamped>(msg->status.accel_status, rclcpp::Time(msg->header.stamp));
  if (!accel_pedal_vec_.empty()) {
    const auto past_accel_ptr =
      getNearestTimeDataFromVec(accel_pedal_ptr_, dif_pedal_time_, accel_pedal_vec_);
    const double raw_accel_pedal_speed =
      getPedalSpeed(past_accel_ptr, accel_pedal_ptr_, accel_pedal_speed_);
    accel_pedal_speed_ = lowpass(accel_pedal_speed_, raw_accel_pedal_speed, 0.5);
    debug_values_.data.at(CURRENT_RAW_ACCEL_SPEED) = raw_accel_pedal_speed;
    debug_values_.data.at(CURRENT_ACCEL_SPEED) = accel_pedal_speed_;
  }
  debug_values_.data.at(CURRENT_ACCEL_PEDAL) = accel_pedal_ptr_->data;
  pushDataToVec(accel_pedal_ptr_, pedal_vec_max_size_, &accel_pedal_vec_);
  delayed_accel_pedal_ptr_ =
    getNearestTimeDataFromVec(accel_pedal_ptr_, pedal_to_accel_delay_, accel_pedal_vec_);

  // get brake data
  brake_pedal_ptr_ =
    std::make_shared<DataStamped>(msg->status.brake_status, rclcpp::Time(msg->header.stamp));
  if (!brake_pedal_vec_.empty()) {
    const auto past_brake_ptr =
      getNearestTimeDataFromVec(brake_pedal_ptr_, dif_pedal_time_, brake_pedal_vec_);
    const double raw_brake_pedal_speed =
      getPedalSpeed(past_brake_ptr, brake_pedal_ptr_, brake_pedal_speed_);
    brake_pedal_speed_ = lowpass(brake_pedal_speed_, raw_brake_pedal_speed, 0.5);
    debug_values_.data.at(CURRENT_RAW_BRAKE_SPEED) = raw_brake_pedal_speed;
    debug_values_.data.at(CURRENT_BRAKE_SPEED) = brake_pedal_speed_;
  }
  debug_values_.data.at(CURRENT_BRAKE_PEDAL) = brake_pedal_ptr_->data;
  pushDataToVec(brake_pedal_ptr_, pedal_vec_max_size_, &brake_pedal_vec_);
  delayed_brake_pedal_ptr_ =
    getNearestTimeDataFromVec(brake_pedal_ptr_, pedal_to_accel_delay_, brake_pedal_vec_);
}

bool AccelBrakeMapCalibrator::callbackUpdateMapService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
  tier4_vehicle_msgs::srv::UpdateAccelBrakeMap::Request::SharedPtr req,
  tier4_vehicle_msgs::srv::UpdateAccelBrakeMap::Response::SharedPtr res)
{
  // if req.path is input, use this as the directory to set updated maps
  std::string update_map_dir = req->path.empty() ? csv_default_map_dir_ : req->path;

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("accel_brake_map_calibrator"),
    "update accel/brake map. directory: " << update_map_dir);
  const auto accel_map_file = update_map_dir + "/accel_map.csv";
  const auto brake_map_file = update_map_dir + "/brake_map.csv";
  if (
    writeMapToCSV(accel_vel_index_, accel_pedal_index_, update_accel_map_value_, accel_map_file) &&
    writeMapToCSV(brake_vel_index_, brake_pedal_index_, update_brake_map_value_, brake_map_file)) {
    res->success = true;
    res->message =
      "Data has been successfully saved on " + update_map_dir + "/(accel/brake)_map.csv";
  } else {
    res->success = false;
    res->message = "Failed to save data. Maybe invalid path?";
  }
  return true;
}

double AccelBrakeMapCalibrator::lowpass(
  const double original, const double current, const double gain)
{
  return current * gain + original * (1.0 - gain);
}

double AccelBrakeMapCalibrator::getPedalSpeed(
  const DataStampedPtr & prev_pedal, const DataStampedPtr & current_pedal,
  const double prev_pedal_speed)
{
  const double dt = (current_pedal->data_time - prev_pedal->data_time).seconds();
  if (dt < 1e-03) {
    // invalid pedal info. return prev pedal speed
    return prev_pedal_speed;
  }

  const double d_pedal = current_pedal->data - prev_pedal->data;
  return d_pedal / dt;
}

double AccelBrakeMapCalibrator::getAccel(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr & prev_twist,
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr & current_twist)
{
  const double dt =
    (rclcpp::Time(current_twist->header.stamp) - rclcpp::Time(prev_twist->header.stamp)).seconds();
  if (dt < 1e-03) {
    // invalid twist. return prev acceleration
    return acceleration_;
  }
  const double dv = current_twist->twist.linear.x - prev_twist->twist.linear.x;
  return std::min(std::max(min_accel_, dv / dt), max_accel_);
}

double AccelBrakeMapCalibrator::getJerk()
{
  const double jerk =
    (acceleration_ - pre_acceleration_) / (acceleration_time_ - pre_acceleration_time_);
  return std::min(std::max(-max_jerk_, jerk), max_jerk_);
}

bool AccelBrakeMapCalibrator::indexValueSearch(
  const std::vector<double> value_index, const double value, const double value_thresh,
  int * searched_index)
{
  if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
    /*  return lower left index.
    +-------+   +:grid
    |    *  |   *:given data
    |       |   o:index to return
    o-------+
    */
    if (value < 0) {
      std::cerr << "error : invalid value." << std::endl;
      return false;
    }
    double old_diff_value = -999;  // TODO(Hirano)
    for (std::size_t i = 0; i < value_index.size(); i++) {
      const double diff_value = value_index.at(i) - value;
      if (diff_value <= 0 && old_diff_value < diff_value) {
        *searched_index = i;
        old_diff_value = diff_value;
      } else {
        return true;
      }
    }
  } else {
    for (std::size_t i = 0; i < value_index.size(); i++) {
      const double diff_value = std::fabs(value_index.at(i) - value);
      if (diff_value <= value_thresh) {
        *searched_index = i;
        return true;
      }
    }
  }
  return false;
}

int AccelBrakeMapCalibrator::nearestValueSearch(
  const std::vector<double> value_index, const double value)
{
  double max_dist = std::numeric_limits<double>::max();
  int nearest_idx = 0;

  for (std::size_t i = 0; i < value_index.size(); i++) {
    const double dist = std::fabs(value - value_index.at(i));
    if (max_dist > dist) {
      nearest_idx = i;
      max_dist = dist;
    }
  }
  return nearest_idx;
}

int AccelBrakeMapCalibrator::nearestPedalSearch()
{
  bool accel_mode;

  if (delayed_accel_pedal_ptr_->data > std::numeric_limits<double>::epsilon()) {
    accel_mode = true;
  } else {
    accel_mode = false;
  }

  int nearest_idx;
  if (accel_mode) {
    nearest_idx = nearestValueSearch(accel_pedal_index_, delayed_accel_pedal_ptr_->data);
  } else {
    nearest_idx = nearestValueSearch(brake_pedal_index_, delayed_brake_pedal_ptr_->data);
  }

  return getUnifiedIndexFromAccelBrakeIndex(accel_mode, nearest_idx);
}

int AccelBrakeMapCalibrator::nearestVelSearch()
{
  const double current_vel = twist_ptr_->twist.linear.x;
  return nearestValueSearch(accel_vel_index_, current_vel);
}

void AccelBrakeMapCalibrator::takeConsistencyOfAccelMap()
{
  const double bit = std::pow(1e-01, precision_);
  for (std::size_t ped_idx = 0; ped_idx < update_accel_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_accel_map_value_.at(0).size() - 1; vel_idx++) {
      const double current_acc = update_accel_map_value_.at(ped_idx).at(vel_idx);
      const double next_ped_acc = update_accel_map_value_.at(ped_idx + 1).at(vel_idx);
      const double next_vel_acc = update_accel_map_value_.at(ped_idx).at(vel_idx + 1);

      if (current_acc <= next_vel_acc) {
        // the higher the velocity, the lower the acceleration
        update_accel_map_value_.at(ped_idx).at(vel_idx + 1) = current_acc - bit;
      }

      if (current_acc >= next_ped_acc) {
        // the higher the accel pedal, the higher the acceleration
        update_accel_map_value_.at(ped_idx + 1).at(vel_idx) = current_acc + bit;
      }
    }
  }
}

void AccelBrakeMapCalibrator::takeConsistencyOfBrakeMap()
{
  const double bit = std::pow(1e-01, precision_);
  for (std::size_t ped_idx = 0; ped_idx < update_brake_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_brake_map_value_.at(0).size() - 1; vel_idx++) {
      const double current_acc = update_brake_map_value_.at(ped_idx).at(vel_idx);
      const double next_ped_acc = update_brake_map_value_.at(ped_idx + 1).at(vel_idx);
      const double next_vel_acc = update_brake_map_value_.at(ped_idx).at(vel_idx + 1);

      if (current_acc <= next_vel_acc) {
        // the higher the velocity, the lower the acceleration
        update_brake_map_value_.at(ped_idx).at(vel_idx + 1) = current_acc - bit;
      }

      if (current_acc <= next_ped_acc) {
        // the higher the brake pedal, the lower the acceleration
        update_brake_map_value_.at(ped_idx + 1).at(vel_idx) = current_acc - bit;
      }
    }
  }
}

bool AccelBrakeMapCalibrator::updateAccelBrakeMap()
{
  // get pedal index
  bool accel_mode = false;
  int accel_pedal_index = 0;
  int brake_pedal_index = 0;
  int accel_vel_index = 0;
  int brake_vel_index = 0;

  if (delayed_accel_pedal_ptr_->data > std::numeric_limits<double>::epsilon()) {
    accel_mode = true;
  } else {
    accel_mode = false;
  }

  if (
    accel_mode && !indexValueSearch(
                    accel_pedal_index_, delayed_accel_pedal_ptr_->data, pedal_diff_threshold_,
                    &accel_pedal_index)) {
    // not match accel pedal output to pedal value in index
    return false;
  }

  if (
    !accel_mode && !indexValueSearch(
                     brake_pedal_index_, delayed_brake_pedal_ptr_->data, pedal_diff_threshold_,
                     &brake_pedal_index)) {
    // not match accel pedal output to pedal value in index
    return false;
  }

  if (
    accel_mode &&
    !indexValueSearch(
      accel_vel_index_, twist_ptr_->twist.linear.x, velocity_diff_threshold_, &accel_vel_index)) {
    // not match current velocity to velocity value in index
    return false;
  }

  if (
    !accel_mode &&
    !indexValueSearch(
      brake_vel_index_, twist_ptr_->twist.linear.x, velocity_diff_threshold_, &brake_vel_index)) {
    // not match current velocity to velocity value in index
    return false;
  }

  // update map
  executeUpdate(accel_mode, accel_pedal_index, accel_vel_index, brake_pedal_index, brake_vel_index);

  // when update 0 pedal index, update another map
  if (accel_mode && accel_pedal_index == 0) {
    // copy accel map value to brake map value
    update_brake_map_value_.at(accel_pedal_index).at(accel_vel_index) =
      update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index);
    if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
      update_brake_map_value_.at(accel_pedal_index).at(accel_vel_index + 1) =
        update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index + 1);
    }
  } else if (!accel_mode && brake_pedal_index == 0) {
    // copy brake map value to accel map value
    update_accel_map_value_.at(brake_pedal_index).at(brake_vel_index) =
      update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index);
    if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
      update_accel_map_value_.at(brake_pedal_index).at(brake_vel_index + 1) =
        update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index + 1);
    }
  }

  // take consistency of map
  takeConsistencyOfAccelMap();
  takeConsistencyOfBrakeMap();

  return true;
}

void AccelBrakeMapCalibrator::executeUpdate(
  const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
  const int brake_pedal_index, const int brake_vel_index)
{
  const double measured_acc = acceleration_ - getPitchCompensatedAcceleration();
  const double map_acc = accel_mode
                           ? update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index)
                           : update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index);
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("accel_brake_map_calibrator"),
    "measured_acc: " << measured_acc << ", map_acc: " << map_acc);

  if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_EACH_CELL) {
    updateEachValOffset(
      accel_mode, accel_pedal_index, accel_vel_index, brake_pedal_index, brake_vel_index,
      measured_acc, map_acc);
  } else if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_TOTAL) {
    updateTotalMapOffset(measured_acc, map_acc);
  } else if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
    updateFourCellAroundOffset(
      accel_mode, accel_pedal_index, accel_vel_index, brake_pedal_index, brake_vel_index,
      measured_acc);
  }

  // add accel data to map
  accel_mode ? map_value_data_.at(getUnifiedIndexFromAccelBrakeIndex(true, accel_pedal_index))
                 .at(accel_vel_index)
                 .emplace_back(measured_acc)
             : map_value_data_.at(getUnifiedIndexFromAccelBrakeIndex(false, brake_pedal_index))
                 .at(brake_vel_index)
                 .emplace_back(measured_acc);
}

bool AccelBrakeMapCalibrator::updateFourCellAroundOffset(
  const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
  const int brake_pedal_index, const int brake_vel_index, const double measured_acc)
{
  // pre-defined
  static std::vector<std::vector<double>> accel_map_offset_vec_(
    accel_map_value_.size(), std::vector<double>(accel_map_value_.at(0).size(), map_offset_));
  static std::vector<std::vector<double>> brake_map_offset_vec_(
    brake_map_value_.size(), std::vector<double>(accel_map_value_.at(0).size(), map_offset_));
  static std::vector<std::vector<Eigen::MatrixXd>> accel_covariance_mat_(
    accel_map_value_.size() - 1,
    std::vector<Eigen::MatrixXd>(
      accel_map_value_.at(0).size() - 1, Eigen::MatrixXd::Identity(4, 4) * covariance_));
  static std::vector<std::vector<Eigen::MatrixXd>> brake_covariance_mat_(
    brake_map_value_.size() - 1,
    std::vector<Eigen::MatrixXd>(
      accel_map_value_.at(0).size() - 1, Eigen::MatrixXd::Identity(4, 4) * covariance_));

  auto & update_map_value = accel_mode ? update_accel_map_value_ : update_brake_map_value_;
  const auto & map_value = accel_mode ? accel_map_value_ : brake_map_value_;
  const auto & pedal_index = accel_mode ? accel_pedal_index : brake_pedal_index;
  const auto & pedal_index_ = accel_mode ? accel_pedal_index_ : brake_pedal_index_;
  const auto & vel_index = accel_mode ? accel_vel_index : brake_vel_index;
  const auto & vel_index_ = accel_mode ? accel_vel_index_ : brake_vel_index_;
  const auto & delayed_pedal =
    accel_mode ? delayed_accel_pedal_ptr_->data : delayed_brake_pedal_ptr_->data;
  auto & map_offset_vec = accel_mode ? accel_map_offset_vec_ : brake_map_offset_vec_;
  auto & covariance_mat = accel_mode ? accel_covariance_mat_ : brake_covariance_mat_;

  const double zll = update_map_value.at(pedal_index + 0).at(vel_index + 0);
  const double zhl = update_map_value.at(pedal_index + 1).at(vel_index + 0);
  const double zlh = update_map_value.at(pedal_index + 0).at(vel_index + 1);
  const double zhh = update_map_value.at(pedal_index + 1).at(vel_index + 1);

  const double xl = pedal_index_.at(pedal_index + 0);
  const double xh = pedal_index_.at(pedal_index + 1);
  const double rx = (delayed_pedal - xl) / (xh - xl);
  const double yl = vel_index_.at(vel_index + 0);
  const double yh = vel_index_.at(vel_index + 1);
  const double ry = (twist_ptr_->twist.linear.x - yl) / (yh - yl);

  Eigen::Vector4d phi(4);
  phi << (1 - rx) * (1 - ry), rx * (1 - ry), (1 - rx) * ry, rx * ry;

  Eigen::Vector4d theta(4);
  theta << zll, zhl, zlh, zhh;

  const int vel_idx_l = vel_index + 0;
  const int vel_idx_h = vel_index + 1;
  const int ped_idx_l = pedal_index + 0;
  const int ped_idx_h = pedal_index + 1;

  Eigen::VectorXd map_offset(4);
  map_offset(0) = map_offset_vec.at(ped_idx_l).at(vel_idx_l);
  map_offset(1) = map_offset_vec.at(ped_idx_h).at(vel_idx_l);
  map_offset(2) = map_offset_vec.at(ped_idx_l).at(vel_idx_h);
  map_offset(3) = map_offset_vec.at(ped_idx_h).at(vel_idx_h);

  Eigen::MatrixXd covariance = covariance_mat.at(ped_idx_l).at(vel_idx_l);

  /* calculate adaptive map offset */
  Eigen::MatrixXd G(4, 4);
  Eigen::RowVectorXd phiT(4);
  phiT = phi.transpose();
  double rk = phiT * covariance * phi;

  G = covariance * phi / (forgetting_factor_ + rk);
  double beta = rk > 0 ? (forgetting_factor_ - (1 - forgetting_factor_) / rk) : 1;
  covariance = covariance - covariance * phi * phiT * covariance / (1 / beta + rk);  // anti-windup
  double eta_hat = phiT * theta;

  const double error_map_offset = measured_acc - eta_hat;
  map_offset = map_offset + G * error_map_offset;

  /* input calculated result and update map */
  map_offset_vec.at(ped_idx_l).at(vel_idx_l) = map_offset(0);
  map_offset_vec.at(ped_idx_h).at(vel_idx_l) = map_offset(1);
  map_offset_vec.at(ped_idx_l).at(vel_idx_h) = map_offset(2);
  map_offset_vec.at(ped_idx_h).at(vel_idx_h) = map_offset(3);

  covariance_mat.at(ped_idx_l).at(vel_idx_l) = covariance;

  update_map_value.at(pedal_index + 0).at(vel_index + 0) =
    map_value.at(pedal_index + 0).at(vel_index + 0) + map_offset(0);
  update_map_value.at(pedal_index + 1).at(vel_index + 0) =
    map_value.at(pedal_index + 1).at(vel_index + 0) + map_offset(1);
  update_map_value.at(pedal_index + 0).at(vel_index + 1) =
    map_value.at(pedal_index + 0).at(vel_index + 1) + map_offset(2);
  update_map_value.at(pedal_index + 1).at(vel_index + 1) =
    map_value.at(pedal_index + 1).at(vel_index + 1) + map_offset(3);

  return true;
}

bool AccelBrakeMapCalibrator::updateEachValOffset(
  const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
  const int brake_pedal_index, const int brake_vel_index, const double measured_acc,
  const double map_acc)
{
  // pre-defined
  static std::vector<std::vector<double>> map_offset_vec_(
    accel_map_value_.size() + brake_map_value_.size() - 1,
    std::vector<double>(accel_map_value_.at(0).size(), map_offset_));
  static std::vector<std::vector<double>> covariance_vec_(
    accel_map_value_.size() + brake_map_value_.size() - 1,
    std::vector<double>(accel_map_value_.at(0).size(), covariance_));

  const int vel_idx = accel_mode ? accel_vel_index : brake_vel_index;
  int ped_idx = accel_mode ? accel_pedal_index : brake_pedal_index;
  ped_idx = getUnifiedIndexFromAccelBrakeIndex(accel_mode, ped_idx);
  double map_offset = map_offset_vec_.at(ped_idx).at(vel_idx);
  double covariance = covariance_vec_.at(ped_idx).at(vel_idx);

  /* calculate adaptive map offset */
  const double phi = 1.0;
  covariance = (covariance - (covariance * phi * phi * covariance) /
                               (forgetting_factor_ + phi * covariance * phi)) /
               forgetting_factor_;

  const double coef = (covariance * phi) / (forgetting_factor_ + phi * covariance * phi);

  const double error_map_offset = measured_acc - map_acc;
  map_offset = map_offset + coef * error_map_offset;

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("accel_brake_map_calibrator"),
    "index: " << ped_idx << ", " << vel_idx << ": map_offset_ = "
              << map_offset_vec_.at(ped_idx).at(vel_idx) << " -> " << map_offset << "\t"
              << " covariance = " << covariance);

  /* input calculated result and update map */
  map_offset_vec_.at(ped_idx).at(vel_idx) = map_offset;
  covariance_vec_.at(ped_idx).at(vel_idx) = covariance;
  if (accel_mode) {
    update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index) =
      accel_map_value_.at(accel_pedal_index).at(accel_vel_index) + map_offset;
  } else {
    update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index) =
      brake_map_value_.at(brake_pedal_index).at(brake_vel_index) + map_offset;
  }
  return true;
}

void AccelBrakeMapCalibrator::updateTotalMapOffset(const double measured_acc, const double map_acc)
{
  /* calculate adaptive map offset */
  const double phi = 1.0;
  covariance_ = (covariance_ - (covariance_ * phi * phi * covariance_) /
                                 (forgetting_factor_ + phi * covariance_ * phi)) /
                forgetting_factor_;

  const double coef = (covariance_ * phi) / (forgetting_factor_ + phi * covariance_ * phi);
  const double error_map_offset = measured_acc - map_acc;
  map_offset_ = map_offset_ + coef * error_map_offset;

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("accel_brake_map_calibrator"),
    "map_offset_ = " << map_offset_ << "\t"
                     << "covariance = " << covariance_);

  /* update map */
  for (std::size_t ped_idx = 0; ped_idx < update_accel_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_accel_map_value_.at(0).size() - 1; vel_idx++) {
      update_accel_map_value_.at(ped_idx).at(vel_idx) =
        accel_map_value_.at(ped_idx).at(vel_idx) + map_offset_;
    }
  }
  for (std::size_t ped_idx = 0; ped_idx < update_brake_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_brake_map_value_.at(0).size() - 1; vel_idx++) {
      update_brake_map_value_.at(ped_idx).at(vel_idx) =
        brake_map_value_.at(ped_idx).at(vel_idx) + map_offset_;
    }
  }
}

std::vector<double> AccelBrakeMapCalibrator::getMapColumnFromUnifiedIndex(
  const std::vector<std::vector<double>> & accel_map_value,
  const std::vector<std::vector<double>> & brake_map_value, const std::size_t index)
{
  if (index < brake_map_value.size()) {
    // input brake map value
    return brake_map_value.at(brake_map_value.size() - index - 1);
  } else {
    // input accel map value
    return accel_map_value.at(index - brake_map_value.size() + 1);
  }
}

double AccelBrakeMapCalibrator::getPedalValueFromUnifiedIndex(const std::size_t index)
{
  if (index < brake_pedal_index_.size()) {
    // brake index ( minus )
    return -brake_pedal_index_.at(brake_pedal_index_.size() - index - 1);
  } else {
    // input accel map value
    return accel_pedal_index_.at(index - brake_pedal_index_.size() + 1);
  }
}

int AccelBrakeMapCalibrator::getUnifiedIndexFromAccelBrakeIndex(
  const bool accel_map, const std::size_t index)
{
  // accel_map=true: accel_map; accel_map=false: brake_map
  if (accel_map) {
    // accel map
    return index + brake_map_value_.size() - 1;
  } else {
    // brake map
    return brake_map_value_.size() - index - 1;
  }
}

double AccelBrakeMapCalibrator::getPitchCompensatedAcceleration()
{
  // It works correctly only when the velocity is positive.
  constexpr double gravity = 9.80665;
  return gravity * std::sin(pitch_);
}

void AccelBrakeMapCalibrator::executeEvaluation()
{
  const double full_orig_accel_sq_error = calculateAccelSquaredError(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    accel_map_, brake_map_);
  pushDataToVec(full_orig_accel_sq_error, full_mse_que_size_, &full_original_accel_mse_que_);
  full_original_accel_rmse_ = getAverage(full_original_accel_mse_que_);
  // std::cerr << "rmse : " << sqrt(full_original_accel_rmse_) << std::endl;

  const double full_orig_accel_l1_error = calculateAccelErrorL1Norm(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    accel_map_, brake_map_);
  const double full_orig_accel_sq_l1_error = full_orig_accel_l1_error * full_orig_accel_l1_error;
  pushDataToVec(full_orig_accel_l1_error, full_mse_que_size_, &full_original_accel_l1_que_);
  pushDataToVec(full_orig_accel_sq_l1_error, full_mse_que_size_, &full_original_accel_sq_l1_que_);
  full_original_accel_error_l1norm_ = getAverage(full_original_accel_l1_que_);

  /*calculate l1norm_covariance*/
  // const double full_original_accel_error_sql1_ = getAverage(full_original_accel_sq_l1_que_);
  // std::cerr << "error_l1norm : " << full_original_accel_error_l1norm_ << std::endl;
  // std::cerr << "error_l1_cov : " <<
  // full_original_accel_error_sql1_-full_original_accel_error_l1norm_*full_original_accel_error_l1norm_
  // << std::endl;

  const double part_orig_accel_sq_error = calculateAccelSquaredError(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    accel_map_, brake_map_);
  pushDataToVec(part_orig_accel_sq_error, part_mse_que_size_, &part_original_accel_mse_que_);
  part_original_accel_rmse_ = getAverage(part_original_accel_mse_que_);

  const double new_accel_sq_error = calculateAccelSquaredError(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    new_accel_map_, new_brake_map_);
  pushDataToVec(new_accel_sq_error, part_mse_que_size_, &new_accel_mse_que_);
  new_accel_rmse_ = getAverage(new_accel_mse_que_);
}

double AccelBrakeMapCalibrator::calculateEstimatedAcc(
  const double throttle, const double brake, const double vel, AccelMap & accel_map,
  BrakeMap & brake_map)
{
  const double pedal = throttle - brake;

  double estimated_acc = 0.0;
  if (pedal > 0.0) {
    accel_map.getAcceleration(pedal, vel, estimated_acc);
  } else {
    brake_map.getAcceleration(-pedal, vel, estimated_acc);
  }

  return estimated_acc;
}

double AccelBrakeMapCalibrator::calculateAccelSquaredError(
  const double throttle, const double brake, const double vel, AccelMap & accel_map,
  BrakeMap & brake_map)
{
  const double estimated_acc = calculateEstimatedAcc(throttle, brake, vel, accel_map, brake_map);
  const double measured_acc = acceleration_ - getPitchCompensatedAcceleration();
  const double dif_acc = measured_acc - estimated_acc;
  return dif_acc * dif_acc;
}

double AccelBrakeMapCalibrator::calculateAccelErrorL1Norm(
  const double throttle, const double brake, const double vel, AccelMap & accel_map,
  BrakeMap & brake_map)
{
  const double estimated_acc = calculateEstimatedAcc(throttle, brake, vel, accel_map, brake_map);
  const double measured_acc = acceleration_ - getPitchCompensatedAcceleration();
  const double dif_acc = measured_acc - estimated_acc;
  return abs(dif_acc);
}

void AccelBrakeMapCalibrator::pushDataToQue(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr & data, const std::size_t max_size,
  std::queue<geometry_msgs::msg::TwistStamped::ConstSharedPtr> * que)
{
  que->push(data);
  while (que->size() > max_size) {
    que->pop();
  }
}

template <class T>
void AccelBrakeMapCalibrator::pushDataToVec(
  const T data, const std::size_t max_size, std::vector<T> * vec)
{
  vec->emplace_back(data);
  while (vec->size() > max_size) {
    vec->erase(vec->begin());
  }
}

template <class T>
T AccelBrakeMapCalibrator::getNearestTimeDataFromVec(
  const T base_data, const double back_time, const std::vector<T> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = rclcpp::Time(base_data->header.stamp).seconds() - back_time;
  T nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = rclcpp::Time(data->header.stamp).seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

DataStampedPtr AccelBrakeMapCalibrator::getNearestTimeDataFromVec(
  DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = base_data->data_time.seconds() - back_time;
  DataStampedPtr nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = data->data_time.seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

double AccelBrakeMapCalibrator::getAverage(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (const auto num : vec) {
    sum += num;
  }
  return sum / vec.size();
}

double AccelBrakeMapCalibrator::getStandardDeviation(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return 0.0;
  }

  const double ave = getAverage(vec);

  double sum = 0.0;
  for (const auto num : vec) {
    sum += std::pow(num - ave, 2);
  }
  return std::sqrt(sum / vec.size());
}

bool AccelBrakeMapCalibrator::isTimeout(
  const builtin_interfaces::msg::Time & stamp, const double timeout_sec)
{
  const double dt = this->now().seconds() - rclcpp::Time(stamp).seconds();
  return dt > timeout_sec;
}

bool AccelBrakeMapCalibrator::isTimeout(
  const DataStampedPtr & data_stamped, const double timeout_sec)
{
  const double dt = (this->now() - data_stamped->data_time).seconds();
  return dt > timeout_sec;
}

nav_msgs::msg::OccupancyGrid AccelBrakeMapCalibrator::getOccMsg(
  const std::string frame_id, const double height, const double width, const double resolution,
  const std::vector<int8_t> & map_value)
{
  nav_msgs::msg::OccupancyGrid occ;
  occ.header.frame_id = frame_id;
  occ.header.stamp = this->now();
  occ.info.height = height;
  occ.info.width = width;
  occ.info.map_load_time = this->now();
  occ.info.origin.position.x = 0;
  occ.info.origin.position.y = 0;
  occ.info.origin.position.z = 0;
  occ.info.origin.orientation.x = 0;
  occ.info.origin.orientation.y = 0;
  occ.info.origin.orientation.z = 0;
  occ.info.origin.orientation.w = 1;
  occ.info.resolution = resolution;
  occ.data = map_value;
  return occ;
}

// function for diagnostics
void AccelBrakeMapCalibrator::checkUpdateSuggest(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using CalibrationStatus = tier4_external_api_msgs::msg::CalibrationStatus;
  CalibrationStatus accel_brake_map_status;
  int8_t level;
  std::string msg;

  accel_brake_map_status.target = CalibrationStatus::ACCEL_BRAKE_MAP;
  if (is_default_map_ == true) {
    accel_brake_map_status.status = CalibrationStatus::NORMAL;
    level = DiagStatus::OK;
    msg = "OK";
  } else {
    accel_brake_map_status.status = CalibrationStatus::UNAVAILABLE;
    level = DiagStatus::ERROR;
    msg = "Default map is not found in " + csv_default_map_dir_;
  }

  if (new_accel_mse_que_.size() < part_mse_que_size_ / 2) {
    // lack of data
    stat.summary(level, msg);
    calibration_status_pub_->publish(accel_brake_map_status);
    return;
  }

  const double rmse_rate = new_accel_rmse_ / part_original_accel_rmse_;
  if (rmse_rate < update_suggest_thresh_) {
    // The accuracy of original accel/brake map is low.
    // Suggest to update accel brake map
    level = DiagStatus::WARN;
    msg = "Accel/brake map Calibration is required.";
    accel_brake_map_status.status = CalibrationStatus::CALIBRATION_REQUIRED;
  }

  stat.summary(level, msg);
  calibration_status_pub_->publish(accel_brake_map_status);
}

// function for debug

void AccelBrakeMapCalibrator::publishMap(
  const std::vector<std::vector<double>> accel_map_value,
  const std::vector<std::vector<double>> brake_map_value, const std::string publish_type)
{
  if (accel_map_value.at(0).size() != brake_map_value.at(0).size()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("accel_brake_map_calibrator"),
      "Invalid map. The number of velocity index of accel map and brake map is different.");
    return;
  }
  const double h = accel_map_value.size() + brake_map_value.size() -
                   1;  // pedal (accel_map_value(0) and brake_map_value(0) is same.)
  const double w = accel_map_value.at(0).size();  // velocity

  // publish occupancy map
  const int8_t MAX_OCC_VALUE = 100;
  std::vector<int8_t> int_map_value;
  int_map_value.resize(h * w);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const double value =
        getMapColumnFromUnifiedIndex(accel_map_value_, brake_map_value_, i).at(j);
      // convert acc to 0~100 int value
      int8_t int_value =
        static_cast<uint8_t>(MAX_OCC_VALUE * ((value - min_accel_) / (max_accel_ - min_accel_)));
      int_map_value.at(i * w + j) = std::max(std::min(MAX_OCC_VALUE, int_value), (int8_t)0);
    }
  }

  if (publish_type == "original") {
    original_map_occ_pub_->publish(getOccMsg("base_link", h, w, map_resolution_, int_map_value));
  } else {
    update_map_occ_pub_->publish(getOccMsg("base_link", h, w, map_resolution_, int_map_value));
  }

  // publish raw map
  std_msgs::msg::Float32MultiArray float_map;

  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim[0].label = "height";
  float_map.layout.dim[1].label = "width";
  float_map.layout.dim[0].size = h;
  float_map.layout.dim[1].size = w;
  float_map.layout.dim[0].stride = h * w;
  float_map.layout.dim[1].stride = w;
  float_map.layout.data_offset = 0;
  std::vector<float> vec(h * w, 0);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      vec[i * w + j] = static_cast<float>(
        getMapColumnFromUnifiedIndex(accel_map_value_, brake_map_value_, i).at(j));
    }
  }
  float_map.data = vec;
  if (publish_type == "original") {
    original_map_raw_pub_->publish(float_map);
  } else {
    update_map_raw_pub_->publish(float_map);
  }
}

void AccelBrakeMapCalibrator::publishFloat32(const std::string publish_type, const double val)
{
  tier4_debug_msgs::msg::Float32Stamped msg;
  msg.stamp = this->now();
  msg.data = val;
  if (publish_type == "current_map_error") {
    current_map_error_pub_->publish(msg);
  } else if (publish_type == "updated_map_error") {
    updated_map_error_pub_->publish(msg);
  } else {
    map_error_ratio_pub_->publish(msg);
  }
}

void AccelBrakeMapCalibrator::publishCountMap()
{
  if (accel_map_value_.at(0).size() != brake_map_value_.at(0).size()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("accel_brake_map_calibrator"),
      "Invalid map. The number of velocity index of accel map and brake map is different.");
    return;
  }
  const double h = accel_map_value_.size() + brake_map_value_.size() -
                   1;  // pedal (accel_map_value(0) and brake_map_value(0) is same.)
  const double w = accel_map_value_.at(0).size();  // velocity
  const int8_t MAX_OCC_VALUE = 100;

  std::vector<int8_t> count_map, ave_map, std_map;
  count_map.resize(h * w);
  ave_map.resize(h * w);
  std_map.resize(h * w);

  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const auto data_vec = map_value_data_.at(i).at(j);
      if (data_vec.empty()) {
        // input *UNKNOWN* value
        count_map.at(i * w + j) = -1;
        ave_map.at(i * w + j) = -1;
      } else {
        const auto count_rate =
          MAX_OCC_VALUE * (static_cast<double>(data_vec.size()) / max_data_count_);
        count_map.at(i * w + j) = static_cast<int8_t>(
          std::max(std::min(static_cast<int>(MAX_OCC_VALUE), static_cast<int>(count_rate)), 0));
        // calculate average
        {
          const double average = getAverage(data_vec);
          int8_t int_average = static_cast<uint8_t>(
            MAX_OCC_VALUE * ((average - min_accel_) / (max_accel_ - min_accel_)));
          ave_map.at(i * w + j) = std::max(std::min(MAX_OCC_VALUE, int_average), (int8_t)0);
        }
        // calculate standard deviation
        {
          const double std_dev = getStandardDeviation(data_vec);
          const double max_std_dev = 0.2;
          const double min_std_dev = 0.0;
          int8_t int_std_dev = static_cast<uint8_t>(
            MAX_OCC_VALUE * ((std_dev - min_std_dev) / (max_std_dev - min_std_dev)));
          std_map.at(i * w + j) = std::max(std::min(MAX_OCC_VALUE, int_std_dev), (int8_t)0);
        }
      }
    }
  }

  // publish average map / standard dev map / count map
  data_ave_pub_->publish(getOccMsg("base_link", h, w, map_resolution_, ave_map));
  data_std_pub_->publish(getOccMsg("base_link", h, w, map_resolution_, std_map));
  data_count_pub_->publish(getOccMsg("base_link", h, w, map_resolution_, count_map));

  // publish count with self pos map
  int nearest_pedal_idx = nearestPedalSearch();
  int nearest_vel_idx = nearestVelSearch();

  // input self pose (to max value / min value ) and publish this
  update_success_
    ? count_map.at(nearest_pedal_idx * w + nearest_vel_idx) = std::numeric_limits<int8_t>::max()
    : count_map.at(nearest_pedal_idx * w + nearest_vel_idx) = std::numeric_limits<int8_t>::min();
  data_count_with_self_pose_pub_->publish(getOccMsg("base_link", h, w, map_resolution_, count_map));
}

void AccelBrakeMapCalibrator::publishIndex()
{
  visualization_msgs::msg::MarkerArray markers;
  const double h = accel_map_value_.size() + brake_map_value_.size() -
                   1;  // pedal (accel_map_value(0) and brake_map_value(0) is same.)
  const double w = accel_map_value_.at(0).size();  // velocity

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->now();
  marker.type = marker.TEXT_VIEW_FACING;
  marker.action = marker.ADD;
  std_msgs::msg::ColorRGBA color;
  color.a = 0.999;
  color.b = 0.1;
  color.g = 0.1;
  color.r = 0.1;
  marker.color = color;
  marker.frame_locked = true;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // pedal value
  for (int pedal_idx = 0; pedal_idx < h; pedal_idx++) {
    const double pedal_value = getPedalValueFromUnifiedIndex(pedal_idx);
    marker.ns = "occ_pedal_index";
    marker.id = pedal_idx;
    marker.pose.position.x = -map_resolution_ * 0.5;
    marker.pose.position.y = map_resolution_ * (0.5 + pedal_idx);
    marker.scale.z = 0.03;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << pedal_value;
    marker.text = stream.str();
    markers.markers.push_back(marker);
  }

  // pedal index name
  marker.ns = "occ_pedal_index";
  marker.id = h;
  marker.pose.position.x = -map_resolution_ * 0.5;
  marker.pose.position.y = map_resolution_ * (0.5 + h);
  marker.scale.z = 0.03;
  marker.text = "(accel_pedal/brake_pedal)";
  markers.markers.push_back(marker);

  // velocity value
  for (int vel_idx = 0; vel_idx < w; vel_idx++) {
    const double vel_value = accel_vel_index_.at(vel_idx);
    marker.ns = "occ_vel_index";
    marker.id = vel_idx;
    marker.pose.position.x = map_resolution_ * (0.5 + vel_idx);
    marker.pose.position.y = -map_resolution_ * 0.2;
    marker.scale.z = 0.02;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << vel_value;
    marker.text = stream.str() + "m/s";
    markers.markers.push_back(marker);
  }

  // velocity index name
  marker.ns = "occ_vel_index";
  marker.id = w;
  marker.pose.position.x = map_resolution_ * (0.5 + w);
  marker.pose.position.y = -map_resolution_ * 0.2;
  marker.scale.z = 0.02;
  marker.text = "(velocity)";
  markers.markers.push_back(marker);

  index_pub_->publish(markers);
}

void AccelBrakeMapCalibrator::publishUpdateSuggestFlag()
{
  std_msgs::msg::Bool update_suggest;

  if (new_accel_mse_que_.size() < part_mse_que_size_ / 2) {
    // lack of data
    update_suggest.data = false;
  } else {
    const double rmse_rate = new_accel_rmse_ / part_original_accel_rmse_;
    update_suggest.data = (rmse_rate < update_suggest_thresh_);
    if (update_suggest.data) {
      auto & clk = *this->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(
        rclcpp::get_logger("accel_brake_map_calibrator"), clk, 5000,
        "suggest to update accel/brake map. evaluation score = " << rmse_rate);
    }
  }

  update_suggest_pub_->publish(update_suggest);
}

bool AccelBrakeMapCalibrator::writeMapToCSV(
  std::vector<double> vel_index, std::vector<double> pedal_index,
  std::vector<std::vector<double>> value_map, std::string filename)
{
  if (update_success_count_ == 0) {
    return false;
  }

  std::ofstream csv_file(filename);

  if (!csv_file.is_open()) {
    RCLCPP_WARN(
      rclcpp::get_logger("accel_brake_map_calibrator"), "Failed to open csv file : %s",
      filename.c_str());
    return false;
  }

  csv_file << "default,";
  for (std::size_t v = 0; v < vel_index.size(); v++) {
    csv_file << vel_index.at(v);
    if (v != vel_index.size() - 1) {
      csv_file << ",";
    }
  }
  csv_file << "\n";

  for (std::size_t p = 0; p < pedal_index.size(); p++) {
    csv_file << pedal_index.at(p) << ",";
    for (std::size_t v = 0; v < vel_index.size(); v++) {
      csv_file << std::fixed << std::setprecision(precision_) << value_map.at(p).at(v);
      if (v != vel_index.size() - 1) {
        csv_file << ",";
      }
    }
    csv_file << "\n";
  }
  csv_file.close();
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("accel_brake_map_calibrator"), "output map to " << filename);
  return true;
}

void AccelBrakeMapCalibrator::addIndexToCSV(std::ofstream * csv_file)
{
  *csv_file << "timestamp,velocity,accel,pitch_comp_accel,final_accel,accel_pedal,brake_pedal,"
            << "accel_pedal_speed,brake_pedal_speed,pitch,steer,jerk,full_original_accel_rmse, "
               "part_original_accel_rmse,new_accel_rmse, rmse_rate"
            << std::endl;
}

void AccelBrakeMapCalibrator::addLogToCSV(
  std::ofstream * csv_file, const double & timestamp, const double velocity, const double accel,
  const double pitched_accel, const double accel_pedal, const double brake_pedal,
  const double accel_pedal_speed, const double brake_pedal_speed, const double pitch,
  const double steer, const double jerk, const double full_original_accel_mse,
  const double part_original_accel_mse, const double new_accel_mse)
{
  const double rmse_rate =
    part_original_accel_mse == 0.0 ? 1.0 : (new_accel_mse / part_original_accel_mse);
  *csv_file << timestamp << ", " << velocity << ", " << accel << ", " << pitched_accel << ", "
            << accel - pitched_accel << ", " << accel_pedal << ", " << brake_pedal << ", "
            << accel_pedal_speed << ", " << brake_pedal_speed << ", " << pitch << ", " << steer
            << ", " << jerk << ", " << full_original_accel_mse << ", " << part_original_accel_mse
            << ", " << new_accel_mse << "," << rmse_rate << std::endl;
}
