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

#ifndef ACCEL_BRAKE_MAP_CALIBRATOR__ACCEL_BRAKE_MAP_CALIBRATOR_NODE_HPP_
#define ACCEL_BRAKE_MAP_CALIBRATOR__ACCEL_BRAKE_MAP_CALIBRATOR_NODE_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "raw_vehicle_cmd_converter/accel_map.hpp"
#include "raw_vehicle_cmd_converter/brake_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <Eigen/Dense>

#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_external_api_msgs/msg/calibration_status.hpp"
#include "tier4_external_api_msgs/msg/calibration_status_array.hpp"
#include "tier4_external_api_msgs/srv/get_accel_brake_map_calibration_data.hpp"
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"
#include "tier4_vehicle_msgs/srv/update_accel_brake_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <fstream>
#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace accel_brake_map_calibrator
{

using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;
using raw_vehicle_cmd_converter::AccelMap;
using raw_vehicle_cmd_converter::BrakeMap;
using std_msgs::msg::Float32MultiArray;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_debug_msgs::msg::Float32Stamped;
using tier4_external_api_msgs::msg::CalibrationStatus;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;
using tier4_vehicle_msgs::msg::ActuationStatusStamped;
using visualization_msgs::msg::MarkerArray;

using tier4_vehicle_msgs::srv::UpdateAccelBrakeMap;

using Map = std::vector<std::vector<double>>;

struct DataStamped
{
  DataStamped(const double _data, const rclcpp::Time & _data_time)
  : data{_data}, data_time{_data_time}
  {
  }
  double data;
  rclcpp::Time data_time;
};
using DataStampedPtr = std::shared_ptr<DataStamped>;

class AccelBrakeMapCalibrator : public rclcpp::Node
{
private:
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;
  std::string csv_default_map_dir_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr original_map_occ_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr update_map_occ_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr original_map_raw_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr update_map_raw_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr offset_covariance_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_count_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_count_with_self_pose_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_ave_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_std_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr index_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr update_suggest_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr current_map_error_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr updated_map_error_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr map_error_ratio_pub_;
  rclcpp::Publisher<CalibrationStatus>::SharedPtr calibration_status_pub_;

  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<SteeringReport>::SharedPtr steer_sub_;
  rclcpp::Subscription<ActuationStatusStamped>::SharedPtr actuation_status_sub_;
  rclcpp::Subscription<ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;

  // Service
  rclcpp::Service<UpdateAccelBrakeMap>::SharedPtr update_map_dir_server_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_output_csv_;
  void initTimer(double period_s);
  void initOutputCSVTimer(double period_s);

  TwistStamped::ConstSharedPtr twist_ptr_;
  std::vector<std::shared_ptr<TwistStamped>> twist_vec_;
  std::vector<DataStampedPtr> accel_pedal_vec_;  // for delayed pedal
  std::vector<DataStampedPtr> brake_pedal_vec_;  // for delayed pedal
  SteeringReport::ConstSharedPtr steer_ptr_;
  DataStampedPtr accel_pedal_ptr_;
  DataStampedPtr brake_pedal_ptr_;
  DataStampedPtr delayed_accel_pedal_ptr_;
  DataStampedPtr delayed_brake_pedal_ptr_;

  // Diagnostic Updater
  std::shared_ptr<diagnostic_updater::Updater> updater_ptr_;
  bool is_default_map_ = true;

  int get_pitch_method_;
  int update_method_;
  int accel_brake_value_source_;
  double acceleration_ = 0.0;
  double acceleration_time_;
  double pre_acceleration_ = 0.0;
  double pre_acceleration_time_;
  double jerk_ = 0.0;
  double accel_pedal_speed_ = 0.0;
  double brake_pedal_speed_ = 0.0;
  double pitch_ = 0.0;
  double update_hz_;
  double velocity_min_threshold_;
  double velocity_diff_threshold_;
  double pedal_diff_threshold_;
  double max_steer_threshold_;
  double max_pitch_threshold_;
  double max_jerk_threshold_;
  double pedal_velocity_thresh_;
  double max_accel_;
  double min_accel_;
  double pedal_to_accel_delay_;
  int precision_;
  std::string csv_calibrated_map_dir_;
  std::string output_accel_file_;
  std::string output_brake_file_;

  // for calculating differential of msg
  const double dif_twist_time_ = 0.2;   // 200ms
  const double dif_pedal_time_ = 0.16;  // 160ms
  const std::size_t twist_vec_max_size_ = 100;
  const std::size_t pedal_vec_max_size_ = 100;
  const double timeout_sec_ = 0.1;
  int max_data_count_;
  const int max_data_save_num_ = 10000;
  const double map_resolution_ = 0.1;
  const double max_jerk_ = 5.0;
  bool pedal_accel_graph_output_ = false;
  bool progress_file_output_ = false;

  // Algorithm
  AccelMap accel_map_;
  BrakeMap brake_map_;

  // for evaluation
  AccelMap new_accel_map_;
  BrakeMap new_brake_map_;
  std::vector<double> part_original_accel_mse_que_;
  std::vector<double> full_original_accel_mse_que_;
  // std::vector<double> full_original_accel_esm_que_;
  std::vector<double> full_original_accel_l1_que_;
  std::vector<double> full_original_accel_sq_l1_que_;
  std::vector<double> new_accel_mse_que_;
  std::size_t full_mse_que_size_ = 100000;
  std::size_t part_mse_que_size_ = 3000;
  double full_original_accel_rmse_ = 0.0;
  double full_original_accel_error_l1norm_ = 0.0;
  double part_original_accel_rmse_ = 0.0;
  double new_accel_rmse_ = 0.0;
  double update_suggest_thresh_;

  // Accel / Brake Map update
  Map accel_map_value_;
  Map brake_map_value_;
  Map update_accel_map_value_;
  Map update_brake_map_value_;
  Map accel_offset_covariance_value_;
  Map brake_offset_covariance_value_;
  std::vector<Map> map_value_data_;
  std::vector<double> accel_vel_index_;
  std::vector<double> brake_vel_index_;
  std::vector<double> accel_pedal_index_;
  std::vector<double> brake_pedal_index_;
  bool update_success_;
  int update_success_count_ = 0;
  int update_count_ = 0;
  int lack_of_data_count_ = 0;
  int failed_to_get_pitch_count_ = 0;
  int too_large_pitch_count_ = 0;
  int too_low_speed_count_ = 0;
  int too_large_steer_count_ = 0;
  int too_large_jerk_count_ = 0;
  int invalid_acc_brake_count_ = 0;
  int too_large_pedal_spd_count_ = 0;
  int update_fail_count_ = 0;

  // for map update
  double map_offset_ = 0.0;
  double map_coef_ = 1.0;
  double covariance_;
  const double forgetting_factor_ = 0.999;
  const double coef_update_skip_thresh_ = 0.1;

  // output log
  std::ofstream output_log_;

  bool getCurrentPitchFromTF(double * pitch);
  void timerCallback();
  void timerCallbackOutputCSV();
  void executeUpdate(
    const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
    const int brake_pedal_index, const int brake_vel_index);
  bool updateFourCellAroundOffset(
    const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
    const int brake_pedal_index, const int brake_vel_index, const double measured_acc);
  bool updateEachValOffset(
    const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
    const int brake_pedal_index, const int brake_vel_index, const double measured_acc,
    const double map_acc);
  void updateTotalMapOffset(const double measured_acc, const double map_acc);
  void callbackActuation(
    const std_msgs::msg::Header header, const double accel, const double brake);
  void callbackActuationCommand(const ActuationCommandStamped::ConstSharedPtr msg);
  void callbackActuationStatus(const ActuationStatusStamped::ConstSharedPtr msg);
  void callbackVelocity(const VelocityReport::ConstSharedPtr msg);
  void callbackSteer(const SteeringReport::ConstSharedPtr msg);
  bool callbackUpdateMapService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    UpdateAccelBrakeMap::Request::SharedPtr req, UpdateAccelBrakeMap::Response::SharedPtr res);
  bool getAccFromMap(const double velocity, const double pedal);
  double lowpass(const double original, const double current, const double gain = 0.8);
  double getPedalSpeed(
    const DataStampedPtr & prev_pedal, const DataStampedPtr & current_pedal,
    const double prev_pedal_speed);
  double getAccel(
    const TwistStamped::ConstSharedPtr & prev_twist,
    const TwistStamped::ConstSharedPtr & current_twist);
  double getJerk();
  bool indexValueSearch(
    const std::vector<double> value_index, const double value, const double value_thresh,
    int * searched_index);
  int nearestValueSearch(const std::vector<double> value_index, const double value);
  int nearestPedalSearch();
  int nearestVelSearch();
  void takeConsistencyOfAccelMap();
  void takeConsistencyOfBrakeMap();
  bool updateAccelBrakeMap();
  void publishFloat32(const std::string publish_type, const double val);
  void publishUpdateSuggestFlag();
  double getPitchCompensatedAcceleration();
  void executeEvaluation();
  double calculateEstimatedAcc(
    const double throttle, const double brake, const double vel, AccelMap & accel_map,
    BrakeMap & brake_map);
  double calculateAccelSquaredError(
    const double throttle, const double brake, const double vel, AccelMap & accel_map,
    BrakeMap & brake_map);
  double calculateAccelErrorL1Norm(
    const double throttle, const double brake, const double vel, AccelMap & accel_map,
    BrakeMap & brake_map);
  std::vector<double> getMapColumnFromUnifiedIndex(
    const Map & accel_map_value, const Map & brake_map_value, const std::size_t index);
  double getPedalValueFromUnifiedIndex(const std::size_t index);
  int getUnifiedIndexFromAccelBrakeIndex(const bool accel_map, const std::size_t index);
  void pushDataToQue(
    const TwistStamped::ConstSharedPtr & data, const std::size_t max_size,
    std::queue<TwistStamped::ConstSharedPtr> * que);
  template <class T>
  void pushDataToVec(const T data, const std::size_t max_size, std::vector<T> * vec);
  template <class T>
  T getNearestTimeDataFromVec(
    const T base_data, const double back_time, const std::vector<T> & vec);
  DataStampedPtr getNearestTimeDataFromVec(
    DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec);
  double getAverage(const std::vector<double> & vec);
  double getStandardDeviation(const std::vector<double> & vec);
  bool isTimeout(const builtin_interfaces::msg::Time & stamp, const double timeout_sec);
  bool isTimeout(const DataStampedPtr & data_stamped, const double timeout_sec);

  /* for covariance calculation */
  // mean value on each cell (counting method depends on the update algorithm)
  Eigen::MatrixXd accel_data_mean_mat_;
  Eigen::MatrixXd brake_data_mean_mat_;
  // calculated variance on each cell
  Eigen::MatrixXd accel_data_covariance_mat_;
  Eigen::MatrixXd brake_data_covariance_mat_;
  // number of data on each cell (counting method depends on the update algorithm)
  Eigen::MatrixXd accel_data_num_;
  Eigen::MatrixXd brake_data_num_;

  OccupancyGrid getOccMsg(
    const std::string frame_id, const double height, const double width, const double resolution,
    const std::vector<int8_t> & map_value);

  /* Diag*/
  void checkUpdateSuggest(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /* Debug */
  void publishMap(
    const Map accel_map_value, const Map brake_map_value, const std::string publish_type);
  void publishOffsetCovMap(const Map accel_map_value, const Map brake_map_value);
  void publishCountMap();
  void publishIndex();
  bool writeMapToCSV(
    std::vector<double> vel_index, std::vector<double> pedal_index, Map value_map,
    std::string filename);
  void addIndexToCSV(std::ofstream * csv_file);
  void addLogToCSV(
    std::ofstream * csv_file, const double & timestamp, const double velocity, const double accel,
    const double pitched_accel, const double accel_pedal, const double brake_pedal,
    const double accel_pedal_speed, const double brake_pedal_speed, const double pitch,
    const double steer, const double jerk, const double full_original_accel_mse,
    const double part_original_accel_mse, const double new_accel_mse);

  mutable Float32MultiArrayStamped debug_values_;
  enum DBGVAL {
    CURRENT_SPEED = 0,
    CURRENT_ACCEL_PEDAL = 1,
    CURRENT_BRAKE_PEDAL = 2,
    CURRENT_RAW_ACCEL = 3,
    CURRENT_ACCEL = 4,
    CURRENT_RAW_ACCEL_SPEED = 5,
    CURRENT_ACCEL_SPEED = 6,
    CURRENT_RAW_BRAKE_SPEED = 7,
    CURRENT_BRAKE_SPEED = 8,
    CURRENT_RAW_PITCH = 9,
    CURRENT_PITCH = 10,
    CURRENT_STEER = 11,
    SUCCESS_TO_UPDATE = 12,
    CURRENT_JERK = 13,
  };
  static constexpr unsigned int num_debug_values_ = 14;

  enum GET_PITCH_METHOD { TF = 0, FILE = 1, NONE = 2 };

  enum UPDATE_METHOD {
    UPDATE_OFFSET_EACH_CELL = 0,
    UPDATE_OFFSET_TOTAL = 1,
    UPDATE_OFFSET_FOUR_CELL_AROUND = 2,
  };

  enum ACCEL_BRAKE_SOURCE {
    STATUS = 0,
    COMMAND = 1,
  };

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

public:
  explicit AccelBrakeMapCalibrator(const rclcpp::NodeOptions & node_options);
};

}  // namespace accel_brake_map_calibrator

#endif  // ACCEL_BRAKE_MAP_CALIBRATOR__ACCEL_BRAKE_MAP_CALIBRATOR_NODE_HPP_
