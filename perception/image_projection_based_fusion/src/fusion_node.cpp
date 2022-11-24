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

#define EIGEN_MPL2_ONLY

#include "image_projection_based_fusion/fusion_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>

#include <boost/optional.hpp>

#include <cmath>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <typeinfo>
namespace image_projection_based_fusion
{

template <class Msg, class ObjType>
FusionNode<Msg, ObjType>::FusionNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // set rois_number
  rois_number_ = static_cast<std::size_t>(declare_parameter("rois_number", 1));
  if (rois_number_ < 1) {
    RCLCPP_WARN(
      this->get_logger(), "minimum rois_number is 1. current rois_number is %zu", rois_number_);
    rois_number_ = 1;
  }
  if (rois_number_ > 8) {
    RCLCPP_WARN(
      this->get_logger(), "maximum rois_number is 8. current rois_number is %zu", rois_number_);
    rois_number_ = 8;
  }

  // Set parameters
  match_threshold_ms_ = declare_parameter<double>("match_threshold_ms");
  timeout_ms_ = declare_parameter<double>("timeout_ms");

  input_offset_ms_ = declare_parameter("input_offset_ms", std::vector<double>{});
  if (!input_offset_ms_.empty() && rois_number_ != input_offset_ms_.size()) {
    throw std::runtime_error("The number of offsets does not match the number of topics.");
  }

  // sub camera info
  camera_info_subs_.resize(rois_number_);
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    std::function<void(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)> fnc =
      std::bind(&FusionNode::cameraInfoCallback, this, std::placeholders::_1, roi_i);
    camera_info_subs_.at(roi_i) = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "input/camera_info" + std::to_string(roi_i), rclcpp::QoS{1}.best_effort(), fnc);
  }

  // sub rois
  rois_subs_.resize(rois_number_);
  roi_stdmap_.resize(rois_number_);
  is_fused_.resize(rois_number_, false);
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    std::function<void(const DetectedObjectsWithFeature::ConstSharedPtr msg)> roi_callback =
      std::bind(&FusionNode::roiCallback, this, std::placeholders::_1, roi_i);
    rois_subs_.at(roi_i) = this->create_subscription<DetectedObjectsWithFeature>(
      "input/rois" + std::to_string(roi_i), rclcpp::QoS{1}.best_effort(), roi_callback);
  }

  // subscribers
  std::function<void(const typename Msg::ConstSharedPtr msg)> sub_callback =
    std::bind(&FusionNode::subCallback, this, std::placeholders::_1);
  sub_ = this->create_subscription<Msg>("input", rclcpp::QoS(1).best_effort(), sub_callback);

  // publisher
  pub_ptr_ = this->create_publisher<Msg>("output", rclcpp::QoS{1});

  // Set timer
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(timeout_ms_));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&FusionNode::timer_callback, this));

  // debugger
  if (declare_parameter("debug_mode", false)) {
    std::size_t image_buffer_size =
      static_cast<std::size_t>(declare_parameter("image_buffer_size", 15));
    debugger_ = std::make_shared<Debugger>(this, rois_number_, image_buffer_size);
  }

  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "fusion_node");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  filter_scope_minx_ = declare_parameter("filter_scope_minx", -100);
  filter_scope_maxx_ = declare_parameter("filter_scope_maxx", 100);
  filter_scope_miny_ = declare_parameter("filter_scope_miny", -100);
  filter_scope_maxy_ = declare_parameter("filter_scope_maxy", 100);
  filter_scope_minz_ = declare_parameter("filter_scope_minz", -100);
  filter_scope_maxz_ = declare_parameter("filter_scope_maxz", 100);
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg,
  const std::size_t camera_id)
{
  camera_info_map_[camera_id] = *input_camera_info_msg;
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::preprocess(Msg & ouput_msg __attribute__((unused)))
{
  // do nothing by default
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::subCallback(const typename Msg::ConstSharedPtr input_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_ms_));
  try {
    setPeriod(period.count());
  } catch (rclcpp::exceptions::RCLError & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
  }
  timer_->reset();

  stop_watch_ptr_->toc("processing_time", true);

  typename Msg::SharedPtr output_msg = std::make_shared<Msg>(*input_msg);

  preprocess(*output_msg);

  int64_t timestamp_nsec =
    (*output_msg).header.stamp.sec * (int64_t)1e9 + (*output_msg).header.stamp.nanosec;

  // if matching rois exist, fuseonsingle
  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    if (camera_info_map_.find(roi_i) == camera_info_map_.end()) {
      RCLCPP_WARN(this->get_logger(), "no camera info. id is %zu", roi_i);
      continue;
    }

    if ((roi_stdmap_.at(roi_i)).size() > 0) {
      int64_t min_interval = 1e9;
      int64_t matched_stamp = -1;
      std::list<int64_t> outdate_stamps;

      for (const auto & [k, v] : roi_stdmap_.at(roi_i)) {
        int64_t newstamp = timestamp_nsec + input_offset_ms_.at(roi_i) * (int64_t)1e6;
        int64_t interval = abs(int64_t(k) - newstamp);

        if (interval <= min_interval && interval <= match_threshold_ms_ * (int64_t)1e6) {
          min_interval = interval;
          matched_stamp = k;
        } else if (int64_t(k) < newstamp && interval > match_threshold_ms_ * (int64_t)1e6) {
          outdate_stamps.push_back(int64_t(k));
        }
      }

      // remove outdated stamps
      for (auto stamp : outdate_stamps) {
        (roi_stdmap_.at(roi_i)).erase(stamp);
      }

      // fuseonSingle
      if (matched_stamp != -1) {
        if (debugger_) {
          debugger_->clear();
        }

        fuseOnSingleImage(
          *input_msg, roi_i, *((roi_stdmap_.at(roi_i))[matched_stamp]), camera_info_map_.at(roi_i),
          *output_msg);
        (roi_stdmap_.at(roi_i)).erase(matched_stamp);
        is_fused_.at(roi_i) = true;

        // add timestamp interval for debug
        if (debug_publisher_) {
          double timestamp_interval_ms = (matched_stamp - timestamp_nsec) / 1e6;
          debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
            "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_ms", timestamp_interval_ms);
          debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
            "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_offset_ms",
            timestamp_interval_ms - input_offset_ms_.at(roi_i));
        }
      }
    }
  }

  // if all camera fused, postprocess; else, publish the old Msg(if exists) and cache the current
  // Msg
  if (std::count(is_fused_.begin(), is_fused_.end(), true) == static_cast<int>(rois_number_)) {
    timer_->cancel();
    postprocess(*output_msg);
    publish(*output_msg);
    std::fill(is_fused_.begin(), is_fused_.end(), false);

    // add processing time for debug
    if (debug_publisher_) {
      const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
      const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/cyclic_time_ms", cyclic_time_ms);
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/processing_time_ms", processing_time_ms);
    }
  } else {
    if (sub_stdpair_.second != nullptr) {
      timer_->cancel();
      postprocess(*(sub_stdpair_.second));
      publish(*(sub_stdpair_.second));
      std::fill(is_fused_.begin(), is_fused_.end(), false);

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

    sub_stdpair_.first = int64_t(timestamp_nsec);
    sub_stdpair_.second = output_msg;
  }
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::roiCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr input_roi_msg, const std::size_t roi_i)
{
  int64_t timestamp_nsec =
    (*input_roi_msg).header.stamp.sec * (int64_t)1e9 + (*input_roi_msg).header.stamp.nanosec;

  // if cached Msg exist, try to match
  if (sub_stdpair_.second != nullptr) {
    int64_t newstamp = sub_stdpair_.first + input_offset_ms_.at(roi_i) * (int64_t)1e6;
    int64_t interval = abs(timestamp_nsec - newstamp);

    if (interval < match_threshold_ms_ * (int64_t)1e6 && is_fused_.at(roi_i) == false) {
      if (camera_info_map_.find(roi_i) == camera_info_map_.end()) {
        RCLCPP_WARN(this->get_logger(), "no camera info. id is %zu", roi_i);
        (roi_stdmap_.at(roi_i))[timestamp_nsec] = input_roi_msg;
        return;
      }
      if (debugger_) {
        debugger_->clear();
      }

      fuseOnSingleImage(
        *(sub_stdpair_.second), roi_i, *input_roi_msg, camera_info_map_.at(roi_i),
        *(sub_stdpair_.second));
      is_fused_.at(roi_i) = true;

      if (debug_publisher_) {
        double timestamp_interval_ms = (timestamp_nsec - sub_stdpair_.first) / 1e6;
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_ms", timestamp_interval_ms);
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug/roi" + std::to_string(roi_i) + "/timestamp_interval_offset_ms",
          timestamp_interval_ms - input_offset_ms_.at(roi_i));
      }

      if (std::count(is_fused_.begin(), is_fused_.end(), true) == static_cast<int>(rois_number_)) {
        timer_->cancel();
        postprocess(*(sub_stdpair_.second));
        publish(*(sub_stdpair_.second));
        std::fill(is_fused_.begin(), is_fused_.end(), false);
        sub_stdpair_.second = nullptr;

        // add processing time for debug
        if (debug_publisher_) {
          const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
          debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
            "debug/cyclic_time_ms", cyclic_time_ms);
        }
      }
      return;
    }
  }
  // store roi msg if not matched
  (roi_stdmap_.at(roi_i))[timestamp_nsec] = input_roi_msg;
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::postprocess(Msg & output_msg __attribute__((unused)))
{
  // do nothing by default
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::timer_callback()
{
  using std::chrono_literals::operator""ms;
  timer_->cancel();
  if (mutex_.try_lock()) {
    // timeout, postprocess cached msg
    if (sub_stdpair_.second != nullptr) {
      postprocess(*(sub_stdpair_.second));
      publish(*(sub_stdpair_.second));
    }
    std::fill(is_fused_.begin(), is_fused_.end(), false);
    sub_stdpair_.second = nullptr;

    // add processing time for debug
    if (debug_publisher_) {
      const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
      debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/cyclic_time_ms", cyclic_time_ms);
    }

    mutex_.unlock();
  } else {
    try {
      std::chrono::nanoseconds period = 10ms;
      setPeriod(period.count());
    } catch (rclcpp::exceptions::RCLError & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    }
    timer_->reset();
  }
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::setPeriod(const int64_t new_period)
{
  if (!timer_) {
    return;
  }
  int64_t old_period = 0;
  rcl_ret_t ret = rcl_timer_get_period(timer_->get_timer_handle().get(), &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get old period");
  }
  ret = rcl_timer_exchange_period(timer_->get_timer_handle().get(), new_period, &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
  }
}

template <class Msg, class Obj>
void FusionNode<Msg, Obj>::publish(const Msg & output_msg)
{
  if (pub_ptr_->get_subscription_count() < 1) {
    return;
  }
  pub_ptr_->publish(output_msg);
}

template class FusionNode<DetectedObjects, DetectedObject>;
template class FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature>;
template class FusionNode<sensor_msgs::msg::PointCloud2, DetectedObjects>;
}  // namespace image_projection_based_fusion
