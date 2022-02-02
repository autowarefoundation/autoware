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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <traffic_light_roi_visualizer/nodelet.hpp>

#include <memory>
#include <string>
#include <utility>

namespace traffic_light
{
TrafficLightRoiVisualizerNodelet::TrafficLightRoiVisualizerNodelet(
  const rclcpp::NodeOptions & options)
: Node("traffic_light_roi_visualizer_node", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  enable_fine_detection_ = this->declare_parameter("enable_fine_detection", false);

  if (enable_fine_detection_) {
    sync_with_rough_roi_.reset(new SyncWithRoughRoi(
      SyncPolicyWithRoughRoi(10), image_sub_, roi_sub_, rough_roi_sub_, traffic_signals_sub_));
    sync_with_rough_roi_->registerCallback(
      std::bind(&TrafficLightRoiVisualizerNodelet::imageRoughRoiCallback, this, _1, _2, _3, _4));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_, traffic_signals_sub_));
    sync_->registerCallback(
      std::bind(&TrafficLightRoiVisualizerNodelet::imageRoiCallback, this, _1, _2, _3));
  }

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightRoiVisualizerNodelet::connectCb, this));

  image_pub_ =
    image_transport::create_publisher(this, "~/output/image", rclcpp::QoS{1}.get_rmw_qos_profile());
}

void TrafficLightRoiVisualizerNodelet::connectCb()
{
  if (image_pub_.getNumSubscribers() == 0) {
    image_sub_.unsubscribe();
    traffic_signals_sub_.unsubscribe();
    roi_sub_.unsubscribe();
    if (enable_fine_detection_) {
      rough_roi_sub_.unsubscribe();
    }
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    traffic_signals_sub_.subscribe(
      this, "~/input/traffic_signals", rclcpp::QoS{1}.get_rmw_qos_profile());
    if (enable_fine_detection_) {
      rough_roi_sub_.subscribe(this, "~/input/rough/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    }
  }
}

bool TrafficLightRoiVisualizerNodelet::createRect(
  cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi,
  const cv::Scalar & color)
{
  cv::rectangle(
    image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 3);
  cv::putText(
    image, std::to_string(tl_roi.id), cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::FONT_HERSHEY_COMPLEX, 1.0, color, 1, CV_AA);
  return true;
}

bool TrafficLightRoiVisualizerNodelet::createRect(
  cv::Mat & image, const autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi,
  const ClassificationResult & result)
{
  cv::Scalar color;
  if (result.label.find("red") != std::string::npos) {
    color = cv::Scalar{255, 0, 0};
  } else if (result.label.find("yellow") != std::string::npos) {
    color = cv::Scalar{0, 255, 0};
  } else if (result.label.find("green") != std::string::npos) {
    color = cv::Scalar{0, 0, 255};
  } else {
    color = cv::Scalar{255, 255, 255};
  }

  cv::rectangle(
    image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 3);

  int offset = 40;
  cv::putText(
    image, std::to_string(result.prob),
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset - (offset * 0)), cv::FONT_HERSHEY_COMPLEX,
    1.1, color, 3);

  cv::putText(
    image, result.label, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset - (offset * 1)),
    cv::FONT_HERSHEY_COMPLEX, 1.1, color, 2);

  return true;
}

void TrafficLightRoiVisualizerNodelet::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
  [[maybe_unused]] const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr &
    input_traffic_signals_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, input_image_msg->encoding);
    for (auto tl_roi : input_tl_roi_msg->rois) {
      createRect(cv_ptr->image, tl_roi, cv::Scalar(0, 255, 0));
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}

bool TrafficLightRoiVisualizerNodelet::getClassificationResult(
  int id, const autoware_auto_perception_msgs::msg::TrafficSignalArray & traffic_signals,
  ClassificationResult & result)
{
  bool has_correspond_traffic_signal = false;
  for (const auto & traffic_signal : traffic_signals.signals) {
    if (id != traffic_signal.map_primitive_id) {
      continue;
    }
    has_correspond_traffic_signal = true;
    for (size_t i = 0; i < traffic_signal.lights.size(); i++) {
      auto light = traffic_signal.lights.at(i);
      // all lamp confidence are the same
      result.prob = light.confidence;
      result.label += (state2label_[light.color] + "-" + state2label_[light.shape]);
      if (i < traffic_signal.lights.size() - 1) {
        result.label += ",";
      }
    }
  }
  return has_correspond_traffic_signal;
}

bool TrafficLightRoiVisualizerNodelet::getRoiFromId(
  int id, const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & rois,
  autoware_auto_perception_msgs::msg::TrafficLightRoi & correspond_roi)
{
  for (const auto roi : rois->rois) {
    if (roi.id == id) {
      correspond_roi = roi;
      return true;
    }
  }
  return false;
}

void TrafficLightRoiVisualizerNodelet::imageRoughRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr &
    input_tl_rough_roi_msg,
  const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr &
    input_traffic_signals_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, input_image_msg->encoding);
    for (auto tl_rough_roi : input_tl_rough_roi_msg->rois) {
      // visualize rough roi
      createRect(cv_ptr->image, tl_rough_roi, cv::Scalar(0, 255, 0));

      ClassificationResult result;
      bool has_correspond_traffic_signal =
        getClassificationResult(tl_rough_roi.id, *input_traffic_signals_msg, result);
      autoware_auto_perception_msgs::msg::TrafficLightRoi tl_roi;
      bool has_correspond_roi = getRoiFromId(tl_rough_roi.id, input_tl_roi_msg, tl_roi);

      if (has_correspond_roi && has_correspond_traffic_signal) {
        // has fine detection and classification results
        createRect(cv_ptr->image, tl_roi, result);
      } else if (has_correspond_roi && !has_correspond_traffic_signal) {
        // has fine detection result and does not have classification result
        createRect(cv_ptr->image, tl_roi, cv::Scalar(255, 255, 255));
      } else if (!has_correspond_roi && has_correspond_traffic_signal) {
        // does not have fine detection result and has classification result
        createRect(cv_ptr->image, tl_rough_roi, result);
      } else {
      }
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}

}  // namespace traffic_light

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightRoiVisualizerNodelet)
