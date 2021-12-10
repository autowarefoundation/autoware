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

#ifndef TRAFFIC_LIGHT_SSD_FINE_DETECTOR__NODELET_HPP_
#define TRAFFIC_LIGHT_SSD_FINE_DETECTOR__NODELET_HPP_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trt_ssd.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

typedef struct Detection
{
  float x, y, w, h, prob;
} Detection;

namespace traffic_light
{
class TrafficLightSSDFineDetectorNodelet : public rclcpp::Node
{
public:
  explicit TrafficLightSSDFineDetectorNodelet(const rclcpp::NodeOptions & options);
  void connectCb();
  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr
      traffic_light_roi_msg);

private:
  bool cvMat2CnnInput(
    const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<float> & data);
  bool cnnOutput2BoxDetection(
    const float * scores, const float * boxes, const int tlr_id,
    const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<Detection> & detections);
  bool rosMsg2CvMat(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image);
  bool fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size);
  void cvRect2TlRoiMsg(
    const cv::Rect & rect, const int32_t id,
    autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi);
  bool readLabelFile(std::string filepath, std::vector<std::string> & labels);
  bool getTlrIdFromLabel(const std::vector<std::string> & labels, int & tlr_id);

  // variables
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::TrafficLightRoiArray> roi_sub_;
  std::mutex connect_mutex_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr
    output_roi_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, autoware_auto_perception_msgs::msg::TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, autoware_auto_perception_msgs::msg::TrafficLightRoiArray>
    ApproximateSyncPolicy;
  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  bool is_approximate_sync_;
  double score_thresh_;

  int tlr_id_;
  int channel_;
  int width_;
  int height_;
  int class_num_;
  int detection_per_class_;

  std::vector<float> mean_;
  std::vector<float> std_;

  std::unique_ptr<ssd::Net> net_ptr_;
};  // TrafficLightSSDFineDetectorNodelet

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_SSD_FINE_DETECTOR__NODELET_HPP_
