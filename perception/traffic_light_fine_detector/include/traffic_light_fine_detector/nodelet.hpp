// Copyright 2023 TIER IV, Inc.
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

#ifndef TRAFFIC_LIGHT_FINE_DETECTOR__NODELET_HPP_
#define TRAFFIC_LIGHT_FINE_DETECTOR__NODELET_HPP_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tensorrt_yolox/tensorrt_yolox.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <fstream>
#include <map>
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
class TrafficLightFineDetectorNodelet : public rclcpp::Node
{
  using TrafficLightRoi = tier4_perception_msgs::msg::TrafficLightRoi;
  using TrafficLightRoiArray = tier4_perception_msgs::msg::TrafficLightRoiArray;

public:
  explicit TrafficLightFineDetectorNodelet(const rclcpp::NodeOptions & options);
  void connectCb();
  /**
   * @brief main process function.
   *
   * @param image_msg        ros image message
   * @param rough_roi_msg    rough rois message
   * @param expect_roi_msg   expect rois message
   */
  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    const TrafficLightRoiArray::ConstSharedPtr rough_roi_msg,
    const TrafficLightRoiArray::ConstSharedPtr expect_roi_msg);

private:
  /**
   * @brief convert the images into data stream ready for gpu process
   *
   * @param in_imgs    cv::Mat images
   * @param num_rois   number of rois
   * @param data       output data stream
   * @return true      succeed
   * @return false     failed
   */
  bool cvMat2CnnInput(
    const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<float> & data);
  /**
   * @brief Calculate the match score. Details will be explained in docs of evalMatchScore
   *
   * @param id2expectRoi
   * @param id2detections
   * @param id2bestDetection
   * @return float
   */
  float evalMatchScore(
    std::map<int, TrafficLightRoi> & id2expectRoi,
    std::map<int, tensorrt_yolox::ObjectArray> & id2detections,
    std::map<int, tensorrt_yolox::Object> & id2bestDetection);
  /**
   * @brief Every traffic light roi might have several possible detections. This function
   * is designed to select the best detection for every traffic light by making use of
   * the relative positions between the traffic lights projected on the image (expect/rois).
   * To be specified, for every detection, all the expect rois will be transferred so that
   * this detection will match the corresponding expect roi. Note that the expect rois
   * of other traffic lights will also be transferred equally. Then, for every expect roi,
   * it will calculate the match score (which is IoU_detection_roi * detection_confidence)
   * with every detection.
   * The combination of detections that will get highest match score sum will be the selected
   * detections
   *
   * @param id2expectRoi    id to expect/roi map
   * @param id2detections   id to detections map
   * @param out_rois        output rois converted from the selected detections
   */
  void detectionMatch(
    std::map<int, TrafficLightRoi> & id2expectRoi,
    std::map<int, tensorrt_yolox::ObjectArray> & id2detections, TrafficLightRoiArray & out_rois);

  /**
   * @brief convert image message to cv::Mat
   *
   * @param image_msg   input image message
   * @param image       output cv::Mat image
   * @param encode      image encode
   * @return true       succeed
   * @return false      failed
   */
  bool rosMsg2CvMat(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image,
    std::string encode = "rgb8");
  bool fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size);
  /**
   * @brief Read the label file to get class number and traffic_light class index of the model
   *
   * @param filepath   path to the label file
   * @param tlr_id     output traffic light class index
   * @param num_class  output class number
   * @return true      succeed
   * @return false     failed
   */
  bool readLabelFile(
    const std::string & filepath, std::vector<int> & tlr_label_id_, int & num_class);

  // variables
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> rough_roi_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> expect_roi_sub_;
  std::mutex connect_mutex_;
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr output_roi_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, TrafficLightRoiArray, TrafficLightRoiArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, TrafficLightRoiArray, TrafficLightRoiArray>
    ApproximateSyncPolicy;
  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  bool is_approximate_sync_;
  double score_thresh_;
  std::vector<int> tlr_label_id_;

  int batch_size_;
  std::unique_ptr<tensorrt_yolox::TrtYoloX> trt_yolox_;
};  // TrafficLightFineDetectorNodelet

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_FINE_DETECTOR__NODELET_HPP_
