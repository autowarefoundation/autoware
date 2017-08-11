#ifndef REGION_TLR_SSD_H
#define REGION_TLR_SSD_H

#include <string>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "Context.h"
#include "autoware_msgs/Signals.h"
#include "traffic_light_recognizer.h"

class RegionTlrSsdRosNode {
 public:
  RegionTlrSsdRosNode();
  ~RegionTlrSsdRosNode();

  void RunRecognition();
  void ImageRawCallback(const sensor_msgs::Image &image);
  void RoiSignalCallback(const autoware_msgs::Signals::ConstPtr &extracted_pos);

  // The vector of data structure to save traffic light state, position, ...etc
  std::vector<Context> contexts_;

 private:
  /* Light state transition probably happen in Japanese traffic light */
  const LightState kStateTransitionMatrix[4][4] = {
    /* current: */
    /* GREEN   , YELLOW    , RED    , UNDEFINED  */
    /* -------------------------------------------  */
    {GREEN     , YELLOW    , YELLOW    , GREEN}  ,  /* | previous = GREEN */
    {UNDEFINED , YELLOW    , RED       , YELLOW} ,  /* | previous = YELLOW */
    {GREEN     , RED       , RED       , RED}    ,  /* | previous = RED */
    {GREEN     , YELLOW    , RED       , UNDEFINED} /* | previous = UNDEFINED */
  };


  void GetRosParam();
  void StartSubscribersAndPublishers();
  LightState DetermineState(LightState previous_state, LightState current_state, int* state_judge_count);
  void PublishTrafficLight(std::vector<Context> contexts);
  void PublishString(std::vector<Context> contexts);
  void PublishMarkerArray(std::vector<Context> contexts);
  void PublishImage(std::vector<Context> contexts);

  // Execution parameter
  std::string image_topic_name_;
  std::string network_definition_file_name_;
  std::string pretrained_model_file_name_;
  bool use_gpu_;
  int gpu_id_;

  // Subscribers
  ros::Subscriber image_subscriber;
  ros::Subscriber roi_signal_subscriber;

  // Publishers
  ros::Publisher signal_state_publisher;
  ros::Publisher signal_state_string_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher superimpose_image_publisher;

  // Flag to show topic will be published in latch manner
  bool kAdvertiseInLatch_;

  // A frame image acquired from topic
  cv::Mat frame_;

  // Timestamp of a frame in process
  std_msgs::Header frame_header_;

  // The instance of the core class of traffic light recognition by SSD
  TrafficLightRecognizer recognizer;

  // The threshold of state detected times to accept the state change
  const int kChangeStateThreshold = 10;

  // constant values to pass recognition states to other nodes
  const int32_t kTrafficLightRed;
  const int32_t kTrafficLightGreen;
  const int32_t kTrafficLightUnknown;
  const std::string kStringRed;
  const std::string kStringGreen;
  const std::string kStringUnknown;
};

#endif  // REGION_TLR_SSD_H
