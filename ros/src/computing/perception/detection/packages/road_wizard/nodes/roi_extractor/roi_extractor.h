#ifndef ROI_EXTRACTOR_H
#define ROI_EXTRACTOR_H

#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include "Context.h"
#include "road_wizard/Signals.h"

class RoiExtractor {
 public:
  explicit RoiExtractor(){};
  ~RoiExtractor(){};

  // Callback functions to obtain images and signal position
  void ImageRawCallback(const sensor_msgs::Image &image);
  void RoiSignalCallback(const road_wizard::Signals::ConstPtr &extracted_pos);
  
  // Utility function to create directory which roi images will be saved
  void CreateTargetDirectory(std::string base_name);

 private:
  // Utility function to count the number of files contained in the specified directory
  int CountFileNum(std::string directory_name);

  // Utility function to create directory tree
  void MakeDirectoryTree(const std::string &target, const std::string &base, const mode_t &mode);

  // Directory path that extracted ROI images will be saved
  std::string target_directory_;

  // One subscribed frame image
  cv::Mat frame_;

  // Time stamp value of subscribed value
  ros::Time frame_timestamp_;
  ros::Time previous_timestamp_;

  // The number of files contained in the target directory
  int file_count_;
};

#endif // ROI_EXTRACTOR_H
