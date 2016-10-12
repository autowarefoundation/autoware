#include "roi_extractor.h"

#include <sys/stat.h>
#include <dirent.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


#include "Context.h"
#include "road_wizard/Signals.h"


void RoiExtractor::ImageRawCallback(const sensor_msgs::Image &image) {
  // Acquire frame image from ros topic
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  frame_ = cv_image->image.clone();

  // Save this topic's time stamp so that same image will not be processed more than twice
  frame_timestamp_ = image.header.stamp;
} // void RoiExtractor::ImageRawCallback()


void RoiExtractor::RoiSignalCallback(const road_wizard::Signals::ConstPtr &extracted_pos) {
  // If frame image has not been updated, do nothing
  if (frame_timestamp_ == previous_timestamp_) {
    return;
  }

  // Aquire signal positions from ros topic
  std::vector<Context> signal_positions;
  Context::SetContexts(signal_positions, extracted_pos, frame_.rows, frame_.cols);

  if (signal_positions.size() == 0) {
    // If signal_positions is empty, no ROI images should be saved
    return;
  }

  // Extract ROI for top signal in vector (top signal has largest estimated radius in every signals projected in a image)
  cv::Mat roi = frame_(cv::Rect(signal_positions.at(0).topLeft, signal_positions.at(0).botRight));
  std::string file_name = target_directory_ + std::to_string(file_count_) + ".png";

  cv::imwrite(file_name.c_str(), roi);
  file_count_++;
  
  previous_timestamp_ = frame_timestamp_;
} // void RoiExtractor::RoiSignalCallback()


void RoiExtractor::CreateTargetDirectory(std::string base_name) {
  // Extracted ROI's images will be saved in "[base_name]/tlr_TrainingDataSet/Images"
  std::string target_directory_name = base_name + "/tlr_TrainingDataSet/Images/";
  
  // Create target directory newly if it doesn't exist
  struct stat directory_info;
  if (stat(target_directory_name.c_str(), &directory_info) != 0) {
    MakeDirectoryTree(target_directory_name, base_name, 0755);
  }

  // Count the number of files contained in the target directory
  // so that saved file is named in continuous number
  file_count_ = CountFileNum(target_directory_name);


  // Save directory name into class member
  target_directory_ = target_directory_name;

} // void RoiExtractor::CreateTargetDirectory


int RoiExtractor::CountFileNum(std::string directory_name) {
  int file_num = 0;
  struct dirent *entry;
  DIR *directory_handler = opendir(directory_name.c_str());

  // Count the number of files contained in the specified directory
  while ((entry = readdir(directory_handler)) != NULL) {
    struct stat status;
    std::string absolute_path = directory_name + std::string(entry->d_name);
    if (stat(absolute_path.c_str(), &status) == 0 &&
        S_ISREG(status.st_mode)) {
      file_num++;
    }
  }

  closedir(directory_handler);

  return file_num;
} //int RoiExtractor::CountFileNum()


void RoiExtractor::MakeDirectoryTree(const std::string &target,
                                     const std::string &base,
                                     const mode_t &mode) {
  // Extract directory subtree structure
  std::string sub_tree = target.substr(base.size());

  // Create directory tree one by one
  size_t separator_start = sub_tree.find("/");
  size_t separator_end = sub_tree.find("/", separator_start + 1);
  std::string path = base;
  while (separator_end != std::string::npos) {
    std::string sub_directory = sub_tree.substr(separator_start, separator_end);
    path = path + sub_directory;
    mkdir(path.c_str(), mode);
    separator_start = separator_end;
    separator_end = sub_tree.find("/", separator_start + 1);
  }
} // void RoiExtractor::MakeDirectoryTree()

// Entry Point of this node
int main (int argc, char *argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "roi_extractor");

  // Get source topic name of image from ROS private parameter
  ros::NodeHandle private_node_handler;
  std::string image_topic_name;
  std::string target_directory_name = std::string(getenv("HOME")) + "/.autoware";
  private_node_handler.param<std::string>("image_raw_topic", image_topic_name, "/image_raw");
  private_node_handler.param<std::string>("target_directory", target_directory_name, target_directory_name);

  // Get directory name which roi images will be saved
  RoiExtractor extractor;
  extractor.CreateTargetDirectory(target_directory_name);

  // Launch callback function to subscribe images and signal position
  ros::NodeHandle node_handler;
  ros::Subscriber image_subscriber = node_handler.subscribe(image_topic_name,
                                                            1,
                                                            &RoiExtractor::ImageRawCallback,
                                                            &extractor);

  ros::Subscriber roi_signal_subscriber = node_handler.subscribe("/roi_signal",
                                                                 1,
                                                                 &RoiExtractor::RoiSignalCallback,
                                                                 &extractor);
  
  ros::spin();

  return 0;
}
