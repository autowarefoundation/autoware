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

  // Reject image if its height is smaller than threshold
  if (roi.size().height < k_minimum_height_) {
    return;
  }

  // Reject image if its similarity level with previous saved ROI is higher than threshold 
  if (k_similarity_threshold_ < CalculateSimilarity(roi, previous_saved_frame_)) {
    return;
  }

  cv::imwrite(file_name.c_str(), roi);
  file_count_++;
  
  previous_timestamp_ = frame_timestamp_;
  previous_saved_frame_ = roi.clone();
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


// calculae similarity of specified two images
// by comparing their histogram, which is sensitive filter for color
double RoiExtractor::CalculateSimilarity(const cv::Mat &image1, const cv::Mat &image2) {
  if (image1.empty() || image2.empty()) {
    return 0.0;
  }

  // Compare by histogram
  cv::Mat image1_hsv, image2_hsv;
  cv::cvtColor(image1, image1_hsv, CV_BGR2HSV);
  cv::cvtColor(image2, image2_hsv, CV_BGR2HSV);

  const int channel[] = {0};

  // Hue range in OpenCV is 0 to 180
  const float hue_ranges[] = {0, 180};
  const float* ranges[] = {hue_ranges};

  // Quantize hue value into 6
  int hist_size[] = {6};

  cv::Mat histogram1;
  cv::calcHist(&image1_hsv,
               1,               // Use this image only to create histogram
               channel,
               cv::Mat(),       // No mask is used
               histogram1,
               1,               // The dimension of histogram is 1
               hist_size,
               ranges);

   cv::Mat histogram2;
   cv::calcHist(&image2_hsv,
                1,              // Use this image only to create histogram
                channel,
                cv::Mat(),      // No mask is used
                histogram2,
                1,              // The dimension of histogram is 1
                hist_size,
                ranges);

   double similarity = cv::compareHist(histogram1, histogram2, CV_COMP_CORREL);

   return similarity;
} // void RoiExtractor::CalculateSimilarity()


// Entry Point of this node
int main (int argc, char *argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "roi_extractor");

  // Get source topic name of image from ROS private parameter
  ros::NodeHandle private_node_handler;
  std::string image_topic_name;
  std::string target_directory_name = std::string(getenv("HOME")) + "/.autoware";
  int minimum_height = 32;
  double similarity_threshold = 0.9;
  private_node_handler.param<std::string>("image_raw_topic", image_topic_name, "/image_raw");
  private_node_handler.param<std::string>("target_directory", target_directory_name, target_directory_name);
  private_node_handler.param<int>("minimum_height", minimum_height, 32); // The default minimum height is 32
  private_node_handler.param<double>("similarity_threshold", similarity_threshold, 0.9); // The default similarity threshold is 0.9

  // Get directory name which roi images will be saved
  RoiExtractor extractor(minimum_height, similarity_threshold);
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
