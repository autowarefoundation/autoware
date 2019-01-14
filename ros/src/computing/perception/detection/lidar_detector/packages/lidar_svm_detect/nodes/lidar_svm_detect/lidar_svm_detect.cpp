/*
 * svm_detect.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: ne0
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <tf/tf.h>

#include <stdio.h>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#if (CV_MAJOR_VERSION != 3)
#include <opencv2/contrib/contrib.hpp>
#endif

class SvmDetect {
public:
  SvmDetect();
  ~SvmDetect();
  void Run();

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber cloud_clusters_sub_;
  ros::Publisher cloud_clusters_pub_;
  ros::Publisher text_pictogram_pub_;

  std::string model_file_path_;

  FILE *model_file_handle_;

  void CloudClustersCallback(
      const autoware_msgs::CloudClusterArray::Ptr &in_cloud_cluster_array_ptr);
  void ClassifyFpfhDescriptor(const std::vector<float> &in_fpfh_descriptor,
                              double &out_label,
                              std::vector<double> &out_scores,
                              double &out_sum_scores);

  void CloseModel();
};

void SvmDetect::CloseModel() { fclose(model_file_handle_); }

SvmDetect::~SvmDetect() {}

SvmDetect::SvmDetect() : node_handle_("~") {}

void SvmDetect::Run() {
  ros::NodeHandle private_node_handle("~");
  std::string clusters_node_name,
      out_clusters_topic_name = "/cloud_clusters_class",
      out_pictograms_topic_name = "/pictogram_clusters_class";

  private_node_handle.param<std::string>("svm_model_file_path",
                                         model_file_path_, "models/svm.model");
  ROS_INFO("svm_model_file_path: %s", model_file_path_.c_str());
  private_node_handle.param<std::string>("clusters_node_name",
                                         clusters_node_name, "/cloud_clusters");
  ROS_INFO("clusters_node_name: %s", clusters_node_name.c_str());

  cloud_clusters_sub_ = node_handle_.subscribe(
      clusters_node_name, 10, &SvmDetect::CloudClustersCallback, this);
  cloud_clusters_pub_ =
      node_handle_.advertise<autoware_msgs::CloudClusterArray>(
          out_clusters_topic_name, 10);
  ROS_INFO("output clusters topic: %s", out_clusters_topic_name.c_str());

  /*text_pictogram_pub_ =
  node_handle_.advertise<jsk_rviz_plugins::PictogramArray>(out_pictograms_topic_name,
  10); ROS_INFO("output pictograms topic: %s",
  out_pictograms_topic_name.c_str());

  model_ptr_ = LoadSvmModel(model_file_path_);

  if(model_ptr_ == NULL)
  {
          ROS_INFO("SvmDetect. Cannot perform classification. Invalid model
  file.");
  }
  else
  {
          ROS_INFO("SvmDetect. Ready, waiting for clusters...");
  }*/
  ros::spin();
}

void SvmDetect::CloudClustersCallback(
    const autoware_msgs::CloudClusterArray::Ptr &in_cloud_cluster_array_ptr) {
  cloud_clusters_pub_.publish(*in_cloud_cluster_array_ptr);
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "svm_lidar_detect");

  SvmDetect node;

  node.Run();

  return 0;
}
