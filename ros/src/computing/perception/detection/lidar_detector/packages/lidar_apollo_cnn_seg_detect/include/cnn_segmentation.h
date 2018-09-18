#ifndef CNN_SEGMENTATION_H
#define CNN_SEGMENTATION_H

#include <ros/ros.h>

#include "caffe/caffe.hpp"

// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "cluster2d.h"
#include "feature_generator.h"
// #include "pcl_types.h"
// #include "modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.h"

struct CellStat {
  CellStat() : point_num(0), valid_point_num(0) {}
  int point_num;
  int valid_point_num;
};

class CNNSegmentation
{
public:
  CNNSegmentation();
  void run();
  void test_run();
private:

  float range_;
  int width_;
  int height_;

  // nodehandle
  ros::NodeHandle nh_;

  // publisher
  ros::Publisher points_pub_;

  // subscriber
  ros::Subscriber points_sub_;

  std::shared_ptr<caffe::Net<float>> caffe_net_;

  // center offset prediction
  boost::shared_ptr<caffe::Blob<float>> instance_pt_blob_;
  // objectness prediction
  boost::shared_ptr<caffe::Blob<float>> category_pt_blob_;
  // fg probability prediction
  boost::shared_ptr<caffe::Blob<float>> confidence_pt_blob_;
  // object height prediction
  boost::shared_ptr<caffe::Blob<float>> height_pt_blob_;
  // raw features to be input into network
  boost::shared_ptr<caffe::Blob<float>> feature_blob_;
  // class prediction
  boost::shared_ptr<caffe::Blob<float>> class_pt_blob_;

  // clustering model for post-processing
  std::shared_ptr<Cluster2D> cluster2d_;

  // bird-view raw feature generator
  std::shared_ptr<FeatureGenerator> feature_generator_;


  // pcl::PointCloud<pcl::PointXYZI> points_;

  // void pointsCallback(const sensor_msgs::PointCloud2& msg);
  bool init();
  bool segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr,
                                const pcl::PointIndices& valid_idx,
                                autoware_msgs::DetectedObjectArray* objects);

  void pointsCallback(const sensor_msgs::PointCloud2& msg);
  void pubColoredPoints(const autoware_msgs::DetectedObjectArray& objects);

  // void drawDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr,
  //                    const pcl::PointIndices& valid_idx,
  //                    int rows, int cols, float range,
  //                    const autoware_msgs::DetectedObjectArray& objects,
  //                    const std::string &result_file);
};

#endif //CNN_SEGMENTATION_H
