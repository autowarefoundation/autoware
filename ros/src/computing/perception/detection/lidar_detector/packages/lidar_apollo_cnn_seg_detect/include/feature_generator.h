#ifndef FEATURE_GENERATOR_H
#define FEATURE_GENERATOR_H

#include "caffe/caffe.hpp"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "util.h"



class  FeatureGenerator
{
public:
  FeatureGenerator(){}
  ~FeatureGenerator(){}

  bool init(caffe::Blob<float>* out_blob);
  void generate(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr);
private:
  int width_ = 0;
  int height_ = 0;
  int range_ = 0;

  float min_height_ = 0.0;
  float max_height_ = 0.0;

  // raw feature data
  float* max_height_data_ = nullptr;
  float* mean_height_data_ = nullptr;
  float* count_data_ = nullptr;
  float* direction_data_ = nullptr;
  float* top_intensity_data_ = nullptr;
  float* mean_intensity_data_ = nullptr;
  float* distance_data_ = nullptr;
  float* nonempty_data_ = nullptr;

  // output Caffe blob
  caffe::Blob<float>* out_blob_ = nullptr;

  std::vector<float> log_table_;

  // point index in feature map
  std::vector<int> map_idx_;

  float logCount(int count);

};

#endif //FEATURE_GENERATOR_H
