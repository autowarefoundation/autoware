#include "cnn_segmentation.h"

#include <caffe/caffe.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_apollo_cnn_seg_detect");
  CNNSegmentation node;
  node.run();
  ros::spin();

  return 0;
}
