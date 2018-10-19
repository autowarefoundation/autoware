#include "cnn_segmentation.h"

// #include "modules/perception/obstacle/lidar/segmentation/cnnseg/proto/cnnseg.pb.h"

#include <caffe/caffe.hpp>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "apollo_lidar_cnn");
  std::cout << "111" << std::endl;
  CNNSegmentation node;
  node.run();
  ros::spin();

  return 0;
}
