
#include "visualize_cloud_cluster.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_cloud_cluster");
  VisualizeCloudCluster app;
  ros::spin();

  return 0;
}
