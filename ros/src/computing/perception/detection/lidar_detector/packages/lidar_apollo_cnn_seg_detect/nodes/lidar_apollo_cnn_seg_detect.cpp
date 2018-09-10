#include "lidar_apollo_cnn_seg_detect.h"

#include <pcl/io/pcd_io.h>

ApolloCNNSeg::ApolloCNNSeg()
{
  a_ = 0;
  points_sub_ = nh_.subscribe("/points_raw", 1, &ApolloCNNSeg::pointsCallback, this);
  pcl::PointCloud<pcl::PointXYZI> cloud;

  double max_x = -999;
  double min_x = 999;
  double max_y = -999;
  double min_y = 999;
  pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/kosuke/apollo/modules/perception/data/cnnseg_test/uscar_12_1470770225_1470770492_1349.pcd", cloud);
  for(size_t i = 0; i < cloud.size(); i++)
  {
    double x = cloud[i].x;
    if(x > max_x)
    {
      max_x = x;
    }
    else if(x < min_x)
    {
      min_x = x;
    }

    double y = cloud[i].y;
    if(y > max_y)
    {
      max_y = y;
    }
    else if(y < min_y)
    {
      min_y = y;
    }
  }

  std::cout << "max x " << max_x << std::endl;
  std::cout << "min x " << min_x << std::endl;
  std::cout << "max y " << max_y << std::endl;
  std::cout << "min y " << min_y << std::endl;
}

void ApolloCNNSeg::run()
{
  std::cout << "a_ " << a_ << std::endl;
  ros::Rate rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void ApolloCNNSeg::pointsCallback(const sensor_msgs::PointCloud2& msg)
{
  pcl::fromROSMsg(msg, points_);
  pcl::io::savePCDFileASCII ("/home/kosuke/test_pcd.pcd", points_);
}
