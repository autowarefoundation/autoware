#pragma once

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "autoware_msgs/DetectedObject.h"

class ShapeEstimater
{
  private:

  public:
    ShapeEstimater();
    ~ShapeEstimater(){};
    bool getShapeAndPose(const std::string &label, const pcl::PointCloud<pcl::PointXYZ> &cluster, autoware_msgs::DetectedObject &output);
};