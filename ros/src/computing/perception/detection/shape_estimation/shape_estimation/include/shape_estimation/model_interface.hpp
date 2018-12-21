#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "autoware_msgs/DetectedObject.h"

class ShapeEstimationModelInterface
{
  public:
    ShapeEstimationModelInterface(){};
    virtual ~ShapeEstimationModelInterface(){};
    virtual bool estimate(const pcl::PointCloud<pcl::PointXYZ> &cluster, autoware_msgs::DetectedObject &ouput) = 0;
};