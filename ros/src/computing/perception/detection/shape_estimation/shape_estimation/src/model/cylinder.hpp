#pragma once
#include "shape_estimation/model_interface.hpp"

class CylinderModel : public ShapeEstimationModelInterface
{
public:
    CylinderModel(){};
    ~CylinderModel(){};
    bool estimate(const pcl::PointCloud<pcl::PointXYZ> &cluster, autoware_msgs::DetectedObject &output) override;
};