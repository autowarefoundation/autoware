#pragma once
#include "shape_estimation/model_interface.hpp"

class BoundingBoxModel : public ShapeEstimationModelInterface
{
  private:
    double calcClosenessCriterion(const std::vector<double> &C_1, const std::vector<double> &C_2);

  public:
    BoundingBoxModel(){};
    ~BoundingBoxModel(){};
    bool estimate(const pcl::PointCloud<pcl::PointXYZ> &cluster, autoware_msgs::DetectedObject &output) override;
};