#include "shape_estimation/shape_estimater.hpp"
#include "shape_estimation/model_interface.hpp"
#include "model/bounding_box.hpp"
#include "model/convex_hull.hpp"
#include "model/cylinder.hpp"
#include <memory>
#include <iostream>

ShapeEstimater::ShapeEstimater() {}
bool ShapeEstimater::getShapeAndPose(const std::string &label,
                                     const pcl::PointCloud<pcl::PointXYZ> &cluster,
                                     autoware_msgs::DetectedObject &output)
{
    std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
    if (label == "car" || label == "vehicle" || label == "truck" || label == "bus")
    {
        model_ptr.reset(new BoundingBoxModel);
    }
    else if (label == "person")
    {
        model_ptr.reset(new CylinderModel);
    }
    else if (label == "motorbike")
    {
        model_ptr.reset(new BoundingBoxModel);
    }
    else if (label == "bicycle")
    {
        model_ptr.reset(new BoundingBoxModel);
    }
    else
    {
//        model_ptr.reset(new CylinderModel);
        model_ptr.reset(new BoundingBoxModel);
    };

    model_ptr->estimate(cluster, output);

    return true;
}
