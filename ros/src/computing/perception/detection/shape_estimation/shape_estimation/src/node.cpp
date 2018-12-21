#include "shape_estimation/node.hpp"
#include "shape_estimation/shape_estimater.hpp"

ShapeEstimationNode::ShapeEstimationNode() : nh_(""), pnh_("~")
{
    sub_ = nh_.subscribe("input", 1, &ShapeEstimationNode::callback, this);
    pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("output", 1, true);
}

void ShapeEstimationNode::callback(const autoware_msgs::DetectedObjectArray::ConstPtr &input_msg)
{
    // Guard
    if (pub_.getNumSubscribers() < 1)
        return;

    // Create output msg
    auto output_msg = *input_msg;

    // Estimate shape for each object and pack msg
    for (auto &object : output_msg.objects)
    {
        // convert ros to pcl
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(object.pointcloud, *cluster);
        // estimate shape and pose
        estimater_.getShapeAndPose(object.label, *cluster, object);
    }

    // Publish
    pub_.publish(output_msg);
    return;
}
