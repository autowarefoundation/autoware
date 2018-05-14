#ifndef OBJECT_TRACKING_VisualizeDetecedObjects_H
#define OBJECT_TRACKING_VisualizeDetecedObjects_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/io/io.h>

#include <vector>
#include <string>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

class VisualizeDetectedObjects
{
private:
  std::string input_topic_;
  std::string pointcloud_frame_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_cloud_array_;

  ros::Publisher pub_arrow_;
  ros::Publisher pub_id_;

  void visMarkers(const autoware_msgs::DetectedObjectArray& input);
  void callBack(const autoware_msgs::DetectedObjectArray& input);

public:
  VisualizeDetectedObjects();
};

#endif  // OBJECT_TRACKING_VisualizeCloudCluster_H
