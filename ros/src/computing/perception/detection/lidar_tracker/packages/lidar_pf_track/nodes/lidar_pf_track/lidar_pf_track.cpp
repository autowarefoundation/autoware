/*
 * pf_track.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: ne0
 */

#include "ros/ros.h"

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

class PfTrack {
public:
  PfTrack();

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber cloud_clusters_sub_;
  ros::Publisher detected_objects_pub_;

  void CloudClustersCallback(
      const autoware_msgs::CloudClusterArray::Ptr &in_cloud_cluster_array_ptr);
};

PfTrack::PfTrack() : node_handle_("~") {
  cloud_clusters_sub_ = node_handle_.subscribe(
      "/cloud_clusters_class", 10, &PfTrack::CloudClustersCallback, this);
  detected_objects_pub_ =
      node_handle_.advertise<autoware_msgs::DetectedObjectArray>(
          "/detected_objects", 10);
}

void PfTrack::CloudClustersCallback(
    const autoware_msgs::CloudClusterArray::Ptr &in_cloud_cluster_array_ptr) {
  autoware_msgs::DetectedObjectArray detected_objects;
  detected_objects.header = in_cloud_cluster_array_ptr->header;
  for (auto i = in_cloud_cluster_array_ptr->clusters.begin();
       i != in_cloud_cluster_array_ptr->clusters.end(); i++) {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = i->header;
    detected_object.id = i->id;
    detected_object.label = i->label;
    detected_object.dimensions = i->bounding_box.dimensions;
    detected_object.pose = i->bounding_box.pose;

    detected_objects.objects.push_back(detected_object);
  }
  detected_objects_pub_.publish(detected_objects);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "lidar_pf_detect");
  PfTrack node;
  ros::spin();

  return 0;
}
