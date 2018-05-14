#include <float.h>
#include <math.h>

#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

ros::Publisher tracked_pub;
ros::Publisher tracked_bba_pub;
ros::Publisher tracked_bba_textlabel_pub;

static std::vector<autoware_msgs::CloudCluster> v_pre_cloud_cluster;
static double threshold_dist;

static double euclid_distance(const geometry_msgs::Point pos1,
                              const geometry_msgs::Point pos2) {
  return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) +
              pow(pos1.z - pos2.z, 2));

} /* static double distance() */

void pos_stamped2pos(geometry_msgs::PointStamped in_pos,
                     geometry_msgs::Point &out_pos) {
  out_pos = in_pos.point;
}

void cluster_cb(
    const autoware_msgs::CloudClusterArray::Ptr &cloud_cluster_array_ptr) {

  autoware_msgs::CloudClusterArray base_msg = *cloud_cluster_array_ptr;
  static int id = 1;

  for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
    geometry_msgs::Point cluster_centroid;
    double min_distance = DBL_MAX;
    bool stamped_id = false;
    pos_stamped2pos(base_msg.clusters.at(i).centroid_point, cluster_centroid);
    for (int j(0); j < (int)v_pre_cloud_cluster.size(); ++j) {
      geometry_msgs::Point pre_cluster_centroid;
      pos_stamped2pos(v_pre_cloud_cluster.at(j).centroid_point,
                      pre_cluster_centroid);
      double distance = euclid_distance(cluster_centroid, pre_cluster_centroid);
      if (distance < min_distance && distance < threshold_dist) {
        min_distance = distance;
        base_msg.clusters.at(i).id = v_pre_cloud_cluster.at(j).id;
        stamped_id = true;
      }
    }
    if (!stamped_id) {
      base_msg.clusters.at(i).id = id;
      ++id;
      if (id > 100)
        id = 1;
    }
  }

  v_pre_cloud_cluster.clear();
  for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
    autoware_msgs::CloudCluster cloud_cluster = base_msg.clusters.at(i);
    v_pre_cloud_cluster.push_back(cloud_cluster);
  }

  autoware_msgs::DetectedObjectArray detected_objects_msg;
  detected_objects_msg.header = base_msg.header;
  for (auto i = base_msg.clusters.begin(); i != base_msg.clusters.end(); i++) {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = i->header;
    detected_object.id = i->id;
    detected_object.label = i->label;
    detected_object.dimensions = i->bounding_box.dimensions;
    detected_object.pose = i->bounding_box.pose;
    detected_objects_msg.objects.push_back(detected_object);
  }

  tracked_pub.publish(detected_objects_msg);

  jsk_recognition_msgs::BoundingBoxArray pub_bb_msg;
  pub_bb_msg.header = base_msg.header;
  for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
    pub_bb_msg.boxes.push_back(base_msg.clusters.at(i).bounding_box);
    pub_bb_msg.boxes.at(i).value = base_msg.clusters.at(i).id;
  }
  tracked_bba_pub.publish(pub_bb_msg);

  visualization_msgs::MarkerArray pub_textlabel_msg;
  std_msgs::ColorRGBA color_white;
  color_white.r = 1.0f;
  color_white.g = 1.0f;
  color_white.b = 1.0f;
  color_white.a = 1.0f;
  for (int i(0); i < (int)base_msg.clusters.size(); ++i) {
    visualization_msgs::Marker marker_textlabel;
    marker_textlabel.header.frame_id = "velodyne";
    marker_textlabel.ns = "cluster";
    marker_textlabel.id = i;
    /* Set the text label */
    marker_textlabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_textlabel.scale.z = 1.0;
    marker_textlabel.text = base_msg.clusters.at(i).label + "@" +
                            std::to_string(base_msg.clusters.at(i).id);
    marker_textlabel.pose.position =
        base_msg.clusters.at(i).bounding_box.pose.position;
    marker_textlabel.pose.orientation.x = 0.0;
    marker_textlabel.pose.orientation.y = 0.0;
    marker_textlabel.pose.orientation.y = 0.0;
    marker_textlabel.pose.orientation.w = 0.0;
    marker_textlabel.pose.position.z += 1.5;
    marker_textlabel.color = color_white;
    marker_textlabel.lifetime = ros::Duration(0.2);
    pub_textlabel_msg.markers.push_back(marker_textlabel);
  }
  tracked_bba_textlabel_pub.publish(pub_textlabel_msg);

} /* void cluster_cb() */

int main(int argc, char *argv[]) {
  /* ROS initialization */
  ros::init(argc, argv, "euclidean_lidar_track");

  ros::NodeHandle n;
  ros::NodeHandle private_n("~");

  if (!private_n.getParam("threshold_dist", threshold_dist)) {
    threshold_dist = 2.0;
  }
  ros::Subscriber cluster_centroids_sub =
      n.subscribe("/cloud_clusters_class", 1, cluster_cb);
  tracked_pub =
      n.advertise<autoware_msgs::DetectedObjectArray>("/detected_objects", 1);
  tracked_bba_pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>(
      "/cloud_cluster_tracked_bounding_box", 1);
  tracked_bba_textlabel_pub = n.advertise<visualization_msgs::MarkerArray>(
      "/cloud_cluster_tracked_text", 1);
  ros::spin();

  return 0;
}
