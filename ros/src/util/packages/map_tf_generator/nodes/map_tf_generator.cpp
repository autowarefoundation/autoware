#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void Callback(const PointCloud::ConstPtr &clouds) {
  const unsigned int sum = clouds->points.size();
  double coordinate[3] = {0, 0, 0};
  for (int i = 0; i < sum; i++) {
    coordinate[0] += clouds->points[i].x;
    coordinate[1] += clouds->points[i].y;
    coordinate[2] += clouds->points[i].z;
  }
  coordinate[0] = -1 * coordinate[0] / sum;
  coordinate[1] = -1 * coordinate[1] / sum;
  coordinate[2] = -1 * coordinate[2] / sum;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(coordinate[0], coordinate[1], coordinate[2]));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_tf_generator");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<PointCloud>("/points_map", 1, &Callback);
  ros::spin();

  return 0;
};
