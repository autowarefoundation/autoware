#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class ApolloCNNSeg
{
public:
  ApolloCNNSeg();
  void run();
private:
  int a_;

  // nodehandle
  ros::NodeHandle nh_;

  ros::Subscriber points_sub_;
  pcl::PointCloud<pcl::PointXYZI> points_;

  void pointsCallback(const sensor_msgs::PointCloud2& msg);
};
