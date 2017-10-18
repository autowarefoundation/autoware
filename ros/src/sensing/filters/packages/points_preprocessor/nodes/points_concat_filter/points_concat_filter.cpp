/*
 * fusion_filter.cpp
 *
 * Created on	: September 4, 2017
 * Author	: Akihito OHSATO
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class PointsConcatFilter
{
public:
  PointsConcatFilter();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudT, PointCloudT> SyncPolicy;
  ros::NodeHandle nh_, pnh_;
  message_filters::Subscriber<PointCloudT> *sub1_, *sub2_;
  message_filters::Synchronizer<SyncPolicy>* sync_;
  ros::Publisher pub_;
  tf::TransformListener tfl;
  std::string sensor_frame;
  void callback(const PointCloudT::ConstPtr& msg1, const PointCloudT::ConstPtr& msg2);
};

PointsConcatFilter::PointsConcatFilter() : nh_(), pnh_("~"), tfl(), sensor_frame("base_link")
{
  pnh_.param("sensor_frame", sensor_frame, sensor_frame);
  sub1_ = new message_filters::Subscriber<PointCloudT>(nh_, "/lidar0/points_raw", 1);
  sub2_ = new message_filters::Subscriber<PointCloudT>(nh_, "/lidar1/points_raw", 1);
  sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub1_, *sub2_);
  sync_->registerCallback(boost::bind(&PointsConcatFilter::callback, this, _1, _2));
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_concat", 1);
}

void PointsConcatFilter::callback(const PointCloudT::ConstPtr& msg1, const PointCloudT::ConstPtr& msg2)
{
  PointCloudT::Ptr cloud1(new PointCloudT);
  PointCloudT::Ptr cloud2(new PointCloudT);
  PointCloudT::Ptr cloud(new PointCloudT);
  // transform points
  try
  {
    tfl.waitForTransform(sensor_frame, msg1->header.frame_id, ros::Time(0), ros::Duration(1.0));
    pcl_ros::transformPointCloud(sensor_frame, *msg1, *cloud1, tfl);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  try
  {
    tfl.waitForTransform(sensor_frame, msg2->header.frame_id, ros::Time(0), ros::Duration(1.0));
    pcl_ros::transformPointCloud(sensor_frame, *msg2, *cloud2, tfl);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  // merge points
  *cloud += *cloud1;
  *cloud += *cloud2;
  cloud->header = msg1->header;
  cloud->header.frame_id = sensor_frame;
  pub_.publish(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_concat_filter");
  PointsConcatFilter node;
  ros::spin();
  return 0;
}
