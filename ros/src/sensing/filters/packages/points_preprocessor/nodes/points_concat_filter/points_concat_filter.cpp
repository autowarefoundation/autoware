/*
 * fusion_filter.cpp
 *
 * Created on   : September 4, 2017
 * Author       : Akihito OHSATO
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
#include "autoware_msgs/ConfigPointsConcatFilter.h"

class PointsConcatFilter
{
public:
  PointsConcatFilter();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT> SyncPolicyT;

  ros::NodeHandle node_handle_, private_node_handle_;
  message_filters::Subscriber<PointCloudMsgT> *cloud_subscriber1_, *cloud_subscriber2_;
  message_filters::Synchronizer<SyncPolicyT>* cloud_synchronizer_;
  ros::Subscriber config_subscriber_;
  ros::Publisher cloud_publisher_;
  tf::TransformListener tf_listener_;
  std::string output_frame_;

  void config_callback(const autoware_msgs::ConfigPointsConcatFilter::ConstPtr& config_msg);
  void pointcloud_callback(const PointCloudMsgT::ConstPtr& cloud_msg1, const PointCloudMsgT::ConstPtr& cloud_msg2);
};

PointsConcatFilter::PointsConcatFilter()
  : node_handle_(), private_node_handle_("~"), tf_listener_(), output_frame_("lidar_frame")
{
  private_node_handle_.param("output_frame", output_frame_, output_frame_);
  config_subscriber_ = node_handle_.subscribe<autoware_msgs::ConfigPointsConcatFilter>(
      "/config/points_concat_filter", 1, &PointsConcatFilter::config_callback, this);
  cloud_subscriber1_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, "/lidar0/points_raw", 1);
  cloud_subscriber2_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, "/lidar1/points_raw", 1);
  cloud_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *cloud_subscriber1_, *cloud_subscriber2_);
  cloud_synchronizer_->registerCallback(boost::bind(&PointsConcatFilter::pointcloud_callback, this, _1, _2));
  cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>("/points_concat", 1);
}

void PointsConcatFilter::config_callback(const autoware_msgs::ConfigPointsConcatFilter::ConstPtr& config_msg)
{
  output_frame_ = config_msg->output_frame;
}

void PointsConcatFilter::pointcloud_callback(const PointCloudMsgT::ConstPtr& cloud_msg1,
                                             const PointCloudMsgT::ConstPtr& cloud_msg2)
{
  PointCloudT::Ptr cloud_source1(new PointCloudT);
  PointCloudT::Ptr cloud_source2(new PointCloudT);
  PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  // Note: If you use kinetic, you can directly receive messages as PointCloutT.
  pcl::fromROSMsg(*cloud_msg1, *cloud_source1);
  pcl::fromROSMsg(*cloud_msg2, *cloud_source2);

  // transform points
  try
  {
    tf_listener_.waitForTransform(output_frame_, cloud_msg1->header.frame_id, ros::Time(0), ros::Duration(1.0));
    pcl_ros::transformPointCloud(output_frame_, *cloud_source1, *cloud_source1, tf_listener_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  try
  {
    tf_listener_.waitForTransform(output_frame_, cloud_msg2->header.frame_id, ros::Time(0), ros::Duration(1.0));
    pcl_ros::transformPointCloud(output_frame_, *cloud_source2, *cloud_source2, tf_listener_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // merge points
  *cloud_concatenated += *cloud_source1;
  *cloud_concatenated += *cloud_source2;
  cloud_concatenated->header = pcl_conversions::toPCL(cloud_msg1->header);
  cloud_concatenated->header.frame_id = output_frame_;
  cloud_publisher_.publish(cloud_concatenated);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_concat_filter");
  PointsConcatFilter node;
  ros::spin();
  return 0;
}
