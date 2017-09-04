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

ros::Publisher pub_;
tf::TransformListener *tfl;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void callback(const PointCloudT::ConstPtr &cloudf, const PointCloudT::ConstPtr &cloudr)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  PointCloudT::Ptr cloudfm(new PointCloudT);
  PointCloudT::Ptr cloudrm(new PointCloudT);
  tfl->waitForTransform("/base_link", cloudf->header.frame_id, ros::Time(0), ros::Duration(1.0));
  pcl_ros::transformPointCloud("/base_link", *cloudf, *cloudfm, *tfl);
  tfl->waitForTransform("/base_link", cloudr->header.frame_id, ros::Time(0), ros::Duration(1.0));
  pcl_ros::transformPointCloud("/base_link", *cloudr, *cloudrm, *tfl);
  *cloud += *cloudfm;
  *cloud += *cloudrm;
  cloud->header = cloudf->header;
  cloud->header.frame_id = "base_link";
  pub_.publish(cloud);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fusion_filter");
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_("~");
  tfl = new tf::TransformListener();
  ros::Subscriber sub1_, sub2_;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudT, PointCloudT> SyncPolicy;
  message_filters::Subscriber<PointCloudT> sub1(nh_, "/front/velodyne_points", 1);
  message_filters::Subscriber<PointCloudT> sub2(nh_, "/rear/velodyne_points", 1);
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(1), sub1, sub2);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_raw", 1);
	ros::spin();

	return 0;
}
