/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <velodyne_pointcloud/point_types.h>
#include <yaml-cpp/yaml.h>

class PointsConcatFilter
{
public:
  PointsConcatFilter();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT>
      SyncPolicyT;

  ros::NodeHandle node_handle_, private_node_handle_;
  message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];
  message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;
  ros::Subscriber config_subscriber_;
  ros::Publisher cloud_publisher_;
  tf::TransformListener tf_listener_;

  size_t input_topics_size_;
  std::string input_topics_;
  std::string output_frame_id_;

  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                           const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                           const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                           const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
};

PointsConcatFilter::PointsConcatFilter() : node_handle_(), private_node_handle_("~"), tf_listener_()
{
  private_node_handle_.param("input_topics", input_topics_, std::string("[/points_alpha, /points_beta]"));
  private_node_handle_.param("output_frame_id", output_frame_id_, std::string("velodyne"));

  YAML::Node topics = YAML::Load(input_topics_);
  input_topics_size_ = topics.size();
  if (input_topics_size_ < 2 || 8 < input_topics_size_)
  {
    ROS_ERROR("The size of input_topics must be between 2 and 8");
    ros::shutdown();
  }
  for (size_t i = 0; i < 8; ++i)
  {
    if (i < input_topics_size_)
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(node_handle_, topics[i].as<std::string>(), 1);
    }
    else
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(node_handle_, topics[0].as<std::string>(), 1);
    }
  }
  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
      *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);
  cloud_synchronizer_->registerCallback(
      boost::bind(&PointsConcatFilter::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
  cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>("/points_concat", 1);
}

void PointsConcatFilter::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                             const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                             const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                             const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
{
  assert(2 <= input_topics_size_ && input_topics_size_ <= 8);

  PointCloudMsgT::ConstPtr msgs[8] = { msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8 };
  PointCloudT::Ptr cloud_sources[8];
  PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  // transform points
  try
  {
    for (size_t i = 0; i < input_topics_size_; ++i)
    {
      // Note: If you use kinetic, you can directly receive messages as
      // PointCloutT.
      cloud_sources[i] = PointCloudT().makeShared();
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
      tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));
      pcl_ros::transformPointCloud(output_frame_id_, *cloud_sources[i], *cloud_sources[i], tf_listener_);
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // merge points
  for (size_t i = 0; i < input_topics_size_; ++i)
  {
    *cloud_concatenated += *cloud_sources[i];
  }

  // publsh points
  cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);
  cloud_concatenated->header.frame_id = output_frame_id_;
  cloud_publisher_.publish(cloud_concatenated);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_concat_filter");
  PointsConcatFilter node;
  ros::spin();
  return 0;
}
