/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef NDT_SLAM_CORE_H
#define NDT_SLAM_CORE_H

#include <deque>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <autoware_config_msgs/ConfigNDTSlam.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <lidar_localizer/map_manager/map_manager.h>
#include <lidar_localizer/ndt/ndt_slam_pcl.h>
#include <lidar_localizer/ndt/ndt_slam_pcl_anh.h>
#include <lidar_localizer/ndt/ndt_slam_pcl_anh_gpu.h>
#include <lidar_localizer/ndt/ndt_slam_pcl_omp.h>
#include <lidar_localizer/pose_linear_interpolator/pose_linear_interpolator.h>

class NdtSlam {
  using PointSource = pcl::PointXYZI;
  using PointTarget = pcl::PointXYZI;
  using SyncPolicyPoints =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                      sensor_msgs::PointCloud2>;

  enum class MethodType {
    PCL_GENERIC = 0,
    PCL_ANH = 1,
    PCL_ANH_GPU = 2,
    PCL_OPENMP = 3,
  };

public:
  NdtSlam(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~NdtSlam();

private:
  void configCallback(
      const autoware_config_msgs::ConfigNDTSlam::ConstPtr &config_msg_ptr);
  void pointsMapUpdatedCallback(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud2_msg_ptr);
  void
  initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr
                          &pose_conv_msg_ptr);
  void
  staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg_ptr);
  void mappingAndLocalizingPointsCallback(
      const sensor_msgs::PointCloud2::ConstPtr &mapping_points_msg_ptr,
      const sensor_msgs::PointCloud2::ConstPtr &localizing_points_msg_ptr);

  void mapping(const boost::shared_ptr<pcl::PointCloud<PointTarget>>
                   &mapping_points_ptr);
  // TODO const ros::Time time_stamp
  void publishPosition(const ros::Time &time_stamp);
  void publishVelocity(const ros::Time &time_stamp);
  void publishPointsMap(const ros::Time &time_stamp);
  void publishTF(const ros::Time &time_stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher points_map_pub_;
  ros::Publisher ndt_pose_pub_;
  ros::Publisher localizer_pose_pub_;
  ros::Publisher estimate_twist_pub_;

  ros::Subscriber config_sub_;
  ros::Subscriber points_map_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber static_pose_sub_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
      mapping_points_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
      localizing_points_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicyPoints>>
      points_synchronizer_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  std::unique_ptr<NdtSlamBase<PointSource, PointTarget>> localizer_ptr_;
  MapManager<PointTarget> map_manager_;
  PoseLinearInterpolator pose_interpolator_;

  MethodType method_type_;
  Eigen::Matrix4f tf_btol_;
  bool with_mapping_;
  bool separate_mapping_;
  bool use_nn_point_z_when_initial_pose_;
  std::string sensor_frame_;
  std::string base_link_frame_;
  std::string map_frame_;
  std::string log_file_directory_path_;
  double min_scan_range_;
  double max_scan_range_;
  double min_add_scan_shift_;

  PoseStamped init_pose_stamped_;
};

#endif
