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

/*
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_FAST_PCL
#include <fast_pcl/filters/voxel_grid.h>
#include <fast_pcl/registration/ndt.h>
#else
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#endif

#include <autoware_config_msgs/ConfigApproximateNDTMapping.h>
#include <autoware_config_msgs/ConfigNDTMappingOutput.h>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

// global variables
static pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom, current_pose,
    current_pose_imu, current_pose_odom, current_pose_imu_odom, ndt_pose, added_pose, localizer_pose;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
    offset_imu_odom_yaw;

static double current_velocity_x = 0.0;
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

static pcl::PointCloud<pcl::PointXYZI> map, submap;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static ros::Publisher guess_pose_linaer_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 5.0;
static double max_scan_range = 200.0;
static double min_add_scan_shift = 1.0;
static double max_submap_size = 100.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool isMapUpdate = true;
static bool _use_openmp = false;
static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;

static std::string _imu_topic = "/imu_raw";

static double fitness_score;

static int submap_num = 0;
static double submap_size = 0.0;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;

static std::ofstream ofs;
static std::string filename;

static void param_callback(const autoware_config_msgs::ConfigApproximateNDTMapping::ConstPtr& input)
{
  ndt_res = input->resolution;
  step_size = input->step_size;
  trans_eps = input->trans_epsilon;
  max_iter = input->max_iterations;
  voxel_leaf_size = input->leaf_size;
  min_scan_range = input->min_scan_range;
  max_scan_range = input->max_scan_range;
  min_add_scan_shift = input->min_add_scan_shift;
  max_submap_size = input->max_submap_size;

  std::cout << "param_callback" << std::endl;
  std::cout << "ndt_res: " << ndt_res << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_epsilon: " << trans_eps << std::endl;
  std::cout << "max_iter: " << max_iter << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "max_scan_range: " << max_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
  std::cout << "max_submap_size: " << max_submap_size << std::endl;
}

static void output_callback(const autoware_config_msgs::ConfigNDTMappingOutput::ConstPtr& input)
{
  double filter_res = input->filter_res;
  std::string filename = input->filename;
  std::cout << "output_callback" << std::endl;
  std::cout << "filter_res: " << filter_res << std::endl;
  std::cout << "filename: " << filename << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  map_ptr->header.frame_id = "map";
  map_filtered->header.frame_id = "map";
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

  // Apply voxelgrid filter
  if (filter_res == 0.0)
  {
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  }
  else
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*map_filtered);
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_filtered, *map_msg_ptr);
  }

  ndt_map_pub.publish(*map_msg_ptr);

  // Writing Point Cloud data to PCD file
  if (filter_res == 0.0)
  {
    pcl::io::savePCDFileASCII(filename, *map_ptr);
    std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
  }
  else
  {
    pcl::io::savePCDFileASCII(filename, *map_filtered);
    std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
  }
}

static void imu_odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;

  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

  previous_time = current_time;
}

static void odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose.x + offset_odom_x;
  guess_pose_odom.y = previous_pose.y + offset_odom_y;
  guess_pose_odom.z = previous_pose.z + offset_odom_z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

  previous_time = current_time;
}

static void imu_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  guess_pose_imu.x = previous_pose.x + offset_imu_x;
  guess_pose_imu.y = previous_pose.y + offset_imu_y;
  guess_pose_imu.z = previous_pose.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

  previous_time = current_time;
}

static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

static void odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  // std::cout << __func__ << std::endl;

  odom = *input;
  odom_calc(input->header.stamp);
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input)
{
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

static void imu_callback(const sensor_msgs::Imu::Ptr& input)
{
  // std::cout << __func__ << std::endl;

  if (_imu_upside_down)
    imuUpsideDown(input);

  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = wrapToPmPi(imu_roll);
  imu_pitch = wrapToPmPi(imu_pitch);
  imu_yaw = wrapToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = imu_roll - previous_imu_roll;

  const double diff_imu_pitch = imu_pitch - previous_imu_pitch;

  double diff_imu_yaw;
  if (fabs(imu_yaw - previous_imu_yaw) > M_PI)
  {
    if (imu_yaw > 0)
      diff_imu_yaw = (imu_yaw - previous_imu_yaw) - M_PI * 2;
    else
      diff_imu_yaw = -M_PI * 2 - (imu_yaw - previous_imu_yaw);
  }
  else
    diff_imu_yaw = imu_yaw - previous_imu_yaw;

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0)
  {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  }
  else
  {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range)
    {
      scan.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputSource(filtered_scan_ptr);

  if (isMapUpdate == true)
  {
    ndt.setInputTarget(map_ptr);
    isMapUpdate = false;
  }

  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;

  if (_use_imu == true && _use_odom == true)
    imu_odom_calc(current_scan_time);
  if (_use_imu == true && _use_odom == false)
    imu_calc(current_scan_time);
  if (_use_imu == false && _use_odom == true)
    odom_calc(current_scan_time);

  pose guess_pose_for_ndt;
  if (_use_imu == true && _use_odom == true)
    guess_pose_for_ndt = guess_pose_imu_odom;
  else if (_use_imu == true && _use_odom == false)
    guess_pose_for_ndt = guess_pose_imu;
  else if (_use_imu == false && _use_odom == true)
    guess_pose_for_ndt = guess_pose_odom;
  else
    guess_pose_for_ndt = guess_pose;

  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  t4_start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
#ifdef USE_FAST_PCL
  if (_use_openmp == true)
  {
    ndt.omp_align(*output_cloud, init_guess);
    fitness_score = ndt.omp_getFitnessScore();
  }
  else
  {
#endif
    ndt.align(*output_cloud, init_guess);
    fitness_score = ndt.getFitnessScore();
#ifdef USE_FAST_PCL
  }
#endif

  t_localizer = ndt.getFinalTransformation();
  t_base_link = t_localizer * tf_ltob;

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update ndt_pose.
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;

  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // Calculate the offset (curren_pos - previous_pos)
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = current_pose.yaw - previous_pose.yaw;
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;

  current_pose_imu.x = current_pose.x;
  current_pose_imu.y = current_pose.y;
  current_pose_imu.z = current_pose.z;
  current_pose_imu.roll = current_pose.roll;
  current_pose_imu.pitch = current_pose.pitch;
  current_pose_imu.yaw = current_pose.yaw;

  current_pose_odom.x = current_pose.x;
  current_pose_odom.y = current_pose.y;
  current_pose_odom.z = current_pose.z;
  current_pose_odom.roll = current_pose.roll;
  current_pose_odom.pitch = current_pose.pitch;
  current_pose_odom.yaw = current_pose.yaw;

  current_pose_imu_odom.x = current_pose.x;
  current_pose_imu_odom.y = current_pose.y;
  current_pose_imu_odom.z = current_pose.z;
  current_pose_imu_odom.roll = current_pose.roll;
  current_pose_imu_odom.pitch = current_pose.pitch;
  current_pose_imu_odom.yaw = current_pose.yaw;

  current_velocity_imu_x = current_velocity_x;
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // Update position and posture. current_pos -> previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift)
  {
    submap_size += shift;
    map += *transformed_scan_ptr;
    submap += *transformed_scan_ptr;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;
    isMapUpdate = true;
  }

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(submap, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  ndt_map_pub.publish(*map_msg_ptr);

  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();

  current_pose_pub.publish(current_pose_msg);

  if (submap_size >= max_submap_size)
  {
    std::string s1 = "submap_";
    std::string s2 = std::to_string(submap_num);
    std::string s3 = ".pcd";
    std::string pcd_filename = s1 + s2 + s3;

    if (submap.size() != 0)
    {
      if (pcl::io::savePCDFileBinary(pcd_filename, submap) == -1)
      {
        std::cout << "Failed saving " << pcd_filename << "." << std::endl;
      }
      std::cout << "Saved " << pcd_filename << " (" << submap.size() << " points)" << std::endl;

      map = submap;
      submap.clear();
      submap_size = 0.0;
    }
    submap_num++;
  }

  // Write log
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << input->header.seq << ","
      << input->header.stamp << ","
      << input->header.frame_id << ","
      << scan_ptr->size() << ","
      << filtered_scan_ptr->size() << ","
      << std::fixed << std::setprecision(5) << current_pose.x << ","
      << std::fixed << std::setprecision(5) << current_pose.y << ","
      << std::fixed << std::setprecision(5) << current_pose.z << ","
      << current_pose.roll << ","
      << current_pose.pitch << ","
      << current_pose.yaw << ","
      << ndt_res << ","
      << step_size << ","
      << trans_eps << ","
      << max_iter << ","
      << voxel_leaf_size << ","
      << min_scan_range << ","
      << max_scan_range << ","
      << min_add_scan_shift << ","
      << max_submap_size << std::endl;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "current submap size: " << submap_size << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;

  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;

  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;

  current_pose_imu.x = 0.0;
  current_pose_imu.y = 0.0;
  current_pose_imu.z = 0.0;
  current_pose_imu.roll = 0.0;
  current_pose_imu.pitch = 0.0;
  current_pose_imu.yaw = 0.0;

  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;

  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;

  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  ros::init(argc, argv, "approximate_ndt_mapping");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  filename = "approximate_ndt_mapping_" + std::string(buffer) + ".csv";
  ofs.open(filename.c_str(), std::ios::app);

  // write header for log file
  if (!ofs)
  {
    std::cerr << "Could not open " << filename << "." << std::endl;
    exit(1);
  }

  ofs << "input->header.seq" << ","
      << "input->header.stamp" << ","
      << "input->header.frame_id" << ","
      << "scan_ptr->size()" << ","
      << "filtered_scan_ptr->size()" << ","
      << "current_pose.x" << ","
      << "current_pose.y" << ","
      << "current_pose.z" << ","
      << "current_pose.roll" << ","
      << "current_pose.pitch" << ","
      << "current_pose.yaw" << ","
      << "ndt_res" << ","
      << "step_size" << ","
      << "trans_eps" << ","
      << "max_iter" << ","
      << "voxel_leaf_size" << ","
      << "min_scan_range" << ","
      << "max_scan_range" << ","
      << "min_add_scan_shift" << ","
      << "max_submap_size" << std::endl;

  // setting parameters
  private_nh.getParam("use_openmp", _use_openmp);
  private_nh.getParam("use_imu", _use_imu);
  private_nh.getParam("use_odom", _use_odom);
  private_nh.getParam("imu_upside_down", _imu_upside_down);
  private_nh.getParam("imu_topic", _imu_topic);

  std::cout << "use_openmp: " << _use_openmp << std::endl;
  std::cout << "use_imu: " << _use_imu << std::endl;
  std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
  std::cout << "use_odom: " << _use_odom << std::endl;
  std::cout << "imu_topic: " << _imu_topic << std::endl;

  if (nh.getParam("tf_x", _tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_y", _tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_z", _tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_roll", _tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_pitch", _tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_yaw", _tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  map.header.frame_id = "map";

  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

  ros::Subscriber param_sub = nh.subscribe("config/approximate_ndt_mapping", 10, param_callback);
  ros::Subscriber output_sub = nh.subscribe("config/ndt_mapping_output", 10, output_callback);
  ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);
  ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom", 100000, odom_callback);
  ros::Subscriber imu_sub = nh.subscribe(_imu_topic, 100000, imu_callback);

  ros::spin();

  return 0;
}
