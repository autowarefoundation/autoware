/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 Localization program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include "autoware_msgs/ConfigICP.h"

#include "autoware_msgs/icp_stat.h"

#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

static pose initial_pose, predict_pose, previous_pose, icp_pose, current_pose, localizer_pose, previous_gnss_pose, current_gnss_pose;

static double offset_x, offset_y, offset_z, offset_yaw;  // current_pos - previous_pose

// Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
static pcl::PointCloud<pcl::PointXYZ> map, add;

// If the map is loaded, map_loaded will be 1.
static int map_loaded = 0;
static int _use_gnss = 1;
static int init_pos_set = 0;

static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

// Default values for ICP
static int maximum_iterations = 100;
static double transformation_epsilon = 0.01;
static double max_correspondence_distance = 1.0;
static double euclidean_fitness_epsilon = 0.1;
static double ransac_outlier_rejection_threshold = 1.0;

static ros::Publisher predict_pose_pub;
static geometry_msgs::PoseStamped predict_pose_msg;

static ros::Publisher icp_pose_pub;
static geometry_msgs::PoseStamped icp_pose_msg;

static geometry_msgs::PoseStamped current_pose_msg;

static ros::Publisher localizer_pose_pub;
static geometry_msgs::PoseStamped localizer_pose_msg;

static ros::Publisher estimate_twist_pub;
static geometry_msgs::TwistStamped estimate_twist_msg;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double exe_time = 0.0;
static double fitness_score = 0.0;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;

static double current_velocity = 0.0, previous_velocity = 0.0, previous_previous_velocity = 0.0;  // [m/s]
static double current_velocity_x = 0.0, previous_velocity_x = 0.0;
static double current_velocity_y = 0.0, previous_velocity_y = 0.0;
static double current_velocity_z = 0.0, previous_velocity_z = 0.0;
// static double current_velocity_yaw = 0.0, previous_velocity_yaw = 0.0;
static double current_velocity_smooth = 0.0;

static double current_accel = 0.0, previous_accel = 0.0;  // [m/s^2]
static double current_accel_x = 0.0;
static double current_accel_y = 0.0;
static double current_accel_z = 0.0;
// static double current_accel_yaw = 0.0;

static double angular_velocity = 0.0;

static int use_predict_pose = 0;

static ros::Publisher estimated_vel_mps_pub, estimated_vel_kmph_pub, estimated_vel_pub;
static std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

static ros::Publisher time_icp_matching_pub;
static std_msgs::Float32 time_icp_matching;

static int _queue_size = 1000;

static ros::Publisher icp_stat_pub;
static autoware_msgs::icp_stat icp_stat_msg;

static double predict_pose_error = 0.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static std::string _localizer = "velodyne";
static std::string _offset = "linear";  // linear, zero, quadratic

static std::ofstream ofs;
static std::string filename;

static void param_callback(const autoware_msgs::ConfigICP::ConstPtr& input)
{
  if (_use_gnss != input->init_pos_gnss)
  {
    init_pos_set = 0;
  }
  else if (_use_gnss == 0 &&
           (initial_pose.x != input->x || initial_pose.y != input->y || initial_pose.z != input->z ||
            initial_pose.roll != input->roll || initial_pose.pitch != input->pitch || initial_pose.yaw != input->yaw))
  {
    init_pos_set = 0;
  }

  _use_gnss = input->init_pos_gnss;

  if (_use_gnss == 0 && init_pos_set == 0)
  {
    initial_pose.x = input->x;
    initial_pose.y = input->y;
    initial_pose.z = input->z;
    initial_pose.roll = input->roll;
    initial_pose.pitch = input->pitch;
    initial_pose.yaw = input->yaw;

    // Setting position and posture for the first time.
    localizer_pose.x = initial_pose.x;
    localizer_pose.y = initial_pose.y;
    localizer_pose.z = initial_pose.z;
    localizer_pose.roll = initial_pose.roll;
    localizer_pose.pitch = initial_pose.pitch;
    localizer_pose.yaw = initial_pose.yaw;

    previous_pose.x = initial_pose.x;
    previous_pose.y = initial_pose.y;
    previous_pose.z = initial_pose.z;
    previous_pose.roll = initial_pose.roll;
    previous_pose.pitch = initial_pose.pitch;
    previous_pose.yaw = initial_pose.yaw;

    current_pose.x = initial_pose.x;
    current_pose.y = initial_pose.y;
    current_pose.z = initial_pose.z;
    current_pose.roll = initial_pose.roll;
    current_pose.pitch = initial_pose.pitch;
    current_pose.yaw = initial_pose.yaw;

    init_pos_set = 1;
  }

  maximum_iterations = input->maximum_iterations;
  transformation_epsilon = input->transformation_epsilon;
  max_correspondence_distance = input->max_correspondence_distance;
  euclidean_fitness_epsilon = input->euclidean_fitness_epsilon;
  ransac_outlier_rejection_threshold = input->ransac_outlier_rejection_threshold;

}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if (map_loaded == 0)
  {
    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));
    // Setting point cloud to be aligned to.
    icp.setInputTarget(map_ptr);
    std::cout << "setInputTarget finished." << std::endl;

    map_loaded = 1;
  }
}

static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  tf::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z,
                        input->pose.orientation.w);
  tf::Matrix3x3 gnss_m(gnss_q);
  current_gnss_pose.x = input->pose.position.x;
  current_gnss_pose.y = input->pose.position.y;
  current_gnss_pose.z = input->pose.position.z;
  gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

  if ((_use_gnss == 1 && init_pos_set == 0) || fitness_score >= 500.0)
  {
    previous_pose.x = previous_gnss_pose.x;
    previous_pose.y = previous_gnss_pose.y;
    previous_pose.z = previous_gnss_pose.z;
    previous_pose.roll = previous_gnss_pose.roll;
    previous_pose.pitch = previous_gnss_pose.pitch;
    previous_pose.yaw = previous_gnss_pose.yaw;

    current_pose.x = current_gnss_pose.x;
    current_pose.y = current_gnss_pose.y;
    current_pose.z = current_gnss_pose.z;
    current_pose.roll = current_gnss_pose.roll;
    current_pose.pitch = current_gnss_pose.pitch;
    current_pose.yaw = current_gnss_pose.yaw;

    offset_x = current_pose.x - previous_pose.x;
    offset_y = current_pose.y - previous_pose.y;
    offset_z = current_pose.z - previous_pose.z;
    offset_yaw = current_pose.yaw - previous_pose.yaw;

    init_pos_set = 1;
  }

  previous_gnss_pose.x = current_gnss_pose.x;
  previous_gnss_pose.y = current_gnss_pose.y;
  previous_gnss_pose.z = current_gnss_pose.z;
  previous_gnss_pose.roll = current_gnss_pose.roll;
  previous_gnss_pose.pitch = current_gnss_pose.pitch;
  previous_gnss_pose.yaw = current_gnss_pose.yaw;
}

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/map", input->header.frame_id, now, ros::Duration(10.0));
    listener.lookupTransform("/map", input->header.frame_id, now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                   input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  current_pose.x = input->pose.pose.position.x + transform.getOrigin().x();
  current_pose.y = input->pose.pose.position.y + transform.getOrigin().y();
  current_pose.z = input->pose.pose.position.z + transform.getOrigin().z();
  m.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  offset_x = 0.0;
  offset_y = 0.0;
  offset_z = 0.0;
  offset_yaw = 0.0;
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if (map_loaded == 1 && init_pos_set == 1)
  {
    matching_start = std::chrono::system_clock::now();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion predict_q, icp_q, current_q, localizer_q;

    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;

    current_scan_time = input->header.stamp;

    pcl::fromROSMsg(*input, filtered_scan);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
    int scan_points_num = filtered_scan_ptr->size();

    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());   // base_link
    Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());  // localizer

    std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start, getFitnessScore_end;
    static double align_time, getFitnessScore_time = 0.0;

    // Setting point cloud to be aligned.
    icp.setInputSource(filtered_scan_ptr);

    // Guess the initial gross estimation of the transformation
    predict_pose.x = previous_pose.x + offset_x;
    predict_pose.y = previous_pose.y + offset_y;
    predict_pose.z = previous_pose.z + offset_z;
    predict_pose.roll = previous_pose.roll;
    predict_pose.pitch = previous_pose.pitch;
    predict_pose.yaw = previous_pose.yaw + offset_yaw;

    Eigen::Translation3f init_translation(predict_pose.x, predict_pose.y, predict_pose.z);
    Eigen::AngleAxisf init_rotation_x(predict_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(predict_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(predict_pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    icp.setMaximumIterations(maximum_iterations);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    icp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);

    align_start = std::chrono::system_clock::now();
    icp.align(*output_cloud, init_guess);
    align_end = std::chrono::system_clock::now();
    align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

    t = icp.getFinalTransformation();  // localizer
    t2 = t * tf_ltob;                  // base_link

    getFitnessScore_start = std::chrono::system_clock::now();
    fitness_score = icp.getFitnessScore();
    getFitnessScore_end = std::chrono::system_clock::now();
    getFitnessScore_time = std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() / 1000.0;

    tf::Matrix3x3 mat_l;  // localizer
    mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

    // Update localizer_pose
    localizer_pose.x = t(0, 3);
    localizer_pose.y = t(1, 3);
    localizer_pose.z = t(2, 3);
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

    tf::Matrix3x3 mat_b;  // base_link
    mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
                   static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
                   static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

    // Update icp_pose
    icp_pose.x = t2(0, 3);
    icp_pose.y = t2(1, 3);
    icp_pose.z = t2(2, 3);
    mat_b.getRPY(icp_pose.roll, icp_pose.pitch, icp_pose.yaw, 1);

    // Calculate the difference between icp_pose and predict_pose
    predict_pose_error = sqrt((icp_pose.x - predict_pose.x) * (icp_pose.x - predict_pose.x) +
                              (icp_pose.y - predict_pose.y) * (icp_pose.y - predict_pose.y) +
                              (icp_pose.z - predict_pose.z) * (icp_pose.z - predict_pose.z));

    if (predict_pose_error <= PREDICT_POSE_THRESHOLD)
    {
      use_predict_pose = 0;
    }
    else
    {
      use_predict_pose = 1;
    }
    use_predict_pose = 0;

    if (use_predict_pose == 0)
    {
      current_pose.x = icp_pose.x;
      current_pose.y = icp_pose.y;
      current_pose.z = icp_pose.z;
      current_pose.roll = icp_pose.roll;
      current_pose.pitch = icp_pose.pitch;
      current_pose.yaw = icp_pose.yaw;
    }
    else
    {
      current_pose.x = predict_pose.x;
      current_pose.y = predict_pose.y;
      current_pose.z = predict_pose.z;
      current_pose.roll = predict_pose.roll;
      current_pose.pitch = predict_pose.pitch;
      current_pose.yaw = predict_pose.yaw;
    }

    // Compute the velocity and acceleration
    scan_duration = current_scan_time - previous_scan_time;
    double secs = scan_duration.toSec();
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = current_pose.yaw - previous_pose.yaw;
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    current_velocity = diff / secs;
    current_velocity_x = diff_x / secs;
    current_velocity_y = diff_y / secs;
    current_velocity_z = diff_z / secs;
    angular_velocity = diff_yaw / secs;

    current_velocity_smooth = (current_velocity + previous_velocity + previous_previous_velocity) / 3.0;
    if (current_velocity_smooth < 0.2)
    {
      current_velocity_smooth = 0.0;
    }

    current_accel = (current_velocity - previous_velocity) / secs;
    current_accel_x = (current_velocity_x - previous_velocity_x) / secs;
    current_accel_y = (current_velocity_y - previous_velocity_y) / secs;
    current_accel_z = (current_velocity_z - previous_velocity_z) / secs;

    estimated_vel_mps.data = current_velocity;
    estimated_vel_kmph.data = current_velocity * 3.6;

    estimated_vel_mps_pub.publish(estimated_vel_mps);
    estimated_vel_kmph_pub.publish(estimated_vel_kmph);

    // Set values for publishing pose
    predict_q.setRPY(predict_pose.roll, predict_pose.pitch, predict_pose.yaw);
    predict_pose_msg.header.frame_id = "/map";
    predict_pose_msg.header.stamp = current_scan_time;
    predict_pose_msg.pose.position.x = predict_pose.x;
    predict_pose_msg.pose.position.y = predict_pose.y;
    predict_pose_msg.pose.position.z = predict_pose.z;
    predict_pose_msg.pose.orientation.x = predict_q.x();
    predict_pose_msg.pose.orientation.y = predict_q.y();
    predict_pose_msg.pose.orientation.z = predict_q.z();
    predict_pose_msg.pose.orientation.w = predict_q.w();

    icp_q.setRPY(icp_pose.roll, icp_pose.pitch, icp_pose.yaw);
    icp_pose_msg.header.frame_id = "/map";
    icp_pose_msg.header.stamp = current_scan_time;
    icp_pose_msg.pose.position.x = icp_pose.x;
    icp_pose_msg.pose.position.y = icp_pose.y;
    icp_pose_msg.pose.position.z = icp_pose.z;
    icp_pose_msg.pose.orientation.x = icp_q.x();
    icp_pose_msg.pose.orientation.y = icp_q.y();
    icp_pose_msg.pose.orientation.z = icp_q.z();
    icp_pose_msg.pose.orientation.w = icp_q.w();

    current_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    current_pose_msg.header.frame_id = "/map";
    current_pose_msg.header.stamp = current_scan_time;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    current_pose_msg.pose.orientation.x = current_q.x();
    current_pose_msg.pose.orientation.y = current_q.y();
    current_pose_msg.pose.orientation.z = current_q.z();
    current_pose_msg.pose.orientation.w = current_q.w();

    localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);
    localizer_pose_msg.header.frame_id = "/map";
    localizer_pose_msg.header.stamp = current_scan_time;
    localizer_pose_msg.pose.position.x = localizer_pose.x;
    localizer_pose_msg.pose.position.y = localizer_pose.y;
    localizer_pose_msg.pose.position.z = localizer_pose.z;
    localizer_pose_msg.pose.orientation.x = localizer_q.x();
    localizer_pose_msg.pose.orientation.y = localizer_q.y();
    localizer_pose_msg.pose.orientation.z = localizer_q.z();
    localizer_pose_msg.pose.orientation.w = localizer_q.w();

    predict_pose_pub.publish(predict_pose_msg);
    icp_pose_pub.publish(icp_pose_msg);
    localizer_pose_pub.publish(localizer_pose_msg);

    // Send TF "/base_link" to "/map"
    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    transform.setRotation(current_q);
    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));

    matching_end = std::chrono::system_clock::now();
    exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;
    time_icp_matching.data = exe_time;
    time_icp_matching_pub.publish(time_icp_matching);

    // Set values for /estimate_twist
    estimate_twist_msg.header.stamp = current_scan_time;
    estimate_twist_msg.twist.linear.x = current_velocity;
    estimate_twist_msg.twist.linear.y = 0.0;
    estimate_twist_msg.twist.linear.z = 0.0;
    estimate_twist_msg.twist.angular.x = 0.0;
    estimate_twist_msg.twist.angular.y = 0.0;
    estimate_twist_msg.twist.angular.z = angular_velocity;

    estimate_twist_pub.publish(estimate_twist_msg);

    geometry_msgs::Vector3Stamped estimate_vel_msg;
    estimate_vel_msg.header.stamp = current_scan_time;
    estimate_vel_msg.vector.x = current_velocity;
    estimated_vel_pub.publish(estimate_vel_msg);

    // Set values for /icp_stat
    icp_stat_msg.header.stamp = current_scan_time;
    icp_stat_msg.exe_time = time_icp_matching.data;
//    icp_stat_msg.iteration = iteration;
    icp_stat_msg.score = fitness_score;
    icp_stat_msg.velocity = current_velocity;
    icp_stat_msg.acceleration = current_accel;
    icp_stat_msg.use_predict_pose = 0;

    icp_stat_pub.publish(icp_stat_msg);

    // Write log
    if (!ofs)
    {
      std::cerr << "Could not open " << filename << "." << std::endl;
      exit(1);
    }
    ofs << input->header.seq << "," << scan_points_num << ","
            << current_pose.x << "," << current_pose.y << "," << current_pose.z << "," << current_pose.roll << ","
            << current_pose.pitch << "," << current_pose.yaw << "," << predict_pose.x << "," << predict_pose.y << ","
            << predict_pose.z << "," << predict_pose.roll << "," << predict_pose.pitch << "," << predict_pose.yaw << ","
            << current_pose.x - predict_pose.x << "," << current_pose.y - predict_pose.y << ","
            << current_pose.z - predict_pose.z << "," << current_pose.roll - predict_pose.roll << ","
            << current_pose.pitch - predict_pose.pitch << "," << current_pose.yaw - predict_pose.yaw << ","
            << predict_pose_error << "," <<  "," << fitness_score << ","
            << "," << current_velocity << "," << current_velocity_smooth << "," << current_accel
            << "," << angular_velocity << "," << exe_time << "," << align_time << "," << getFitnessScore_time << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence: " << input->header.seq << std::endl;
    std::cout << "Timestamp: " << input->header.stamp << std::endl;
    std::cout << "Frame ID: " << input->header.frame_id << std::endl;
    //		std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
    std::cout << "ICP has converged: " << icp.hasConverged() << std::endl;
    std::cout << "Fitness Score: " << fitness_score << std::endl;

    std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
              << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix: " << std::endl;
    std::cout << t << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    // Update offset
    if (_offset == "linear")
    {
      offset_x = diff_x;
      offset_y = diff_y;
      offset_z = diff_z;
      offset_yaw = diff_yaw;
    }
    else if (_offset == "quadratic")
    {
      offset_x = (current_velocity_x + current_accel_x * secs) * secs;
      offset_y = (current_velocity_y + current_accel_y * secs) * secs;
      offset_z = diff_z;
      offset_yaw = diff_yaw;
    }
    else if (_offset == "zero")
    {
      offset_x = 0.0;
      offset_y = 0.0;
      offset_z = 0.0;
      offset_yaw = 0.0;
    }

    // Update previous_***
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time.sec = current_scan_time.sec;
    previous_scan_time.nsec = current_scan_time.nsec;

    previous_previous_velocity = previous_velocity;
    previous_velocity = current_velocity;
    previous_velocity_x = current_velocity_x;
    previous_velocity_y = current_velocity_y;
    previous_velocity_z = current_velocity_z;
    previous_accel = current_accel;

    previous_estimated_vel_kmph.data = estimated_vel_kmph.data;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_matching");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm *pnow = std::localtime(&now);
  std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
  filename = "icp_matching_" + std::string(buffer) + ".csv";
  ofs.open(filename.c_str(), std::ios::app);

  // setting parameters
  private_nh.getParam("use_gnss", _use_gnss);
  private_nh.getParam("queue_size", _queue_size);
  private_nh.getParam("offset", _offset);

  if (nh.getParam("localizer", _localizer) == false)
  {
    std::cout << "localizer is not set." << std::endl;
    return 1;
  }

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

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Log file: " << filename << std::endl;
  std::cout << "use_gnss: " << _use_gnss << std::endl;
  std::cout << "queue_size: " << _queue_size << std::endl;
  std::cout << "offset: " << _offset << std::endl;
  std::cout << "localizer: " << _localizer << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * _tf_x, (-1.0) * _tf_y, (-1.0) * _tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * _tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * _tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * _tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  // Updated in initialpose_callback or gnss_callback
  initial_pose.x = 0.0;
  initial_pose.y = 0.0;
  initial_pose.z = 0.0;
  initial_pose.roll = 0.0;
  initial_pose.pitch = 0.0;
  initial_pose.yaw = 0.0;

  // Publishers
  predict_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose", 1000);
  icp_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/icp_pose", 1000);
  localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 1000);
  estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 1000);
  estimated_vel_mps_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 1000);
  estimated_vel_kmph_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 1000);
  estimated_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/estimated_vel", 1000);
  time_icp_matching_pub = nh.advertise<std_msgs::Float32>("/time_icp_matching", 1000);
  icp_stat_pub = nh.advertise<autoware_msgs::icp_stat>("/icp_stat", 1000);

  // Subscribers
  ros::Subscriber param_sub = nh.subscribe("config/icp", 10, param_callback);
  ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, gnss_callback);
  ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialpose_callback);
  ros::Subscriber points_sub = nh.subscribe("filtered_points", _queue_size, points_callback);

  ros::spin();

  return 0;
}
