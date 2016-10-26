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
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT // If you want to output "position_log.txt", "#define OUTPUT".

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_FAST_PCL
#include <fast_pcl/registration/ndt.h>
#include <fast_pcl/filters/voxel_grid.h>
#else
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#endif

#include <runtime_manager/ConfigNdtMapping.h>
#include <runtime_manager/ConfigNdtMappingOutput.h>

struct pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

// global variables
static pose previous_pose, guess_pose, current_pose, ndt_pose, added_pose, localizer_pose;

static double offset_x, offset_y, offset_z, offset_yaw; // current_pose - previous_pose

static pcl::PointCloud<pcl::PointXYZI> map;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static int iter = 30; // Maximum iterations
static float ndt_res = 1.0; // Resolution
static double step_size = 0.1; // Step size
static double trans_eps = 0.01; // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end, t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double RANGE = 0.0;
static double SHIFT = 0.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool isMapUpdate = true;
static bool _use_openmp = false;

static double fitness_score;

static void param_callback(const runtime_manager::ConfigNdtMapping::ConstPtr& input)
{
  ndt_res = input->resolution;
  step_size = input->step_size;
  trans_eps = input->trans_eps;
  voxel_leaf_size = input->leaf_size;

  std::cout << "param_callback" << std::endl;
  std::cout << "ndt_res: " << ndt_res << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_eps: " << trans_eps << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
}

static void output_callback(const runtime_manager::ConfigNdtMappingOutput::ConstPtr& input)
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
  if(filter_res == 0.0){
    std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  }else{
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
  if(filter_res == 0.0){
    pcl::io::savePCDFileASCII(filename, *map_ptr);
    std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
  }else{
    pcl::io::savePCDFileASCII(filename, *map_filtered);
    std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
  }    
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    double r;
    pcl::PointXYZI p; 
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr (new pcl::PointCloud<pcl::PointXYZI>());
    tf::Quaternion q;

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Time scan_time = input->header.stamp;

    pcl::fromROSMsg(*input, tmp);

    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++){
    	p.x = (double) item->x;
    	p.y = (double) item->y;
    	p.z = (double) item->z;
    	p.intensity = (double) item->intensity;

    	r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    	if(r > RANGE){
    		scan.push_back(p);
    	}
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    
    // Add initial point cloud to velodyne_map
    if(initial_scan_loaded == 0){
      map += *scan_ptr;
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
    ndt.setMaximumIterations(iter);
    ndt.setInputSource(filtered_scan_ptr);
    
    if(isMapUpdate == true){
    	ndt.setInputTarget(map_ptr);
    	isMapUpdate = false;
    }

    guess_pose.x = previous_pose.x + offset_x;
    guess_pose.y = previous_pose.y + offset_y;
    guess_pose.z = previous_pose.z + offset_z;
    guess_pose.roll = previous_pose.roll;
    guess_pose.pitch = previous_pose.pitch;
    guess_pose.yaw = previous_pose.yaw + offset_yaw;
    
    Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);
    
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;
    
    t3_end = ros::Time::now();
    d3 = t3_end - t3_start;
    
    t4_start = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
#ifdef USE_FAST_PCL
    if(_use_openmp == true){
    	ndt.omp_align(*output_cloud, init_guess);
    	fitness_score = ndt.omp_getFitnessScore();
    }else{
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

    mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)), static_cast<double>(t_localizer(0, 2)),
		  static_cast<double>(t_localizer(1, 0)), static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
		  static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)), static_cast<double>(t_localizer(2, 2)));
    
    
    mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)), static_cast<double>(t_base_link(0, 2)),
		  static_cast<double>(t_base_link(1, 0)), static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
		  static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)), static_cast<double>(t_base_link(2, 2)));
    
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
    
    br.sendTransform(tf::StampedTransform(transform, scan_time, "map", "base_link"));
    
    // Calculate the offset (curren_pos - previous_pos)
    offset_x = current_pose.x - previous_pose.x;
    offset_y = current_pose.y - previous_pose.y;
    offset_z = current_pose.z - previous_pose.z;
    offset_yaw = current_pose.yaw - previous_pose.yaw;
    
    // Update position and posture. current_pos -> previous_pos
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;
    
    // Calculate the shift between added_pos and current_pos
    double shift = sqrt(pow(current_pose.x-added_pose.x, 2.0) + pow(current_pose.y-added_pose.y, 2.0));
    if(shift >= SHIFT){
      map += *transformed_scan_ptr;
      added_pose.x = current_pose.x;
      added_pose.y = current_pose.y;
      added_pose.z = current_pose.z;
      added_pose.roll = current_pose.roll;
      added_pose.pitch = current_pose.pitch;
      added_pose.yaw = current_pose.yaw;
      isMapUpdate = true;
    }
    
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    ndt_map_pub.publish(*map_msg_ptr);
    
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = scan_time;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();
    
    current_pose_pub.publish(current_pose_msg);
    
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
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    
}

int main(int argc, char **argv)
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

    offset_x = 0.0;
    offset_y = 0.0;
    offset_z = 0.0;
    offset_yaw = 0.0;

    ros::init(argc, argv, "ndt_mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // setting parameters
    private_nh.getParam("range", RANGE);
    std::cout << "RANGE: " << RANGE << std::endl;
    private_nh.getParam("shift", SHIFT);
    std::cout << "SHIFT: " << SHIFT << std::endl;
    private_nh.getParam("use_openmp", _use_openmp);
    std::cout << "use_openmp: " << _use_openmp << std::endl;

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

    map.header.frame_id = "map";

    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

    ros::Subscriber param_sub = nh.subscribe("config/ndt_mapping", 10, param_callback);
    ros::Subscriber output_sub = nh.subscribe("config/ndt_mapping_output", 10, output_callback);
    ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);

    ros::spin();

    return 0;
}
