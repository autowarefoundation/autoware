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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <velodyne_pointcloud/point_types.h>

#include "autoware_config_msgs/ConfigRingFilter.h"

#include <points_downsampler/PointsDownsamplerInfo.h>

#include <chrono>

#define MAX_MEASUREMENT_RANGE 200.0

ros::Publisher filtered_points_pub;

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

int ring_max = 0;
int ring_div = 3;

static ros::Publisher points_downsampler_info_pub;
static points_downsampler::PointsDownsamplerInfo points_downsampler_info_msg;

static std::chrono::time_point<std::chrono::system_clock> filter_start, filter_end;

static bool _output_log = false;
static std::ofstream ofs;
static std::string filename;

static std::string POINTS_TOPIC;
static double measurement_range = MAX_MEASUREMENT_RANGE;

static void config_callback(const autoware_config_msgs::ConfigRingFilter::ConstPtr& input)
{
  ring_div = input->ring_div;
  voxel_leaf_size = input->voxel_leaf_size;
  measurement_range = input->measurement_range;
}

static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> tmp;
  sensor_msgs::PointCloud2 filtered_msg;

  pcl::fromROSMsg(*input, scan);
  pcl::fromROSMsg(*input, tmp);

  filter_start = std::chrono::system_clock::now();

  scan.points.clear();

  double square_measurement_range = measurement_range*measurement_range;

  for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    pcl::PointXYZI p;
    p.x = item->x;
    p.y = item->y;
    p.z = item->z;
    p.intensity = item->intensity;

    double square_distance = p.x * p.x + p.y * p.y;

    if (item->ring % ring_div == 0 && square_distance <= square_measurement_range)
    {
      scan.points.push_back(p);
    }
    if (item->ring > ring_max)
    {
      ring_max = item->ring;
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  if (voxel_leaf_size >= 0.1)
  {
    // Downsampling the velodyne scan using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
  }
  else
  {
    pcl::toROSMsg(*scan_ptr, filtered_msg);
  }

  filter_end = std::chrono::system_clock::now();

  filtered_msg.header = input->header;
  filtered_points_pub.publish(filtered_msg);

  points_downsampler_info_msg.header = input->header;
  points_downsampler_info_msg.filter_name = "ring_filter";
  points_downsampler_info_msg.measurement_range = measurement_range;
  points_downsampler_info_msg.original_points_size = scan.size();
  if (voxel_leaf_size >= 0.1)
  {
    points_downsampler_info_msg.filtered_points_size = filtered_scan_ptr->size();
  }
  else
  {
    points_downsampler_info_msg.filtered_points_size = scan_ptr->size();
  }
  points_downsampler_info_msg.original_ring_size = ring_max;
  points_downsampler_info_msg.filtered_ring_size = ring_max / ring_div;
  points_downsampler_info_msg.exe_time = std::chrono::duration_cast<std::chrono::microseconds>(filter_end - filter_start).count() / 1000.0;
  points_downsampler_info_pub.publish(points_downsampler_info_msg);

  if(_output_log == true){
	  if(!ofs){
		  std::cerr << "Could not open " << filename << "." << std::endl;
		  exit(1);
	  }
	  ofs << points_downsampler_info_msg.header.seq << ","
		  << points_downsampler_info_msg.header.stamp << ","
		  << points_downsampler_info_msg.header.frame_id << ","
		  << points_downsampler_info_msg.filter_name << ","
		  << points_downsampler_info_msg.original_points_size << ","
		  << points_downsampler_info_msg.filtered_points_size << ","
		  << points_downsampler_info_msg.original_ring_size << ","
		  << points_downsampler_info_msg.filtered_ring_size << ","
		  << points_downsampler_info_msg.exe_time << ","
		  << std::endl;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ring_filter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("points_topic", POINTS_TOPIC);
  private_nh.getParam("output_log", _output_log);
  if(_output_log == true){
	  char buffer[80];
	  std::time_t now = std::time(NULL);
	  std::tm *pnow = std::localtime(&now);
	  std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
	  filename = "ring_filter_" + std::string(buffer) + ".csv";
	  ofs.open(filename.c_str(), std::ios::app);
  }

  // Publishers
  filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
  points_downsampler_info_pub = nh.advertise<points_downsampler::PointsDownsamplerInfo>("/points_downsampler_info", 1000);

  // Subscribers
  ros::Subscriber config_sub = nh.subscribe("config/ring_filter", 10, config_callback);
  ros::Subscriber scan_sub = nh.subscribe(POINTS_TOPIC, 10, scan_callback);

  ros::spin();

  return 0;
}
