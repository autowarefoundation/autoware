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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <runtime_manager/ConfigDistanceFilter.h>

#include <points_downsampler/PointsDownsamplerInfo.h>

#include <chrono>

ros::Publisher filtered_points_pub;

static int sample_num = 1000;

static ros::Publisher points_downsampler_info_pub;
static points_downsampler::PointsDownsamplerInfo points_downsampler_info_msg;

static std::chrono::time_point<std::chrono::system_clock> filter_start, filter_end;

static bool _output_log = false;
static std::ofstream ofs;
static std::string filename;

static void config_callback(const runtime_manager::ConfigDistanceFilter::ConstPtr& input)
{
  sample_num = input->sample_num;
}

static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointXYZI sampled_p;
  pcl::PointCloud<pcl::PointXYZI> scan;

  pcl::fromROSMsg(*input, scan);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  filtered_scan_ptr->header = scan.header;

  int points_num = scan.size();

  double w_total = 0.0;
  double w_step = 0.0;
  int m = 0;
  double c = 0.0;

  filter_start = std::chrono::system_clock::now();

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin(); item != scan.end(); item++)
  {
    w_total += item->x * item->x + item->y * item->y + item->z * item->z;
  }
  w_step = w_total / sample_num;

  pcl::PointCloud<pcl::PointXYZI>::const_iterator item = scan.begin();
  for (m = 0; m < sample_num; m++)
  {
    while (m * w_step > c)
    {
      item++;
      c += item->x * item->x + item->y * item->y + item->z * item->z;
    }
    sampled_p.x = item->x;
    sampled_p.y = item->y;
    sampled_p.z = item->z;
    sampled_p.intensity = item->intensity;
    filtered_scan_ptr->points.push_back(sampled_p);
  }

  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);

  filter_end = std::chrono::system_clock::now();

  filtered_msg.header = input->header;
  filtered_points_pub.publish(filtered_msg);

  points_downsampler_info_msg.header = input->header;
  points_downsampler_info_msg.filter_name = "distance_filter";
  points_downsampler_info_msg.original_points_size = points_num;
  points_downsampler_info_msg.filtered_points_size = filtered_scan_ptr->size();
  points_downsampler_info_msg.original_ring_size = 0;
  points_downsampler_info_msg.original_ring_size = 0;
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
  ros::init(argc, argv, "distance_filter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("output_log", _output_log);
  if(_output_log == true){
	  char buffer[80];
	  std::time_t now = std::time(NULL);
	  std::tm *pnow = std::localtime(&now);
	  std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
	  filename = "distance_filter_" + std::string(buffer) + ".csv";
	  ofs.open(filename.c_str(), std::ios::app);
  }

  // Publishers
  filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
  points_downsampler_info_pub = nh.advertise<points_downsampler::PointsDownsamplerInfo>("/points_downsampler_info", 1000);

  // Subscribers
  ros::Subscriber config_sub = nh.subscribe("config/distance_filter", 10, config_callback);
  ros::Subscriber scan_sub = nh.subscribe("points_raw", 10, scan_callback);

  ros::spin();

  return 0;
}
