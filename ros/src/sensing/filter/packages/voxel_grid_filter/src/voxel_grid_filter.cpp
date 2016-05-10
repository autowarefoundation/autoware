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

ros::Publisher filtered_points_pub;

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ> scan;
    pcl::fromROSMsg(*input, scan);
    // pcl::PointCloud<velodyne_pointcloud::PointXYZIR> tmp;
    // pcl::fromROSMsg(*input, tmp);
    // scan.points.clear();
    // for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = tmp.begin(); item != tmp.end(); item++) {
    //     p.x = (double) item->x;
    //     p.y = (double) item->y;
    //     p.z = (double) item->z;
    //     if(item->ring >= min && item->ring <= max && item->ring % layer == 0 ){
    //         scan.points.push_back(p);
    //     }
    // }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
    // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
    if (voxel_leaf_size >= 0.1) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

        // Downsampling the velodyne scan using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(scan_ptr);
        voxel_grid_filter.filter(*filtered_scan_ptr);

        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
        filtered_points_pub.publish(filtered_msg);
    } else {
        filtered_points_pub.publish(*input);
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "voxel_grid_filter");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

    // Publishers
    filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

	// Subscribers
	ros::Subscriber scan_sub = nh.subscribe("points_raw", 10, scan_callback);

	ros::spin();

	return 0;
}
