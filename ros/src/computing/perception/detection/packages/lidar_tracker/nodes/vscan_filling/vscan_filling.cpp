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

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



static ros::Publisher _pub;
const std::string FRAME = "/velodyne";

static void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ> vscan;
	pcl::fromROSMsg(*msg, vscan);

	pcl::PointCloud<pcl::PointXYZ> filling_cloud;

	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = vscan.begin(); item != vscan.end(); item++) {
		if ((item->x == 0 && item->y == 0))
			continue;

		pcl::PointXYZ p;

		//push back bottom pointcloud
    p.x = item->x;
		p.y = item->y;
		p.z = item->z;
    filling_cloud.points.push_back(p);

		double bottom_z = item->z;
	//	std::cout << "bottom : " <<   bottom_z << std::endl;
		item++;

		//move to top pointcloud
		double top_z = item->z;
	//	std::cout << "top : " <<   top_z << std::endl;

		//filling pointcloud
		double step = 0.3; //meter
		int i = 1;
		while(1){
			p.z = bottom_z + step * (double)i;
		//	std::cout << i << " : " <<   p.z << std::endl;

			if(p.z > top_z)
				break;
			filling_cloud.points.push_back(p);
			i++;
		}

		//push back top pointcloud
		p.z = item->z;
		filling_cloud.points.push_back(p);

	}

	sensor_msgs::PointCloud2::Ptr map_ptr(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(filling_cloud, *map_ptr);
	map_ptr->header = msg->header;
	_pub.publish(*map_ptr);

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "vscan_filling");
	ros::NodeHandle nh;

	_pub = nh.advertise<sensor_msgs::PointCloud2>("/vscan_filling_cloud", 1, true);
	ros::Subscriber sub = nh.subscribe("vscan_points", 100, Callback);

	ros::spin();
	return 0;
}
