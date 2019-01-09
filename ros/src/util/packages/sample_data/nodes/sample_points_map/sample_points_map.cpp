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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

ros::Publisher pub;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcd_read");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/points_map", 1, true);

	// skip argv[0]
	argc--;
	argv++;

	if (argc <= 0) {
		fprintf(stderr, "file name ?\n");
		return 0;
	}

	sensor_msgs::PointCloud2 msg, add;

	pcl::io::loadPCDFile(*argv++, msg);
	// ToDo: error check
	argc--;

	while (argc > 0) {
		//sensor_msgs::PointCloud2 add;
		pcl::io::loadPCDFile(*argv++, add);
		// ToDo: error check
		argc--;

		msg.width += add.width;
		msg.row_step += add.row_step;
		msg.data.insert(msg.data.end(), add.data.begin(), add.data.end());

		fprintf(stderr, "%d%c", argc, argc ? ' ' : '\n');
	}
	msg.header.frame_id = "/map";

	pub.publish(msg);
	ros::spin();
	return 0;
}
