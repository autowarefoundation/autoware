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
