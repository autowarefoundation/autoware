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
#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#define UPDATE_RATE	200
#define DEBUG_PRINT

ros::Publisher pub;
int show = 0;
int swap_x_y = 0;
int file_num = 0;

/* for pcdfilelist.csv */
struct PcdFileRange {
  std::string name;
  double x_min;
  double y_min;
  double z_min;
  double x_max;
  double y_max;
  double z_max;
};

std::vector<PcdFileRange> files;

typedef std::vector<std::vector<std::string>> Tbl;

Tbl read_csv(const char* filename)
{
  std::ifstream ifs(filename);
  std::string line;

  Tbl tbl;

  while (std::getline(ifs, line)) {
    std::istringstream ss(line);

    std::vector<std::string> columns;
    std::string column;
    while (std::getline(ss, column, ',')) {
      columns.push_back(column);
    }
    tbl.push_back(columns);
  }
  return tbl;
}

std::vector<PcdFileRange> read_pcdfilerange(const char* filename)
{
  Tbl tbl = read_csv(filename);
  size_t i, n = tbl.size();
  std::vector<PcdFileRange> ret(n);
  for (i=0; i<n; i++) {
    ret[i].name = tbl[i][0];
    ret[i].x_min = std::stod(tbl[i][1]);
    ret[i].y_min = std::stod(tbl[i][2]);
    ret[i].z_min = std::stod(tbl[i][3]);
    ret[i].x_max = std::stod(tbl[i][4]);
    ret[i].y_max = std::stod(tbl[i][5]);
    ret[i].z_max = std::stod(tbl[i][6]);
  }
  return ret;
}


void trajectoryCallback(const visualization_msgs::Marker msg)
{

	if(show++%UPDATE_RATE != 0) return;

	sensor_msgs::PointCloud2 pcd, add;
	int loaded = 0;

	for (int i = 1; i < (int)files.size(); i++) {
	  if(swap_x_y) {
	    if(files[i].y_min < msg.pose.position.x && msg.pose.position.x < files[i].y_max &&
	       files[i].x_min < msg.pose.position.y && msg.pose.position.y < files[i].x_max &&
	       files[i].z_min < msg.pose.position.z && msg.pose.position.z < files[i].z_max) ;
	    else continue;
	  } else {

	    if(files[i].x_min < msg.pose.position.x && msg.pose.position.x < files[i].x_max &&
	       files[i].y_min < msg.pose.position.y && msg.pose.position.y < files[i].y_max &&
	       files[i].z_min < msg.pose.position.z && msg.pose.position.z < files[i].z_max) ;
	    else continue;
	  }

	  if(loaded == 0) {
	    if(pcl::io::loadPCDFile(files[i].name.c_str(), pcd) == -1) 
	    {
	      fprintf(stderr, "load failed %s\n", files[i].name.c_str());
	    } else loaded = 1;
	  } else {
	    if(pcl::io::loadPCDFile(files[i].name.c_str(), add) == -1) 
	    {
	      fprintf(stderr, "load failed %s\n", files[i].name.c_str());
	    }

	    pcd.width += add.width;
	    pcd.row_step += add.row_step;
	    pcd.data.insert(pcd.data.end(), add.data.begin(), add.data.end());
	  }
#ifdef DEBUG_PRINT
	  fprintf(stderr, "load %s\n", files[i].name.c_str());
#endif
	}

#ifdef DEBUG_PRINT
	  fprintf(stderr, "---\n");
#endif

	if(loaded == 1) {
	  pcd.header.frame_id = "/map";
	  pub.publish(pcd);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcd_read");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("sample_trajectory", 1000, trajectoryCallback);
	pub = n.advertise<sensor_msgs::PointCloud2>("/points_map", 1, true);

	// skip argv[0]
	argc--;
	argv++;

	if (argc <= 0) {
	  fprintf(stderr, "file name ?\n");
	  return 0;
	}

	if(std::string(argv[0]) == "swap_x_y_on") {
	  printf("swap_x_y: on\n");
	  swap_x_y = 1;
	} else {
	  printf("swap_x_y : off\n");
	}
	if(argc == 2) {
	  files = read_pcdfilerange(argv[1]);
	} else {
	  std::vector<PcdFileRange> filelists = read_pcdfilerange(argv[1]);
	  argc -= 2;
	  argv += 2;
	  while(argc > 0) {
	  fprintf(stderr, "** %d, name=%s **\n", (int)strlen(*argv), *argv);
	    for(int i = 0; i < (int)filelists.size(); i++) {
	      if(filelists[i].name.compare(*argv) == 0) {
	        files.push_back(filelists[i]);
		continue;
	      }
	    }
	    argc--;
	    argv++;
	  }
	  fprintf(stderr, "files.size()=%d\n", (int)files.size());
	}

	ros::spin();
	return 0;
}
