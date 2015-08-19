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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <map_db.h>

#define UPDATE_RATE	200
#define MARGIN		100
#define DEBUG_PRINT

ros::Publisher pub;
ros::Publisher stat_publisher;
std_msgs::Bool pmap_stat_msg;
int show = 0;
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
std::vector<std::string> pcd_file_list;

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

std::vector<PcdFileRange> read_pcdfilerange(const char* filename, double margin)
{
  Tbl tbl = read_csv(filename);
  size_t i, n = tbl.size();
  std::vector<PcdFileRange> ret(n);
  for (i=0; i<n; i++) {
    ret[i].name = tbl[i][0];
    ret[i].x_min = std::stod(tbl[i][1]) - margin;
    ret[i].y_min = std::stod(tbl[i][2]) - margin;
    ret[i].z_min = std::stod(tbl[i][3]) - margin;
    ret[i].x_max = std::stod(tbl[i][4]) + margin;
    ret[i].y_max = std::stod(tbl[i][5]) + margin;
    ret[i].z_max = std::stod(tbl[i][6]) + margin;
  }
  return ret;
}



void gnssposeCallback(const geometry_msgs::PoseStamped msg)
{

	if(show++%UPDATE_RATE != 0) return;

	sensor_msgs::PointCloud2 pcd, add;
	int loaded = 0;

	for (int i = 0; i < (int)files.size(); i++) {
#if 0
	  if(files[i].x_min < msg.pose.position.x && msg.pose.position.x < files[i].x_max &&
	     files[i].y_min < msg.pose.position.y && msg.pose.position.y < files[i].y_max &&
	     files[i].z_min < msg.pose.position.z && msg.pose.position.z < files[i].z_max) ;
#else
	  if(files[i].x_min < msg.pose.position.x && msg.pose.position.x < files[i].x_max &&
	     files[i].y_min < msg.pose.position.y && msg.pose.position.y < files[i].y_max);
#endif
	  else continue;

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
	  pmap_stat_msg.data = true;

	  pub.publish(pcd);
	  stat_publisher.publish(pmap_stat_msg);
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcd_read");
	ros::NodeHandle n;
	ros::Subscriber gnss_pose_sub;
	pub = n.advertise<sensor_msgs::PointCloud2>("/points_map", 1, true);
	stat_publisher = n.advertise<std_msgs::Bool>("/pmap_stat", 100);

	//download PCD files
	static GetFile gf;
	if(argc >= 3){
	  std::string host_name = argv[1];
	  int port = std::atoi(argv[2]);
	  gf = GetFile(host_name, port);
	  argc--;
	  argv++;
	  argc--;
	  argv++;
	} else {
	  gf = GetFile();
	}

	// prepare download dir
	struct stat st;
	std::string dirname = "/data/map/japan/aichi/nagoya/moriyama/pointcloud";
	std::string tmp_dir = "/tmp" + dirname;
	if(stat(tmp_dir.c_str(), &st) != 0) {
	  std::istringstream ss(dirname);
	  std::string column;
	  std::getline(ss, column, '/');
	  tmp_dir = "/tmp";
	  while (std::getline(ss, column, '/')) {
	    tmp_dir += "/" + column;
	    errno = 0;
	    int ret = mkdir(tmp_dir.c_str(), 0755);
	    if(ret < 0 && errno != EEXIST) perror("mkdir");
	  }
	  std::cerr << "mkdir " << tmp_dir << std::endl;
	}
	std::cerr << "start get file = arealist.txt ... ";
	if(gf.GetHTTPFile(dirname+"/"+"arealist.txt") == 0) {
	  std::cerr << " download done" << std::endl;
	} else {
	  std::cerr << " download failed" << std::endl;
	}

	int update = 1;

	// skip argv[0]
	argc--;
	argv++;

	if (argc < 1) {
	  fprintf(stderr, "Usage: map_db points_map_quuery <1x1|3x3|5x5|7x7|9x9|noupdate>\n");
	  return 0;
	}

	double margin = 0;
	std::string area(argv[0]);
	if(area == "1x1") {
	  ;
	} else if(area == "3x3") {
	  margin = MARGIN * 1;
	} else if(area == "5x5") {
	  margin = MARGIN * 2;
	} else if(area == "7x7") {
	  margin = MARGIN * 3;
	} else if(area == "9x9") {
	  margin = MARGIN * 4;
	} else if(area == "noupdate") {
	  update = 0;
	} else {
	  argc++;
	  argv--;
	}
	argc--;
	argv++;

	tmp_dir = "/tmp" + dirname+"/arealist.txt";
	files = read_pcdfilerange(tmp_dir.c_str(), margin);

	if (update) {
	  gnss_pose_sub = n.subscribe("gnss_pose", 1000, gnssposeCallback);
	  if(argc == 1) {
	    files = read_pcdfilerange(argv[0], margin);
	  } else {
	    std::vector<PcdFileRange> filelists = read_pcdfilerange(argv[0], margin);
	    argc--;
	    argv++;
	    while(argc > 0) {
#ifdef DEBUG_PRINT
	      fprintf(stderr, "** name=%s **\n", *argv);
#endif
	      for(int i = 0; i < (int)filelists.size(); i++) {
		if(filelists[i].name.compare(*argv) == 0) {
		  files.push_back(filelists[i]);
		  continue;
		}
	      }
	      argc--;
	      argv++;
	    }
	  }

	} else if (update == 0) {
	  fprintf(stderr, "noupdate files.size()=%d\n", (int)files.size());
	  sensor_msgs::PointCloud2 pcd, add;
	  int loaded = 0;

	  for (int i = 0; i < (int)files.size(); i++) {
	    std::string filename = dirname+"/"+basename((char *)files[i].name.c_str());
std::cerr << "start get file = " << filename;
	    if(gf.GetHTTPFile(filename) == 0) {
	      pcd_file_list.push_back("/tmp"+filename);
	      std::cerr << " download done" << std::endl;
	    } else {
	      std::cerr << " download failed" << std::endl;
	    }
	  }
	  for(auto x: pcd_file_list) {
	    if(loaded == 0) {
	      if(pcl::io::loadPCDFile(x.c_str(), pcd) == -1) 
	      {
		fprintf(stderr, "load failed %s\n", x.c_str());
	      } else loaded = 1;
	    } else {
	      if(pcl::io::loadPCDFile(x.c_str(), add) == -1) 
	      {
		fprintf(stderr, "load failed %s\n", x.c_str());
	      }

	      pcd.width += add.width;
	      pcd.row_step += add.row_step;
	      pcd.data.insert(pcd.data.end(), add.data.begin(), add.data.end());
	    }
#ifdef DEBUG_PRINT
	    fprintf(stderr, "load %s\n", x.c_str());
#endif

	  }

	  pmap_stat_msg.data = true;

	  pcd.header.frame_id = "/map";
	  pub.publish(pcd);
	  stat_publisher.publish(pmap_stat_msg);
	  fprintf(stderr, "\npoint_map published\n");
	} else {
	  fprintf(stderr, "  Usage: map_db points_map_query <1x1|3x3|5x5|7x7|9x9|noupdate> \n");
	  std::exit(1);
	}

	ros::spin();
	return 0;
}
