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
#include <std_msgs/String.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <map_db.h>
#include <sys/stat.h>

#define UPDATE_RATE	200
#define MARGIN		100
#define DEBUG_PRINT

int update_rate;

ros::Publisher pub;
ros::Publisher stat_publisher;
std_msgs::Bool pmap_stat_msg;
int gnss_show = 0;
int current_pose_show = 0;
int file_num = 0;
double margin = 0;

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
std::string dirname;
GetFile gf;
int download = 0;
std::vector<std::string> load_arealists;

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

std::vector<PcdFileRange> read_local_pcdfilerange(const char* filename, double margin)
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

std::vector<PcdFileRange> read_pcdfilerange(const char* filename, double margin, std::string dirname)
{
	Tbl tbl = read_csv(filename);
	size_t i, n = tbl.size();
	std::vector<PcdFileRange> ret(n);
	for (i=0; i<n; i++) {
		ret[i].name = dirname + basename((char *)tbl[i][0].c_str());
		ret[i].x_min = std::stod(tbl[i][1]) - margin;
		ret[i].y_min = std::stod(tbl[i][2]) - margin;
		ret[i].z_min = std::stod(tbl[i][3]) - margin;
		ret[i].x_max = std::stod(tbl[i][4]) + margin;
		ret[i].y_max = std::stod(tbl[i][5]) + margin;
		ret[i].z_max = std::stod(tbl[i][6]) + margin;
	}
	return ret;
}

int get_arealist(std::string dirname, GetFile gf) {
	if(dirname.empty() == true) return -1;
	//std::cerr << "++ dirname=" << dirname << " ++\n";
	struct stat st;
	std::string tmp_dir = "/tmp" + dirname + "/arealist.txt";
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
		if(gf.GetHTTPFile(dirname+"/"+"arealist.txt") == 0) {
			std::cerr << " download done" << std::endl;
		} else {
			std::cerr << " download failed" << std::endl;
			return -1;
		}
	}

	tmp_dir = "/tmp" + dirname+"/arealist.txt";
	std::vector<PcdFileRange> tmp_files;
	tmp_files = read_pcdfilerange(tmp_dir.c_str(), margin, "/tmp"+dirname);
	for(auto file: tmp_files) {
		files.push_back(file);
	}
	std::cerr << "read " << tmp_dir << " done\n";

	return 0;
}

int check_load_pcdfile(double x, double y) {
	if(download) {
		std::string tmp_dir1 = "/data/map/";
		std::string tmp_dir2 = "/data/map/";
		int x_min = (int)(x - margin);
		int x_max = (int)(x + margin);
		int y_min = (int)(y - margin);
		int y_max = (int)(y + margin);
		x_min -= x_min % 1000;
		x_max -= x_max % 1000;
		y_min -= y_min % 1000;
		y_max -= y_max % 1000;

		tmp_dir1 += std::to_string(y_min) +  "/" + std::to_string(x_min) + "/pointcloud/";
		tmp_dir2 += std::to_string(y_max) +  "/" + std::to_string(x_max) + "/pointcloud/";

		int loaded = 0;
		for(auto list: load_arealists) {
			if(list.compare(tmp_dir1) == 0) {
				loaded = 1;
				break;
			}
		}
		if(loaded == 0) {
			if(get_arealist(tmp_dir1, gf) == 0) {
				load_arealists.insert(load_arealists.begin(), tmp_dir1);
			}
		}

		if(tmp_dir1 != tmp_dir2) {
			loaded = 0;
			for(auto list: load_arealists) {
				if(list.compare(tmp_dir2) == 0) {
					loaded = 1;
					break;
				}
			}

			if(loaded == 0) {
				if(get_arealist(tmp_dir2, gf) == 0) {
					load_arealists.insert(load_arealists.begin(), tmp_dir2);
				}
			}
		}
	}

	sensor_msgs::PointCloud2 pcd, add;
	int loaded = 0;

	for (int i = 0; i < (int)files.size(); i++) {
#if 0
		if(files[i].x_min < x && msg.pose.position.x < files[i].x_max &&
		   files[i].y_min < y && msg.pose.position.y < files[i].y_max &&
		   files[i].z_min < z && msg.pose.position.z < files[i].z_max) ;
#else
		if(files[i].x_min < x && x < files[i].x_max &&
		   files[i].y_min < y && y < files[i].y_max);
#endif
	  else continue;

		if(download) {
			struct stat st;
			if(stat(files[i].name.c_str(), &st) != 0) {
			  std::string filename = files[i].name.substr(4);
			  std::cerr << "start get file=" << filename;
			  if(gf.GetHTTPFile(filename) == 0) {
				std::cerr << " download done" << std::endl;
			  } else {
				std::cerr << " download failed" << std::endl;
			  }
			}
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
	  pmap_stat_msg.data = true;

	  pub.publish(pcd);
	  stat_publisher.publish(pmap_stat_msg);
	}

	return 0;
}


void gnss_pose_callback(const geometry_msgs::PoseStamped msg)
{
	if(gnss_show++%update_rate != 0) return;

#ifdef DEBUG_PRINT
	std::cerr << "call gnss_pose_callback\n";
#endif
	check_load_pcdfile(msg.pose.position.x, msg.pose.position.y);
	gnss_show = 1;
}

static void current_pose_callback(const geometry_msgs::PoseStamped &pose)
{
//	std::cerr << "call current_pose_callback\n";
//	std::cerr << "x=" << pose.pose.position.x << ", y=" << pose.pose.position.y << ", gnss_show=" << gnss_show << std::endl;

	gnss_show = 1;
	if(current_pose_show++%update_rate != 0) return;

	check_load_pcdfile(pose.pose.position.x, pose.pose.position.y);
	current_pose_show = 1;
	gnss_show = 1;
}

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input)
{
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try{
		ros::Time now = ros::Time(0);
		listener.waitForTransform("/map", "/world", now, ros::Duration(10.0));
		listener.lookupTransform("/map", "world", now, transform);
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s", ex.what());
	}

	std::cout << "x: " << transform.getOrigin().x() << std::endl;
	std::cout << "y: " << transform.getOrigin().y() << std::endl;
	std::cout << "z: " << transform.getOrigin().z() << std::endl;
	std::cerr << "x=" << input->pose.pose.position.x + transform.getOrigin().x() << std::endl;
	std::cerr << "y=" << input->pose.pose.position.y + transform.getOrigin().y() << std::endl;

	check_load_pcdfile(input->pose.pose.position.x + transform.getOrigin().x(), input->pose.pose.position.y + transform.getOrigin().y());

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcd_read");
	ros::NodeHandle n;
	ros::Subscriber gnss_pose_sub;
	ros::Subscriber current_pose_sub;
	ros::Subscriber initial_pose_sub;

	pub = n.advertise<sensor_msgs::PointCloud2>("/points_map", 1, true);
	stat_publisher = n.advertise<std_msgs::Bool>("/pmap_stat", 100);
	n.param<int>("points_map_loader/update_rate", update_rate, UPDATE_RATE);
	std::cout << "update_rate=" << update_rate << std::endl;

	int update = 1;

	// skip argv[0]
	argc--;
	argv++;

	if (argc < 2) {
	  fprintf(stderr, "Usage: map_file points_map_loader <1x1|3x3|5x5|7x7|9x9|noupdate> <arealist file> [pcd files]\n");
	  return 0;
	}

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

	std::string name(argv[0]);
	if(name == "download") {
		std::cerr << "pmap_loader download mode\n";
		download = 1;
		argc--;
		argv++;

		if(argc < 2 && update == 0){
			std::cerr << "Usage: points_map_loader noupdate download <x> <y>\n";
			std::exit(1);
		} else if(argc >=2) {
			dirname = "/data/map/";
			int i = std::atoi(argv[0]);
			i -= i % 1000;
			dirname += std::to_string(i);
			dirname += "/";
			i = std::atoi(argv[1]);
			i -= i % 1000;
			dirname += std::to_string(i);
			dirname += "/pointcloud/";
			argc -= 2;
			argv++;
			argv++;
		}
		if(argc >= 2){
			std::string host_name = argv[0];
			int port = std::atoi(argv[1]);
			gf = GetFile(host_name, port);
			argc -= 2;
			argv++;
			argv++;
		} else {
			gf = GetFile();
		}

		get_arealist(dirname, gf);
	} else {
		std::cerr << "pmap_loader local file mode\n";
	}

	if (update) {
		gnss_pose_sub = n.subscribe("gnss_pose", 1000, gnss_pose_callback);
		current_pose_sub = n.subscribe("current_pose", 1000, current_pose_callback);
		initial_pose_sub = n.subscribe("initialpose", 1000, initialpose_callback);

	  if(argc == 1) {
			files = read_local_pcdfilerange(argv[0], margin);
		} else if(argc > 1) {
			std::vector<PcdFileRange> filelists = read_local_pcdfilerange(argv[0], margin);
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

	}
	if (update == 0 || (update == 1 && download == 1) ) {
	  sensor_msgs::PointCloud2 pcd, add;
		if(download) {
			for (int i = 0; i < (int)files.size(); i++) {
				std::string filename = dirname+"/"+basename((char *)files[i].name.c_str());
				struct stat st;
				std::string tmp_file = "/tmp" + filename;
				if(stat(tmp_file.c_str(), &st) == 0) {
					pcd_file_list.push_back("/tmp"+filename);
					continue;
				}
				std::cerr << "** start get file = " << filename;
				if(gf.GetHTTPFile(filename) == 0) {
					pcd_file_list.push_back("/tmp"+filename);
					std::cerr << " download done **" << std::endl;
				} else {
					std::cerr << " download failed" << std::endl;
				}
			}
		} else {
			while(argc > 0) {
				std::cerr << "file=" << argv[0] << std::endl;
				pcd_file_list.push_back(argv[0]);
				argc--;
				argv++;
			}
		}
	  int loaded = 0;
		for(auto x: pcd_file_list) {
#ifdef DEBUG_PRINT
			fprintf(stderr, "load %s\n", x.c_str());
#endif
	    if(loaded == 0) {
				if(pcl::io::loadPCDFile(x.c_str(), pcd) == -1) {
					fprintf(stderr, "  load failed %s\n", x.c_str());
	      } else loaded = 1;
	    } else {
				if(pcl::io::loadPCDFile(x.c_str(), add) == -1) {
					fprintf(stderr, "  load failed %s\n", x.c_str());
	      }

	      pcd.width += add.width;
	      pcd.row_step += add.row_step;
	      pcd.data.insert(pcd.data.end(), add.data.begin(), add.data.end());
	    }
	  }

	  pmap_stat_msg.data = true;

	  pcd.header.frame_id = "/map";
	  pub.publish(pcd);
	  stat_publisher.publish(pmap_stat_msg);
	  fprintf(stderr, "\npoint_map published\n");

	} else if(update == 0 && download == 0) {
	  fprintf(stderr, "  Usage: map_file points_map_loader <1x1|3x3|5x5|7x7|9x9|noupdate> <arealist file> [pcd files]\n");
	  std::exit(1);
	}

	ros::spin();
	return 0;
}
