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

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <waypoint_follower/LaneArray.h>

class RequestQueue {
private:
	std::queue<geometry_msgs::Point> queue_;
	std::queue<geometry_msgs::Point> look_ahead_queue_;
	std::mutex mtx_;
	std::condition_variable cv_;

public:
	void enqueue(const geometry_msgs::Point& p);
	void enqueue_look_ahead(const geometry_msgs::Point& p);
	void clear();
	void clear_look_ahead();
	geometry_msgs::Point dequeue();
};

void RequestQueue::enqueue(const geometry_msgs::Point& p)
{
	std::unique_lock<std::mutex> lock(mtx_);
	queue_.push(p);
	cv_.notify_all();
}

void RequestQueue::enqueue_look_ahead(const geometry_msgs::Point& p)
{
	std::unique_lock<std::mutex> lock(mtx_);
	look_ahead_queue_.push(p);
	cv_.notify_all();
}

void RequestQueue::clear()
{
	std::unique_lock<std::mutex> lock(mtx_);
	while (!queue_.empty())
		queue_.pop();
}

void RequestQueue::clear_look_ahead()
{
	std::unique_lock<std::mutex> lock(mtx_);
	while (!look_ahead_queue_.empty())
		look_ahead_queue_.pop();
}

geometry_msgs::Point RequestQueue::dequeue()
{
	std::unique_lock<std::mutex> lock(mtx_);
	while (queue_.empty() && look_ahead_queue_.empty())
		cv_.wait(lock);
	if (!queue_.empty()) {
		geometry_msgs::Point p = queue_.front();
		queue_.pop();
		return p;
	} else {
		geometry_msgs::Point p = look_ahead_queue_.front();
		look_ahead_queue_.pop();
		return p;
	}
}

#define UPDATE_RATE	1000
#define MARGIN		100
#define DEBUG_PRINT

int update_rate;
int fallback_time;

ros::Publisher pub;
ros::Publisher stat_publisher;
ros::Time gnss_check_time;
ros::Time current_check_time;
std_msgs::Bool pmap_stat_msg;
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
std::vector<PcdFileRange> areas;
std::mutex areas_mtx;
std::vector<std::string> pcd_file_list;
RequestQueue request_queue;
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
	  fprintf(stderr, "load %s\n", files[i].name.c_str());
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
	ros::Time now = ros::Time::now();
	if (((now - current_check_time).toSec() * 1000) < fallback_time)
		return;
	if (((now - gnss_check_time).toSec() * 1000) < update_rate)
		return;
	check_load_pcdfile(msg.pose.position.x, msg.pose.position.y);
	gnss_check_time = now;
}

static void current_pose_callback(const geometry_msgs::PoseStamped &pose)
{
	ros::Time now = ros::Time::now();
	if (((now - current_check_time).toSec() * 1000) < update_rate)
		return;
	check_load_pcdfile(pose.pose.position.x, pose.pose.position.y);
	current_check_time = now;
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

static void waypoints_callback(const waypoint_follower::LaneArray& msg)
{
	if (msg.lanes.size() == 0)
		return;

	request_queue.clear_look_ahead();

	for (const waypoint_follower::lane& l : msg.lanes) {
		size_t end = l.waypoints.size() - 1;
		double distance = 0;
		double threshold = (MARGIN / 2) + margin; // better way?
		for (size_t i = 0; i <= end; ++i) {
			if (i == 0 || i == end) {
				geometry_msgs::Point p;
				p.x = l.waypoints[i].pose.pose.position.x;
				p.y = l.waypoints[i].pose.pose.position.y;
				request_queue.enqueue_look_ahead(p);
			} else {
				geometry_msgs::Point p1, p2;
				p1.x = l.waypoints[i].pose.pose.position.x;
				p1.y = l.waypoints[i].pose.pose.position.y;
				p2.x = l.waypoints[i - 1].pose.pose.position.x;
				p2.y = l.waypoints[i - 1].pose.pose.position.y;
				distance += hypot(p2.x - p1.x, p2.y - p1.y);
				if (distance > threshold) {
					request_queue.enqueue_look_ahead(p1);
					distance = 0;
				}
			}
		}
	}
}

static void download_map()
{
	while (true) {
		geometry_msgs::Point p = request_queue.dequeue();

		std::string tmp_dir1 = "/data/map/";
		std::string tmp_dir2 = "/data/map/";
		int x_min = (int)(p.x - margin);
		int x_max = (int)(p.x + margin);
		int y_min = (int)(p.y - margin);
		int y_max = (int)(p.y + margin);
		x_min -= x_min % 1000;
		x_max -= x_max % 1000;
		y_min -= y_min % 1000;
		y_max -= y_max % 1000;

		tmp_dir1 += std::to_string(y_min) +  "/" + std::to_string(x_min) + "/pointcloud/";
		tmp_dir2 += std::to_string(y_max) +  "/" + std::to_string(x_max) + "/pointcloud/";

		bool loaded = false;
		for (auto list: load_arealists) {
			if (list.compare(tmp_dir1) == 0) {
				loaded = true;
				break;
			}
		}
		if (!loaded) {
			if (get_arealist(tmp_dir1, gf) == 0)
				load_arealists.insert(load_arealists.begin(), tmp_dir1);
		}

		if (tmp_dir1 != tmp_dir2) {
			loaded = false;
			for (auto list: load_arealists) {
				if (list.compare(tmp_dir2) == 0) {
					loaded = true;
					break;
				}
			}
			if (!loaded) {
				if (get_arealist(tmp_dir2, gf) == 0)
					load_arealists.insert(load_arealists.begin(), tmp_dir2);
			}
		}

		for (size_t i = 0; i < files.size(); ++i) {
			if (files[i].x_min < p.x && p.x < files[i].x_max &&
			    files[i].y_min < p.y && p.y < files[i].y_max) {
				bool loaded = false;
				for (const PcdFileRange& f: areas) {
					if (f.name.compare(files[i].name) == 0) {
						loaded = true;
						break;
					}
				}
				if (!loaded) {
					struct stat st;
					if (stat(files[i].name.c_str(), &st) != 0) {
						if (gf.GetHTTPFile(files[i].name.substr(4)) == 0) {
							std::unique_lock<std::mutex> lock(areas_mtx);
							areas.push_back(files[i]);
						}
					} else {
						std::unique_lock<std::mutex> lock(areas_mtx);
						areas.push_back(files[i]);
					}
				}
			}
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcd_read");
	ros::NodeHandle n;
	ros::Subscriber gnss_pose_sub;
	ros::Subscriber current_pose_sub;
	ros::Subscriber initial_pose_sub;
	ros::Subscriber waypoints_sub;

	pub = n.advertise<sensor_msgs::PointCloud2>("/points_map", 1, true);
	stat_publisher = n.advertise<std_msgs::Bool>("/pmap_stat", 1, true);
	n.param<int>("points_map_loader/update_rate", update_rate, UPDATE_RATE);
	fallback_time = update_rate * 2;
	std::string host_name;
	n.param<std::string>("points_map_loader/host_name", host_name, HTTP_HOSTNAME);
	int port;
	n.param<int>("points_map_loader/port", port, HTTP_PORT);
	std::string user;
	n.param<std::string>("points_map_loader/user", user, HTTP_USER);
	std::string password;
	n.param<std::string>("points_map_loader/password", password, HTTP_PASSWORD);
	std::cout << "update_rate=" << update_rate << std::endl;

	pmap_stat_msg.data = false;
	stat_publisher.publish(pmap_stat_msg);

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

		gf = GetFile(host_name, port, user, password);
	} else {
		std::cerr << "pmap_loader local file mode\n";
	}

	if (update) {
		gnss_check_time = current_check_time = ros::Time::now();
		gnss_pose_sub = n.subscribe("gnss_pose", 1000, gnss_pose_callback);
		current_pose_sub = n.subscribe("current_pose", 1000, current_pose_callback);
		initial_pose_sub = n.subscribe("initialpose", 1000, initialpose_callback);
		if (download == 1) {
			waypoints_sub = n.subscribe("traffic_waypoints_array", 1, waypoints_callback);
			try {
				std::thread downloader(download_map);
				downloader.detach();
			} catch (std::exception &ex) {
				std::cerr << "failed to create thread from " << ex.what() << std::endl;
			}
		}

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
	    fprintf(stderr, "load %s\n", x.c_str());
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
