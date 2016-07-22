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

#include <condition_variable>
#include <queue>
#include <thread>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include <waypoint_follower/LaneArray.h>

#include <map_db.h> // XXX wrong name

namespace {

class RequestQueue {
private:
	std::queue<geometry_msgs::Point> queue_; // takes priority over look_ahead_queue_
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

struct Area {
	std::string path;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
};

typedef std::vector<Area> AreaList;
typedef std::vector<std::vector<std::string>> Tbl;

constexpr int DEFAULT_UPDATE_RATE = 1000; // ms
constexpr double MARGIN_UNIT = 100; // meter
constexpr int ROUNDING_UNIT = 1000; // meter
const std::string AREALIST_FILENAME = "arealist.txt";
const std::string TEMPORARY_DIRNAME = "/tmp/";

int update_rate;
int fallback_rate;
double margin;
bool can_download;

ros::Time gnss_time;
ros::Time current_time;

ros::Publisher pcd_pub;
ros::Publisher stat_pub;
std_msgs::Bool stat_msg;

AreaList all_areas;
AreaList downloaded_areas;
std::mutex downloaded_areas_mtx;
std::vector<std::string> cached_arealist_paths;

GetFile gf;
RequestQueue request_queue;

Tbl read_csv(const std::string& path)
{
	std::ifstream ifs(path.c_str());
	std::string line;
	Tbl ret;
	while (std::getline(ifs, line)) {
		std::istringstream iss(line);
		std::string col;
		std::vector<std::string> cols;
		while (std::getline(iss, col, ','))
			cols.push_back(col);
		ret.push_back(cols);
	}
	return ret;
}

void write_csv(const std::string& path, const Tbl& tbl)
{
	std::ofstream ofs(path.c_str());
	for (const std::vector<std::string>& cols : tbl) {
		std::string line;
		for (size_t i = 0; i < cols.size(); ++i) {
			if (i == 0)
				line += cols[i];
			else
				line += "," + cols[i];
		}
		ofs << line << std::endl;
	}
}

AreaList read_arealist(const std::string& path)
{
	Tbl tbl = read_csv(path);
	AreaList ret;
	for (const std::vector<std::string>& cols : tbl) {
		Area area;
		area.path = cols[0];
		area.x_min = std::stod(cols[1]);
		area.y_min = std::stod(cols[2]);
		area.z_min = std::stod(cols[3]);
		area.x_max = std::stod(cols[4]);
		area.y_max = std::stod(cols[5]);
		area.z_max = std::stod(cols[6]);
		ret.push_back(area);
	}
	return ret;
}

void write_arealist(const std::string& path, const AreaList& areas)
{
	Tbl tbl;
	for (const Area& area : areas) {
		std::vector<std::string> cols;
		cols.push_back(area.path);
		cols.push_back(std::to_string(area.x_min));
		cols.push_back(std::to_string(area.y_min));
		cols.push_back(std::to_string(area.z_min));
		cols.push_back(std::to_string(area.x_max));
		cols.push_back(std::to_string(area.y_max));
		cols.push_back(std::to_string(area.z_max));
		tbl.push_back(cols);
	}
	write_csv(path, tbl);
}

bool is_downloaded(const std::string& path)
{
	struct stat st;
	return (stat(path.c_str(), &st) == 0);
}

bool is_in_area(double x, double y, const Area& area, double m)
{
	return ((area.x_min - m) <= x && x <= (area.x_max + m) && (area.y_min - m) <= y && y <= (area.y_max + m));
}

std::string create_location(int x, int y)
{
	x -= x % ROUNDING_UNIT;
	y -= y % ROUNDING_UNIT;
	return ("data/map/" + std::to_string(y) + "/" + std::to_string(x) + "/pointcloud/");
}

void cache_arealist(const Area& area, AreaList& areas)
{
	for (const Area& a : areas) {
		if (a.path == area.path)
			return;
	}
	areas.push_back(area);
}

int download(GetFile gf, const std::string& tmp, const std::string& loc, const std::string& filename)
{
	std::string pathname;
	pathname += tmp;
	std::istringstream iss(loc);
	std::string col;
	while (std::getline(iss, col, '/')) {
		pathname += col + "/";
		mkdir(pathname.c_str(), 0755);
	}

	return gf.GetHTTPFile(loc + filename);
}

void download_map()
{
	while (true) {
		geometry_msgs::Point p = request_queue.dequeue();

		int x = static_cast<int>(p.x);
		int y = static_cast<int>(p.y);
		int x_min = static_cast<int>(p.x - margin);
		int y_min = static_cast<int>(p.y - margin);
		int x_max = static_cast<int>(p.x + margin);
		int y_max = static_cast<int>(p.y + margin);

		std::vector<std::string> locs;
		locs.push_back(create_location(x, y));
		locs.push_back(create_location(x_min, y_min));
		locs.push_back(create_location(x_min, y_max));
		locs.push_back(create_location(x_max, y_min));
		locs.push_back(create_location(x_max, y_max));
		for (const std::string& loc : locs) { // XXX better way?
			std::string arealist_path = TEMPORARY_DIRNAME + loc + AREALIST_FILENAME;

			bool cached = false;
			for (const std::string& path : cached_arealist_paths) {
				if (path == arealist_path) {
					cached = true;
					break;
				}
			}
			if (cached)
				continue;

			AreaList areas;
			if (is_downloaded(arealist_path))
				areas = read_arealist(arealist_path);
			else {
				if (download(gf, TEMPORARY_DIRNAME, loc, AREALIST_FILENAME) != 0)
					continue;
				areas = read_arealist(arealist_path);
				for (Area& area : areas)
					area.path = TEMPORARY_DIRNAME + loc + basename(area.path.c_str());
				write_arealist(arealist_path, areas);
			}
			for (const Area& area : areas)
				cache_arealist(area, all_areas);
			cached_arealist_paths.push_back(arealist_path);
		}

		for (const Area& area : all_areas) {
			if (is_in_area(p.x, p.y, area, margin)) {
				int x_area = static_cast<int>(area.x_max - MARGIN_UNIT);
				int y_area = static_cast<int>(area.y_max - MARGIN_UNIT);
				std::string loc = create_location(x_area, y_area);
				if (is_downloaded(area.path) ||
				    download(gf, TEMPORARY_DIRNAME, loc, basename(area.path.c_str())) == 0) {
					std::unique_lock<std::mutex> lock(downloaded_areas_mtx);
					cache_arealist(area, downloaded_areas);
				}
			}
		}
	}
}

sensor_msgs::PointCloud2 create_pcd(const geometry_msgs::Point& p)
{
	sensor_msgs::PointCloud2 pcd, part;
	std::unique_lock<std::mutex> lock(downloaded_areas_mtx);
	for (const Area& area : downloaded_areas) {
		if (is_in_area(p.x, p.y, area, margin)) {
			if (pcd.width == 0)
				pcl::io::loadPCDFile(area.path.c_str(), pcd);
			else {
				pcl::io::loadPCDFile(area.path.c_str(), part);
				pcd.width += part.width;
				pcd.row_step += part.row_step;
				pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
			}
		}
	}

	return pcd;
}

sensor_msgs::PointCloud2 create_pcd(const std::vector<std::string>& pcd_paths, int* ret_err = NULL)
{
	sensor_msgs::PointCloud2 pcd, part;
	for (const std::string& path : pcd_paths) {
		// Following outputs are used for progress bar of Runtime Manager.
		if (pcd.width == 0) {
			if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << path << std::endl;
				if (ret_err) *ret_err = 1;
			}
		} else {
			if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
				std::cerr << "load failed " << path << std::endl;
				if (ret_err) *ret_err = 1;
			}
			pcd.width += part.width;
			pcd.row_step += part.row_step;
			pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
		}
		std::cerr << "load " << path << std::endl;
		if (!ros::ok()) break;
	}

	return pcd;
}

void publish_pcd(sensor_msgs::PointCloud2 pcd, const int* errp = NULL)
{
	if (pcd.width != 0) {
		pcd.header.frame_id = "map";
		pcd_pub.publish(pcd);

		if (errp == NULL || *errp == 0) {
			stat_msg.data = true;
			stat_pub.publish(stat_msg);
		}
	}
}

void publish_gnss_pcd(const geometry_msgs::PoseStamped& msg)
{
	ros::Time now = ros::Time::now();
	if (((now - current_time).toSec() * 1000) < fallback_rate)
		return;
	if (((now - gnss_time).toSec() * 1000) < update_rate)
		return;
	gnss_time = now;

	if (can_download)
		request_queue.enqueue(msg.pose.position);

	publish_pcd(create_pcd(msg.pose.position));
}

void publish_current_pcd(const geometry_msgs::PoseStamped& msg)
{
	ros::Time now = ros::Time::now();
	if (((now - current_time).toSec() * 1000) < update_rate)
		return;
	current_time = now;

	if (can_download)
		request_queue.enqueue(msg.pose.position);

	publish_pcd(create_pcd(msg.pose.position));
}

void publish_dragged_pcd(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		ros::Time zero = ros::Time(0);
		listener.waitForTransform("map", "world", zero, ros::Duration(10));
		listener.lookupTransform("map", "world", zero, transform);
	} catch (tf::TransformException &ex) {
		ROS_ERROR_STREAM("failed to create transform from " << ex.what());
	}

	geometry_msgs::Point p;
	p.x = msg.pose.pose.position.x + transform.getOrigin().x();
	p.y = msg.pose.pose.position.y + transform.getOrigin().y();

	if (can_download)
		request_queue.enqueue(p);

	publish_pcd(create_pcd(p));
}

void request_lookahead_download(const waypoint_follower::LaneArray& msg)
{
	request_queue.clear_look_ahead();

	for (const waypoint_follower::lane& l : msg.lanes) {
		size_t end = l.waypoints.size() - 1;
		double distance = 0;
		double threshold = (MARGIN_UNIT / 2) + margin; // XXX better way?
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

void print_usage()
{
	ROS_ERROR_STREAM("Usage:");
	ROS_ERROR_STREAM("rosrun map_file points_map_loader noupdate [PCD]...");
	ROS_ERROR_STREAM("rosrun map_file points_map_loader {1x1|3x3|5x5|7x7|9x9} AREALIST [PCD]...");
	ROS_ERROR_STREAM("rosrun map_file points_map_loader {1x1|3x3|5x5|7x7|9x9} download");
}

} // namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_map_loader");

	ros::NodeHandle n;

	if (argc < 3) {
		print_usage();
		return EXIT_FAILURE;
	}

	std::string area(argv[1]);
	if (area == "noupdate")
		margin = -1;
	else if (area == "1x1")
		margin = 0;
	else if (area == "3x3")
		margin = MARGIN_UNIT * 1;
	else if (area == "5x5")
		margin = MARGIN_UNIT * 2;
	else if (area == "7x7")
		margin = MARGIN_UNIT * 3;
	else if (area == "9x9")
		margin = MARGIN_UNIT * 4;
	else {
		print_usage();
		return EXIT_FAILURE;
	}

	std::string arealist_path;
	std::vector<std::string> pcd_paths;
	if (margin < 0) {
		can_download = false;
		for (int i = 2; i < argc; ++i) {
			std::string path(argv[i]);
			pcd_paths.push_back(path);
		}
	} else {
		std::string mode(argv[2]);
		if (mode == "download") {
			can_download = true;
			std::string host_name;
			n.param<std::string>("points_map_loader/host_name", host_name, HTTP_HOSTNAME);
			int port;
			n.param<int>("points_map_loader/port", port, HTTP_PORT);
			std::string user;
			n.param<std::string>("points_map_loader/user", user, HTTP_USER);
			std::string password;
			n.param<std::string>("points_map_loader/password", password, HTTP_PASSWORD);
			gf = GetFile(host_name, port, user, password);
		} else {
			can_download = false;
			arealist_path += argv[2];
			for (int i = 3; i < argc; ++i) {
				std::string path(argv[i]);
				pcd_paths.push_back(path);
			}
		}
	}

	pcd_pub = n.advertise<sensor_msgs::PointCloud2>("points_map", 1, true);
	stat_pub = n.advertise<std_msgs::Bool>("pmap_stat", 1, true);

	stat_msg.data = false;
	stat_pub.publish(stat_msg);

	ros::Subscriber gnss_sub;
	ros::Subscriber current_sub;
	ros::Subscriber initial_sub;
	ros::Subscriber waypoints_sub;
	if (margin < 0) {
		int err = 0;
		publish_pcd(create_pcd(pcd_paths, &err), &err);
	} else {
		n.param<int>("points_map_loader/update_rate", update_rate, DEFAULT_UPDATE_RATE);
		fallback_rate = update_rate * 2; // XXX better way?

		gnss_sub = n.subscribe("gnss_pose", 1000, publish_gnss_pcd);
		current_sub = n.subscribe("current_pose", 1000, publish_current_pcd);
		initial_sub = n.subscribe("initialpose", 1, publish_dragged_pcd);

		if (can_download) {
			waypoints_sub = n.subscribe("traffic_waypoints_array", 1, request_lookahead_download);
			try {
				std::thread downloader(download_map);
				downloader.detach();
			} catch (std::exception &ex) {
				ROS_ERROR_STREAM("failed to create thread from " << ex.what());
			}
		} else {
			AreaList areas = read_arealist(arealist_path);
			for (const Area& area : areas) {
				for (const std::string& path : pcd_paths) {
					if (path == area.path)
						cache_arealist(area, downloaded_areas);
				}
			}
		}

		gnss_time = current_time = ros::Time::now();
	}

	ros::spin();

	return 0;
}
