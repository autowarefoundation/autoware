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

/*
pos_downloader
This node get location data from db server and
publish data as ractangular plane
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <pthread.h>

#include <geo_pos_conv.hh>
#include <pos_db.h>

#define MYNAME		"pos_downloader"
#define MARKERNAME	"mo_marker"
#define STARTTIME	(0)		// now
#define DELAYSEC	(3)

using namespace std;

static string host_name = "db3.ertl.jp";
static int db_port = 5678;
static int sleep_msec = 500;
static double life_time = 5.0;

static ros::Publisher pub;

static SendData sd;

static std::vector<std::string> split(const string& input, char delimiter)
{
    std::istringstream stream(input);

    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

static int result_to_marker(const std::string& result, visualization_msgs::Marker& marker, int is_swap)
{
  std::vector<std::string> columns = split(result, '\t');
  static int id = 1;

  // id,x,y,z,or_x,or_y,or_z,or_w,lon,lat,tm
  if(columns.size() != 11)
    return -1;

  if (columns[0].find("ndt_pose", 0) != string::npos) {
    marker.type = visualization_msgs::Marker::ARROW;
    marker.lifetime = ros::Duration();
    marker.id = 0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
  } else if (columns[0].find("pedestrian_pose", 0) != string::npos) {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(life_time);
    marker.id = id++;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
  } else if (columns[0].find("car_pose", 0) != string::npos) {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(life_time);
    marker.id = id++;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
  } else {
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration(life_time);
    marker.id = id++;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
  }

  if (is_swap) {
    marker.pose.position.x = std::stod(columns[2]);
    marker.pose.position.y = std::stod(columns[1]);
    marker.pose.orientation.x = std::stod(columns[5]);
    marker.pose.orientation.y = std::stod(columns[4]);
  } else {
    marker.pose.position.x = std::stod(columns[1]);
    marker.pose.position.y = std::stod(columns[2]);
    marker.pose.orientation.x = std::stod(columns[4]);
    marker.pose.orientation.y = std::stod(columns[5]);
  }
  marker.pose.position.z = std::stod(columns[3]);
  marker.pose.orientation.z = std::stod(columns[6]);
  marker.pose.orientation.w = std::stod(columns[7]);
  if (marker.pose.position.x < -1.79E308 ||
	marker.pose.position.y < -1.79E308 ||
	marker.pose.position.z < -1.79E308) {
    geo_pos_conv geo;
    double lon = std::stod(columns[8]);
    double lat = std::stod(columns[9]);
    geo.set_plane(7); // Aichi-ken
    geo.set_llh_nmea_degrees(lat, lon, 0/*h*/);
    if (is_swap) {
      marker.pose.position.x = geo.y();
      marker.pose.position.y = geo.x();
    } else {
      marker.pose.position.x = geo.x();
      marker.pose.position.y = geo.y();
    }
    marker.pose.position.z = geo.z();
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
  }

  std::cout << "pose " << marker.pose.position.x << ","
	<< marker.pose.position.y << ","
	<< marker.pose.position.z << ":"
	<< marker.pose.orientation.x << ","
	<< marker.pose.orientation.y << ","
	<< marker.pose.orientation.z << ","
	<< marker.pose.orientation.w << std::endl;

  return 0;
}

static void marker_publisher(const std_msgs::String& msg)
{
  std::vector<std::string> db_data = split(msg.data, '\n');

  for (const std::string& row : db_data) {
    if(row.empty())
      continue;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = MARKERNAME;
    marker.action = visualization_msgs::Marker::ADD;
    int ret = result_to_marker(row, marker, 1);
    if (ret != 0)
      continue;

    pub.publish(marker);
  }
}

// create "YYYY-mm-dd HH:MM:SS.sss"
static int create_timestr(time_t sec, int nsec, char *str, size_t size)
{
  std::tm *nowtm;

  nowtm = std::localtime(&sec);
  return std::snprintf(str, size, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
	nowtm->tm_year + 1900, nowtm->tm_mon + 1, nowtm->tm_mday,
	nowtm->tm_hour, nowtm->tm_min, nowtm->tm_sec, nsec/1000/1000);
}

//wrap SendData class
static uint64_t send_sql(time_t diff_sec, uint64_t prev)
{
  std::string data = make_header(1, 1);
  string db_response;
  std_msgs::String msg;
  ros::Time now = ros::Time::now();
  time_t now_sec = now.toSec() - diff_sec;
  int now_nsec = now.toNSec()%(1000*1000*1000);
  char timestr[2][64]; // "YYYY-mm-dd HH:MM:SS.sss"

  if (prev == 0) {
    time_t psec = now_sec - sleep_msec/1000;
    int pnsec = now_nsec - (sleep_msec%1000)*1000*1000;
    if (pnsec < 0) {
	psec--;
	pnsec += 1000*1000*1000;
    }
    create_timestr(psec, pnsec, timestr[0], sizeof(timestr[0]));
  } else {
    create_timestr(prev/1000/1000/1000, prev%(1000*1000*1000),
	timestr[0], sizeof(timestr[0]));
  }
  create_timestr(now_sec, now_nsec, timestr[1], sizeof(timestr[1]));

  data += "SELECT id,x,y,z,or_x,or_y,or_z,or_w,lon,lat,tm FROM POS "
	"WHERE tm >= '";
  data += timestr[0];
  data += "' AND tm < '";
  data += timestr[1];
  data += "';\r\n";

  int ret = sd.Sender(data, db_response);
  if (ret == -1) {
    std::cerr << "sd.Sender() failed" << std::endl;
  } else {
    std::cout << "return data: \"" << db_response << "\"" << std::endl;
    if (db_response.size() > 4) {
      msg.data = db_response.substr(4).c_str();
      marker_publisher(msg);
    }
  }

  return now_sec*1000*1000*1000 + now_nsec;
}

static void* intervalCall(void *unused)
{
  double *args = (double *)unused;
  double diff_sec = args[1];
  time_t prev_sec = 0;

  if (args[0] != 0)
    diff_sec += ros::Time::now().toSec() - args[0];
  cout << "diff=" << diff_sec << endl;

  while (1) {
    prev_sec = send_sql((time_t)diff_sec, prev_sec);
    usleep(sleep_msec*1000);
  }

  return nullptr;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, MYNAME) ;
  ros::NodeHandle nh;
  pthread_t th;
  double args[2];

  cout << MYNAME << endl;

  pub = nh.advertise<visualization_msgs::Marker>(MARKERNAME, 1);
  nh.param<double>(MYNAME "/time", args[0], STARTTIME);
  cout << "time=" << args[0] << endl;
  nh.param<double>(MYNAME "/delay", args[1], DELAYSEC);
  cout << "delay=" << args[1] << endl;

  sd = SendData(host_name, db_port);

  if (pthread_create(&th, nullptr, intervalCall, (void *)args) != 0) {
    std::perror("pthread_create");
    std::exit(1);
  }

  pthread_detach(th);

  ros::spin();

  return 0;
}
