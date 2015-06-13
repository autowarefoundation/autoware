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

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define MYNAME		"pos_downloader"
#define MARKERNAME	"mo_marker"
#define STARTTIME	(0)		// sec since 1970-01-01 (0==now)
#define DELAYSEC	(3)		// delay sec for pos_uploader
#define LIFETIME	(1)		// anonymous marker's lifetime
#define POSUP_DZ	(40)		// z offset of PosUp
#define PEDESTRIAN_DZ	(-2)		// z offset of pedestrian_pose

using namespace std;

static string host_name = "db3.ertl.jp";
static int db_port = 5678;
static int sleep_msec = 500;		// period
static double life_time;
static double posup_dz;
static double pedestrian_dz;

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

static void dbg_out_marker(visualization_msgs::Marker marker)
{
  std::cout << marker.id << " : "
	<< marker.pose.position.x << ","
	<< marker.pose.position.y << ","
	<< marker.pose.position.z << " : "
	<< marker.pose.orientation.x << ","
	<< marker.pose.orientation.y << ","
	<< marker.pose.orientation.z << ","
	<< marker.pose.orientation.w << std::endl;
}

static int publish_car(int id, int is_ndt, ros::Time now, geometry_msgs::Pose& pose)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = now;
  marker.ns = MARKERNAME;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = id++;
  marker.pose = pose;
  if (is_ndt) {
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://pos_db/model/prius_model.dae";
    marker.mesh_use_embedded_materials = true;
    marker.lifetime = ros::Duration();
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    tf::Quaternion q1;
    q1.setRPY(M_PI/2, 0, M_PI);
    tf::Quaternion q2(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
    tf::Quaternion q3;
    q3 = q2 * q1;

    marker.pose.position.z -= 2.0;
    marker.pose.orientation.x = q3.x();
    marker.pose.orientation.y = q3.y();
    marker.pose.orientation.z = q3.z();
    marker.pose.orientation.w = q3.w();

  } else {
    marker.type = visualization_msgs::Marker::CUBE;
    marker.lifetime = ros::Duration(life_time);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.color.a = 1.0;
    marker.scale.x = 4.4; // #A
    marker.scale.y = 1.6;
    marker.scale.z = 1.0; // #1
    marker.pose.position.z += 0.5; // == #1/2
    pub.publish(marker);
    dbg_out_marker(marker);

    marker.id = id++;
    marker.scale.x = 3.0; // #B
    marker.scale.y = 1.6;
    marker.scale.z = 0.6; // #2
    marker.pose = pose;
    marker.pose.position.x += (4.4 - 3.0) / 2; // == (#A - #B)/2
    marker.pose.position.z += 1.0 + 0.3; // == #1 + #2/2
  }

  pub.publish(marker);
  dbg_out_marker(marker);

  return id;
}

static int publish_pedestrian(int id, int is_pedestrian, ros::Time now,
			      geometry_msgs::Pose& pose)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = now;
  marker.ns = MARKERNAME;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.lifetime = ros::Duration(life_time);
  marker.id = id++;
  if (is_pedestrian) {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    pose.position.z += pedestrian_dz;
  } else {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    pose.position.z += posup_dz;
  }
  marker.color.a = 1.0;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 1.2; // #1
  marker.pose = pose;
  marker.pose.position.z += 0.6; // == #1/2
  pub.publish(marker);
  dbg_out_marker(marker);

  marker.id = id++;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.6; // #2
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;
  marker.pose = pose;
  marker.pose.position.z += 1.2 + 0.3 + 0.1; // == #1 + #2/2 + alpha
  pub.publish(marker);
  dbg_out_marker(marker);

  return id;
}

static int result_to_marker(const string& idstr, ros::Time now,
			    geometry_msgs::Pose& pose, int is_swap)
{
  static int id = 2;

  if (idstr.find("ndt_pose", 0) != string::npos) {
    int nid = 0;
    if(idstr.length() >= 21) nid = 0x7f000000 | (std::strtol((idstr.substr(15,6)).c_str(), NULL, 16)) << 1;
    publish_car(nid, 1, now, pose);
  } else if (idstr.find("car_pose", 0) != string::npos) {
    id = publish_car(id, 0, now, pose);
  } else if (idstr.find("pedestrian_pose", 0) != string::npos) {
    id = publish_pedestrian(id, 1, now, pose);
  } else {
    id = publish_pedestrian(id, 0, now, pose);
  }

  return 0;
}

static void marker_publisher(const std_msgs::String& msg, int is_swap)
{
  std::vector<std::string> db_data = split(msg.data, '\n');
  std::vector<std::string> cols;
  ros::Time now = ros::Time::now();
  geometry_msgs::Pose pose;

  for (const std::string& row : db_data) {
    if(row.empty())
      continue;
    cols = split(row, '\t');
    // id,x,y,z,or_x,or_y,or_z,or_w,lon,lat,tm
    if(cols.size() != 11)
      continue;

    if (is_swap) {
      pose.position.x = std::stod(cols[2]);
      pose.position.y = std::stod(cols[1]);
      pose.orientation.x = std::stod(cols[5]);
      pose.orientation.y = std::stod(cols[4]);
    } else {
      pose.position.x = std::stod(cols[1]);
      pose.position.y = std::stod(cols[2]);
      pose.orientation.x = std::stod(cols[4]);
      pose.orientation.y = std::stod(cols[5]);
    }
    pose.position.z = std::stod(cols[3]);
    pose.orientation.z = std::stod(cols[6]);
    pose.orientation.w = std::stod(cols[7]);
    if (pose.position.x < -1.79E308 ||
	pose.position.y < -1.79E308 ||
	pose.position.z < -1.79E308) {
      geo_pos_conv geo;
      double lon = std::stod(cols[8]);
      double lat = std::stod(cols[9]);
      geo.set_plane(7); // Aichi-ken
      geo.llh_to_xyz(lat, lon, 0/*h*/);
      if (is_swap) {
	pose.position.x = geo.y();
	pose.position.y = geo.x();
      } else {
	pose.position.x = geo.x();
	pose.position.y = geo.y();
      }
      pose.position.z = geo.z();
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
    }
    result_to_marker(cols[0], now, pose, is_swap);
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
      marker_publisher(msg, 1);
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
  nh.param<double>(MYNAME "/life_time", life_time, LIFETIME);
  cout << "life_time=" << life_time << endl;
  nh.param<double>(MYNAME "/posup_dz", posup_dz, POSUP_DZ);
  cout << "posup_dz=" << posup_dz << endl;
  nh.param<double>(MYNAME "/pedestrian_dz", pedestrian_dz, PEDESTRIAN_DZ);
  cout << "pedestrian_dz=" << pedestrian_dz << endl;

  sd = SendData(host_name, db_port);

  if (pthread_create(&th, nullptr, intervalCall, (void *)args) != 0) {
    std::perror("pthread_create");
    std::exit(1);
  }

  pthread_detach(th);

  ros::spin();

  return 0;
}
