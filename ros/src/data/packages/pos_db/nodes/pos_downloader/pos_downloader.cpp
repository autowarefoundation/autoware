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
#ifndef CURRENT_CAR_DIRECTLY
#include <map>
#endif /* ! CURRENT_CAR_DIRECTLY */
#include <pthread.h>

#include <geo_pos_conv.hh>
#include <pos_db.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define MYNAME		"pos_downloader"
#define MARKERNAME	"mo_marker"
#define STARTTIME	(0)		// sec since 1970-01-01 (0==now)
#define DELAYSEC	(0)		// delay sec for pos_uploader
#define POSUP_DZ	(40)		// z offset of PosUp
#define PEDESTRIAN_DZ	(-2)		// z offset of pedestrian_pose
#define DOWNLOAD_PERIOD	(250)		// period (msec)
#define TOTAL_LIFETIME	(10.0)		// total lifetime (sec)

#define TYPE_OWN	(1)
#define TYPE_CAR	(2)
#define TYPE_PEDESTRIAN	(3)

#define ANON_MARKER_ID_MIN	(2)
#define ANON_MARKER_ID_MAX	(0x7f000000)

using namespace std;

static string db_host_name;
static int db_port;
static string sshpubkey;
static string sshprivatekey;
static int ssh_port;
static string sshtunnelhost;
static int sleep_msec = DOWNLOAD_PERIOD;	// period
static double life_time = 1.0;			// sec
static double posup_dz;
static double pedestrian_dz;

static ros::Publisher pub;

static SendData sd;

static char mac_addr[MAC_ADDRBUFSIZ];
static int ignore_my_pose = 1;

#ifndef CURRENT_CAR_DIRECTLY
static map<int, geometry_msgs::Pose> car_map;
static map<int, ros::Time> now_map;
static ros::Time prev_time;
#endif /* ! CURRENT_CAR_DIRECTLY */

#ifdef NEVER
static double color_percent(int diffmsec)
{
  return (1.0 - diffmsec/TOTAL_LIFETIME/1000.0)*((256-48)/256.0) + (48/256.0);
}
#endif /* NEVER */
static double alpha_percent(int diffmsec)
{
  return (1.0 - diffmsec/TOTAL_LIFETIME/1000.0);
}

static int create_markerid(geometry_msgs::Pose& pose, int type)
{
  return (int)(pose.position.x*11 + pose.position.y*13 + pose.position.z + type*5) % ANON_MARKER_ID_MAX;
}

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
#ifdef POS_DB_VERBOSE
  std::cout << marker.id << " : "
	<< marker.pose.position.x << ","
	<< marker.pose.position.y << ","
	<< marker.pose.position.z << " : "
	<< marker.pose.orientation.x << ","
	<< marker.pose.orientation.y << ","
	<< marker.pose.orientation.z << ","
	<< marker.pose.orientation.w << std::endl;
#endif /* POS_DB_VERBOSE */
}

static void publish_car(int id, int is_current, ros::Time now,
		       geometry_msgs::Pose& pose, int diffmsec)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = now;
  marker.ns = MARKERNAME;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  if (is_current) {
#ifdef CURRENT_CAR_DIRECTLY
    marker.id = id;
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

    pub.publish(marker);
    dbg_out_marker(marker);
#else /* CURRENT_CAR_DIRECTLY */
    ros::Time newnow = now - ros::Duration(diffmsec/1000.0);
    if (now_map.count(id) == 0 || newnow >= now_map[id]) {
      car_map[id] = pose;
      now_map[id] = newnow;
    }
#endif /* CURRENT_CAR_DIRECTLY */

  } else {
    marker.id = create_markerid(pose, 1);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(life_time);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = alpha_percent(diffmsec);
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.pose.position.z += 0.5; // == #1/2
    pub.publish(marker);
    dbg_out_marker(marker);
  }
}

#ifndef CURRENT_CAR_DIRECTLY
static void publish_car_summary(ros::Time now)
{
  visualization_msgs::Marker marker;
  map<int, geometry_msgs::Pose>::iterator itr;

  for(itr = car_map.begin(); itr != car_map.end(); itr++) {
    int id = itr->first;
    geometry_msgs::Pose pose = itr->second;
    ros::Time cur = now_map[id];
    if (cur <= prev_time) {
      continue;
    }
    marker.header.frame_id = "/map";
    marker.header.stamp = cur;
    marker.ns = MARKERNAME;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.pose = pose;
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

    pub.publish(marker);
    dbg_out_marker(marker);
    prev_time = cur;
  }
  car_map.clear();
  now_map.clear();
}
#endif /* ! CURRENT_CAR_DIRECTLY */

static void publish_pedestrian(int id, int is_pedestrian, ros::Time now,
			      geometry_msgs::Pose& pose, int diffmsec)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = now;
  marker.ns = MARKERNAME;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.lifetime = ros::Duration(life_time);
  marker.id = create_markerid(pose, 2);
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
  marker.color.a = alpha_percent(diffmsec);
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 1.2; // #1
  marker.pose = pose;
  marker.pose.position.z += 0.6; // == #1/2
  pub.publish(marker);
  dbg_out_marker(marker);

  marker.id = 1 + create_markerid(pose, 2);
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.6; // #2
  marker.scale.y = 0.6;
  marker.scale.z = 0.6;
  marker.pose = pose;
  marker.pose.position.z += 1.2 + 0.3 + 0.1; // == #1 + #2/2 + alpha
  pub.publish(marker);
  dbg_out_marker(marker);
}

static int result_to_marker(const string& idstr, ros::Time now,
			    geometry_msgs::Pose& pose, int type,
			    int diffmsec, int is_swap)
{
  int nid;

  switch (type) {
  case TYPE_OWN:
    /* use lower 6 bytes */
    nid = ANON_MARKER_ID_MAX |
      (std::strtol(idstr.substr(6, 6).c_str(), NULL, 16)) << 1;
    publish_car(nid, 1, now, pose, diffmsec);
    break;
  case TYPE_CAR:
    publish_car(0, 0, now, pose, diffmsec);
    break;
  case TYPE_PEDESTRIAN:
    publish_pedestrian(0, 1, now, pose, diffmsec);
    break;

  /* backward compatibility */
  default:
    if (idstr.find("current_pose", 0) != string::npos) {
      /* current_pose:DEF012345678 */
      if (idstr.length() >= 25) {
	/* use lower 6 bytes */
	nid = ANON_MARKER_ID_MAX |
	  (std::strtol((idstr.substr(19, 6)).c_str(), NULL, 16)) << 1;
        publish_car(nid, 1, now, pose, diffmsec);
      }
    } else if (idstr.find("ndt_pose", 0) != string::npos) {
      /* ndt_pose:9ABCDEF01234 */
      if (idstr.length() >= 21) {
	/* use lower 6 bytes */
	nid = ANON_MARKER_ID_MAX |
	  (std::strtol((idstr.substr(15, 6)).c_str(), NULL, 16)) << 1;
        publish_car(nid, 1, now, pose, diffmsec);
      }
    } else if (idstr.find("car_pose", 0) != string::npos) {
      publish_car(0, 0, now, pose, diffmsec);
    } else if (idstr.find("pedestrian_pose", 0) != string::npos) {
      publish_pedestrian(0, 1, now, pose, diffmsec);
    } else {
      publish_pedestrian(0, 0, now, pose, diffmsec); // PosUp
    }
  }

  return 0;
}

static int get_timeval(const char *tstr, time_t *sp, int *np)
{
  struct tm tm;
  const char *p;

  if (sscanf(tstr, "%d-%d-%d %d:%d:%d", &tm.tm_year, &tm.tm_mon,
	&tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec) != 6) {
    std::cerr << "Cannot convert time \"" << tstr << "\"" << std::endl;
    return -1;
  }
  if ((p = strchr(tstr, '.')) != NULL) {
    sscanf(++p, "%d", np);
    for (int i = strlen(p); i < 9; i++, *np*=10);
  } else {
    *np = 0;
  }
  tm.tm_year -= 1900;
  tm.tm_mon -= 1;
  *sp = mktime(&tm);
  return 0;
}

static void marker_publisher(const std_msgs::String& msg, int is_swap)
{
  std::vector<std::string> db_data = split(msg.data, '\n');
  std::vector<std::string> cols;
  ros::Time now = ros::Time::now();
  geometry_msgs::Pose pose;
  int type;
  time_t now_sec, prv_sec = 0;
  int now_nsec, prv_nsec;

  for (const std::string& row : db_data) {
    if(row.empty())
      continue;
    cols = split(row, '\t');
    // id,x,y,z,or_x,or_y,or_z,or_w,lon,lat,tm,type
    if(cols.size() != 12)
      continue;

    type = std::stoi(cols[11]);
    if(ignore_my_pose &&
       (type == TYPE_OWN ||
	cols[0].find("current_pose", 0) != string::npos ||
	cols[0].find("ndt_pose", 0) != string::npos) &&
       cols[0].find(mac_addr, 0) != string::npos) {
      continue;	// don't publish Marker of my pose
    }

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
    // VoltDB returns not NULL but -1.7976931348623157E308
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
    now_sec = now.toSec();
    now_nsec = now.toNSec()%(1000*1000*1000);
    get_timeval(cols[10].c_str(), &prv_sec, &prv_nsec);
    result_to_marker(cols[0], now, pose, type,
      (now_sec-prv_sec)*1000+(now_nsec-prv_nsec)/1000/1000, is_swap);
  }

#ifndef CURRENT_CAR_DIRECTLY
  publish_car_summary(now);
#endif /* ! CURRENT_CAR_DIRECTLY */
}

// create "YYYY-mm-dd HH:MM:SS.sss"
static int create_timestr(time_t sec, int nsec, char *str, size_t size)
{
  std::tm *nowtm;

  nowtm = std::gmtime(&sec);
  return std::snprintf(str, size, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
	nowtm->tm_year + 1900, nowtm->tm_mon + 1, nowtm->tm_mday,
	nowtm->tm_hour, nowtm->tm_min, nowtm->tm_sec, nsec/1000/1000);
}

//wrap SendData class
static void send_sql(time_t diff_sec)
{
  std::string data;
  string db_response;
  std_msgs::String msg;
  ros::Time now = ros::Time::now();
  time_t now_sec = now.toSec() - diff_sec;
  int now_nsec = now.toNSec()%(1000*1000*1000);
  char timestr[64]; // "YYYY-mm-dd HH:MM:SS.sss..."
  char prevstr[64]; // "YYYY-mm-dd HH:MM:SS.sss..."

  create_timestr(now_sec-TOTAL_LIFETIME, now_nsec, prevstr, sizeof(prevstr));
  create_timestr(now_sec, now_nsec, timestr, sizeof(timestr));

  data = make_header(1, 1);
  // select pos data between previous latest timestamp and now
  data += "SELECT id,x,y,z,or_x,or_y,or_z,or_w,lon,lat,tm,type FROM POS "
	"WHERE tm > '";
  data += prevstr;
  data += "' AND tm < '";
  data += timestr;
  data += "' ORDER BY tm;\r\n";

  int ret = sd.Sender(data, db_response, 0);
  if (ret < 0) {
    std::cerr << "sd.Sender() failed" << std::endl;
  } else {
#ifdef POS_DB_VERBOSE
    std::cout << "return data: \"" << db_response << "\"" << std::endl;
#endif /* POS_DB_VERBOSE */
    msg.data = db_response.c_str();
    marker_publisher(msg, 1);
  }
}

static void* intervalCall(void *unused)
{
  double *args = (double *)unused;
  double diff_sec = args[1];

  if (args[0] != 0)
    diff_sec += ros::Time::now().toSec() - args[0];
#ifdef POS_DB_VERBOSE
  cout << "diff=" << diff_sec << endl;
#endif /* POS_DB_VERBOSE */

  while (1) {
    send_sql((time_t)diff_sec);
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

  if(argc < 2) {
    std::cerr << "usage : \n\trosrun " << MYNAME << " <user name> [show_my_pose]" << std::endl;
    return -1;
  }
  if(argc > 2) {
    if(strncmp(argv[2], "show_my_pose", 12) == 0) ignore_my_pose = 0;
  }
  std::cerr << "ignore_my_pose=" << ignore_my_pose << std::endl;

  probe_mac_addr(mac_addr);
  std::cerr <<  "mac_addr=" << mac_addr << std::endl;

  string home_dir = getenv("HOME");

  pub = nh.advertise<visualization_msgs::Marker>(MARKERNAME, 1);
  nh.param<double>(MYNAME "/time", args[0], STARTTIME);
  cout << "time=" << args[0] << endl;
  nh.param<double>(MYNAME "/delay", args[1], DELAYSEC);
  cout << "delay=" << args[1] << endl;
  nh.param<double>(MYNAME "/posup_dz", posup_dz, POSUP_DZ);
  cout << "posup_dz=" << posup_dz << endl;
  nh.param<double>(MYNAME "/pedestrian_dz", pedestrian_dz, PEDESTRIAN_DZ);
  cout << "pedestrian_dz=" << pedestrian_dz << endl;

  nh.param<string>("pos_db/db_host_name", db_host_name, DB_HOSTNAME);
  cout << "db_host_name=" << db_host_name << endl;
  nh.param<int>("pos_db/db_port", db_port, DB_PORT);
  cout << "db_port=" << db_port << endl;
  nh.param<string>("pos_db/sshpubkey", sshpubkey, home_dir+SSHPUBKEY);
  cout << "sshpubkey=" << sshpubkey << endl;
  nh.param<string>("pos_db/sshprivatekey", sshprivatekey, home_dir+SSHPRIVATEKEY);
  cout << "sshprivatekey=" << sshprivatekey << endl;
  nh.param<int>("pos_db/ssh_port", ssh_port, SSHPORT);
  cout << "ssh_port=" << ssh_port << endl;
  nh.param<string>("pos_db/sshtunnelhost", sshtunnelhost, SSHTUNNELHOST);
  cout << "sshtunnelhost=" << sshtunnelhost << endl;

  sd = SendData(db_host_name, db_port, argv[1], sshpubkey, sshprivatekey, ssh_port, sshtunnelhost);

  if (pthread_create(&th, nullptr, intervalCall, (void *)args) != 0) {
    std::perror("pthread_create");
    std::exit(1);
  }

  pthread_detach(th);

  ros::spin();

  return 0;
}
