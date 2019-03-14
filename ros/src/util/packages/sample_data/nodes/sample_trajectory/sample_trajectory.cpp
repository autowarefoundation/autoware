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

#include <fstream>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <gnss/geo_pos_conv.hpp>

#define SELF_TRANS	0
int swap_x_y = 0;

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
    if(columns[0].compare("$GPGGA") != 0) continue;

    if(columns[2] != "" && columns[4] != "" && columns[9] != "") {
      tbl.push_back(columns);
    }
  }
  return tbl;
}

/* for gnssdata.csv */
struct GnssData {
  double x;
  double y;
  double z;
};


std::vector<GnssData> read_gnssdata(const char* filename)
{
  Tbl tbl = read_csv(filename);
  size_t i, n = tbl.size();
  std::vector<GnssData> ret(n);

  for (i=0; i<n; i++) {
    ret[i].x = std::stod(tbl[i][2]);
    ret[i].y = std::stod(tbl[i][4]);
    ret[i].z = std::stod(tbl[i][9]);
  }

  return ret;
}


void set_marker_data(visualization_msgs::Marker* marker,
		    double px, double py, double pz, double ox, double oy, double oz, double ow,
		    double sx, double sy, double sz, double r, double g, double b, double a)
{
  if(swap_x_y) {
    marker->pose.position.x = py;
    marker->pose.position.y = px;
    marker->pose.orientation.x = oy;
    marker->pose.orientation.y = ox;
  } else {
    marker->pose.position.x = px;
    marker->pose.position.y = py;
    marker->pose.orientation.x = ox;
    marker->pose.orientation.y = oy;
  }
  marker->pose.position.z = pz;

  marker->pose.orientation.z = oz;
  marker->pose.orientation.w = ow;

  marker->scale.x = sx;
  marker->scale.y = sy;
  marker->scale.z = sz;

  marker->color.r = r;
  marker->color.g = g;
  marker->color.b = b;
  marker->color.a = a;
}

void publish_marker(visualization_msgs::Marker* marker,
		    ros::Publisher& pub,
		    ros::Rate& rate)
{
#if SELF_TRANS
    if(swap_x_y) {
      marker->pose.position.x += 16635;
      marker->pose.position.y += 86432;
    } else {
      marker->pose.position.x += 86432;
      marker->pose.position.y += 16635;
    }
    marker->pose.position.z += -50; // -50
#endif

    ros::ok();
    pub.publish(*marker);
    rate.sleep();
    //    marker->id++;
}


int main(int argc, char **argv)
{

/*

#!/bin/sh
rosrun sample_data sample_trajectory gnss.log <swap_x_y_off|swap_x_y_on>

# EOF

 */

  ros::init(argc, argv, "sample_trajectory");
  ros::NodeHandle n;
  //  ros::Publisher pub = n.advertise<visualization_msgs::Marker>("/vector_map", 10, true);
  ros::Publisher pub = n.advertise<visualization_msgs::Marker>("/sample_trajectory", 10, true);


  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " csv_file"
              << std::endl;
    std::exit(1);
  }

  geo_pos_conv geo;
  std::vector<GnssData> gnssdatas = read_gnssdata(argv[1]);

  if(std::string(argv[2]) == "swap_x_y_on") {
    printf("swap_x_y: on\n");
    swap_x_y = 1;
  } else {
    printf("swap_x_y : off\n");
  }

  visualization_msgs::Marker marker;
#if SELF_TRANS
  marker.header.frame_id = "/map2";
#else
  marker.header.frame_id = "/map";
#endif
  marker.header.stamp = ros::Time::now();

  marker.ns = "vector_map";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::SPHERE;

  ros::Rate rate(50);

  size_t i;

  // show gnssdata
  geo.set_plane(7);
  for (i=0; i<gnssdatas.size(); i++) {
    geo.set_llh_nmea_degrees(gnssdatas[i].x,gnssdatas[i].y,gnssdatas[i].z);
    set_marker_data(&marker,
		    geo.x()-1.0, 
		    geo.y()-1.0, 
		    geo.z()-1.0,
		    0, 0, 0, 1,
		    2.0, 2.0, 2.0,
		    0, 1, 0, 1);
    publish_marker(&marker, pub, rate);
  }

  return 0;
}
