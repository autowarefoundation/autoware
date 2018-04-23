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

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <gnss/geo_pos_conv.hpp>

static ros::Publisher pub;

struct car_info {
  int gps_id;
  double lat;
  double lon;
  double ele;
  std::string timestamp;
  double x;
  double y;
  double z;

  void dump() const;
};

void car_info::dump() const
{
    std::cout << "gps_id: " << gps_id << std::endl;
    std::cout << "lat: " << lat << std::endl;
    std::cout << "lon: " << lon << std::endl;
    std::cout << "ele: " << ele << std::endl;
    std::cout << "timestamp: " << timestamp << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;
}

static std::vector<std::string> split(const std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

static void publish_markers(const visualization_msgs::Marker& markers)
{
  ros::Rate rate(50);
  while(ros::ok()){
    pub.publish(markers);
    ros::spinOnce();
    rate.sleep();
  }
}

static void init_markers(visualization_msgs::Marker& markers)
{
  markers.header.frame_id = "/mobility";
  markers.header.stamp = ros::Time::now();
  markers.ns = "mo_marker";
  markers.id = 0;
  markers.type = visualization_msgs::Marker::SPHERE_LIST;
  markers.action = visualization_msgs::Marker::ADD;
  markers.lifetime = ros::Duration();
  markers.color.r = 1.0;
  markers.color.g = 0.0;
  markers.color.b = 0.0;
  markers.color.a = 1.0;
  markers.scale.x = 10.0;
  markers.scale.y = 10.0;
  markers.scale.z = 10.0;
}

static void db_callback(const std_msgs::String::ConstPtr &msg)
{
  visualization_msgs::Marker sphere_list;
  init_markers(sphere_list);

  geo_pos_conv geo;
  geo.set_plane(7);

  std::vector<car_info> cars;
  std::vector<std::string> db_data = split(msg->data,'\n');
  for(const auto& row : db_data){
    if(row.empty())
      continue;

    std::vector<std::string> car_data = split(row, '\t');
    if(car_data.size()!=5)
      continue;

    car_info car;
    car.gps_id = std::stoi(car_data[0]);
    car.lat = std::stod(car_data[1]);
    car.lon = std::stod(car_data[2]);
    car.ele = std::stod(car_data[3]);
    car.timestamp = car_data[4];

    // Convert from lat,lon to x,y
    // geo.set_llh_nmea_degrees(car.lat, car.lon, car.ele);
    geo.llh_to_xyz(car.lat, car.lon, car.ele);
    car.x = geo.x();
    car.y = geo.y();
    car.z = geo.z();

    // swap x and y
    geometry_msgs::Point p;
    p.x = car.y;
    p.y = car.x;
    p.z = car.z;
    cars.push_back(car);
    sphere_list.points.push_back(p);

    car.dump();
  }

  publish_markers(sphere_list);
}

int main(int argc, char **argv)
{
  std::cout << "-------------------\n"
	    << "sample_mobility.cpp\n"
	    << "-------------------" << std::endl;

  ros::init(argc, argv, "sample_mobility");
  ros::NodeHandle n;
  pub = n.advertise<visualization_msgs::Marker>("mo_marker", 10);
  // Subscribing the data from DB
  ros::Subscriber sub = n.subscribe("mo", 1000, db_callback);
  ros::spin();
  return 0;
}
