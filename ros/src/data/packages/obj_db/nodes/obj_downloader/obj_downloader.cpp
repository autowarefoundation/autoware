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

/*
obj_downloader
This node get location data from db server and
publish data as ractangular plane
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <gnss/geo_pos_conv.hpp>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <pthread.h>

#include <obj_db.h>

using namespace std;

static string host_name = "db3.ertl.jp";
static int db_port = 5678;

enum DataType {
  NORMAL = 10000,
  DB1    = 10001,
  TEST   = 10002,
  RANGE  = 10003,
};

struct CarInformation {
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

void CarInformation::dump() const
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

static ros::Publisher pub;

static double positionRange[4];
static double geoPosition[4];//rectangular coordinate for sql condition

static SendData sd;
static DataType send_data_type;

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

static bool isNumeric(const std::string& str){
  if(str.find_first_not_of("-0123456789. Ee\t") != string::npos) return false;
  return true;
}

static int result_to_car_info(const std::string& result, CarInformation& car)
{
  std::vector<std::string> columns = split(result, '\t');
  if(columns.size() != 8)
    return -1;

  car.gps_id = std::stoi(columns[0]);
  car.lat = std::stod(columns[1]);
  car.lon = std::stod(columns[2]);
  car.ele = std::stod(columns[3]);
  car.x = std::stod(columns[4]);
  car.y = std::stod(columns[5]);
  car.z = std::stod(columns[6]);
  car.timestamp = columns[7];

  return 0;
}

static void marker_publisher(const std_msgs::String& msg)
{
  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = "/mobility";
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "mo_pictograms";
  sphere_list.id = 0;
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.lifetime = ros::Duration();
  sphere_list.color.r = 1.0;
  sphere_list.color.g = 0.0;
  sphere_list.color.b = 0.0;
  sphere_list.color.a = 1.0;
  sphere_list.scale.x = 10.0;
  sphere_list.scale.y = 10.0;
  sphere_list.scale.z = 10.0;

  geo_pos_conv geo;
  geo.set_plane(7);
  // Loading data.
  std::vector<std::string> db_data = split(msg.data,'\n');

  std::vector<CarInformation> cars;
  for(const std::string& row : db_data) {
    if(row.empty())
      continue;

    CarInformation car;
    int ret = result_to_car_info(row, car);
    if (ret != 0)
      continue;

    // Convert from lat,lon to x,y
    // geo.set_llh_nmea_degrees(car.lat, car.lon, car.ele);
    if(car.lat >= -180 && car.lat <= 180 && car.lon >= -180 && car.lon <= 180){
      geo.llh_to_xyz(car.lat, car.lon, car.ele);
      car.x = geo.x();
      car.y = geo.y();
      car.z = geo.z();
    }

    geometry_msgs::Point p;
    // swap x and y
    p.x = car.y;
    p.y = car.x;
    p.z = car.z;

    cars.push_back(car);
    sphere_list.points.push_back(p);

    car.dump();
  }

  pub.publish(sphere_list);
}

static std::string construct_select_statement(DataType type)
{
  std::stringstream ss;

  switch (type) {
  case RANGE:
     ss << "SELECT 0,lat,lon,x,y,z,tm FROM pos"
        << " WHERE ((lat >= "<< fixed << setprecision(7) << positionRange[0]
        << " AND lat < " << fixed << setprecision(7) << positionRange[1]
        << " AND lon >= " << fixed << setprecision(7) << positionRange[2]
        << " AND lon < " << fixed << setprecision(7) << positionRange[3] << ")"
        << " OR (x >= " << fixed << setprecision(7) << geoPosition[0]
        << " AND x < "  << fixed << setprecision(7) << geoPosition[1]
        << " AND y >= " << fixed << setprecision(7) << geoPosition[2]
        << " AND y < " << fixed << setprecision(7) << geoPosition[3] << "))"
        << " AND tm > TO_TIMESTAMP(Second,SINCE_EPOCH(Second,current_timestamp)-1) and tm <= current_timestamp;";
      break;
  case TEST:
     ss << "SELECT 0,lat,lon,0,x,y,z,tm FROM pos"
        << " WHERE ((lat >= " << fixed << setprecision(7) << positionRange[0]
        << " AND lat < " << fixed << setprecision(7) << positionRange[1]
        << " AND lon >= " << fixed << setprecision(7) << positionRange[2]
        << " AND lon < " << fixed << setprecision(7) << positionRange[3] << ")"
        << " OR (x >= " << fixed << setprecision(7) << geoPosition[0]
        << " AND x < " << fixed << setprecision(7) << geoPosition[1]
        << " AND y >= " << fixed << setprecision(7) << geoPosition[2]
        << " AND y < " << fixed << setprecision(7) << geoPosition[3] << "))"
        << " AND id = '0' ORDER BY tm DESC LIMIT 1;";
      break;
  case DB1:
    ss << "SELECT id,lat,lon,ele,timestamp FROM select_test"
       << " WHERE timestamp = (select max(timestamp) from select_test)"
       << " AND lat >= " << fixed << setprecision(7) << positionRange[0]
       << " AND lat < " << fixed << setprecision(7) << positionRange[1]
       << " AND lon >= " << fixed << setprecision(7) << positionRange[2]
       << " AND lon < " << fixed << setprecision(7) << positionRange[3] << ";<E>";
    break;
  case NORMAL:
  default:
    ss << "SELECT 0,lat,lon,0,tm FROM pos"
       << " WHERE lat >= 30 AND lat < 40"
       << " AND lon >= 130 AND lon < 140"
       << " AND tm > TO_TIMESTAMP(Second,SINCE_EPOCH(Second,current_timestamp)-1) AND tm <= current_timestamp;";
    break;
  }

  return ss.str();
}

//wrap SendData class
static void send_sql()
{
  //I assume that values has 4 value ex: "0 0 0 0"   "1 2 3 4"
  //And if setting the other number of value , sendData will be failed.

  //create header
  std::string data = make_header(1, 1);

  data += construct_select_statement(send_data_type);
  data += "\n";

  string db_response;
  int ret = sd.Sender(data, db_response);
  if (ret == -1) {
    std::cerr << "Failed: sd.Sender" << std::endl;
    return;
  }

  std::cout << "return data: " << db_response << std::endl;

  std_msgs::String msg;
  msg.data = db_response.c_str();

  marker_publisher(msg);
}

static void* intervalCall(void *unused)
{
  while(1){
    send_sql();
    sleep(1);
  }

  return nullptr;
}

int main(int argc, char **argv)
{
  ros::init(argc ,argv, "obj_downloader") ;
  ros::NodeHandle nh;

  cout << "obj_downloader" << endl;

  pub = nh.advertise<visualization_msgs::Marker>("mo_pictograms",1);

  if(argc == 1){
    std::cout << "normal execution" << std::endl;
    send_data_type = NORMAL;
  } else if(argc == 5){
    DataType type = static_cast<DataType>(std::atoi(argv[1]));
    switch (type) {
    case NORMAL:
      std::cout << "normal access" << std::endl;
      send_data_type = NORMAL;
      break;
    case DB1:
      std::cout << "fixed range access" << std::endl;
      positionRange[0] = 35.2038955;
      positionRange[1] = 35.2711311;
      positionRange[2] = 136.9813925;
      positionRange[3] = 137.055852;
      host_name = "db1.ertl.jp";
      db_port = 5700;
      send_data_type = DB1;
      break;
    case TEST:
      std::cout << "test access" << std::endl;
      positionRange[0] = 34.5;
      positionRange[1] = 35.4;
      positionRange[2] = 136.6;
      positionRange[3] = 137.8;
      send_data_type = TEST;
      break;
    case RANGE:
      std::cout << "current data get test access\n" << std::endl;
      positionRange[0] = 34.5;
      positionRange[1] = 35.4;
      positionRange[2] = 136.6;
      positionRange[3] = 137.8;
      send_data_type = RANGE;
      break;
    default:
      printf("range access\n");
      for(int i=1; i<5 ;i++){
        std::string arg(argv[i]);
        if(!isNumeric(arg)){
          std::cerr << "argment '" << arg << "' is not numeric" << std::endl;
          return -1;
        }
        positionRange[i-1] = std::stod(arg);

        if(!(positionRange[i-1]>=-360 && positionRange[i-1]<=360)){
          std::cerr << "Error: invalid range" << std::endl;
          return -1;
        }
      }
      send_data_type = RANGE;
      break;
    }
  } else{
    std::cerr << "The number of argment is invalid." << std::endl;
    return -1;
  }

  geo_pos_conv geo;
  geo.set_plane(7);
  geo.llh_to_xyz(positionRange[0], positionRange[2], 0);
  geoPosition[0] = geo.x();
  geoPosition[2] = geo.y();

  geo.llh_to_xyz(positionRange[1], positionRange[3], 0);
  geoPosition[1] = geo.x();
  geoPosition[3] = geo.y();

  sd = SendData(host_name,db_port);

  pthread_t th;
  if(pthread_create(&th, nullptr, intervalCall, nullptr)){
    std::perror("pthread_create");
    std::exit(1);
  }

  pthread_detach(th);

  ros::spin();

  return 0;
}
