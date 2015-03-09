#include <fstream>
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <geo_pos_conv.hh>
#include "ros/ros.h"

ros::Publisher pub;

typedef struct{
  int gps_id;
  double lat;
  double lon;
  double ele;
  std::string timestamp;
  double x;
  double y;
  double z;
}car_info;


std::vector<std::string> split(const std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}


void db_callback(const std_msgs::String::ConstPtr &msg)
{
  std::vector<car_info> cars;
  std::vector<std::string> db_data, tmp;
  visualization_msgs::Marker sphere_list;
  car_info a;
  sphere_list.header.frame_id = "/mobility";
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "mo_marker";
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
  ros::Rate rate(50);
  geo_pos_conv geo;
  geo.set_plane(7);
  // Loading data.
  db_data = split(msg->data.c_str(),'\n');
  for(int i = 0; i < (int)db_data.size(); i++){
    geometry_msgs::Point p;
    if(db_data[i].compare("")==0) continue;
    tmp = split(db_data[i], '\t');
    if(tmp.size()!=5) continue;
    a.gps_id = std::stoi(tmp[0]);
    a.lat = std::stod(tmp[1].c_str());
    a.lon = std::stod(tmp[2].c_str());
    a.ele = std::stod(tmp[3].c_str());
    a.timestamp = tmp[4];
    // Convert from lat,lon to x,y
    // geo.set_llh_nmea_degrees(a.lat, a.lon, a.ele);
    geo.llh_to_xyz(a.lat, a.lon, a.ele);
    a.x = geo.x();
    a.y = geo.y();
    a.z = geo.z();
    // swap x and y
    p.x = a.y;
    p.y = a.x;
    p.z = a.z;
    cars.push_back(a);
    sphere_list.points.push_back(p);
    std::cout << "gps_id: " << cars[i].gps_id << std::endl;
    std::cout << "lat: " << cars[i].lat << std::endl;
    std::cout << "lon: " << cars[i].lon << std::endl;
    std::cout << "ele: " << cars[i].ele << std::endl;
    std::cout << "timestamp: " << cars[i].timestamp << std::endl;
    std::cout << "x: " << cars[i].x << std::endl;
    std::cout << "y: " << cars[i].y << std::endl;
    std::cout << "z: " << cars[i].z << std::endl;
  }
  // Publish data
  while(ros::ok()){
    pub.publish(sphere_list);
    ros::spinOnce();
    rate.sleep();
  }
  // Free vector
  std::vector<car_info>().swap(cars);
}


int main(int argc, char **argv)
{
  std::cout << "-------------------" << std::endl;
  std::cout << "sample_mobility.cpp" << std::endl;
  std::cout << "-------------------" << std::endl;
  ros::init(argc, argv, "sample_mobility");
  ros::NodeHandle n;
  pub = n.advertise<visualization_msgs::Marker>("mo_marker", 10);
  // Subscribing the data from DB
  ros::Subscriber sub = n.subscribe("mo", 1000, db_callback);
  ros::spin();
  return 0;
}
