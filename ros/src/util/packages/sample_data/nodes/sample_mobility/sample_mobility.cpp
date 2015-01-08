#include <fstream>
#include "ros/ros.h"
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <geo_pos_conv.hh>

#define SELF_TRANS	0
int swap_x_y = 0;

ros::Publisher pub;

typedef std::vector<std::vector<std::string>> Tbl;

typedef struct{
  int gps_id;
  double lat;
  double lon;
  double ele;
  std::string timestamp;
  double x;
  double y;
  double z;
}Car;

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
  
  
  while(ros::ok()){
    
    pub.publish(*marker);
    ros::spinOnce();
    rate.sleep();
    std::cout << "Published." << std::endl;
    //    marker->id++;
  }  
}


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

  std::cout << "db_callback" << std::endl;

  std::vector<Car> cars;
  std::vector<std::string> db_data, tmp;

  visualization_msgs::Marker sphere_list;
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

  Car a;
  /*
  marker.header.frame_id = "/mobility";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mo_marker";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  */
  ros::Rate rate(50);

  geo_pos_conv geo;
  geo.set_plane(7);

  // Loading data.
  db_data = split(msg->data.c_str(),',');
  int counter=0;

  for(int i = 0; i < db_data.size(); i++){

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
    //    geo.set_llh_nmea_degrees(a.lat, a.lon, a.ele);
    geo.llh_to_xyz(a.lat, a.lon, a.ele);
    a.x = geo.x();
    a.y = geo.y();
    a.z = geo.z();
    p.x = geo.x();
    p.y = geo.y();
    p.z = geo.z();
    /*
    p.x = 1000.0 + i * 10.0;
    p.y = 1000.0 + i * 10.0;
    p.z = 0.0;
    */
    /*
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 1.0;
    */

    cars.push_back(a);
    sphere_list.points.push_back(p);

    //    std::cout << cars[i].gps_id << " : " << cars[i].lat << " : " << cars[i].lon << " : " << cars[i].ele << " : " << cars[i].timestamp << " : " << cars[i].x << " : " << cars[i].y << " : " << cars[i].z << std::endl;

    std::cout << "gps_id: " << cars[counter].gps_id << std::endl;
    std::cout << "lat: " << cars[counter].lat << std::endl;
    std::cout << "lon: " << cars[counter].lon << std::endl;
    std::cout << "ele: " << cars[counter].ele << std::endl;
    std::cout << "timestamp: " << cars[counter].timestamp << std::endl;
    std::cout << "x: " << cars[counter].x << std::endl;
    std::cout << "y: " << cars[counter].y << std::endl;
    std::cout << "z: " << cars[counter].z << std::endl;
    counter++;
    /*
    set_marker_data(&marker,
		    a.x,
		    a.y,
		    a.z,
		    0, 0, 0, 1,
		    20.0, 20.0, 20.0,
		    0, 1, 0, 1);
    publish_marker(&marker, pub, rate);
    */
  }
  // Publish data
  while(ros::ok()){
    /*
    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id = "/mobility";
    sphere_list.header.stamp = ros::Time::now();
    sphere_list.ns = "mo_marker";
    sphere_list.id = 0;
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere_list.action = visualization_msgs::Marker::ADD;

    sphere_list.color.r = 1.0;
    sphere_list.color.g = 0.0;
    sphere_list.color.b = 0.0;
    sphere_list.color.a = 1.0;

    sphere_list.lifetime = ros::Duration();
    */

    pub.publish(sphere_list);

    ros::spinOnce();

    rate.sleep();
  }
  // Free vector
  std::vector<Car>().swap(cars);

}


int main(int argc, char **argv)
{

  std::cout << "---------------------------------------------" << std::endl;
  std::cout << "sample_mobility.cpp coded by Yuki KITSUKAWA" << std::endl;
  std::cout << "---------------------------------------------" << std::endl;
/*

#!/bin/sh
rosrun sample_data sample_trajectory gnss.log <swap_x_y_off|swap_x_y_on>

# EOF

*/

  ros::init(argc, argv, "sample_mobility");

  ros::NodeHandle n;
  pub = n.advertise<visualization_msgs::Marker>("mo_marker", 10);

  //  std::vector<GnssData> gnssdatas = read_gnssdata(argv[1]);

  /*
  if(std::string(argv[2]) == "swap_x_y_on") {
    printf("swap_x_y: on\n");
    swap_x_y = 1;
  } else {
    printf("swap_x_y : off\n");
  }
  */

  // Subscribing the data from DB
  ros::Subscriber sub = n.subscribe("mo", 1000, db_callback);

  ros::spin();

  return 0;
}
