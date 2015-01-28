#include <fstream>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <geo_pos_conv.hh>

#include "ui_socket/route_cmd.h"


#define SELF_TRANS	0
#define HEIGHT	50
//#define DEBUG_PRINT 
int swap_x_y = 0;
ros::Publisher pub;
visualization_msgs::Marker marker;

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


void RouteCmdCallback(const ui_socket::route_cmd msg)
{
  geo_pos_conv geo;
  ros::Rate rate(1000);
  double x,y,z, a;

  geo.set_plane(7);
#ifdef DEBUG_PRINT
  fprintf(stderr, "point.size()=%d\n", (int)msg.point.size());
#endif

  for(int i = 0; i < msg.point.size(); i++) {
#ifdef DEBUG_PRINT
    fprintf(stderr, "%d: point[i].lat=%f, point[i].lon=%f\n", i, msg.point[i].lat, msg.point[i].lon);
#endif 

    geo.llh_to_xyz(msg.point[i].lat, msg.point[i].lon, HEIGHT);

#ifdef DEBUG_PRINT
    fprintf(stderr, "  geo.x()=%f, geo.y()=%f, geo.z()=%f\n", geo.x(),geo.y(),geo.z()); 
#endif
    set_marker_data(&marker,
		    geo.x()-1.0, 
		    geo.y()-1.0, 
		    geo.z()-1.0,
		    0, 0, 0, 1,
		    2.0, 2.0, 2.0,
		    1, 0, 0, 1);
    publish_marker(&marker, pub, rate);
    sleep(1);
  }
}

int main(int argc, char **argv)
{

/*

#!/bin/sh
rosrun lane_navigator lane_navigator <swap_x_y_off|swap_x_y_on>

# EOF

*/

  ros::init(argc, argv, "lane_navigator");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("route_cmd", 1000, RouteCmdCallback);
  pub = n.advertise<visualization_msgs::Marker>("/lane_navigator", 10, true);



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

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <swap_x_y_on|swap_x_y_off>"
              << std::endl;
    std::exit(1);
  }

  if(std::string(argv[1]) == "swap_x_y_on") {
    printf("swap_x_y: on\n");
    swap_x_y = 1;
  } else {
    printf("swap_x_y : off\n");
  }


  ros::spin();
  return 0;
}
