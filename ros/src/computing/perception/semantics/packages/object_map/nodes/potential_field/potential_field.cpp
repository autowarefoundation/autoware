#include "tf/transform_listener.h"
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
//#include <omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace grid_map;
ros::Publisher publisher;

double ver_y_p = 0.7;
double ver_x_p = 0.7;
double target_ver_y_p = 1.0;
double target_ver_x_p = 1.0;

double tf_x = 1.2;
double tf_z = 2.0;

static GridMap map({"obstacle_potential_field", "potential_field",
                    "waypoint_field", "vscan_field"});

void obj_callback(jsk_recognition_msgs::BoundingBoxArray::ConstPtr
                      obj_msg) { // Create grid map.

  // Add data to grid map.
  ros::Time time = ros::Time::now();
#if 0
#ifdef _OPENMP
  std::vector<float> v_map;
  {
    GridMapIterator it(map);
    int j;
    std::cout << "openMP" << std::endl;
    v_map.resize(
        (int)(40.0 * 20.0 / (map.getResolution() * map.getResolution())));
    double pos_x;
    double pos_y;
    double len_x;
    double len_y;
    Position position;
#pragma omp parallel for private(map, position, pos_x, pos_y, len_x, len_y)
    for (j = 0;
         j < (int)(40.0 * 20.0 / (map.getResolution() * map.getResolution()));
         ++j) {
      if (j != 0)
        ++it;
      map.getPosition(*it, position);
      v_map.at(j) = -0.1;
      for (int i(0); i < (int)obj_msg->boxes.size(); ++i) {
        pos_x = obj_msg->boxes.at(i).pose.position.x + 2.0;
        pos_y = obj_msg->boxes.at(i).pose.position.y;
        len_x = obj_msg->boxes.at(i).dimensions.x / 2.0;
        len_y = obj_msg->boxes.at(i).dimensions.y / 2.0;

        if (pos_x - len_x < position.x() && position.x() < pos_x + len_x) {
          if (pos_y - len_y < position.y() && position.y() < pos_y + len_y) {
            v_map.at(j) += std::exp(0.0);
          }
        }
      }
    }
  }
  GridMapIterator it(map);
  int j;

  for (j = 0;
       j < (int)(40.0 * 20.0 / (map.getResolution() * map.getResolution()));
       ++j) {
    if (j != 0)
      ++it;
    map.at("obstacle_potential_field", *it) = v_map.at(j);
  }
#endif
#else
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {

    Position position;
    map.getPosition(*it, position);
    map.at("obstacle_potential_field", *it) = 0.0;
    for (int i(0); i < (int)obj_msg->boxes.size(); ++i) {
      double pos_x = obj_msg->boxes.at(i).pose.position.x + tf_z;
      double pos_y = obj_msg->boxes.at(i).pose.position.y;
      double len_x = obj_msg->boxes.at(i).dimensions.x / 2.0;
      double len_y = obj_msg->boxes.at(i).dimensions.y / 2.0;

      if (-0.5 < pos_x && pos_x < 4.0) {
        if (-1.0 < pos_y && pos_y < 1.0)
          continue;
      }
      if (pos_x - len_x < position.x() && position.x() < pos_x + len_x) {
        if (pos_y - len_y < position.y() && position.y() < pos_y + len_y) {
          map.at("obstacle_potential_field", *it) += std::exp(0.0);
        } else if (position.y() < pos_y - len_y) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.y() - (pos_y - len_y)), 2.0) /
                       std::pow(2.0 * ver_y_p, 2.0))));
        } else if (pos_y + len_y < position.y()) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.y() - (pos_y + len_y)), 2.0) /
                       std::pow(2.0 * ver_y_p, 2.0))));
        }
      } else if (pos_y - len_y < position.y() && position.y() < pos_y + len_y) {
        if (pos_x - len_x < position.x() && position.x() < pos_x + len_x) {
          map.at("obstacle_potential_field", *it) += std::exp(0.0);
        } else if (position.x() < pos_x - len_x) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.x() - (pos_x - len_x)), 2.0) /
                       std::pow(2.0 * ver_x_p, 2.0))));
        } else if (pos_x + len_x < position.x()) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.x() - (pos_x + len_x)), 2.0) /
                       std::pow(2.0 * ver_x_p, 2.0))));
        }
      } else if (position.x() < pos_x - len_x) {
        if (position.y() < pos_y - len_y) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.y() - (pos_y - len_y)), 2.0) /
                       std::pow(2.0 * ver_y_p, 2.0))) +
              (-1.0 * (std::pow((position.x() - (pos_x - len_x)), 2.0) /
                       std::pow(2.0 * ver_x_p, 2.0))));
        } else if (pos_y + len_y < position.y()) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.y() - (pos_y + len_y)), 2.0) /
                       std::pow(2.0 * ver_y_p, 2.0))) +
              (-1.0 * (std::pow((position.x() - (pos_x - len_x)), 2.0) /
                       std::pow(2.0 * ver_x_p, 2.0))));
        }
      } else if (pos_x + len_x < position.x()) {
        if (position.y() < pos_y - len_y) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.y() - (pos_y - len_y)), 2.0) /
                       std::pow(2.0 * ver_y_p, 2.0))) +
              (-1.0 * (std::pow((position.x() - (pos_x + len_x)), 2.0) /
                       std::pow(2.0 * ver_x_p, 2.0))));
        } else if (pos_y + len_y / 2.0 < position.y()) {
          map.at("obstacle_potential_field", *it) += std::exp(
              (-1.0 * (std::pow((position.y() - (pos_y + len_y)), 2.0) /
                       std::pow(2.0 * ver_y_p, 2.0))) +
              (-1.0 * (std::pow((position.x() - (pos_x + len_x)), 2.0) /
                       std::pow(2.0 * ver_x_p, 2.0))));
        }
      }
    }
  }
#endif
  // Publish grid map.
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  // map["potential_field"] = map["obstacle_potential_field"] +
  //                          map["waypoint_field"] + map["vscan_field"];
  map["potential_field"] =
      map["obstacle_potential_field"].cwiseMax(map["vscan_field"]) +
      map["waypoint_field"];

  GridMapRosConverter::toMessage(map, message);
  publisher.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                    message.info.header.stamp.toSec());
}
void target_poiny_callback(
    visualization_msgs::Marker::ConstPtr target_point_msgs) {
  ros::Time time = ros::Time::now();
  geometry_msgs::PointStamped in, out;
  in.header = target_point_msgs->header;
  in.point = target_point_msgs->pose.position;
  tf::TransformListener tflistener;
  try {
    ros::Time now = ros::Time(0);
    tflistener.waitForTransform("/map", "/base_link", now, ros::Duration(10.0));
    tflistener.transformPoint("/base_link", in.header.stamp, in, "/map", out);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("waypoint_field", *it) = 0.0;
    map.at("waypoint_field", *it) -=
        0.5 * std::exp((-1.0 * (std::pow((position.y() - (out.point.y)), 2.0) /
                                std::pow(2.0 * target_ver_y_p, 2.0))) +
                       (-1.0 * (std::pow((position.x() - (out.point.x)), 2.0) /
                                std::pow(2.0 * target_ver_x_p, 2.0))));
  }
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  // map["potential_field"] = map["obstacle_potential_field"] +
  //                          map["waypoint_field"] + map["vscan_field"];
  map["potential_field"] =
      map["obstacle_potential_field"].cwiseMax(map["vscan_field"]) +
      map["waypoint_field"];

  GridMapRosConverter::toMessage(map, message);
  publisher.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                    message.info.header.stamp.toSec());
}
void vscan_points_callback(sensor_msgs::PointCloud2::ConstPtr vscan_msg) {
  ros::Time time = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ> pcl_vscan;
  pcl::fromROSMsg(*vscan_msg, pcl_vscan);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_vscan_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(pcl_vscan));
  // for (int i(0); i < (int)pcl_vscan.size(); ++i)
  //   std::cout << pcl_vscan.at(i).x << "," << pcl_vscan.at(i).y << std::endl;

  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("vscan_field", *it) = 0.0;
    for (int i(0); i < (int)pcl_vscan.size(); ++i) {
      if (3.5 < pcl_vscan.at(i).z + tf_x || pcl_vscan.at(i).z + tf_x < 0.2)
        continue;
      if (pcl_vscan.at(i).x + tf_z - map.getResolution() < position.x() &&
          position.x() < pcl_vscan.at(i).x + tf_z + map.getResolution()) {
        if (pcl_vscan.at(i).y - map.getResolution() < position.y() &&
            position.y() < pcl_vscan.at(i).y + map.getResolution()) {
          map.at("vscan_field", *it) = std::exp(0.0);
        }
      }
    }
  }
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  // map["potential_field"] = map["obstacle_potential_field"] +
  //                          map["waypoint_field"] + map["vscan_field"];
  map["potential_field"] =
      map["obstacle_potential_field"].cwiseMax(map["vscan_field"]) +
      map["waypoint_field"];
  GridMapRosConverter::toMessage(map, message);
  publisher.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                    message.info.header.stamp.toSec());
}
int main(int argc, char **argv) {
  // Initialize node and publisher.
  ros::init(argc, argv, "potential_field");
  ros::NodeHandle nh;

  publisher = nh.advertise<grid_map_msgs::GridMap>("/potential_field", 1, true);
  ros::Subscriber obj_subscriber =
      nh.subscribe("/bounding_boxes", 1, obj_callback);
  ros::Subscriber waypoint_subscriber =
      nh.subscribe("/next_target_mark", 1, target_poiny_callback);
  ros::Subscriber vscan_subscriber =
      nh.subscribe("/vscan_points", 1, vscan_points_callback);
  ROS_INFO("Created map");
  map.setFrameId("/base_link");
  map.setGeometry(Length(40.0, 25.0), 0.5);
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("obstacle_potential_field", *it) = 0.0;
    map.at("waypoint_field", *it) = 0.0;
    map.at("vscan_field", *it) = 0.0;
  }
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(), map.getSize()(0),
           map.getSize()(1));
  ros::spin();
  return 0;
}
