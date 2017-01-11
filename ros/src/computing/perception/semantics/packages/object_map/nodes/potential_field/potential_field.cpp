#include <cmath>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>

using namespace grid_map;
ros::Publisher publisher;

double ver_y_p = 1.0;
double ver_x_p = 1.0;

void obj_callback(jsk_recognition_msgs::BoundingBoxArray::ConstPtr
                      obj_msg) { // Create grid map.
  ROS_INFO("Created map");
  GridMap map({"potential_field"});
  map.setFrameId("velodyne");
  map.setGeometry(Length(30.0, 30.0), 0.5);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(), map.getSize()(0),
           map.getSize()(1));

  // Add data to grid map.
  ros::Time time = ros::Time::now();
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("potential_field", *it) = -0.04;
    for (int i(0); i < (int)obj_msg->boxes.size(); ++i) {
      if (obj_msg->boxes.at(i).pose.position.x -
                  obj_msg->boxes.at(i).dimensions.x <
              position.x() &&
          position.x() < obj_msg->boxes.at(i).pose.position.x +
                             obj_msg->boxes.at(i).dimensions.x)
        if (obj_msg->boxes.at(i).pose.position.y -
                    obj_msg->boxes.at(i).dimensions.y <
                position.y() &&
            position.y() < obj_msg->boxes.at(i).pose.position.y +
                               obj_msg->boxes.at(i).dimensions.y)
          map.at("potential_field", *it) = 1.0;

      map.at("potential_field", *it) += std::exp(
          (-1.0 *
           (std::pow((position.y() - (obj_msg->boxes.at(i).pose.position.y -
                                      obj_msg->boxes.at(i).dimensions.y)),
                     2.0) /
            std::pow(2.0 * ver_y_p, 2.0))) +
          (-1.0 *
           (std::pow((position.x() - (obj_msg->boxes.at(i).pose.position.x -
                                      obj_msg->boxes.at(i).dimensions.x)),
                     2.0) /
            std::pow(2.0 * ver_x_p, 2.0))));
      map.at("potential_field", *it) += std::exp(
          (-1.0 *
           (std::pow((position.y() - (obj_msg->boxes.at(i).pose.position.y +
                                      obj_msg->boxes.at(i).dimensions.y)),
                     2.0) /
            std::pow(2.0 * ver_y_p, 2.0))) +
          (-1.0 *
           (std::pow((position.x() - (obj_msg->boxes.at(i).pose.position.x -
                                      obj_msg->boxes.at(i).dimensions.x)),
                     2.0) /
            std::pow(2.0 * ver_x_p, 2.0))));
      map.at("potential_field", *it) += std::exp(
          (-1.0 *
           (std::pow((position.y() - (obj_msg->boxes.at(i).pose.position.y -
                                      obj_msg->boxes.at(i).dimensions.y)),
                     2.0) /
            std::pow(2.0 * ver_y_p, 2.0))) +
          (-1.0 *
           (std::pow((position.x() - (obj_msg->boxes.at(i).pose.position.x +
                                      obj_msg->boxes.at(i).dimensions.x)),
                     2.0) /
            std::pow(2.0 * ver_x_p, 2.0))));
      map.at("potential_field", *it) += std::exp(
          (-1.0 *
           (std::pow((position.y() - (obj_msg->boxes.at(i).pose.position.y +
                                      obj_msg->boxes.at(i).dimensions.y)),
                     2.0) /
            std::pow(2.0 * ver_y_p, 2.0))) +
          (-1.0 *
           (std::pow((position.x() - (obj_msg->boxes.at(i).pose.position.x +
                                      obj_msg->boxes.at(i).dimensions.x)),
                     2.0) /
            std::pow(2.0 * ver_x_p, 2.0))));
    }
    // map.at("potential_field", *it) = std::exp(
    //     (-1.0 * (std::pow((position.y()), 2) / std::pow(2.0 * ver_y_p, 2.0)))
    //     +
    //     (-1.0 *
    //      (std::pow((position.x() - 3.0), 2) / std::pow(2.0 * ver_x_p,
    //      2.0))));
  }
  // Publish grid map.
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  publisher.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                    message.info.header.stamp.toSec());
}

int main(int argc, char **argv) {
  // Initialize node and publisher.
  ros::init(argc, argv, "potential_field");
  ros::NodeHandle nh("~");
  publisher = nh.advertise<grid_map_msgs::GridMap>("/potential_field", 1, true);
  ros::Subscriber subscriber = nh.subscribe("/bounding_boxes", 1, obj_callback);
  ros::spin();
  return 0;
}
