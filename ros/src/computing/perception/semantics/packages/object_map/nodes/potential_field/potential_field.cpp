#include "tf/transform_listener.h"
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace grid_map;

class PotentialField {
private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Subscriber waypoint_subscriber_;
  ros::Subscriber vscan_subscriber_;
  ros::Subscriber obj_subscriber_;
  bool use_target_waypoint_;
  bool use_obstacle_box_;
  bool use_vscan_points_;
  double map_x_size_;
  double map_y_size_;
  double map_resolution_;
  double tf_x_;
  double tf_z_;
  double map_x_offset_;
  GridMap map_;
  class ObstacleFieldParameter {
  public:
    ObstacleFieldParameter() : ver_x_p(0.9), ver_y_p(0.9) {}
    double ver_x_p;
    double ver_y_p;
  };
  class TargetWaypointFieldParamater {
  public:
    TargetWaypointFieldParamater() : ver_x_p(1.0), ver_y_p(1.0) {}
    double ver_x_p;
    double ver_y_p;
  };
  class VscanPointsFieldParamater {
  public:
    VscanPointsFieldParamater() : around_x(0.5), around_y(0.5) {}
    double around_x;
    double around_y;
  };

  void obj_callback(autoware_msgs::DetectedObjectArray::ConstPtr obj_msg);
  void target_waypoint_callback(
      visualization_msgs::Marker::ConstPtr target_point_msgs);
  void vscan_points_callback(sensor_msgs::PointCloud2::ConstPtr vscan_msg);
  void publish_potential_field();

public:
  PotentialField();
  void run();
  void init();
};

PotentialField::PotentialField()
    : tf_x_(1.2), tf_z_(2.0),
      map_({"potential_field", "obstacle_field", "target_waypoint_field",
            "vscan_points_field"}) {
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParam("use_obstacle_box", use_obstacle_box_)) {
    ROS_INFO("use obstacle_box");
    use_obstacle_box_ = true;
  }
  if (!private_nh.getParam("use_vscan_points", use_vscan_points_)) {
    ROS_INFO("use vscan points");
    use_vscan_points_ = true;
  }
  if (!private_nh.getParam("use_target_waypoint", use_target_waypoint_)) {
    ROS_INFO("don't use target_waypoint");
    use_target_waypoint_ = false;
  }
  if (!private_nh.getParam("map_resolution", map_resolution_)) {
    map_resolution_ = 0.25;
    ROS_INFO("map resolution %f", map_resolution_);
  }
  if (!private_nh.getParam("map_x_size", map_x_size_)) {
    map_x_size_ = 40.0;
    ROS_INFO("map x size %f", map_x_size_);
  }
  if (!private_nh.getParam("map_y_size", map_y_size_)) {
    map_y_size_ = 25.0;
    ROS_INFO("map y size %f", map_y_size_);
  }
  if (!private_nh.getParam("map_x_offset", map_x_offset_)) {
    map_x_offset_ = 10.0;
    ROS_INFO("map x offset %f", map_x_offset_);
  }
  publisher_ =
      nh_.advertise<grid_map_msgs::GridMap>("/potential_field", 1, true);

  if (use_obstacle_box_)
    obj_subscriber_ = nh_.subscribe("/detected_objects", 1,
                                    &PotentialField::obj_callback, this);
  if (use_vscan_points_)
    vscan_subscriber_ = nh_.subscribe(
        "/vscan_points", 1, &PotentialField::vscan_points_callback, this);
  if (use_target_waypoint_)
    waypoint_subscriber_ =
        nh_.subscribe("/next_target_mark", 1,
                      &PotentialField::target_waypoint_callback, this);
}

void PotentialField::init() {
  ROS_INFO("Created map");
  map_.setFrameId("/potential_field_link");
  map_.setGeometry(Length(map_x_size_, map_y_size_), map_resolution_);
  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    Position position;
    map_.getPosition(*it, position);
    map_.at("obstacle_field", *it) = 0.0;
    map_.at("target_waypoint_field", *it) = 0.0;
    map_.at("vscan_points_field", *it) = 0.0;
    map_.at("potential_field", *it) = 0.0;
  }
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
           map_.getLength().x(), map_.getLength().y(), map_.getSize()(0),
           map_.getSize()(1));
}
void PotentialField::run() { ros::spin(); }

void PotentialField::publish_potential_field() {
  grid_map_msgs::GridMap message;

  map_["potential_field"] =
      map_["obstacle_field"].cwiseMax(map_["vscan_points_field"]) +
      map_["target_waypoint_field"];
  GridMapRosConverter::toMessage(map_, message);
  publisher_.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                    message.info.header.stamp.toSec());
}
void PotentialField::obj_callback(
    autoware_msgs::DetectedObjectArray::ConstPtr obj_msg) { // Create grid map.
  static ObstacleFieldParameter param;
  double ver_x_p(param.ver_x_p);
  double ver_y_p(param.ver_y_p);

  // Add data to grid map.
  ros::Time time = ros::Time::now();

  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {

    Position position;
    map_.getPosition(*it, position);
    map_.at("obstacle_field", *it) = 0.0;
    for (int i(0); i < (int)obj_msg->objects.size(); ++i) {
      double pos_x =
          obj_msg->objects.at(i).pose.position.x + tf_x_ - map_x_offset_;
      double pos_y = obj_msg->objects.at(i).pose.position.y;
      double len_x = obj_msg->objects.at(i).dimensions.x / 2.0;
      double len_y = obj_msg->objects.at(i).dimensions.y / 2.0;

      double r, p, y;
      tf::Quaternion quat(obj_msg->objects.at(i).pose.orientation.x,
                          obj_msg->objects.at(i).pose.orientation.y,
                          obj_msg->objects.at(i).pose.orientation.z,
                          obj_msg->objects.at(i).pose.orientation.w);
      tf::Matrix3x3(quat).getRPY(r, p, y);

      double rotated_pos_x = std::cos(-1.0 * y) * (position.x() - pos_x) -
                             std::sin(-1.0 * y) * (position.y() - pos_y) +
                             pos_x;
      double rotated_pos_y = std::sin(-1.0 * y) * (position.x() - pos_x) +
                             std::cos(-1.0 * y) * (position.y() - pos_y) +
                             pos_y;

      if (-0.5 < pos_x && pos_x < 4.0) {
        if (-1.0 < pos_y && pos_y < 1.0)
          continue;
      }
      if (pos_x - len_x < rotated_pos_x && rotated_pos_x < pos_x + len_x) {
        if (pos_y - len_y < rotated_pos_y && rotated_pos_y < pos_y + len_y) {
          map_.at("obstacle_field", *it) =
              std::max(std::exp(0.0),
                       static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (rotated_pos_y < pos_y - len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y - len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y + len_y < rotated_pos_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y + len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        }
      } else if (rotated_pos_x < pos_x - len_x) {
        if (rotated_pos_y < pos_y - len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y - len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x - len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y + len_y < rotated_pos_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y + len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x - len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y - len_y < rotated_pos_y &&
                   rotated_pos_y < pos_y + len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x - len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        }
      } else if (pos_x + len_x < rotated_pos_x) {
        if (rotated_pos_y < pos_y - len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y - len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x + len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y + len_y / 2.0 < rotated_pos_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y + len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x + len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y - len_y < rotated_pos_y &&
                   rotated_pos_y < pos_y + len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x + len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        }
      }
    }
  }
  // Publish grid map.
  map_.setTimestamp(time.toNSec());
  publish_potential_field();
}

void PotentialField::target_waypoint_callback(
    visualization_msgs::Marker::ConstPtr target_point_msgs) {
  static TargetWaypointFieldParamater param;
  double ver_x_p(param.ver_x_p);
  double ver_y_p(param.ver_y_p);
  ros::Time time = ros::Time::now();
  geometry_msgs::PointStamped in, out;
  in.header = target_point_msgs->header;
  in.point = target_point_msgs->pose.position;
  tf::TransformListener tflistener;
  try {
    ros::Time now = ros::Time(0);
    tflistener.waitForTransform("/map", "/potential_field_link", now,
                                ros::Duration(10.0));
    tflistener.transformPoint("/potential_field_link", in.header.stamp, in,
                              "/map", out);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    Position position;
    map_.getPosition(*it, position);
    map_.at("target_waypoint_field", *it) = 0.0;
    map_.at("target_waypoint_field", *it) -=
        0.5 * std::exp((-1.0 * (std::pow((position.y() - (out.point.y)), 2.0) /
                                std::pow(2.0 * ver_y_p, 2.0))) +
                       (-1.0 * (std::pow((position.x() - (out.point.x)), 2.0) /
                                std::pow(2.0 * ver_x_p, 2.0))));
  }
  map_.setTimestamp(time.toNSec());
}

void PotentialField::vscan_points_callback(
    sensor_msgs::PointCloud2::ConstPtr vscan_msg) {
  static VscanPointsFieldParamater param;
  double around_x(param.around_x);
  double around_y(param.around_y);

  ros::Time time = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ> pcl_vscan;
  pcl::fromROSMsg(*vscan_msg, pcl_vscan);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_vscan_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(pcl_vscan));

  double length_x = map_.getLength().x() / 2.0;
  double length_y = map_.getLength().y() / 2.0;

  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    Position position;
    map_.getPosition(*it, position);
    map_.at("vscan_points_field", *it) = 0.0;
    for (int i(0); i < (int)pcl_vscan.size(); ++i) {
      double point_x = pcl_vscan.at(i).x - map_x_offset_;
      if (3.0 < pcl_vscan.at(i).z + tf_z_ || pcl_vscan.at(i).z + tf_z_ < 0.3)
        continue;
      if (length_x < point_x && point_x < -1.0 * length_x)
        continue;
      if (length_y < pcl_vscan.at(i).y && pcl_vscan.at(i).y < -1.0 * length_y)
        continue;
      if ((point_x + tf_x_) - around_x < position.x() &&
          position.x() < point_x + tf_x_ + around_x) {
        if (pcl_vscan.at(i).y - around_y < position.y() &&
            position.y() < pcl_vscan.at(i).y + around_y) {
          map_.at("vscan_points_field", *it) = 1.0; // std::exp(0.0) ;
        }
      }
    }
  }
  map_.setTimestamp(time.toNSec());
  publish_potential_field();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "potential_field");

  PotentialField potential_field;
  potential_field.init();
  potential_field.run();
  return 0;
}
