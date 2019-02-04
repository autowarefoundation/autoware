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

#include <geometry_msgs/PoseStamped.h>
#include "autoware_msgs/Lane.h"
#include <visualization_msgs/MarkerArray.h>
#include "vector_map/vector_map.h"

#include "vector_map_server/GetWhiteLine.h"
#include "vector_map_server/GetStopLine.h"
#include "vector_map_server/GetCrossWalk.h"
#include "vector_map_server/GetSignal.h"

using vector_map::VectorMap;
using vector_map::Category;
using vector_map::Color;
using vector_map::Key;

using vector_map::Vector;
using vector_map::Line;
using vector_map::Area;
using vector_map::Pole;
using vector_map::Signal;

using vector_map::isValidMarker;
using vector_map::createVectorMarker;
using vector_map::createLineMarker;
using vector_map::createAreaMarker;
using vector_map::createPoleMarker;

namespace
{
class VectorMapClient
{
private:
  geometry_msgs::PoseStamped pose_;
  autoware_msgs::Lane waypoints_;

public:
  VectorMapClient()
  {
  }

  geometry_msgs::PoseStamped getPose() const
  {
    return pose_;
  }

  autoware_msgs::Lane getWaypoints() const
  {
    return waypoints_;
  }

  void setPose(const geometry_msgs::PoseStamped& pose)
  {
    pose_ = pose;
  }

  void setWaypoints(const autoware_msgs::Lane& waypoints)
  {
    waypoints_ = waypoints;
  }
};
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vector_map_client");

  ros::NodeHandle nh;
  VectorMapClient vmc;

  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vector_map_client", 10, true);

  VectorMap vmap;
  vmap.subscribe(nh,
                 Category::POINT | Category::VECTOR | Category::LINE | Category::AREA | Category::POLE |
                 Category::WHITE_LINE | Category::STOP_LINE | Category::CROSS_WALK | Category::SIGNAL,
                 ros::Duration(0)); // non-blocking

  ros::Subscriber pose_sub = nh.subscribe("current_pose", 1, &VectorMapClient::setPose, &vmc);
  ros::Subscriber waypoints_sub = nh.subscribe("final_waypoints", 1, &VectorMapClient::setWaypoints, &vmc);

  visualization_msgs::MarkerArray marker_array;
  ros::ServiceClient white_line_cli =
    nh.serviceClient<vector_map_server::GetWhiteLine>("vector_map_server/get_white_line");
  ros::ServiceClient stop_line_cli =
    nh.serviceClient<vector_map_server::GetStopLine>("vector_map_server/get_stop_line");
  ros::ServiceClient cross_walk_cli =
    nh.serviceClient<vector_map_server::GetCrossWalk>("vector_map_server/get_cross_walk");
  ros::ServiceClient signal_cli =
    nh.serviceClient<vector_map_server::GetSignal>("vector_map_server/get_signal");
  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();

    visualization_msgs::MarkerArray marker_array_buffer;
    int id = 0;

    vector_map_server::GetWhiteLine white_line_srv;
    white_line_srv.request.pose = vmc.getPose();
    white_line_srv.request.waypoints = vmc.getWaypoints();
    if (white_line_cli.call(white_line_srv))
    {
      for (const auto& white_line : white_line_srv.response.objects.data)
      {
        if (white_line.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(white_line.lid));
        if (line.lid == 0)
          continue;

        visualization_msgs::Marker marker = createLineMarker("white_line", id++, Color::GREEN, vmap, line);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);
        }
      }
    }

    vector_map_server::GetStopLine stop_line_srv;
    stop_line_srv.request.pose = vmc.getPose();
    stop_line_srv.request.waypoints = vmc.getWaypoints();
    if (stop_line_cli.call(stop_line_srv))
    {
      for (const auto& stop_line : stop_line_srv.response.objects.data)
      {
        if (stop_line.lid == 0)
          continue;

        Line line = vmap.findByKey(Key<Line>(stop_line.lid));
        if (line.lid == 0)
          continue;

        visualization_msgs::Marker marker = createLineMarker("stop_line", id++, Color::RED, vmap, line);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);
        }
      }
    }

    vector_map_server::GetCrossWalk cross_walk_srv;
    cross_walk_srv.request.pose = vmc.getPose();
    cross_walk_srv.request.waypoints = vmc.getWaypoints();
    if (cross_walk_cli.call(cross_walk_srv))
    {
      for (const auto& cross_walk : cross_walk_srv.response.objects.data)
      {
        if (cross_walk.aid == 0)
          continue;

        Area area = vmap.findByKey(Key<Area>(cross_walk.aid));
        if (area.aid == 0)
          continue;

        visualization_msgs::Marker marker = createAreaMarker("cross_walk", id++, Color::BLUE, vmap, area);
        if (isValidMarker(marker))
        {
          marker.scale.x = 0.4;
          marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(marker);
        }
      }
    }

    vector_map_server::GetSignal signal_srv;
    signal_srv.request.pose = vmc.getPose();
    signal_srv.request.waypoints = vmc.getWaypoints();
    if (signal_cli.call(signal_srv))
    {
      for (const auto& signal : signal_srv.response.objects.data)
      {
        if (signal.vid == 0)
          continue;

        Vector vector = vmap.findByKey(Key<Vector>(signal.vid));
        if (vector.vid == 0)
          continue;

        Pole pole;
        if (signal.plid != 0)
        {
          pole = vmap.findByKey(Key<Pole>(signal.plid));
          if (pole.plid == 0)
            continue;
        }

        visualization_msgs::Marker vector_marker;
        switch (signal.type)
        {
        case Signal::RED:
        case Signal::PEDESTRIAN_RED:
          vector_marker = createVectorMarker("signal", id++, Color::RED, vmap, vector);
          break;
        case Signal::BLUE:
        case Signal::PEDESTRIAN_BLUE:
          vector_marker = createVectorMarker("signal", id++, Color::BLUE, vmap, vector);
          break;
        case Signal::YELLOW:
          vector_marker = createVectorMarker("signal", id++, Color::YELLOW, vmap, vector);
          break;
        case Signal::RED_LEFT:
          vector_marker = createVectorMarker("signal", id++, Color::LIGHT_RED, vmap, vector);
          break;
        case Signal::BLUE_LEFT:
          vector_marker = createVectorMarker("signal", id++, Color::LIGHT_GREEN, vmap, vector);
          break;
        case Signal::YELLOW_LEFT:
          vector_marker = createVectorMarker("signal", id++, Color::LIGHT_YELLOW, vmap, vector);
          break;
        case Signal::OTHER:
          vector_marker = createVectorMarker("signal", id++, Color::CYAN, vmap, vector);
          break;
        default:
          vector_marker = createVectorMarker("signal", id++, Color::GRAY, vmap, vector);
          break;
        }
        if (isValidMarker(vector_marker))
        {
          vector_marker.type = visualization_msgs::Marker::CUBE;
          vector_marker.scale.x = 0.4;
          vector_marker.scale.y = 0.4;
          vector_marker.scale.z = 0.4;
          vector_marker.color.a = 0.4;
          marker_array_buffer.markers.push_back(vector_marker);
        }

        if (signal.plid != 0)
        {
          visualization_msgs::Marker pole_marker = createPoleMarker("signal", id++, Color::MAGENTA, vmap, pole);
          if (isValidMarker(pole_marker))
          {
            pole_marker.type = visualization_msgs::Marker::CUBE;
            pole_marker.scale.x += 0.4;
            pole_marker.scale.y += 0.4;
            pole_marker.color.a = 0.4;
            marker_array_buffer.markers.push_back(pole_marker);
          }
        }
      }
    }

    if (!marker_array.markers.empty())
    {
      for (auto& marker : marker_array.markers)
        marker.action = visualization_msgs::Marker::DELETE;
      marker_array_pub.publish(marker_array); // clear previous marker
    }
    marker_array = marker_array_buffer;
    marker_array_pub.publish(marker_array);

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
