/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
#ifndef __AUTOWARE_MAP_VISUALIZATION_HPP__
#define __AUTOWARE_MAP_VISUALIZATION_HPP__

#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_map/map_handler.hpp>
#include <autoware_map/visualization.h>
#include <autoware_map/util.h>

const double MARKER_SCALE_POINT = 0.1;
const double MARKER_SCALE_VECTOR = 0.08;
const double MARKER_SCALE_VECTOR_LENGTH = 0.64;

using autoware_map::AutowareMapHandler;
using autoware_map::Area;
using autoware_map::Lane;
using autoware_map::Point;
using autoware_map::Signal;
using autoware_map::SignalLight;
using autoware_map::Wayarea;
using autoware_map::Waypoint;
using autoware_map::WaypointRelation;
using autoware_map::WaypointSignalRelation;

std_msgs::ColorRGBA createColorRGBA(Color color)
{
    const double COLOR_VALUE_MIN = 0.0;
    const double COLOR_VALUE_MAX = 1.0;
    const double COLOR_VALUE_MEDIAN = 0.5;
    const double COLOR_VALUE_LIGHT_LOW = 0.56;
    const double COLOR_VALUE_LIGHT_HIGH = 0.93;

    std_msgs::ColorRGBA color_rgba;
    color_rgba.r = COLOR_VALUE_MIN;
    color_rgba.g = COLOR_VALUE_MIN;
    color_rgba.b = COLOR_VALUE_MIN;
    color_rgba.a = COLOR_VALUE_MEDIAN;

    switch (color)
    {
        case BLACK:
            break;
        case GRAY:
            color_rgba.r = COLOR_VALUE_MEDIAN;
            color_rgba.g = COLOR_VALUE_MEDIAN;
            color_rgba.b = COLOR_VALUE_MEDIAN;
            break;
        case LIGHT_RED:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_GREEN:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_BLUE:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case LIGHT_YELLOW:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_LOW;
            break;
        case LIGHT_CYAN:
            color_rgba.r = COLOR_VALUE_LIGHT_LOW;
            color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case LIGHT_MAGENTA:
            color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
            color_rgba.g = COLOR_VALUE_LIGHT_LOW;
            color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
            break;
        case RED:
            color_rgba.r = COLOR_VALUE_MAX;
            break;
        case GREEN:
            color_rgba.g = COLOR_VALUE_MAX;
            break;
        case BLUE:
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case YELLOW:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.g = COLOR_VALUE_MAX;
            break;
        case CYAN:
            color_rgba.g = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case MAGENTA:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        case WHITE:
            color_rgba.r = COLOR_VALUE_MAX;
            color_rgba.g = COLOR_VALUE_MAX;
            color_rgba.b = COLOR_VALUE_MAX;
            break;
        default:
            color_rgba.a = COLOR_VALUE_MIN; // hide color from view
            break;
    }

    return color_rgba;
}


void enableMarker(visualization_msgs::Marker& marker)
{
    marker.action = visualization_msgs::Marker::ADD;
}

void disableMarker(visualization_msgs::Marker& marker)
{
    marker.action = visualization_msgs::Marker::DELETE;
}

bool isValidMarker(const visualization_msgs::Marker& marker)
{
    return marker.action == visualization_msgs::Marker::ADD;
}

void insertMarkerArray(visualization_msgs::MarkerArray& a1, const visualization_msgs::MarkerArray& a2)
{
    a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

visualization_msgs::Marker createMarker(const std::string& ns, int id, int type)
{
    visualization_msgs::Marker marker;
    // NOTE: Autoware want to use map messages with or without /use_sim_time.
    // Therefore we don't set marker.header.stamp.
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.lifetime = ros::Duration();
    marker.frame_locked = true;
    disableMarker(marker);
    return marker;
}

visualization_msgs::MarkerArray createAreaMarkerArray(const AutowareMapHandler& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto& area : autoware_map.findByFilter([] (const Area &area){return true; }))
    {
        visualization_msgs::Marker line_strip = createMarker("area", id++, visualization_msgs::Marker::LINE_STRIP);;
        for (auto point_id: area.point_ids){
          Point point = autoware_map.findById<Point>(point_id);
          line_strip.points.push_back(convertPointToGeomPoint(point));
        }
        Point closing_point = autoware_map.findById<Point>(area.point_ids.front());

        line_strip.points.push_back(convertPointToGeomPoint(closing_point));
        line_strip.scale.x = MARKER_SCALE_POINT;
        line_strip.color = createColorRGBA(color);
        enableMarker(line_strip);
        marker_array.markers.push_back(line_strip);
    }
    return marker_array;
}


visualization_msgs::Marker createVectorMarkerFromSignal(const std::string& ns, int id, Color color, const AutowareMapHandler& autoware_map,
                                              const SignalLight signal_light)
{
    visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::ARROW);

    Point point = autoware_map.findById<Point>(signal_light.point_id);
    if (point.point_id == 0)
        return marker;

    marker.pose.position = convertPointToGeomPoint(point);
    marker.pose.orientation = convertAngleToGeomQuaternion(signal_light.horizontal_angle, signal_light.vertical_angle);
    marker.scale.x = MARKER_SCALE_VECTOR_LENGTH;
    marker.scale.y = MARKER_SCALE_VECTOR;
    marker.scale.z = MARKER_SCALE_VECTOR;
    marker.color = createColorRGBA(color);

    enableMarker(marker);
    return marker;
}

visualization_msgs::MarkerArray createSignalMarkerArray(const AutowareMapHandler& autoware_map, Color red_color, Color blue_color,
                                                        Color yellow_color, Color other_color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (const auto& signal_light : autoware_map.findByFilter([] (const SignalLight &signal){return true; }))
    {
        visualization_msgs::Marker vector_marker;
        switch (signal_light.color_type)
        {
            case 1: //RED
            case 11: //RED flashing
                vector_marker = createVectorMarkerFromSignal("signal", id++, red_color, autoware_map, signal_light);
                break;
            case 2:  //GREEN
            case 12: //GREEN Flashing
                vector_marker = createVectorMarkerFromSignal("signal", id++, blue_color, autoware_map, signal_light);
                break;
            case 3: //YELLOW
            case 13: //YELLOW Flashing
                vector_marker = createVectorMarkerFromSignal("signal", id++, yellow_color, autoware_map, signal_light);
                break;
            default:
                ROS_WARN_STREAM("[createSignalMarkerArray] unknown signal.type: " << signal_light.color_type << " Creating Marker as OTHER.");
                vector_marker = createVectorMarkerFromSignal("signal", id++, other_color, autoware_map, signal_light);
                break;
        }
        if (isValidMarker(vector_marker))
            marker_array.markers.push_back(vector_marker);
        else
            ROS_ERROR_STREAM("[createSignalMarkerArray] failed createVectorMarker: " << signal_light);
    }
    return marker_array;
}

visualization_msgs::MarkerArray createWaypointMarkerArray(const AutowareMapHandler& autoware_map, Color color, Color stop_color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    visualization_msgs::Marker marker = createMarker("waypoints", id++, visualization_msgs::Marker::POINTS);
    visualization_msgs::Marker stop_marker = createMarker("waypoints", id++, visualization_msgs::Marker::POINTS);
    for ( auto waypoint : autoware_map.findByFilter([] (const Waypoint &){return true; }))
    {
        if(waypoint.stop_line == 1) {
            stop_marker.points.push_back(convertPointToGeomPoint(getPointFromWaypointId(waypoint.waypoint_id,autoware_map)));
        }
        else
        {
            marker.points.push_back(convertPointToGeomPoint(getPointFromWaypointId(waypoint.waypoint_id,autoware_map)));
        }
    }
    marker.scale.x = stop_marker.scale.x = MARKER_SCALE_POINT;
    marker.scale.y = stop_marker.scale.y = MARKER_SCALE_POINT;
    marker.color = createColorRGBA(color);
    stop_marker.color = createColorRGBA(stop_color);
    enableMarker(marker);
    enableMarker(stop_marker);
    marker_array.markers.push_back(marker);
    marker_array.markers.push_back(stop_marker);
    return marker_array;
}

visualization_msgs::MarkerArray createWaypointSignalRelationMarkerArray(const AutowareMapHandler& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    for ( auto relation : autoware_map.findByFilter([] (const WaypointSignalRelation &){return true; }))
    {
        Waypoint waypoint = autoware_map.findById<Waypoint>(relation.waypoint_id);
        // Waypoint waypoint = autoware_map.findByKey(autoware_map::Key<Waypoint>(relation.waypoint_id));
        Point waypoint_point = autoware_map.findById<Point>(waypoint.point_id);
        std::vector<SignalLight> signals = autoware_map.findByFilter([&](SignalLight light){return light.signal_id == relation.signal_id; });
        if(!signals.empty())
        {
            Point signal_point = autoware_map.findById<Point>(signals.front().point_id);
            visualization_msgs::Marker marker = createMarker("waypoint_signal_relation", id++, visualization_msgs::Marker::ARROW );
            marker.points.push_back(convertPointToGeomPoint(signal_point));
            marker.points.push_back(convertPointToGeomPoint(waypoint_point));
            marker.scale.x = 0.1; // shaft diameter
            marker.scale.y = 1; //head_diameter
            marker.scale.z = 1; //head_length
            marker.color = createColorRGBA(color);
            enableMarker(marker);
            marker_array.markers.push_back(marker);
        }
    }
    return marker_array;
}
visualization_msgs::MarkerArray createWaypointRelationMarkerArray(const AutowareMapHandler& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    for ( auto relation : autoware_map.findByFilter([] (const WaypointRelation &){return true; }))
    {
        Point point = getPointFromWaypointId(relation.waypoint_id, autoware_map);
        Point next_point = getPointFromWaypointId(relation.next_waypoint_id, autoware_map);

        visualization_msgs::Marker marker = createMarker("waypoint_relations", id++, visualization_msgs::Marker::ARROW );
        marker.points.push_back(convertPointToGeomPoint(point));
        marker.points.push_back(convertPointToGeomPoint(next_point));
        marker.scale.x = 0.1;     // shaft diameter
        marker.scale.y = 0.3;     //head_diameter
        marker.scale.z = 0.3;     //head_length
        marker.color = createColorRGBA(color);
        enableMarker(marker);
        marker_array.markers.push_back(marker);

    }
    return marker_array;
}
visualization_msgs::MarkerArray createPointMarkerArray(const AutowareMapHandler& autoware_map, Color color)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    visualization_msgs::Marker marker = createMarker("points", id++, visualization_msgs::Marker::POINTS);
    for ( auto point : autoware_map.findByFilter([] (const Point &){return true; }))
    {
        marker.points.push_back(convertPointToGeomPoint(point));
    }
    marker.scale.x = MARKER_SCALE_POINT;
    marker.scale.y = MARKER_SCALE_POINT;
    marker.color = createColorRGBA(color);
    enableMarker(marker);
    marker_array.markers.push_back(marker);
    return marker_array;
}

#endif
