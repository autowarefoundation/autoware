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
#ifndef __AUTOWARE_MAP_UTIL_H__
#define __AUTOWARE_MAP_UTIL_H__

#include <autoware_map/map_handler.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

inline double degreeToRadian(double x) {return x * M_PI / 180.0;}

autoware_map_msgs::Point getPointFromWaypointId(int waypoint_id, autoware_map::AutowareMapHandler autoware_map);
geometry_msgs::Point convertPointToGeomPoint(const autoware_map_msgs::Point& autoware_point);
geometry_msgs::Quaternion convertAngleToGeomQuaternion(const double horizontal_angle, const double vertical_angle);
bool isJapaneseCoordinate(int epsg);

//
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Lane& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneAttributeRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneSignalLightRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneChangeRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::OppositeLaneRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Point& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Area& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Signal& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::SignalLight& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Wayarea& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Waypoint& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointLaneRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointSignalRelation& obj);
std::ostream& operator<<(std::ostream& os, const autoware_map::Category& cat);

//
std::istream& operator>>(std::istream& os, autoware_map_msgs::Lane& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneAttributeRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneSignalLightRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::LaneChangeRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::OppositeLaneRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Point& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Area& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Signal& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::SignalLight& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Wayarea& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::Waypoint& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::WaypointLaneRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::WaypointSignalRelation& obj);
std::istream& operator>>(std::istream& os, autoware_map_msgs::WaypointRelation& obj);

#endif
