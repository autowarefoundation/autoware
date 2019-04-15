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
#include <autoware_map/util.h>
#include <tf/transform_datatypes.h>

autoware_map_msgs::Point getPointFromWaypointId(int waypoint_id, autoware_map::AutowareMapHandler autoware_map)
{
    autoware_map_msgs::Waypoint waypoint = autoware_map.findById<autoware_map_msgs::Waypoint>(waypoint_id);
    return autoware_map.findById<autoware_map_msgs::Point>(waypoint.point_id);
}

bool isJapaneseCoordinate(int epsg)
{
    //EPSG CODE 2443~2461 for JGD2000
    //EPSG CODE 6669~6687 for JGD2011
    return (epsg >= 2443 && epsg <= 2461) || (epsg >= 6669 && epsg <= 6687);
}

geometry_msgs::Quaternion convertAngleToGeomQuaternion(const double horizontal_angle, const double vertical_angle)
{

    double pitch = degreeToRadian(vertical_angle - 90); // convert vertical angle to pitch
    double yaw = degreeToRadian(-horizontal_angle + 90); // convert horizontal angle to yaw
    return tf::createQuaternionMsgFromRollPitchYaw(0, pitch, yaw);
}

geometry_msgs::Point convertPointToGeomPoint(const autoware_map_msgs::Point& autoware_point)
{
    // NOTE:
    // We swap x and y axis if
    // Japan Plane Rectangular Coordinate System is used.
    geometry_msgs::Point geom_point;
    if(isJapaneseCoordinate(autoware_point.epsg))
    {
        geom_point.x = autoware_point.y;
        geom_point.y = autoware_point.x;
        geom_point.z = autoware_point.z;
    }else
    {
        geom_point.x = autoware_point.x;
        geom_point.y = autoware_point.y;
        geom_point.z = autoware_point.z;
    }
    return geom_point;
}


std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Lane& obj)
{
    os << std::fixed << obj.lane_id << ","
    << obj.start_waypoint_id << ","
    << obj.end_waypoint_id << ","
    << obj.lane_number << ","
    << obj.num_of_lanes << ","
    << obj.speed_limit << ","
    << obj.length << ","
    << obj.width_limit << ","
    << obj.height_limit << ","
    << obj.weight_limit;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneAttributeRelation& obj)
{
    os << std::fixed << obj.lane_id << ","
    << obj.attribute_type << ","
    << obj.area_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneRelation& obj)
{
    os << std::fixed << obj.lane_id << ","
    << obj.next_lane_id << ","
    << obj.blinker;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneSignalLightRelation& obj)
{
    os << std::fixed << obj.lane_id << ","
    << obj.signal_light_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::LaneChangeRelation& obj)
{
    os << std::fixed << obj.lane_id << ","
    << obj.next_lane_id << ","
    << obj.blinker;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::OppositeLaneRelation& obj)
{
    os << std::fixed << obj.lane_id << ","
    << obj.opposite_lane_id;
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Point& obj)
{
    os << std::fixed << obj.point_id << ","
    << obj.x << ","
    << obj.y << ","
    << obj.z << ","
    << obj.lat << ","
    << obj.lng << ","
    << obj.pcd << ","
    << obj.mgrs << ","
    << obj.epsg;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Area& obj)
{
    os << std::fixed << obj.area_id << ",";
    for ( auto id = obj.point_ids.begin(); id != obj.point_ids.end(); id++)
    {
        os << std::fixed << *id;
        if( id + 1 != obj.point_ids.end() )
            os << std::fixed << ":";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Signal& obj)
{
    os << std::fixed << obj.signal_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::SignalLight& obj)
{
    os << std::fixed << obj.signal_light_id << ","
    << obj.signal_id << ","
    << obj.point_id << ","
    << obj.horizontal_angle << ","
    << obj.vertical_angle << ","
    << obj.color_type << ","
    << obj.arrow_type;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Wayarea& obj)
{
    os << std::fixed << obj.wayarea_id << ","
    << obj.area_id;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::Waypoint& obj)
{
    os << std::fixed << obj.waypoint_id << ","
    << obj.point_id << ","
    << obj.velocity << ","
    << obj.stop_line << ","
    << obj.left_width << ","
    << obj.right_width << ","
    << obj.height;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointLaneRelation& obj)
{
    os << std::fixed << obj.waypoint_id << ","
    << obj.lane_id << ","
    << obj.order;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointRelation& obj)
{
    os << std::fixed << obj.waypoint_id << ","
    << obj.next_waypoint_id << ","
    << obj.yaw << ","
    << obj.blinker << ","
    << obj.distance;
    return os;
}
std::ostream& operator<<(std::ostream& os, const autoware_map_msgs::WaypointSignalRelation& obj)
{
    os << std::fixed << obj.waypoint_id << ","
    << obj.signal_id;
    return os;
}


std::ostream& operator<<(std::ostream& os, const autoware_map::Category& cat)
{
    if(cat == autoware_map::Category::NONE)
    {
        os << std::fixed << "NONE ";
        return os;
    }
    if( cat == autoware_map::Category::ALL)
    {
        os << std::fixed << "ALL ";
        return os;
    }
    if( cat & autoware_map::Category::LANE)
    {
        os << std::fixed << "LANE ";
    }
    if( cat & autoware_map::Category::LANE_ATTRIBUTE_RELATION)
    {
        os << std::fixed << "LANE_ATTRIBUTE_RELATION ";
    }
    if( cat & autoware_map::Category::LANE_RELATION)
    {
        os << std::fixed << "LANE_RELATION ";
    }
    if( cat & autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION)
    {
        os << std::fixed << "LANE_SIGNAL_LIGHT_RELATION ";
    }
    if( cat & autoware_map::Category::LANE_CHANGE_RELATION)
    {
        os << std::fixed << "LANE_CHANGE_RELATION ";
    }
    if( cat & autoware_map::Category::OPPOSITE_LANE_RELATION)
    {
        os << std::fixed << "OPPOSITE_LANE_RELATION ";
    }
    if( cat & autoware_map::Category::POINT)
    {
        os << std::fixed << "POINT ";
    }
    if( cat & autoware_map::Category::AREA)
    {
        os << std::fixed << "AREA ";
    }
    if( cat & autoware_map::Category::SIGNAL)
    {
        os << std::fixed << "SIGNAL ";
    }
    if( cat & autoware_map::Category::SIGNAL_LIGHT)
    {
        os << std::fixed << "SIGNAL_LIGHT ";
    }
    if( cat & autoware_map::Category::WAYAREA)
    {
        os << std::fixed << "WAYAREA ";
    }
    if( cat & autoware_map::Category::WAYPOINT)
    {
        os << std::fixed << "WAYPOINT ";
    }
    if( cat & autoware_map::Category::WAYPOINT_LANE_RELATION)
    {
        os << std::fixed << "WAYPOINT_LANE_RELATION ";
    }
    if( cat & autoware_map::Category::WAYPOINT_RELATION)
    {
        os << std::fixed << "WAYPOINT_RELATION ";
    }
    if( cat & autoware_map::Category::WAYPOINT_SIGNAL_RELATION)
    {
        os << "WAYPOINT_SIGNAL_RELATION ";
    }

    return os;
}


std::istream& operator>>(std::istream& is, autoware_map_msgs::Lane& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns.at(0));
    obj.start_waypoint_id = std::stoi(columns.at(1));
    obj.end_waypoint_id = std::stoi(columns.at(2));
    obj.lane_number = std::stoi(columns.at(3));
    obj.num_of_lanes = std::stoi(columns.at(4));
    obj.speed_limit = std::stod(columns.at(5));
    obj.length = std::stod(columns.at(6));
    obj.width_limit = std::stod(columns.at(7));
    obj.height_limit = std::stod(columns.at(8));
    obj.weight_limit = std::stod(columns.at(9));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneAttributeRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns.at(0));
    obj.attribute_type = std::stoi(columns.at(1));
    obj.area_id = std::stoi(columns.at(2));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns.at(0));
    obj.next_lane_id = std::stoi(columns.at(1));
    obj.blinker = std::stoi(columns.at(2));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneSignalLightRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns.at(0));
    obj.signal_light_id = std::stoi(columns.at(1));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::LaneChangeRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns.at(0));
    obj.next_lane_id = std::stoi(columns.at(1));
    obj.blinker = std::stoi(columns.at(2));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::OppositeLaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.lane_id = std::stoi(columns.at(0));
    obj.opposite_lane_id = std::stoi(columns.at(1));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Point& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.point_id = std::stoi(columns.at(0));
    obj.x = std::stod(columns.at(1));
    obj.y = std::stod(columns.at(2));
    obj.z = std::stod(columns.at(3));
    obj.lat = std::stod(columns.at(4));
    obj.lng = std::stod(columns.at(5));
    obj.pcd = columns.at(6);
    try{
        obj.mgrs = std::stoi(columns.at(7));
    }
    catch (const std::invalid_argument& e)
    {
        ROS_WARN_STREAM("invalid argument for mgrs: " << e.what());
        obj.mgrs = 0;
    }
    obj.epsg = std::stoi(columns.at(8));

    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Area& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.area_id = std::stoi(columns.at(0));
    std::stringstream ss(columns.at(1));
    while (std::getline(ss, column, ':' )) {
        obj.point_ids.push_back( std::stoi(column) );
    }

    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Signal& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.signal_id = std::stoi(columns.at(0));
    // obj.signal_light_id = std::stoi(columns.at(1));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::SignalLight& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.signal_light_id = std::stoi(columns.at(0));
    obj.signal_id = std::stoi(columns.at(1));
    obj.point_id = std::stoi(columns.at(2));
    obj.horizontal_angle = std::stod(columns.at(3));
    obj.vertical_angle = std::stod(columns.at(4));
    obj.color_type = std::stoi(columns.at(5));
    obj.arrow_type = std::stoi(columns.at(6));
    return is;
}

std::istream& operator>>(std::istream& is, autoware_map_msgs::Wayarea& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.wayarea_id = std::stoi(columns.at(0));
    obj.area_id = std::stoi(columns.at(1));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::Waypoint& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns.at(0));
    obj.point_id = std::stoi(columns.at(1));
    obj.velocity = std::stod(columns.at(2));
    obj.stop_line = std::stoi(columns.at(3));
    obj.left_width = std::stod(columns.at(4));
    obj.right_width = std::stod(columns.at(4));
    obj.height = std::stod(columns.at(5));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointLaneRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns.at(0));
    obj.lane_id = std::stoi(columns.at(1));
    obj.order = std::stoi(columns.at(2));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns.at(0));
    obj.next_waypoint_id = std::stoi(columns.at(1));
    obj.yaw = std::stod(columns.at(2));
    obj.blinker = std::stoi(columns.at(3));
    obj.distance = std::stod(columns.at(4));
    return is;
}
std::istream& operator>>(std::istream& is, autoware_map_msgs::WaypointSignalRelation& obj)
{
    std::vector<std::string> columns;
    std::string column;
    while (std::getline(is, column, ','))
    {
        columns.push_back(column);
    }
    obj.waypoint_id = std::stoi(columns.at(0));
    obj.signal_id = std::stoi(columns.at(1));
    return is;
}
