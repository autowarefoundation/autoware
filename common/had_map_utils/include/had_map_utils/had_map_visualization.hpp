// Copyright 2020 Tier IV, Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HAD_MAP_UTILS__HAD_MAP_VISUALIZATION_HPP_
#define HAD_MAP_UTILS__HAD_MAP_VISUALIZATION_HPP_

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <chrono>
#include <unordered_set>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "had_map_utils/visibility_control.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace autoware
{
namespace common
{
namespace had_map_utils
{
/**
 * \brief Set set rgba information to a Color Object
 * \param[out] cl color object to be set
 * \param r red value
 * \param g green value
 * \param b blue value
 * \param a alpha value
 */
void HAD_MAP_UTILS_PUBLIC setColor(
  std_msgs::msg::ColorRGBA * cl,
  const float32_t & r, const float32_t & g, const float32_t & b, const float32_t & a);

/**
 * \brief Set the header information to a marker object
 * \param m input marker
 * \param id id of the marker
 * \param t timestamp of the marker
 * \param frame_id frame of the marker
 * \param ns namespace of the marker
 * \param c color of the marker
 * \param action action used to visualize the marker
 * \param type type of the marker
 * \param scale scale of the marker
 * \return visualization_msgs::msg::Marker
 */
void HAD_MAP_UTILS_PUBLIC setMarkerHeader(
  visualization_msgs::msg::Marker * m,
  const int32_t & id, const rclcpp::Time & t,
  const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const int32_t & action, const int32_t & type,
  const float32_t & scale);

/**
 * \brief creates marker with type LINE_STRIP from a lanelet::LineString3d object
 * \param t timestamp set to the marker
 * \param ls input linestring
 * \param frame_id frame id set to the marker
 * \param ns namespace set to the marker
 * \param c color of the marker
 * \param lss linestrip scale (i.e. width)
 * \return created visualization_msgs::msg::Marker
 */
visualization_msgs::msg::Marker HAD_MAP_UTILS_PUBLIC lineString2Marker(
  const rclcpp::Time & t,
  const lanelet::LineString3d & ls,
  const std::string & frame_id, const std::string & ns, const std_msgs::msg::ColorRGBA & c,
  const float32_t & lss);

/**
 * \brief creates marker with type LINE_STRIP from a lanelet::ConstLineString3d object
 * \param t timestamp set to the marker
 * \param ls input linestring
 * \param frame_id frame id set to the marker
 * \param ns namespace set to the marker
 * \param c color of the marker
 * \param lss linestrip scale (i.e. width)
 * \return created visualization_msgs::msg::Marker
 */
visualization_msgs::msg::Marker HAD_MAP_UTILS_PUBLIC lineString2Marker(
  const rclcpp::Time & t,
  const lanelet::ConstLineString3d & ls,
  const std::string & frame_id, const std::string & ns, const std_msgs::msg::ColorRGBA & c,
  const float32_t & lss);

/**
 * \brief converts lanelet::LineString into markers with type LINE_STRIP
 * \param t time set to returned marker message
 * \param ns namespace set to the marker
 * \param linestrings input linestring objects
 * \param c color of the marker
 * \return created visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC
lineStringsAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns,
  const lanelet::LineStrings3d & linestrings,
  const std_msgs::msg::ColorRGBA & c);

/**
 * \brief converts outer bound of lanelet::Lanelet into markers with type LINE_STRIP
 * \param t time set to returned marker message
 * \param lanelets input lanelet objects
 * \param c color of the marker
 * \param viz_centerline option to add centerline to the marker array
 * \return created visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC laneletsBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c,
  const bool8_t & viz_centerline);

/**
 * \brief creates marker with type LINE_STRIP from a lanelet::BasicPolygon object
 * \param t timestamp set to the marker
 * \param line_id id set to the marker
 * \param pg input polygon
 * \param frame_id frame id set to the marker
 * \param ns namespace set to the marker
 * \param c color of the marker
 * \param lss linestrip scale (i.e. width)
 * \return created visualization_msgs::msg::Marker
 */
visualization_msgs::msg::Marker HAD_MAP_UTILS_PUBLIC basicPolygon2Marker(
  const rclcpp::Time & t,
  const int32_t & line_id, const lanelet::BasicPolygon3d & pg,
  const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const float32_t & lss);

/**
 * \brief converts outer bound of lanelet::Area into markers with type LINE_STRIP
 * \param t time set to returned marker message
 * \param ns namespace set to the marker
 * \param areas input area objects
 * \param c color of the marker
 * \return created visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC areasBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns, const lanelet::Areas & areas,
  const std_msgs::msg::ColorRGBA & c);

/**
 * \brief converts outer bound of lanelet::Polygon into markers with type LINE_STRIP
 * \param t Time set to returned marker message
 * \param ns namespace set to the marker
 * \param polygons input polygons
 * \param c color of the marker
 * \return created visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC polygonsBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns, const lanelet::Polygons3d & polygons,
  const std_msgs::msg::ColorRGBA & c);

/**
 * \brief creates marker with type LINE_STRIP from a bounding box
 * \param t Time set to returned marker message
 * \param line_id id set to marker
 * \param lower lower bound of the bounding box with length 3(x,y,z)
 * \param upper upper bound of the bounding box with length 3(x,y,z)
 * \param frame_id frame id set to the marker
 * \param ns namespace set to the marker
 * \param c color of the marker
 * \param lss linestrip scale (i.e. width)
 * \return created visualization_msgs::msg::Marker
 */
visualization_msgs::msg::Marker HAD_MAP_UTILS_PUBLIC bbox2Marker(
  const rclcpp::Time & t, const int32_t & line_id,
  const float64_t lower[], const float64_t upper[],
  const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const float32_t & lss);

/**
 * \brief creates marker array from bounding box
 * \param t Time set to returned marker message
 * \param ns Namespace set to returned marker message
 * \param upper upper bound of the bounding box with length 3(x,y,z)
 * \param lower lower bound of the bounding box with length 3(x,y,z)
 * \param c Color of the marker array
 * \return created visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC boundingBoxAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns, const float64_t upper[], const float64_t lower[],
  const std_msgs::msg::ColorRGBA & c);

/**
 * \brief converts area enclosed by lanelet::Lanelet into list of triangles.
 * \param ll input lanelet
 * \return result of triangulation
 */
std::vector<geometry_msgs::msg::Polygon> HAD_MAP_UTILS_PUBLIC lanelet2Triangle(
  const lanelet::ConstLanelet & ll);

/**
 * \brief converts area enclosed by geometry_msg::msg::Polygon into list of triangles.
 * \param polygon input polygon
 * \return result of triangulation
 */
std::vector<geometry_msgs::msg::Polygon> HAD_MAP_UTILS_PUBLIC polygon2Triangle(
  const geometry_msgs::msg::Polygon & polygon);

/**
 * \brief converts lanelet::Area into geometry_msgs::msg::Polygon type
 * \param area input area
 * \return converted geometry_msgs::msg::Polygon
 */
geometry_msgs::msg::Polygon HAD_MAP_UTILS_PUBLIC area2Polygon(
  const lanelet::ConstArea & area);

/**
 * \brief converts lanelet::Lanelet into geometry_msgs::msg::Polygon type
 * \param ll input lanelet
 * \return converted geometry_msgs::msg::Polygon
 */
geometry_msgs::msg::Polygon HAD_MAP_UTILS_PUBLIC lanelet2Polygon(
  const lanelet::ConstLanelet & ll);

/**
 * \brief converts bounded area by lanelet::Lanelet into triangle markers
 * \param t Time set to returned marker message
 * \param ns Namespace set to returned marker message
 * \param lanelets input lanelet::Lanelet
 * \param c Color of the marker array
 * \return Converted triangle markers enclosed by the Lanelet
 */
visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC laneletsAsTriangleMarkerArray(
  const rclcpp::Time & t, const std::string & ns, const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c);

/**
 * \brief converts bounded area by lanelet::Area into triangle markers
 * \param t Time set to returned marker message
 * \param ns Namespace set to returned marker message
 * \param areas input lanelet::Area objects
 * \param c Color of the marker array
 * \return Converted triangle markers enclosed by the area
 */
visualization_msgs::msg::MarkerArray HAD_MAP_UTILS_PUBLIC areasAsTriangleMarkerArray(
  const rclcpp::Time & t, const std::string & ns, const lanelet::Areas & areas,
  const std_msgs::msg::ColorRGBA & c);

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware

#endif  // HAD_MAP_UTILS__HAD_MAP_VISUALIZATION_HPP_
