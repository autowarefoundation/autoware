// Copyright 2020 TierIV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

//lint -e537 pclint vs cpplint NOLINT

#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <common/types.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include "had_map_utils/had_map_visualization.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace had_map_utils
{

template<typename T>
bool8_t exists(const std::unordered_set<T> & set, const T & element)
{
  return std::find(set.begin(), set.end(), element) != set.end();
}

void setColor(
  std_msgs::msg::ColorRGBA * cl,
  const float32_t & r, const float32_t & g, const float32_t & b, const float32_t & a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void setMarkerHeader(
  visualization_msgs::msg::Marker * m,
  const int32_t & id,
  const rclcpp::Time & t,
  const std::string & frame_id,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & c,
  const int32_t & action,
  const int32_t & type,
  const float32_t & scale)
{
  m->header.frame_id = frame_id;
  m->header.stamp = t;
  m->ns = ns;
  m->action = action;
  m->type = type;

  m->id = id;
  m->pose.position.x = 0.0;
  m->pose.position.y = 0.0;
  m->pose.position.z = 0.0;
  m->pose.orientation.x = 0.0;
  m->pose.orientation.y = 0.0;
  m->pose.orientation.z = 0.0;
  m->pose.orientation.w = 1.0;
  m->scale.x = scale;
  m->scale.y = scale;
  m->scale.z = scale;
  m->color = c;
}

visualization_msgs::msg::Marker lineString2Marker(
  const rclcpp::Time & t,
  const lanelet::LineString3d & ls,
  const std::string & frame_id,
  const std::string & ns, const std_msgs::msg::ColorRGBA & c, const float32_t & lss)
{
  visualization_msgs::msg::Marker line_strip;
  setMarkerHeader(
    &line_strip, static_cast<int32_t>(ls.id()), t, frame_id, ns, c,
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::LINE_STRIP,
    lss);

  for (auto i = ls.begin(); i != ls.end(); i++) {
    geometry_msgs::msg::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = (*i).z();
    line_strip.points.push_back(p);
  }
  return line_strip;
}

visualization_msgs::msg::Marker lineString2Marker(
  const rclcpp::Time & t,
  const lanelet::ConstLineString3d & ls,
  const std::string & frame_id, const std::string & ns, const std_msgs::msg::ColorRGBA & c,
  const float32_t & lss)
{
  visualization_msgs::msg::Marker line_strip;
  setMarkerHeader(
    &line_strip, static_cast<int32_t>(ls.id()), t, frame_id, ns, c,
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::LINE_STRIP,
    lss);

  for (auto i = ls.begin(); i != ls.end(); i++) {
    geometry_msgs::msg::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = (*i).z();
    line_strip.points.push_back(p);
  }
  return line_strip;
}

visualization_msgs::msg::MarkerArray lineStringsAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns,
  const lanelet::LineStrings3d & linestrings,
  const std_msgs::msg::ColorRGBA & c)
{
  float32_t lss = 0.1f;
  visualization_msgs::msg::MarkerArray marker_array;

  for (auto lsi = linestrings.begin(); lsi != linestrings.end(); lsi++) {
    lanelet::LineString3d ls = *lsi;
    visualization_msgs::msg::Marker line_strip = lineString2Marker(t, ls, "map", ns, c, lss);
    marker_array.markers.push_back(line_strip);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray laneletsBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c,
  const bool8_t & viz_centerline)
{
  float32_t lss = 0.1f;
  std::unordered_set<lanelet::Id> added;
  visualization_msgs::msg::MarkerArray marker_array;

  for (auto li = lanelets.begin(); li != lanelets.end(); li++) {
    lanelet::ConstLanelet lll = *li;
    lanelet::ConstLineString3d left_ls = lll.leftBound();
    lanelet::ConstLineString3d right_ls = lll.rightBound();
    lanelet::ConstLineString3d center_ls = lll.centerline();

    visualization_msgs::msg::Marker left_line_strip, right_line_strip, center_line_strip;
    if (!exists(added, left_ls.id())) {
      left_line_strip = lineString2Marker(t, left_ls, "map", "left_lane_bound", c, lss);
      marker_array.markers.push_back(left_line_strip);
      added.insert(left_ls.id());
    }
    if (!exists(added, right_ls.id())) {
      right_line_strip = lineString2Marker(
        t, right_ls, "map", "right_lane_bound", c, lss);
      marker_array.markers.push_back(right_line_strip);
      added.insert(right_ls.id());
    }
    if (viz_centerline && !exists(added, center_ls.id())) {
      center_line_strip = lineString2Marker(
        t, center_ls, "map", "center_lane_line",
        c, std::max(lss * 0.1f, 0.01f));
      marker_array.markers.push_back(center_line_strip);
      added.insert(center_ls.id());
    }
  }
  return marker_array;
}

visualization_msgs::msg::Marker basicPolygon2Marker(
  const rclcpp::Time & t,
  const int32_t & line_id,
  const lanelet::BasicPolygon3d & pg,
  const std::string & frame_id,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & c,
  const float32_t & lss)
{
  visualization_msgs::msg::Marker line_strip;
  setMarkerHeader(
    &line_strip, line_id, t, frame_id, ns, c,
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::LINE_STRIP,
    lss);

  for (auto i = pg.begin(); i != pg.end(); i++) {
    geometry_msgs::msg::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = (*i).z();
    line_strip.points.push_back(p);
  }
  geometry_msgs::msg::Point pb;
  auto i = pg.begin();
  pb.x = (*i).x();
  pb.y = (*i).y();
  pb.z = (*i).z();
  line_strip.points.push_back(pb);
  return line_strip;
}

visualization_msgs::msg::MarkerArray areasBoundaryAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns,
  const lanelet::Areas & areas,
  const std_msgs::msg::ColorRGBA & c)
{
  int pg_count = 0;
  float32_t lss = 0.1f;
  visualization_msgs::msg::MarkerArray marker_array;
  for (auto area : areas) {
    lanelet::CompoundPolygon3d cpg = area.outerBoundPolygon();
    lanelet::BasicPolygon3d bpg = cpg.basicPolygon();

    visualization_msgs::msg::Marker line_strip = basicPolygon2Marker(
      t, pg_count, bpg, "map", ns, c,
      lss);
    marker_array.markers.push_back(line_strip);
    pg_count++;
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray polygonsBoundaryAsMarkerArray(
  const rclcpp::Time & t, const std::string & ns,
  const lanelet::Polygons3d & polygons, const std_msgs::msg::ColorRGBA & c)
{
  int32_t pg_count = 0;
  float32_t lss = 0.1f;
  visualization_msgs::msg::MarkerArray marker_array;
  for (auto poly : polygons) {
    lanelet::BasicPolygon3d bpg = poly.basicPolygon();
    visualization_msgs::msg::Marker line_strip = basicPolygon2Marker(
      t, pg_count, bpg, "map", ns, c,
      lss);
    marker_array.markers.push_back(line_strip);
    pg_count++;
  }
  return marker_array;
}

visualization_msgs::msg::Marker bbox2Marker(
  const rclcpp::Time & t, const int32_t & line_id, const float64_t lower[], const float64_t upper[],
  const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const float32_t & lss)
{
  visualization_msgs::msg::Marker line_strip;
  setMarkerHeader(
    &line_strip, line_id, t, frame_id, ns, c,
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::LINE_STRIP,
    lss);

  geometry_msgs::msg::Point bl, br, tl, tr;
  bl.x = lower[0]; bl.y = lower[0]; bl.z = 0.0;
  br.x = upper[0]; br.y = lower[0]; br.z = 0.0;
  tl.x = lower[0]; tl.y = upper[0]; tl.z = 0.0;
  tr.x = upper[0]; tr.y = upper[0]; tr.z = 0.0;

  line_strip.points.push_back(bl);
  line_strip.points.push_back(br);
  line_strip.points.push_back(tr);
  line_strip.points.push_back(tl);
  line_strip.points.push_back(bl);
  return line_strip;
}

visualization_msgs::msg::MarkerArray boundingBoxAsMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns, const float64_t upper[], const float64_t lower[],
  const std_msgs::msg::ColorRGBA & c)
{
  float32_t lss = 0.2f;
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker line_strip = bbox2Marker(t, 0, upper, lower, "map", ns, c, lss);
  marker_array.markers.push_back(line_strip);
  return marker_array;
}

geometry_msgs::msg::Point toGeomMsgPt(const geometry_msgs::msg::Point32 & src)
{
  geometry_msgs::msg::Point dst;
  dst.x = static_cast<float32_t>(src.x);
  dst.y = static_cast<float32_t>(src.y);
  dst.z = static_cast<float32_t>(src.z);
  return dst;
}

geometry_msgs::msg::Point32 toGeomMsgPt32(const lanelet::BasicPoint3d & src)
{
  geometry_msgs::msg::Point32 dst;
  dst.x = static_cast<float32_t>(src.x());
  dst.y = static_cast<float32_t>(src.y());
  dst.z = static_cast<float32_t>(src.z());
  return dst;
}
void adjacentPoints(
  const size_t i, const size_t N,
  const geometry_msgs::msg::Polygon poly,
  geometry_msgs::msg::Point32 * p0,
  geometry_msgs::msg::Point32 * p1,
  geometry_msgs::msg::Point32 * p2)
{
  *p1 = poly.points[i];
  if (i == 0) {
    *p0 = poly.points[N - 1];
  } else {
    *p0 = poly.points[i - 1];
  }
  if (i < N - 1) {
    *p2 = poly.points[i + 1];
  } else {
    *p2 = poly.points[0];
  }
}

std::vector<geometry_msgs::msg::Polygon> lanelet2Triangle(
  const lanelet::ConstLanelet & ll)
{
  geometry_msgs::msg::Polygon ls_poly = lanelet2Polygon(ll);
  return polygon2Triangle(ls_poly);
}

std::vector<geometry_msgs::msg::Polygon> area2Triangle(
  const lanelet::Area & area)
{
  geometry_msgs::msg::Polygon ls_poly = area2Polygon(area);
  return polygon2Triangle(ls_poly);
}

// Is angle AOB less than 180?
// https://qiita.com/fujii-kotaro/items/a411f2a45627ed2f156e
bool8_t isAcuteAngle(
  const geometry_msgs::msg::Point32 & vertex_a,
  const geometry_msgs::msg::Point32 & vertex_o,
  const geometry_msgs::msg::Point32 & vertex_b)
{
  return (vertex_a.x - vertex_o.x) * (vertex_b.y - vertex_o.y) -
         (vertex_a.y - vertex_o.y) * (vertex_b.x - vertex_o.x) >= 0;
}

// https://qiita.com/fujii-kotaro/items/a411f2a45627ed2f156e
bool8_t isWithinTriangle(
  const geometry_msgs::msg::Point32 & vertex_a,
  const geometry_msgs::msg::Point32 & vertex_b,
  const geometry_msgs::msg::Point32 & vertex_c,
  const geometry_msgs::msg::Point32 & pt)
{
  float64_t c1 = (vertex_b.x - vertex_a.x) * (pt.y - vertex_b.y) -
    (vertex_b.y - vertex_a.y) * (pt.x - vertex_b.x);
  float64_t c2 = (vertex_c.x - vertex_b.x) * (pt.y - vertex_c.y) -
    (vertex_c.y - vertex_b.y) * (pt.x - vertex_c.x);
  float64_t c3 = (vertex_a.x - vertex_c.x) * (pt.y - vertex_a.y) -
    (vertex_a.y - vertex_c.y) * (pt.x - vertex_a.x);

  return (c1 > 0.0 && c2 > 0.0 && c3 > 0.0) || (c1 < 0.0 && c2 < 0.0 && c3 < 0.0);
}

std::vector<geometry_msgs::msg::Polygon> polygon2Triangle(
  const geometry_msgs::msg::Polygon & polygon)
{
  std::vector<geometry_msgs::msg::Polygon> triangles;
  geometry_msgs::msg::Polygon poly = polygon;
  size_t num_vertices = poly.points.size();

  std::vector<bool8_t> is_acute_angle;
  is_acute_angle.assign(num_vertices, false);
  for (size_t i = 0; i < num_vertices; i++) {
    geometry_msgs::msg::Point32 p0, p1, p2;

    adjacentPoints(i, num_vertices, poly, &p0, &p1, &p2);
    is_acute_angle.at(i) = isAcuteAngle(p0, p1, p2);
  }
  while (num_vertices >= 3) {
    size_t clipped_vertex = 0;

    for (size_t i = 0; i < num_vertices; i++) {
      bool8_t theta = is_acute_angle.at(i);
      if (theta == true) {
        geometry_msgs::msg::Point32 p0, p1, p2;
        adjacentPoints(i, num_vertices, poly, &p0, &p1, &p2);

        size_t j_begin = (i + 2) % num_vertices;
        size_t j_end = (i - 1 + num_vertices) % num_vertices;
        bool8_t is_ear = true;
        for (size_t j = j_begin; j != j_end; j = (j + 1) % num_vertices) {
          if (isWithinTriangle(p0, p1, p2, poly.points.at(j))) {
            is_ear = false;
            break;
          }
        }

        if (is_ear) {
          clipped_vertex = i;
          break;
        }
      }
    }
    geometry_msgs::msg::Point32 p0, p1, p2;
    adjacentPoints(clipped_vertex, num_vertices, poly, &p0, &p1, &p2);
    geometry_msgs::msg::Polygon triangle;
    triangle.points.push_back(p0);
    triangle.points.push_back(p1);
    triangle.points.push_back(p2);
    triangles.push_back(triangle);
    auto it = poly.points.begin();
    std::advance(it, clipped_vertex);
    poly.points.erase(it);

    auto it_angle = is_acute_angle.begin();
    std::advance(it_angle, clipped_vertex);
    is_acute_angle.erase(it_angle);

    num_vertices = poly.points.size();
    if (clipped_vertex == num_vertices) {
      clipped_vertex = 0;
    }
    adjacentPoints(clipped_vertex, num_vertices, poly, &p0, &p1, &p2);
    is_acute_angle.at(clipped_vertex) = isAcuteAngle(p0, p1, p2);

    size_t i_prev = (clipped_vertex == 0) ? num_vertices - 1 : clipped_vertex - 1;
    adjacentPoints(i_prev, num_vertices, poly, &p0, &p1, &p2);
    is_acute_angle.at(i_prev) = isAcuteAngle(p0, p1, p2);
  }
  return triangles;
}


geometry_msgs::msg::Polygon area2Polygon(
  const lanelet::ConstArea & area)
{
  geometry_msgs::msg::Polygon polygon;
  polygon.points.clear();
  polygon.points.reserve(area.outerBoundPolygon().size());

  std::transform(
    area.outerBoundPolygon().begin(),
    area.outerBoundPolygon().end(),
    std::back_inserter(polygon.points),
    [](lanelet::ConstPoint3d pt) {return toGeomMsgPt32(pt.basicPoint());});
  return polygon;
}

geometry_msgs::msg::Polygon lanelet2Polygon(
  const lanelet::ConstLanelet & ll)
{
  geometry_msgs::msg::Polygon polygon;

  const lanelet::CompoundPolygon3d & ll_poly = ll.polygon3d();
  polygon.points.clear();
  polygon.points.reserve(ll_poly.size());

  std::transform(
    ll_poly.begin(),
    ll_poly.end(),
    std::back_inserter(polygon.points),
    [](lanelet::ConstPoint3d pt) {return toGeomMsgPt32(pt.basicPoint());});
  return polygon;
}

visualization_msgs::msg::MarkerArray laneletsAsTriangleMarkerArray(
  const rclcpp::Time & t,
  const std::string & ns,
  const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  if (lanelets.empty()) {
    return marker_array;
  }

  setMarkerHeader(
    &marker, 0, t, "map", ns, c,
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::TRIANGLE_LIST,
    1.0);

  for (auto ll : lanelets) {
    std::vector<geometry_msgs::msg::Polygon> triangles = lanelet2Triangle(ll);

    for (auto tri : triangles) {
      geometry_msgs::msg::Point tri0[3];

      for (size_t i = 0; i < 3; i++) {
        tri0[i] = toGeomMsgPt(tri.points[i]);

        marker.points.push_back(tri0[i]);
        marker.colors.push_back(c);
      }
    }
  }
  if (!marker.points.empty()) {
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray areasAsTriangleMarkerArray(
  const rclcpp::Time & t, const std::string & ns, const lanelet::Areas & areas,
  const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  if (areas.empty()) {
    return marker_array;
  }

  setMarkerHeader(
    &marker, 0, t, "map", ns, c,
    visualization_msgs::msg::Marker::ADD,
    visualization_msgs::msg::Marker::TRIANGLE_LIST,
    1.0);

  for (auto area : areas) {
    std::vector<geometry_msgs::msg::Polygon> triangles = area2Triangle(area);
    for (auto tri : triangles) {
      for (size_t i = 0; i < 3; i++) {
        marker.points.push_back(toGeomMsgPt(tri.points[i]));
        marker.colors.push_back(c);
      }
    }
  }

  if (!marker.points.empty()) {
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware
