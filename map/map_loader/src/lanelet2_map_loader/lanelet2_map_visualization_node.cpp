// Copyright 2021 Tier IV, Inc.
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
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "map_loader/lanelet2_map_visualization_node.hpp"

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <vector>

namespace
{
void insertMarkerArray(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

void setColor(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}
}  // namespace

Lanelet2MapVisualizationNode::Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_visualization", options)
{
  using std::placeholders::_1;

  viz_lanelets_centerline_ = this->declare_parameter("viz_lanelets_centerline", true);

  sub_map_bin_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&Lanelet2MapVisualizationNode::onMapBin, this, _1));

  pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "output/lanelet2_map_marker", rclcpp::QoS{1}.transient_local());
}

void Lanelet2MapVisualizationNode::onMapBin(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*msg, viz_lanelet_map);
  RCLCPP_INFO(this->get_logger(), "Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets shoulder_lanelets = lanelet::utils::query::shoulderLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLineStrings3d partitions = lanelet::utils::query::getAllPartitions(viz_lanelet_map);
  lanelet::ConstLineStrings3d pedestrian_markings =
    lanelet::utils::query::getAllPedestrianMarkings(viz_lanelet_map);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines =
    lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems =
    lanelet::utils::query::trafficLights(all_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  std::vector<lanelet::NoStoppingAreaConstPtr> no_reg_elems =
    lanelet::utils::query::noStoppingAreas(all_lanelets);
  std::vector<lanelet::SpeedBumpConstPtr> sb_reg_elems =
    lanelet::utils::query::speedBumps(all_lanelets);
  std::vector<lanelet::CrosswalkConstPtr> cw_reg_elems =
    lanelet::utils::query::crosswalks(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(viz_lanelet_map);
  lanelet::ConstPolygons3d parking_lots = lanelet::utils::query::getAllParkingLots(viz_lanelet_map);
  lanelet::ConstPolygons3d obstacle_polygons =
    lanelet::utils::query::getAllObstaclePolygons(viz_lanelet_map);
  lanelet::ConstPolygons3d no_obstacle_segmentation_area =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map, "no_obstacle_segmentation_area");
  lanelet::ConstPolygons3d no_obstacle_segmentation_area_for_run_out =
    lanelet::utils::query::getAllPolygonsByType(
      viz_lanelet_map, "no_obstacle_segmentation_area_for_run_out");
  lanelet::ConstPolygons3d hatched_road_markings_area =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map, "hatched_road_markings");
  lanelet::ConstPolygons3d intersection_areas =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map, "intersection_area");
  std::vector<lanelet::NoParkingAreaConstPtr> no_parking_reg_elems =
    lanelet::utils::query::noParkingAreas(all_lanelets);
  lanelet::ConstLineStrings3d curbstones = lanelet::utils::query::curbstones(viz_lanelet_map);

  std_msgs::msg::ColorRGBA cl_road, cl_shoulder, cl_cross, cl_partitions, cl_pedestrian_markings,
    cl_ll_borders, cl_shoulder_borders, cl_stoplines, cl_trafficlights, cl_detection_areas,
    cl_speed_bumps, cl_crosswalks, cl_parking_lots, cl_parking_spaces, cl_lanelet_id,
    cl_obstacle_polygons, cl_no_stopping_areas, cl_no_obstacle_segmentation_area,
    cl_no_obstacle_segmentation_area_for_run_out, cl_hatched_road_markings_area,
    cl_hatched_road_markings_line, cl_no_parking_areas, cl_curbstones, cl_intersection_area;
  setColor(&cl_road, 0.27, 0.27, 0.27, 0.999);
  setColor(&cl_shoulder, 0.15, 0.15, 0.15, 0.999);
  setColor(&cl_cross, 0.27, 0.3, 0.27, 0.5);
  setColor(&cl_partitions, 0.25, 0.25, 0.25, 0.999);
  setColor(&cl_pedestrian_markings, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_ll_borders, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_shoulder_borders, 0.2, 0.2, 0.2, 0.999);
  setColor(&cl_stoplines, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_trafficlights, 0.5, 0.5, 0.5, 0.8);
  setColor(&cl_detection_areas, 0.27, 0.27, 0.37, 0.5);
  setColor(&cl_no_stopping_areas, 0.37, 0.37, 0.37, 0.5);
  setColor(&cl_speed_bumps, 0.56, 0.40, 0.27, 0.5);
  setColor(&cl_crosswalks, 0.80, 0.80, 0.0, 0.5);
  setColor(&cl_obstacle_polygons, 0.4, 0.27, 0.27, 0.5);
  setColor(&cl_parking_lots, 0.5, 0.5, 0.0, 0.3);
  setColor(&cl_parking_spaces, 1.0, 0.647, 0.0, 0.6);
  setColor(&cl_lanelet_id, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_no_obstacle_segmentation_area, 0.37, 0.37, 0.27, 0.5);
  setColor(&cl_no_obstacle_segmentation_area_for_run_out, 0.37, 0.7, 0.27, 0.5);
  setColor(&cl_hatched_road_markings_area, 0.3, 0.3, 0.3, 0.5);
  setColor(&cl_hatched_road_markings_line, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_no_parking_areas, 0.42, 0.42, 0.42, 0.5);
  setColor(&cl_curbstones, 0.1, 0.1, 0.2, 0.999);
  setColor(&cl_intersection_area, 0.16, 1.0, 0.69, 0.5);

  visualization_msgs::msg::MarkerArray map_marker_array;

  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.5));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(partitions, "partitions", cl_partitions, 0.1));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::laneletDirectionAsMarkerArray(shoulder_lanelets, "shoulder_"));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::pedestrianMarkingsAsMarkerArray(
                         pedestrian_markings, cl_pedestrian_markings));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "walkway_lanelets", walkway_lanelets, cl_cross));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::obstaclePolygonsAsMarkerArray(obstacle_polygons, cl_obstacle_polygons));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::noStoppingAreasAsMarkerArray(no_reg_elems, cl_no_stopping_areas));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::speedBumpsAsMarkerArray(sb_reg_elems, cl_speed_bumps));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::crosswalkAreasAsMarkerArray(cw_reg_elems, cl_crosswalks));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(
      shoulder_lanelets, cl_shoulder_borders, viz_lanelets_centerline_, "shoulder_"));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                         road_lanelets, cl_ll_borders, viz_lanelets_centerline_));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::generateTrafficLightIdMaker(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::generateLaneletIdMarker(shoulder_lanelets, cl_lanelet_id));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::generateLaneletIdMarker(
                         crosswalk_lanelets, cl_lanelet_id, "crosswalk_lanelet_id"));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "shoulder_road_lanelets", shoulder_lanelets, cl_shoulder));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::noObstacleSegmentationAreaAsMarkerArray(
                         no_obstacle_segmentation_area, cl_no_obstacle_segmentation_area));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::noObstacleSegmentationAreaForRunOutAsMarkerArray(
      no_obstacle_segmentation_area_for_run_out, cl_no_obstacle_segmentation_area_for_run_out));

  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::hatchedRoadMarkingsAreaAsMarkerArray(
      hatched_road_markings_area, cl_hatched_road_markings_area, cl_hatched_road_markings_line));

  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::noParkingAreasAsMarkerArray(no_parking_reg_elems, cl_no_parking_areas));

  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(curbstones, "curbstone", cl_curbstones, 0.2));

  insertMarkerArray(
    &map_marker_array, lanelet::visualization::intersectionAreaAsMarkerArray(
                         intersection_areas, cl_intersection_area));

  pub_marker_->publish(map_marker_array);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapVisualizationNode)
