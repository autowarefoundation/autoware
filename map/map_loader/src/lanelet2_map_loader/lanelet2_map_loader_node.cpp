// Copyright 2021 TierIV
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

#include "map_loader/lanelet2_map_loader_node.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <string>

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  const auto lanelet2_filename = declare_parameter("lanelet2_map_path", "");
  const auto lanelet2_map_projector_type = declare_parameter("lanelet2_map_projector_type", "MGRS");
  const auto center_line_resolution = declare_parameter("center_line_resolution", 5.0);

  // load map from file
  const auto map = load_map(*this, lanelet2_filename, lanelet2_map_projector_type);
  if (!map) {
    return;
  }

  // overwrite centerline
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  // create map bin msg
  const auto map_bin_msg = create_map_bin_msg(map, lanelet2_filename, now());

  // create publisher and publish
  pub_map_bin_ =
    create_publisher<HADMapBin>("output/lanelet2_map", rclcpp::QoS{1}.transient_local());
  pub_map_bin_->publish(map_bin_msg);
}

lanelet::LaneletMapPtr Lanelet2MapLoaderNode::load_map(
  rclcpp::Node & node, const std::string & lanelet2_filename,
  const std::string & lanelet2_map_projector_type)
{
  lanelet::ErrorMessages errors{};
  if (lanelet2_map_projector_type == "MGRS") {
    lanelet::projection::MGRSProjector projector{};
    const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);
    if (errors.empty()) {
      return map;
    }
  } else if (lanelet2_map_projector_type == "UTM") {
    const double map_origin_lat = node.declare_parameter("latitude", 0.0);
    const double map_origin_lon = node.declare_parameter("longitude", 0.0);
    lanelet::GPSPoint position{map_origin_lat, map_origin_lon};
    lanelet::Origin origin{position};
    lanelet::projection::UtmProjector projector{origin};

    const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);
    if (errors.empty()) {
      return map;
    }
  } else if (lanelet2_map_projector_type == "local") {
    // Use MGRSProjector as parser
    lanelet::projection::MGRSProjector projector{};
    const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);

    // overwrite local_x, local_y
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) {
        point.x() = point.attribute("local_x").asDouble().value();
      }
      if (point.hasAttribute("local_y")) {
        point.y() = point.attribute("local_y").asDouble().value();
      }
    }

    // realign lanelet borders using updated points
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }

    return map;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "lanelet2_map_projector_type is not supported");
    return nullptr;
  }

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
  }
  return nullptr;
}

HADMapBin Lanelet2MapLoaderNode::create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  HADMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Lanelet2MapLoaderNode)
