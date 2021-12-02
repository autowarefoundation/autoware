// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
//
// Authors: Ryohsuke Mitsudome

#ifndef LANELET2_EXTENSION__IO__AUTOWARE_OSM_PARSER_HPP_
#define LANELET2_EXTENSION__IO__AUTOWARE_OSM_PARSER_HPP_

#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <memory>
#include <string>

namespace lanelet
{
namespace io_handlers
{
class AutowareOsmParser : public OsmParser
{
public:
  using OsmParser::OsmParser;

  /**
   * [parse parse osm file to laneletMap. It is generally same as default
   * OsmParser, but it will overwrite x and y value with local_x and local_y
   * tags if present]
   * @param  filename [path to osm file]
   * @param  errors   [any errors caught during parsing]
   * @return          [returns LaneletMap]
   */
  std::unique_ptr<LaneletMap> parse(
    const std::string & filename, ErrorMessages & errors) const;  // NOLINT

  /**
   * [parseVersions parses MetaInfo tags from osm file]
   * @param filename       [path to osm file]
   * @param format_version [parsed information about map format version]
   * @param map_version    [parsed information about map version]
   */
  static void parseVersions(
    const std::string & filename, std::string * format_version, std::string * map_version);

  static constexpr const char * extension() { return ".osm"; }

  static constexpr const char * name() { return "autoware_osm_handler"; }
};

}  // namespace io_handlers
}  // namespace lanelet

#endif  // LANELET2_EXTENSION__IO__AUTOWARE_OSM_PARSER_HPP_
