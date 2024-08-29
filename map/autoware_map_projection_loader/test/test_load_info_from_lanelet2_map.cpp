// Copyright 2023 TIER IV, Inc.
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

#include "autoware/map_projection_loader/load_info_from_lanelet2_map.hpp"

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <gmock/gmock.h>

#include <fstream>

void save_dummy_mgrs_lanelet2_map(const std::string & mgrs_coord, const std::string & output_path)
{
  int zone = std::stoi(mgrs_coord.substr(0, 2));
  bool is_north = false;
  int precision = 0;
  double utm_x{};
  double utm_y{};
  double lat{};
  double lon{};
  GeographicLib::MGRS::Reverse(mgrs_coord, zone, is_north, utm_x, utm_y, precision, false);
  GeographicLib::UTMUPS::Reverse(zone, is_north, utm_x, utm_y, lat, lon);

  std::ofstream file(output_path);
  if (!file) {
    std::cerr << "Unable to open file.\n";
    return;
  }

  file << "<?xml version=\"1.0\"?>\n";
  file << "<osm version=\"0.6\" generator=\"lanelet2\">\n";
  file << R"(  <node id="1" lat=")" << lat << "\" lon=\"" << lon << "\"/>\n";
  file << "</osm>";

  file.close();
}

void save_dummy_local_lanelet2_map(const std::string & output_path)
{
  std::ofstream file(output_path);
  if (!file) {
    std::cerr << "Unable to open file.\n";
    return;
  }

  file << "<?xml version=\"1.0\"?>\n";
  file << "<osm version=\"0.6\" generator=\"lanelet2\">\n";
  file << "  <node id=\"1\" lat=\"\" lon=\"\"/>\n";
  file << "  <node id=\"2\" lat=\"\" lon=\"\"/>\n";
  file << "  <node id=\"3\" lat=\"\" lon=\"\"/>\n";
  file << "</osm>";

  file.close();
}

void save_dummy_mgrs_lanelet2_map_with_one_zero_point(const std::string & output_path)
{
  std::ofstream file(output_path);
  if (!file) {
    std::cerr << "Unable to open file.\n";
    return;
  }

  file << "<?xml version=\"1.0\"?>\n";
  file << "<osm version=\"0.6\" generator=\"lanelet2\">\n";
  file << "  <node id=\"1\" lat=\"0.0\" lon=\"0.0\"/>\n";
  file << "  <node id=\"2\" lat=\"0.00001\" lon=\"0.00001\"/>\n";
  file << "  <node id=\"3\" lat=\"0.00002\" lon=\"0.00002\"/>\n";
  file << "</osm>";

  file.close();
}

TEST(TestLoadFromLanelet2Map, LoadMGRSGrid)
{
  // Save dummy lanelet2 map
  const std::string mgrs_grid = "54SUE";
  const std::string mgrs_coord = mgrs_grid + "1000010000";
  const std::string output_path = "/tmp/test_load_info_from_lanelet2_map.osm";
  save_dummy_mgrs_lanelet2_map(mgrs_coord, output_path);

  // Test the function
  const auto projector_info =
    autoware::map_projection_loader::load_info_from_lanelet2_map(output_path);

  // Check the result
  EXPECT_EQ(projector_info.projector_type, "MGRS");
  EXPECT_EQ(projector_info.mgrs_grid, mgrs_grid);
}

TEST(TestLoadFromLanelet2Map, LoadLocalGrid)
{
  // Save dummy lanelet2 map
  const std::string output_path = "/tmp/test_load_info_from_lanelet2_map.osm";
  save_dummy_local_lanelet2_map(output_path);

  // Test the function
  const auto projector_info =
    autoware::map_projection_loader::load_info_from_lanelet2_map(output_path);

  // Check the result
  EXPECT_EQ(projector_info.projector_type, "local");
}

TEST(TestLoadFromLanelet2Map, LoadNoLocalGrid)
{
  // Save dummy lanelet2 map
  const std::string output_path = "/tmp/test_load_info_from_lanelet2_map.osm";
  save_dummy_mgrs_lanelet2_map_with_one_zero_point(output_path);

  // Test the function
  const auto projector_info =
    autoware::map_projection_loader::load_info_from_lanelet2_map(output_path);

  // Check the result
  EXPECT_EQ(projector_info.projector_type, "MGRS");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
