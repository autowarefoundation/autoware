// Copyright 2022 The Autoware Contributors
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

#include <costmap_generator/points_to_costmap.hpp>

#include <gtest/gtest.h>
using pointcloud = pcl::PointCloud<pcl::PointXYZ>;
class PointsToCostmapTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  ~PointsToCostmapTest() override { rclcpp::shutdown(); }

  grid_map::GridMap construct_gridmap();

public:
  const double grid_resolution_ = 1.0;
  const double grid_length_x_ = 20.0;
  const double grid_length_y_ = 20.0;
  const double grid_position_x_ = 0.0;
  const double grid_position_y_ = 0.0;
};

grid_map::GridMap PointsToCostmapTest::construct_gridmap()
{
  grid_map::GridMap gm;

  gm.setFrameId("map");
  // set gridmap size,resolution
  gm.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_);
  // center of grid to position p in map frame
  gm.setPosition(grid_map::Position(grid_position_x_, grid_position_y_));

  // set initial value
  gm.add("points", 0);

  return gm;
}

// grid_y
// |  map_x-------
// |            |
// |            |
// |            |
// |            map_y
// |__________________grid_x
TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_validPoints)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;

  in_sensor_points.width = 3;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  in_sensor_points.points[0].x = 0.7;
  in_sensor_points.points[0].y = 1;
  in_sensor_points.points[0].z = 1;

  in_sensor_points.points[1].x = 1.1;
  in_sensor_points.points[1].y = 1;
  in_sensor_points.points[1].z = 4;

  in_sensor_points.points[2].x = 1.4;
  in_sensor_points.points[2].y = 2;
  in_sensor_points.points[2].z = 2.7;

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 5.0;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        // std::cout << "i:"<< i <<",j:"<<j<< std::endl;
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 3);
}

TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_invalidPoints_biggerThanMaximumHeightThres)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;
  in_sensor_points.width = 1;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  in_sensor_points.points[0].x = 0.7;
  in_sensor_points.points[0].y = 1;
  in_sensor_points.points[0].z = 1;  // out of [maximum_height_thres,minimum_lidar_height_thres]

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 0.99;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 0);
}

TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_invalidPoints_lessThanMinimumHeightThres)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;
  in_sensor_points.width = 1;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  in_sensor_points.points[0].x = 0.7;
  in_sensor_points.points[0].y = 1;
  in_sensor_points.points[0].z = -0.1;  // out of [maximum_height_thres,minimum_lidar_height_thres]

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 0.99;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 0);
}

TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_invalidPoints_outOfGrid)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;
  in_sensor_points.width = 1;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  // when we construct gridmap,we set grid map center to (0,0) in map frame
  // so it would be outside of grid map if absolute value of point.x bigger than half of
  // grid_length_x_
  in_sensor_points.points[0].x = 1 + grid_length_x_ / 2.0;
  in_sensor_points.points[0].y = 1 + grid_length_y_ / 2.0;
  in_sensor_points.points[0].z = 0.5;

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 0.99;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 0);
}
