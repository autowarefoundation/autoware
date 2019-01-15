/*
 * GridMapRosTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Peter Fankhauser, Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/gtest_eigen.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_msgs/GridMap.h"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

// STD
#include <string>
#include <vector>
#include <stdlib.h>
#include <iterator>

// ROS
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace grid_map;

TEST(RosMessageConversion, roundTrip)
{
  GridMap mapIn({"layer"});
  mapIn.setGeometry(Length(2.0, 3.0), 0.5, Position(1.0, 1.5));
  mapIn["layer"].setRandom();

  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(mapIn, message);
  GridMap mapOut;
  GridMapRosConverter::fromMessage(message, mapOut);

  for (size_t i = 0; i < mapIn.getLayers().size(); ++i) {
    EXPECT_EQ(mapIn.getLayers().at(i), mapOut.getLayers().at(i));
    const std::string layer = mapIn.getLayers().at(i);
    EXPECT_TRUE((mapIn[layer].array() == mapOut[layer].array()).all());
  }

  EXPECT_EQ(mapIn.getFrameId(), mapOut.getFrameId());
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getPosition().array() == mapOut.getPosition().array()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(RosbagHandling, saveLoad)
{
  string layer = "layer";
  string pathToBag = "test.bag";
  string topic = "topic";
  vector<string> layers;
  layers.push_back(layer);
  grid_map::GridMap gridMapIn(layers), gridMapOut;
  gridMapIn.setGeometry(grid_map::Length(1.0, 1.0), 0.5);
  double a = 1.0;
  for (grid_map::GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    gridMapIn.at(layer, *iterator) = a;
    a += 1.0;
  }

  EXPECT_FALSE(gridMapOut.exists(layer));

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    EXPECT_DOUBLE_EQ(gridMapIn.at(layer, *iterator), gridMapOut.at(layer, *iterator));
  }
}

TEST(RosbagHandling, saveLoadWithTime)
{
  string layer = "layer";
  string pathToBag = "test.bag";
  string topic = "topic";
  vector<string> layers;
  layers.push_back(layer);
  GridMap gridMapIn(layers), gridMapOut;
  gridMapIn.setGeometry(grid_map::Length(1.0, 1.0), 0.5);
  double a = 1.0;
  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    gridMapIn.at(layer, *iterator) = a;
    a += 1.0;
  }

  EXPECT_FALSE(gridMapOut.exists(layer));

  if (!ros::Time::isValid()) ros::Time::init();
  // TODO Do other time than now.
  gridMapIn.setTimestamp(ros::Time::now().toNSec());

  EXPECT_TRUE(GridMapRosConverter::saveToBag(gridMapIn, pathToBag, topic));
  EXPECT_TRUE(GridMapRosConverter::loadFromBag(pathToBag, topic, gridMapOut));

  EXPECT_TRUE(gridMapOut.exists(layer));

  for (GridMapIterator iterator(gridMapIn); !iterator.isPastEnd(); ++iterator) {
    EXPECT_DOUBLE_EQ(gridMapIn.at(layer, *iterator), gridMapOut.at(layer, *iterator));
  }
}

TEST(OccupancyGridConversion, withMove)
{
  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(8.0, 5.0), 0.5, grid_map::Position(0.0, 0.0));
  map.add("layer", 1.0);

  // Convert to OccupancyGrid msg.
  nav_msgs::OccupancyGrid occupancyGrid;
  GridMapRosConverter::toOccupancyGrid(map, "layer", 0.0, 1.0, occupancyGrid);

  // Expect the (0, 0) cell to have value 100.
  EXPECT_DOUBLE_EQ(100.0, occupancyGrid.data[0]);

  // Move the map, so the cell (0, 0) will move to unobserved space.
  map.move(grid_map::Position(-1.0, -1.0));

  // Convert again to OccupancyGrid msg.
  nav_msgs::OccupancyGrid occupancyGridNew;
  GridMapRosConverter::toOccupancyGrid(map, "layer", 0.0, 1.0, occupancyGridNew);

  // Now the (0, 0) cell should be unobserved (-1).
  EXPECT_DOUBLE_EQ(-1.0, occupancyGridNew.data[0]);
}

TEST(OccupancyGridConversion, roundTrip)
{
  // Create occupancy grid.
  nav_msgs::OccupancyGrid occupancyGrid;
  occupancyGrid.header.stamp = ros::Time(5.0);
  occupancyGrid.header.frame_id = "map";
  occupancyGrid.info.resolution = 0.1;
  occupancyGrid.info.width = 50;
  occupancyGrid.info.height = 100;
  occupancyGrid.info.origin.position.x = 3.0;
  occupancyGrid.info.origin.position.y = 6.0;
  occupancyGrid.info.origin.orientation.w = 1.0;
  occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

  for (auto& cell : occupancyGrid.data) {
    cell = rand() % 102 - 1; // [-1, 100]
  }

  // Convert to grid map.
  GridMap gridMap;
  GridMapRosConverter::fromOccupancyGrid(occupancyGrid, "layer", gridMap);

  // Convert back to occupancy grid.
  nav_msgs::OccupancyGrid occupancyGridResult;
  GridMapRosConverter::toOccupancyGrid(gridMap, "layer", -1.0, 100.0, occupancyGridResult);

  // Check map info.
  EXPECT_EQ(occupancyGrid.header.stamp, occupancyGridResult.header.stamp);
  EXPECT_EQ(occupancyGrid.header.frame_id, occupancyGridResult.header.frame_id);
  EXPECT_EQ(occupancyGrid.info.width, occupancyGridResult.info.width);
  EXPECT_EQ(occupancyGrid.info.height, occupancyGridResult.info.height);
  EXPECT_DOUBLE_EQ(occupancyGrid.info.origin.position.x, occupancyGridResult.info.origin.position.x);
  EXPECT_DOUBLE_EQ(occupancyGrid.info.origin.position.x, occupancyGridResult.info.origin.position.x);
  EXPECT_DOUBLE_EQ(occupancyGrid.info.origin.orientation.x, occupancyGridResult.info.origin.orientation.x);
  EXPECT_DOUBLE_EQ(occupancyGrid.info.origin.orientation.y, occupancyGridResult.info.origin.orientation.y);
  EXPECT_DOUBLE_EQ(occupancyGrid.info.origin.orientation.z, occupancyGridResult.info.origin.orientation.z);
  EXPECT_DOUBLE_EQ(occupancyGrid.info.origin.orientation.w, occupancyGridResult.info.origin.orientation.w);

  // Check map data.
  for (std::vector<int8_t>::iterator iterator = occupancyGrid.data.begin();
      iterator != occupancyGrid.data.end(); ++iterator) {
    size_t i = std::distance(occupancyGrid.data.begin(), iterator);
    EXPECT_EQ((int)*iterator, (int)occupancyGridResult.data[i]);
  }
}

TEST(ImageConversion, roundTripBGRA8)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn["layer"].setRandom();
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image message.
  sensor_msgs::Image image;
  GridMapRosConverter::toImage(mapIn, "layer", sensor_msgs::image_encodings::BGRA8, minValue,
                               maxValue, image);

  // Convert back to grid map.
  GridMap mapOut;
  GridMapRosConverter::initializeFromImage(image, mapIn.getResolution(), mapOut);
  GridMapRosConverter::addLayerFromImage(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  const float resolution = (maxValue - minValue) / (float) std::numeric_limits<unsigned char>::max();
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}

TEST(ImageConversion, roundTripMONO16)
{
  // Create grid map.
  GridMap mapIn({"layer"});
  mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
  mapIn["layer"].setRandom();
  const float minValue = -1.0;
  const float maxValue = 1.0;

  // Convert to image message.
  sensor_msgs::Image image;
  GridMapRosConverter::toImage(mapIn, "layer", sensor_msgs::image_encodings::MONO16,
                               minValue, maxValue, image);

  // Convert back to grid map.
  GridMap mapOut;
  GridMapRosConverter::initializeFromImage(image, mapIn.getResolution(), mapOut);
  GridMapRosConverter::addLayerFromImage(image, "layer", mapOut, minValue, maxValue);

  // Check data.
  // TODO Why is factor 300 necessary?
  const float resolution = 300.0 * (maxValue - minValue) / (float) std::numeric_limits<unsigned short>::max();
  expectNear(mapIn["layer"], mapOut["layer"], resolution, "");
  EXPECT_EQ(mapIn.getTimestamp(), mapOut.getTimestamp());
  EXPECT_TRUE((mapIn.getLength() == mapOut.getLength()).all());
  EXPECT_TRUE((mapIn.getSize() == mapOut.getSize()).all());
}
