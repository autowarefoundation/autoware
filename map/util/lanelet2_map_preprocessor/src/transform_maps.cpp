// Copyright 2020 Tier IV, Inc.
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

#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <iostream>
#include <unordered_set>
#include <vector>

void printUsage()
{
  std::cerr << "Please set following private parameters:" << std::endl
            << "llt_map_path" << std::endl
            << "pcd_map_path" << std::endl
            << "llt_output_path" << std::endl
            << "pcd_output_path" << std::endl;
}

bool loadLaneletMap(
  const std::string & llt_map_path, lanelet::LaneletMapPtr & lanelet_map_ptr,
  lanelet::Projector & projector)
{
  lanelet::LaneletMapPtr lanelet_map;
  lanelet::ErrorMessages errors;
  lanelet_map_ptr = lanelet::load(llt_map_path, "autoware_osm_handler", projector, &errors);

  for (const auto & error : errors) {
    ROS_ERROR_STREAM(error);
  }
  if (!errors.empty()) {
    return false;
  }
  std::cout << "Loaded Lanelet2 map" << std::endl;
  return true;
}

bool loadPCDMap(const std::string & pcd_map_path, pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map_ptr)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_map_path, *pcd_map_ptr) == -1) {  //* load the file
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return false;
  }
  std::cout << "Loaded " << pcd_map_ptr->width * pcd_map_ptr->height << " data points."
            << std::endl;
  return true;
}

void transformMaps(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map_ptr, lanelet::LaneletMapPtr & lanelet_map_ptr,
  const Eigen::Affine3d affine)
{
  {
    for (lanelet::Point3d pt : lanelet_map_ptr->pointLayer) {
      Eigen::Vector3d eigen_pt(pt.x(), pt.y(), pt.z());
      auto transformed_pt = affine * eigen_pt;
      pt.x() = transformed_pt.x();
      pt.y() = transformed_pt.y();
      pt.z() = transformed_pt.z();
    }
  }

  {
    for (auto & pt : pcd_map_ptr->points) {
      Eigen::Vector3d eigen_pt(pt.x, pt.y, pt.z);
      auto transformed_pt = affine * eigen_pt;
      pt.x = transformed_pt.x();
      pt.y = transformed_pt.y();
      pt.z = transformed_pt.z();
    }
  }
}

Eigen::Affine3d createAffineMatrixFromXYZRPY(
  const double x, const double y, const double z, const double roll, const double pitch,
  const double yaw)
{
  double roll_rad = roll * M_PI / 180.0;
  double pitch_rad = pitch * M_PI / 180.0;
  double yaw_rad = yaw * M_PI / 180.0;

  Eigen::Translation<double, 3> trans(x, y, z);
  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
  return trans * rot;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "lanelet_map_height_adjuster");
  ros::NodeHandle pnh("~");

  if (!pnh.hasParam("llt_map_path")) {
    printUsage();
    return EXIT_FAILURE;
  }
  if (!pnh.hasParam("pcd_map_path")) {
    printUsage();
    return EXIT_FAILURE;
  }
  if (!pnh.hasParam("llt_output_path")) {
    printUsage();
    return EXIT_FAILURE;
  }
  if (!pnh.hasParam("pcd_output_path")) {
    printUsage();
    return EXIT_FAILURE;
  }
  std::string llt_map_path, pcd_map_path, llt_output_path, pcd_output_path;
  double x, y, z, roll, pitch, yaw;
  pnh.getParam("llt_map_path", llt_map_path);
  pnh.getParam("pcd_map_path", pcd_map_path);
  pnh.getParam("llt_output_path", llt_output_path);
  pnh.getParam("pcd_output_path", pcd_output_path);
  pnh.param("x", x, 0.0);
  pnh.param("y", y, 0.0);
  pnh.param("z", z, 0.0);
  pnh.param("roll", roll, 0.0);
  pnh.param("pitch", pitch, 0.0);
  pnh.param("yaw", yaw, 0.0);

  std::cout << "transforming maps with following parameters" << std::endl
            << "x " << x << std::endl
            << "y " << y << std::endl
            << "z " << z << std::endl
            << "roll " << roll << std::endl
            << "pitch " << pitch << std::endl
            << "yaw " << yaw << std::endl;

  lanelet::LaneletMapPtr llt_map_ptr(new lanelet::LaneletMap);
  lanelet::projection::MGRSProjector projector;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (!loadLaneletMap(llt_map_path, llt_map_ptr, projector)) {
    return EXIT_FAILURE;
  }
  if (!loadPCDMap(pcd_map_path, pcd_map_ptr)) {
    return EXIT_FAILURE;
  }
  Eigen::Affine3d affine = createAffineMatrixFromXYZRPY(x, y, z, roll, pitch, yaw);

  std::string mgrs_grid;
  if (pnh.hasParam("mgrs_grid")) {
    pnh.param("mgrs_grid", mgrs_grid, mgrs_grid);
    projector.setMGRSCode(mgrs_grid);
  } else {
    std::cout << "no mgrs code set. using last projected grid instead" << std::endl;
    mgrs_grid = projector.getProjectedMGRSGrid();
  }

  std::cout << "using mgrs grid: " << mgrs_grid << std::endl;

  transformMaps(pcd_map_ptr, llt_map_ptr, affine);
  lanelet::write(llt_output_path, *llt_map_ptr, projector);
  pcl::io::savePCDFileBinary(pcd_output_path, *pcd_map_ptr);
  return 0;
}
