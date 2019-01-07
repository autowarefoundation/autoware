/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <autoware_config_msgs/ConfigPoints2Polygon.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl_conversions/pcl_conversions.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1;

static constexpr bool USE_PCD_FILE = false;
static const std::string VELODYNE_POINTS_PCD = "/tmp/velodyne_points.pcd";
static const std::string POINTS_POLYGON_VTK = "/tmp/points_polygon.vtk";

static int config_k_search = 20;
static double config_search_radius = 0.025;
static double config_mu = 2.5;
static int config_maximum_nearest_neighbors = 100;
static double config_maximum_surface_angle = M_PI / 4; // 45 degrees
static double config_minimum_angle = M_PI / 18; // 10 degrees
static double config_maximum_angle = 2 * M_PI / 3; // 120 degrees
static bool config_normal_consistency = false;

static bool use_pcd_file;
static std::string velodyne_points_pcd;
static std::string points_polygon_vtk;

static void points_to_polygon(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr
		normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr
		tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(config_k_search);
	n.compute(*normals);

	// Concatenate the XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr
		cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	// Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr
		tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points
	// (maximum edge length)
	gp3.setSearchRadius(config_search_radius);

	// Set typical values for the parameters
	gp3.setMu(config_mu);
	gp3.setMaximumNearestNeighbors(config_maximum_nearest_neighbors);
	gp3.setMaximumSurfaceAngle(config_maximum_surface_angle);
	gp3.setMinimumAngle(config_minimum_angle);
	gp3.setMaximumAngle(config_maximum_angle);
	gp3.setNormalConsistency(config_normal_consistency);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// Save result into a output file
	pcl::io::saveVTKFile(points_polygon_vtk, triangles);
}

static void config_callback(const autoware_config_msgs::ConfigPoints2Polygon& msg)
{
	config_k_search = msg.k_search;
	config_search_radius = msg.search_radius;
	config_mu = msg.mu;
	config_maximum_nearest_neighbors = msg.maximum_nearest_neighbors;
	config_maximum_surface_angle = msg.maximum_surface_angle;
	config_minimum_angle = msg.minimum_angle;
	config_maximum_angle = msg.maximum_angle;
	config_normal_consistency = msg.normal_consistency;

	if (use_pcd_file) {
		// Load input file into a PointCloud<T> with an appropriate
		// type
		pcl::PointCloud<pcl::PointXYZ>::Ptr
			cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(velodyne_points_pcd, cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

		points_to_polygon(cloud);
	}
}

static void velodyne_points_callback(const sensor_msgs::PointCloud2& msg)
{
	if (use_pcd_file)
		ROS_WARN("use_pcd_file: true");
	else {
		// Load ROS message into a PointCloud<T> with an appropriate
		// type
		pcl::PointCloud<pcl::PointXYZ>::Ptr
			cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(msg, *cloud);

		points_to_polygon(cloud);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "points2polygon");

	ros::NodeHandle n;
	n.param<bool>("points2polygon/use_pcd_file",
		      use_pcd_file, USE_PCD_FILE);
	n.param<std::string>("points2polygon/velodyne_points_pcd",
			     velodyne_points_pcd, VELODYNE_POINTS_PCD);
	n.param<std::string>("points2polygon/points_polygon_vtk",
			     points_polygon_vtk, POINTS_POLYGON_VTK);

	ros::Subscriber sub_config = n.subscribe("config/points2polygon",
						 SUBSCRIBE_QUEUE_SIZE,
						 config_callback);
	ros::Subscriber sub_points = n.subscribe("points_raw",
						 SUBSCRIBE_QUEUE_SIZE,
						 velodyne_points_callback);

	if (use_pcd_file) {
		// Load input file into a PointCloud<T> with an appropriate
		// type
		pcl::PointCloud<pcl::PointXYZ>::Ptr
			cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(velodyne_points_pcd, cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

		points_to_polygon(cloud);
	}

	ros::spin();

	return 0;
}
