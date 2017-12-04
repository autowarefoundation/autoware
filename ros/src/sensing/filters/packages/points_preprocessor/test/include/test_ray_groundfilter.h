
#include <cmath>

#include <ros/ros.h>

#include "ray_ground_filter.h"

// test fixtures are necessary to use friend classes
TEST(RayGroundFilter, clipCloud)
{
  char* argv = "test_points_preprocessor";
  int argc = 1;
  ros::init(argc, &argv, "test_raygroundfilter_clipcloud");
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  RayGroundFilter rgfilter;

  // add a few test points
  pcl::PointXYZI pt; 
  pt.x = 0.0F; pt.y = 0.0F; pt.z = 1.0F;
  in_cloud_ptr->push_back(pt);
  pt.x = 0.0F; pt.y = 1.0F; pt.z = 1.3F;
  in_cloud_ptr->push_back(pt);
  pt.x = 0.0F; pt.y = 2.0F; pt.z = 1.5F;
  in_cloud_ptr->push_back(pt);
  pt.x = 0.0F; pt.y = 3.0F; pt.z = 1.7F;
  in_cloud_ptr->push_back(pt);
  pt.x = 0.0F; pt.y = 4.0F; pt.z = 1.9F;
  in_cloud_ptr->push_back(pt);
  pt.x = 0.0F; pt.y = 5.0F; pt.z = 1.7F;
  in_cloud_ptr->push_back(pt);
  pt.x = 0.0F; pt.y = 6.0F; pt.z = 1.5F;
  in_cloud_ptr->push_back(pt);

  // clip cloud
  const float CLIP_HEIGHT = 1.5F;
  rgfilter.ClipCloud(in_cloud_ptr, CLIP_HEIGHT, out_cloud_ptr);

  // make sure everything worked correctly
  ASSERT_EQ(out_cloud_ptr->points.size(), 4);
  const float TOL = 1.0E-6F;
  ASSERT_LT(fabsf(out_cloud_ptr->points[0].x), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[0].y), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[0].z - 1.0F), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[1].x), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[1].y - 1.0F), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[1].z - 1.3F), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[2].x), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[2].y - 2.0F), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[2].z - 1.5F), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[3].x), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[3].y - 6.0F), TOL);
  ASSERT_LT(fabsf(out_cloud_ptr->points[3].z - 1.5F), TOL);
}
