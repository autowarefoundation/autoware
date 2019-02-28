/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

/**
* @file test_point_pillars.cpp
* @brief unit test file
* @author Kosuke Murakami
* @date 2019/02/26
*/

#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "point_pillars_ros.h"
#include "preprocess_points.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  {
  }
  ~TestSuite()
  {
  }
};

class TestClass
{
public:
  TestClass(const int MAX_NUM_PILLARS, const int MAX_NUM_POINTS_PER_PILLAR, const int GRID_X_SIZE,
            const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
            const float PILLAR_Z_SIZE, const float MIN_X_RANGE, const float MIN_Y_RANGE, const float MIN_Z_RANGE,
            const int NUM_INDS_FOR_SCAN, const int NUM_BOX_CORNERS);
  const int MAX_NUM_PILLARS_;
  const int MAX_NUM_POINTS_PER_PILLAR_;
  const int GRID_X_SIZE_;
  const int GRID_Y_SIZE_;
  const int GRID_Z_SIZE_;
  const float PILLAR_X_SIZE_;
  const float PILLAR_Y_SIZE_;
  const float PILLAR_Z_SIZE_;
  const float MIN_X_RANGE_;
  const float MIN_Y_RANGE_;
  const float MIN_Z_RANGE_;
  const int NUM_INDS_FOR_SCAN_;
  const int NUM_BOX_CORNERS_;

  void loadPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr, const std::string& in_file);
  void pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array);
  void preprocess(const float* in_points_array, int in_num_points, int* x_coors, int* y_coors,
                  float* num_points_per_pillar, float* pillar_x, float* pillar_y, float* pillar_z, float* pillar_i,
                  float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped, float* pillar_feature_mask,
                  float* sparse_pillar_map, int* host_pillar_count);

private:
  std::unique_ptr<PreprocessPoints> preprocess_points_ptr_;
};

TestClass::TestClass(const int MAX_NUM_PILLARS, const int MAX_NUM_POINTS_PER_PILLAR, const int GRID_X_SIZE,
                     const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
                     const float PILLAR_Z_SIZE, const float MIN_X_RANGE, const float MIN_Y_RANGE,
                     const float MIN_Z_RANGE, const int NUM_INDS_FOR_SCAN, const int NUM_BOX_CORNERS)
  : MAX_NUM_PILLARS_(MAX_NUM_PILLARS)
  , MAX_NUM_POINTS_PER_PILLAR_(MAX_NUM_POINTS_PER_PILLAR)
  , GRID_X_SIZE_(GRID_X_SIZE)
  , GRID_Y_SIZE_(GRID_Y_SIZE)
  , GRID_Z_SIZE_(GRID_Z_SIZE)
  , PILLAR_X_SIZE_(PILLAR_X_SIZE)
  , PILLAR_Y_SIZE_(PILLAR_Y_SIZE)
  , PILLAR_Z_SIZE_(PILLAR_Z_SIZE)
  , MIN_X_RANGE_(MIN_X_RANGE)
  , MIN_Y_RANGE_(MIN_Y_RANGE)
  , MIN_Z_RANGE_(MIN_Z_RANGE)
  , NUM_INDS_FOR_SCAN_(NUM_INDS_FOR_SCAN)
  , NUM_BOX_CORNERS_(NUM_BOX_CORNERS)
{
  preprocess_points_ptr_.reset(new PreprocessPoints(
      MAX_NUM_PILLARS_, MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_, GRID_Y_SIZE_, GRID_Z_SIZE_, PILLAR_X_SIZE_,
      PILLAR_Y_SIZE_, PILLAR_Z_SIZE_, MIN_X_RANGE_, MIN_Y_RANGE_, MIN_Z_RANGE_, NUM_INDS_FOR_SCAN_, NUM_BOX_CORNERS_));
};

void TestClass::preprocess(const float* in_points_array, int in_num_points, int* x_coors, int* y_coors,
                           float* num_points_per_pillar, float* pillar_x, float* pillar_y, float* pillar_z,
                           float* pillar_i, float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped,
                           float* pillar_feature_mask, float* sparse_pillar_map, int* host_pillar_count)
{
  preprocess_points_ptr_->preprocess(in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar, pillar_x,
                                     pillar_y, pillar_z, pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped,
                                     pillar_feature_mask, sparse_pillar_map, host_pillar_count);
}

void TestClass::pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array)
{
  for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++)
  {
    pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
    out_points_array[i * 4 + 0] = point.x;
    out_points_array[i * 4 + 1] = point.y;
    out_points_array[i * 4 + 2] = point.z;
    out_points_array[i * 4 + 3] = point.intensity;
  }
}

void TestClass::loadPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr, const std::string& in_file)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *in_pcl_pc_ptr) == -1)
  {
    ROS_ERROR("Couldn't read test pcd file \n");
  }
}

TEST(TestSuite, CheckPreprocessPointsCPU)
{
  TestClass test_obj(12000, 100, 432, 496, 1, 0.16, 0.16, 4.0, 0, -39.68, -3.0, 512, 4);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  std::string package_path = ros::package::getPath("lidar_point_pillars");
  std::string in_file = package_path + "/test/data/1527836009720148.pcd";
  test_obj.loadPoints(pcl_pc_ptr, in_file);

  float* points_array = new float[pcl_pc_ptr->size() * 4];
  test_obj.pclToArray(pcl_pc_ptr, points_array);

  int x_coors[test_obj.MAX_NUM_PILLARS_] = { 0 };
  int y_coors[test_obj.MAX_NUM_PILLARS_] = { 0 };
  float num_points_per_pillar[test_obj.MAX_NUM_PILLARS_] = { 0 };
  float* pillar_x = new float[test_obj.MAX_NUM_PILLARS_ * test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_y = new float[test_obj.MAX_NUM_PILLARS_ * test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_z = new float[test_obj.MAX_NUM_PILLARS_ * test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_i = new float[test_obj.MAX_NUM_PILLARS_ * test_obj.MAX_NUM_POINTS_PER_PILLAR_];

  float* x_coors_for_sub_shaped = new float[test_obj.MAX_NUM_PILLARS_ * test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* y_coors_for_sub_shaped = new float[test_obj.MAX_NUM_PILLARS_ * test_obj.MAX_NUM_POINTS_PER_PILLAR_];
  float* pillar_feature_mask = new float[test_obj.MAX_NUM_PILLARS_ * test_obj.MAX_NUM_POINTS_PER_PILLAR_];

  float* sparse_pillar_map = new float[512 * 512];

  int host_pillar_count[1] = { 0 };
  test_obj.preprocess(points_array, pcl_pc_ptr->size(), x_coors, y_coors, num_points_per_pillar, pillar_x, pillar_y,
                      pillar_z, pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped, pillar_feature_mask,
                      sparse_pillar_map, host_pillar_count);
  EXPECT_EQ(15, num_points_per_pillar[0]);
  EXPECT_FLOAT_EQ(3.7517259, pillar_x[14]);
  EXPECT_EQ(71, x_coors[100]);
  EXPECT_EQ(177, y_coors[100]);
  EXPECT_EQ(1, sparse_pillar_map[177 * 512 + 71]);
  EXPECT_EQ(6270, host_pillar_count[0]);
  delete[] points_array;
  delete[] pillar_x;
  delete[] pillar_y;
  delete[] pillar_z;
  delete[] pillar_i;
  delete[] x_coors_for_sub_shaped;
  delete[] y_coors_for_sub_shaped;
  delete[] pillar_feature_mask;
  delete[] sparse_pillar_map;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
