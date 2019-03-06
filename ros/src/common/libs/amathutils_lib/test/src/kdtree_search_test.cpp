/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 * v1.0 Yukihiro Saito
 */

#include <gtest/gtest.h>
#include <vector>
#include "amathutils_lib/kdtree_search.hpp"
#include "amathutils_lib/numerical_comparision.hpp"

using namespace amathutils;
class KdtreeSearchTestSuite : public ::testing::Test
{
  public:
    KdtreeSearchTestSuite() {}
    ~KdtreeSearchTestSuite() {}
};

TEST(TestSuite, Check3dSearch)
{
    {
        std::vector<Vector3d> v_vec;
        {
            v_vec.push_back(Vector3d(0, 0, 0));
            v_vec.push_back(Vector3d(1, 1, 1));
            v_vec.push_back(Vector3d(2, 2, 2));
            v_vec.push_back(Vector3d(3, 3, 3));
            v_vec.push_back(Vector3d(4, 4, 4));
            v_vec.push_back(Vector3d(5, 5, 5));
        }
        KdTreeSearcher kdtree(v_vec);
        {
            int index;
            kdtree.searchNearestNeighbor(Vector3d(2.1, 2.1, 2.1), index);
            ASSERT_EQ(index == 2, true);
        }
        {
            int index;
            kdtree.searchNearestNeighbor(Vector3d(-100, -100, -100), index);
            ASSERT_EQ(index == 0, true);
        }
        {
            std::vector<int> v_index;
            kdtree.searchNearestK(Vector3d(1.1, 1.1, 1.1), 3, v_index);
            ASSERT_EQ(v_index.size() == 3 &&
                          v_index.at(0) == 1 &&
                          v_index.at(1) == 2 &&
                          v_index.at(2) == 0,
                      true);
        }
        {
            std::vector<int> v_index;
            kdtree.searchRadius(Vector3d(1.1, 1.1, 1.1), std::sqrt(3), v_index);
            bool is_vec1(false), is_vec2(false);
            for (const auto &index : v_index)
            {
                if (!is_vec1 && index == 1)
                {
                    is_vec1 = true;
                }
                if (!is_vec2 && index == 2)
                {
                    is_vec2 = true;
                }
            }
            ASSERT_EQ((is_vec1 &&
                       is_vec2),
                      true);
        }
    }
}

TEST(TestSuite, Check2dSearch)
{
    {
        std::vector<Vector2d> v_vec;
        {
            v_vec.push_back(Vector2d(0, 0));
            v_vec.push_back(Vector2d(1, 1));
            v_vec.push_back(Vector2d(2, 2));
            v_vec.push_back(Vector2d(3, 3));
            v_vec.push_back(Vector2d(4, 4));
            v_vec.push_back(Vector2d(5, 5));
        }
        KdTreeSearcher kdtree(v_vec);
        {
            int index;
            kdtree.searchNearestNeighbor(Vector2d(2.1, 2.1), index);
            ASSERT_EQ(index == 2, true);
        }
        {
            int index;
            kdtree.searchNearestNeighbor(Vector2d(-100, -100), index);
            ASSERT_EQ(index == 0, true);
        }
        {
            std::vector<int> v_index;
            kdtree.searchNearestK(Vector2d(1.1, 1.1), 3, v_index);
            ASSERT_EQ(v_index.size() == 3 &&
                          v_index.at(0) == 1 &&
                          v_index.at(1) == 2 &&
                          v_index.at(2) == 0,
                      true);
        }
        {
            std::vector<int> v_index;
            kdtree.searchRadius(Vector2d(1.1, 1.1), std::sqrt(2), v_index);
            bool is_vec1(false), is_vec2(false);
            for (const auto &index : v_index)
            {
                if (!is_vec1 && index == 1)
                {
                    is_vec1 = true;
                }
                if (!is_vec2 && index == 2)
                {
                    is_vec2 = true;
                }
            }
            ASSERT_EQ((is_vec1 &&
                       is_vec2),
                      true);
        }
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
