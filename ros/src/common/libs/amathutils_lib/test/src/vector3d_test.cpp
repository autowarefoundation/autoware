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
#include "amathutils_lib/vector3d.hpp"
#include "amathutils_lib/numerical_comparision.hpp"

using namespace amathutils;
class Vector3dTestSuite : public ::testing::Test
{
  public:
    Vector3dTestSuite() {}
    ~Vector3dTestSuite() {}
};

TEST(TestSuite, CheckSimple)
{
    {
        Vector3d vec(1, 2, 3);
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), 2.0) &&
                      approximatelyEqual(vec.getZ(), 3.0),
                  true);
        vec.x() = 2;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 2.0) &&
                      approximatelyEqual(vec.getY(), 2.0) &&
                      approximatelyEqual(vec.getZ(), 3.0),
                  true);
        vec.y() = 3;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 2.0) &&
                      approximatelyEqual(vec.getY(), 3.0) &&
                      approximatelyEqual(vec.getZ(), 3.0),
                  true);
        vec.z() = 4;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 2.0) &&
                      approximatelyEqual(vec.getY(), 3.0) &&
                      approximatelyEqual(vec.getZ(), 4.0),
                  true);
    }
    {
        Vector3d vec = Vector3d::Zero();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.0) &&
                      approximatelyEqual(vec.getY(), 0.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    {
        Vector3d vec = Vector3d::Identity();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), 1.0) &&
                      approximatelyEqual(vec.getZ(), 1.0),
                  true);
    }
    {
        Vector3d vec = Vector3d::UnitX();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), 0.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    {
        Vector3d vec = Vector3d::UnitY();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.0) &&
                      approximatelyEqual(vec.getY(), 1.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    {
        Vector3d vec = Vector3d::UnitZ();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.0) &&
                      approximatelyEqual(vec.getY(), 0.0) &&
                      approximatelyEqual(vec.getZ(), 1.0),
                  true);
    }

    { // operator +
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(0, 1, 0);
        Vector3d vec = vec1 + vec2;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), 1.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    { // operator -
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(0, 1, 0);
        Vector3d vec = vec1 - vec2;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), -1.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    { // operator *
        Vector3d vec(1, 1, 1);
        vec = vec * 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 10.0) &&
                      approximatelyEqual(vec.getY(), 10.0) &&
                      approximatelyEqual(vec.getZ(), 10.0),
                  true);
    }
    { // operator /
        Vector3d vec(1, 1, 1);
        vec = vec / 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.1) &&
                      approximatelyEqual(vec.getY(), 0.1) &&
                      approximatelyEqual(vec.getZ(), 0.1),
                  true);
    }
    { // operator +=
        Vector3d vec(1, 0, 0);
        Vector3d vec1(0, 1, 0);
        vec += vec1;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), 1.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    { // operator -=
        Vector3d vec(1, 0, 0);
        Vector3d vec1(0, 1, 0);
        vec -= vec1;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), -1.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    { // operator *=
        Vector3d vec(1, 1, 1);
        vec *= 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 10.0) &&
                      approximatelyEqual(vec.getY(), 10.0) &&
                      approximatelyEqual(vec.getZ(), 10.0),
                  true);
    }
    { // operator /=
        Vector3d vec(1, 1, 1);
        vec /= 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.1) &&
                      approximatelyEqual(vec.getY(), 0.1) &&
                      approximatelyEqual(vec.getZ(), 0.1),
                  true);
    }
    { // operator ==
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(0, 1, 0);
        ASSERT_EQ(vec1 == vec2, false);

        vec1 = Vector3d(1, 0, 0);
        vec2 = Vector3d(1, 0, 0);
        ASSERT_EQ(vec1 == vec2, true);
    }
    { // operator !=
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(0, 1, 0);
        ASSERT_EQ(vec1 != vec2, true);

        vec1 = Vector3d(1, 0, 0);
        vec2 = Vector3d(1, 0, 0);
        ASSERT_EQ(vec1 != vec2, false);
    }
}

TEST(TestSuite, CheckDistance)
{
    {
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(0, 0, 1);
        ASSERT_EQ(approximatelyEqual(vec1.getDistance(vec2), std::sqrt(2)), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::getDistance(vec1, vec2), std::sqrt(2)), true);
    }
    {
        Vector3d vec1(1, 1, 1);
        Vector3d vec2(-1, -1, -1);
        ASSERT_EQ(approximatelyEqual(vec1.getDistance(vec2), std::sqrt(3.0) * 2.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::getDistance(vec1, vec2), std::sqrt(3.0) * 2.0), true);
    }
}

TEST(TestSuite, CheckDot)
{
    {
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(0, 0, 1);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec1, vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec2, vec1), 0.0), true);
    }
    {
        Vector3d vec1(0, 0, 0);
        Vector3d vec2(0, 0, 0);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec1, vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec2, vec1), 0.0), true);
    }
    {
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(10, 0, 0);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), 10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec1, vec2), 10.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), 10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec2, vec1), 10.0), true);
    }
    {
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(-10, 0, 0);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), -10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec1, vec2), -10.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), -10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector3d::dot(vec2, vec1), -10.0), true);
    }
}

TEST(TestSuite, CheckCross)
{
    {
        Vector3d vec1(1, 2, 0);
        Vector3d vec2(0, 1, -1);
        ASSERT_EQ(vec1.cross(vec2) == Vector3d(-2, 1, 1), true);
        ASSERT_EQ(Vector3d::cross(vec1, vec2) == Vector3d(-2, 1, 1), true);
        ASSERT_EQ(vec2.cross(vec1) == Vector3d(2, -1, -1), true);
        ASSERT_EQ(Vector3d::cross(vec2, vec1) == Vector3d(2, -1, -1), true);
    }
    {
        Vector3d vec1(0, 0, 0);
        Vector3d vec2(0, 0, 0);
        ASSERT_EQ(vec1.cross(vec2) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(Vector3d::cross(vec1, vec2) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(vec2.cross(vec1) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(Vector3d::cross(vec2, vec1) == Vector3d(0, 0, 0), true);
    }
    {
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(10, 0, 0);
        ASSERT_EQ(vec1.cross(vec2) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(Vector3d::cross(vec1, vec2) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(vec2.cross(vec1) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(Vector3d::cross(vec2, vec1) == Vector3d(0, 0, 0), true);
    }
    {
        Vector3d vec1(1, 0, 0);
        Vector3d vec2(-10, 0, 0);
        ASSERT_EQ(vec1.cross(vec2) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(Vector3d::cross(vec1, vec2) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(vec2.cross(vec1) == Vector3d(0, 0, 0), true);
        ASSERT_EQ(Vector3d::cross(vec2, vec1) == Vector3d(0, 0, 0), true);
    }
}

TEST(TestSuite, CheckNormalization)
{
    {
        Vector3d vec(10, 0, 0);
        vec.normalize();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), 0.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    {
        Vector3d vec = Vector3d::normalized(Vector3d(10, 0, 0));
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) &&
                      approximatelyEqual(vec.getY(), 0.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    {
        Vector3d vec(-10, 0, 0);
        vec.normalize();
        ASSERT_EQ(approximatelyEqual(vec.getX(), -1.0) &&
                      approximatelyEqual(vec.getY(), 0.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
    {
        Vector3d vec = Vector3d::normalized(Vector3d(-10, 0, 0));
        ASSERT_EQ(approximatelyEqual(vec.getX(), -1.0) &&
                      approximatelyEqual(vec.getY(), 0.0) &&
                      approximatelyEqual(vec.getZ(), 0.0),
                  true);
    }
}

TEST(TestSuite, CheckNorm)
{
    {
        Vector3d vec(3, 4, 5);
        ASSERT_EQ(approximatelyEqual(vec.getL1norm(), 12.0), true);
    }
    {
        Vector3d vec(-3, 4, -5);
        ASSERT_EQ(approximatelyEqual(vec.getL1norm(), -4.0), true);
    }
    {
        Vector3d vec(3, -4, 5);
        ASSERT_EQ(approximatelyEqual(vec.getL1norm(), 4.0), true);
    }
    {
        Vector3d vec(3, 4, 5);
        ASSERT_EQ(approximatelyEqual(vec.getL2norm(), std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + 5.0 * 5.0)), true);
    }
    {
        Vector3d vec(-3, 4, -5);
        ASSERT_EQ(approximatelyEqual(vec.getL2norm(), std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + 5.0 * 5.0)), true);
    }
    {
        Vector3d vec(3, -4, 5);
        ASSERT_EQ(approximatelyEqual(vec.getL2norm(), std::sqrt(3.0 * 3.0 + 4.0 * 4.0 + 5.0 * 5.0)), true);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
