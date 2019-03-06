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
#include "amathutils_lib/vector2d.hpp"
#include "amathutils_lib/numerical_comparision.hpp"

using namespace amathutils;
class Vector2dTestSuite : public ::testing::Test
{
  public:
    Vector2dTestSuite() {}
    ~Vector2dTestSuite() {}
};

TEST(TestSuite, CheckSimple)
{
    {
        Vector2d vec(1, 2);
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 2.0), true);
        vec.x() = 2;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 2.0) && approximatelyEqual(vec.getY(), 2.0), true);
        vec.y() = 3;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 2.0) && approximatelyEqual(vec.getY(), 3.0), true);
    }
    {
        Vector2d vec = Vector2d::Zero();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.0) && approximatelyEqual(vec.getY(), 0.0), true);
    }
    {
        Vector2d vec = Vector2d::Identity();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 1.0), true);
    }
    {
        Vector2d vec = Vector2d::UnitX();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 0.0), true);
    }
    {
        Vector2d vec = Vector2d::UnitY();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.0) && approximatelyEqual(vec.getY(), 1.0), true);
    }

    { // operator +
        Vector2d vec1(1, 0);
        Vector2d vec2(0, 1);
        Vector2d vec = vec1 + vec2;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 1.0), true);
    }
    { // operator -
        Vector2d vec1(1, 0);
        Vector2d vec2(0, 1);
        Vector2d vec = vec1 - vec2;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), -1.0), true);
    }
    { // operator *
        Vector2d vec(1, 1);
        vec = vec * 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 10.0) && approximatelyEqual(vec.getY(), 10.0), true);
    }
    { // operator /
        Vector2d vec(1, 1);
        vec = vec / 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.1) && approximatelyEqual(vec.getY(), 0.1), true);
    }
    { // operator +=
        Vector2d vec(1, 0);
        Vector2d vec1(0, 1);
        vec += vec1;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 1.0), true);
    }
    { // operator -=
        Vector2d vec(1, 0);
        Vector2d vec1(0, 1);
        vec -= vec1;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), -1.0), true);
    }
    { // operator *=
        Vector2d vec(1, 1);
        vec *= 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 10.0) && approximatelyEqual(vec.getY(), 10.0), true);
    }
    { // operator /=
        Vector2d vec(1, 1);
        vec /= 10.0;
        ASSERT_EQ(approximatelyEqual(vec.getX(), 0.1) && approximatelyEqual(vec.getY(), 0.1), true);
    }
    { // operator ==
        Vector2d vec1(1, 0);
        Vector2d vec2(0, 1);
        ASSERT_EQ(vec1 == vec2, false);

        vec1 = Vector2d(1, 0);
        vec2 = Vector2d(1, 0);
        ASSERT_EQ(vec1 == vec2, true);
    }
    { // operator !=
        Vector2d vec1(1, 0);
        Vector2d vec2(0, 1);
        ASSERT_EQ(vec1 != vec2, true);

        vec1 = Vector2d(1, 0);
        vec2 = Vector2d(1, 0);
        ASSERT_EQ(vec1 != vec2, false);
    }
}

TEST(TestSuite, CheckDistance)
{
    {
        Vector2d vec1(1, 0);
        Vector2d vec2(0, 1);
        ASSERT_EQ(approximatelyEqual(vec1.getDistance(vec2), std::sqrt(2)), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::getDistance(vec1, vec2), std::sqrt(2)), true);
    }
    {
        Vector2d vec1(1, 1);
        Vector2d vec2(-1, -1);
        ASSERT_EQ(approximatelyEqual(vec1.getDistance(vec2), std::sqrt(2) * 2.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::getDistance(vec1, vec2), std::sqrt(2) * 2.0), true);
    }
}

TEST(TestSuite, CheckDot)
{
    {
        Vector2d vec1(1, 0);
        Vector2d vec2(0, 1);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec1, vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec2, vec1), 0.0), true);
    }
    {
        Vector2d vec1(0, 0);
        Vector2d vec2(0, 0);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec1, vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec2, vec1), 0.0), true);
    }
    {
        Vector2d vec1(1, 0);
        Vector2d vec2(10, 0);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), 10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec1, vec2), 10.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), 10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec2, vec1), 10.0), true);
    }
    {
        Vector2d vec1(1, 0);
        Vector2d vec2(-10, 0);
        ASSERT_EQ(approximatelyEqual(vec1.dot(vec2), -10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec1, vec2), -10.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.dot(vec1), -10.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::dot(vec2, vec1), -10.0), true);
    }
}

TEST(TestSuite, CheckCross)
{
    {
        Vector2d vec1(1, 0);
        Vector2d vec2(0, 1);
        ASSERT_EQ(approximatelyEqual(vec1.cross(vec2), 1.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec1, vec2), 1.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.cross(vec1), -1.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec2, vec1), -1.0), true);
    }
    {
        Vector2d vec1(0, 0);
        Vector2d vec2(0, 0);
        ASSERT_EQ(approximatelyEqual(vec1.cross(vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec1, vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.cross(vec1), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec2, vec1), 0.0), true);
    }
    {
        Vector2d vec1(1, 0);
        Vector2d vec2(10, 0);
        ASSERT_EQ(approximatelyEqual(vec1.cross(vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec1, vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.cross(vec1), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec2, vec1), 0.0), true);
    }
    {
        Vector2d vec1(1, 0);
        Vector2d vec2(-10, 0);
        ASSERT_EQ(approximatelyEqual(vec1.cross(vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec1, vec2), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec2.cross(vec1), 0.0), true);
        ASSERT_EQ(approximatelyEqual(Vector2d::cross(vec2, vec1), 0.0), true);
    }
}

TEST(TestSuite, CheckNormalization)
{
    {
        Vector2d vec(10, 0);
        vec.normalize();
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 0.0), true);
    }
    {
        Vector2d vec = Vector2d::normalized(Vector2d(10, 0));
        ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 0.0), true);
    }
    {
        Vector2d vec(-10, 0);
        vec.normalize();
        ASSERT_EQ(approximatelyEqual(vec.getX(), -1.0) && approximatelyEqual(vec.getY(), 0.0), true);
    }
    {
        Vector2d vec = Vector2d::normalized(Vector2d(-10, 0));
        ASSERT_EQ(approximatelyEqual(vec.getX(), -1.0) && approximatelyEqual(vec.getY(), 0.0), true);
    }
}

TEST(TestSuite, CheckAngle)
{
    {
        {
            Vector2d vec = Vector2d::UnitFromAngle(0.0);
            ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 0.0), true);
        }
        {
            Vector2d vec = Vector2d::UnitFromAngle(M_PI_2);
            ASSERT_EQ(approximatelyEqual(vec.getX(), 0.0) && approximatelyEqual(vec.getY(), 1.0), true);
        }
        {
            Vector2d vec = Vector2d::UnitFromAngle(M_PI);
            ASSERT_EQ(approximatelyEqual(vec.getX(), -1.0) && approximatelyEqual(vec.getY(), 0.0), true);
        }
        {
            Vector2d vec = Vector2d::UnitFromAngle(M_PI + M_PI_2);
            ASSERT_EQ(approximatelyEqual(vec.getX(), 0.0) && approximatelyEqual(vec.getY(), -1.0), true);
        }
        {
            Vector2d vec = Vector2d::UnitFromAngle(2 * M_PI);
            ASSERT_EQ(approximatelyEqual(vec.getX(), 1.0) && approximatelyEqual(vec.getY(), 0.0), true);
        }
        {
            Vector2d vec = Vector2d::UnitFromAngle(0.5);
            ASSERT_EQ(approximatelyEqual(vec.getAngle(), 0.5), true);
        }
    }
}

TEST(TestSuite, CheckNorm)
{
    {
        Vector2d vec(3, 4);
        ASSERT_EQ(approximatelyEqual(vec.getL1norm(), 7.0), true);
    }
    {
        Vector2d vec(-3, 4);
        ASSERT_EQ(approximatelyEqual(vec.getL1norm(), 1.0), true);
    }
    {
        Vector2d vec(3, -4);
        ASSERT_EQ(approximatelyEqual(vec.getL1norm(), -1.0), true);
    }
    {
        Vector2d vec(3, 4);
        ASSERT_EQ(approximatelyEqual(vec.getL2norm(), 5.0), true);
    }
    {
        Vector2d vec(-3, 4);
        ASSERT_EQ(approximatelyEqual(vec.getL2norm(), 5.0), true);
    }
    {
        Vector2d vec(3, -4);
        ASSERT_EQ(approximatelyEqual(vec.getL2norm(), 5.0), true);
    }
}

TEST(TestSuite, CheckRotate)
{
    {
        Vector2d vec(1, 0);
        ASSERT_EQ(approximatelyEqual(vec.rotate(M_PI_2).getX(), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec.rotate(M_PI_2).getY(), 1.0), true);
    }
    {
        Vector2d vec(0, 1);
        ASSERT_EQ(approximatelyEqual(vec.rotate(M_PI_2).getX(), -1.0), true);
        ASSERT_EQ(approximatelyEqual(vec.rotate(M_PI_2).getY(), 0.0), true);
    }
    {
        Vector2d vec(1, 0);
        ASSERT_EQ(approximatelyEqual(vec.rotate(-M_PI_2).getX(), 0.0), true);
        ASSERT_EQ(approximatelyEqual(vec.rotate(-M_PI_2).getY(), -1.0), true);
    }
    {
        Vector2d vec(0, 1);
        ASSERT_EQ(approximatelyEqual(vec.rotate(-M_PI_2).getX(), 1.0), true);
        ASSERT_EQ(approximatelyEqual(vec.rotate(-M_PI_2).getY(), 0.0), true);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
