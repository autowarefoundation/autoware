// Copyright 2017-2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TEST_BOUNDING_BOX_HPP_
#define TEST_BOUNDING_BOX_HPP_

#include "geometry/bounding_box/lfit.hpp"
#include "geometry/bounding_box/rotating_calipers.hpp"

#include <geometry_msgs/msg/point32.hpp>

#include <limits>
#include <list>
#include <vector>

using autoware::common::geometry::point_adapter::x_;
using autoware::common::geometry::point_adapter::xr_;
using autoware::common::geometry::point_adapter::y_;
using autoware::common::geometry::point_adapter::yr_;
using autoware_auto_perception_msgs::msg::BoundingBox;

template <typename PointT>
class BoxTest : public ::testing::Test
{
protected:
  std::list<PointT> points;
  BoundingBox box;

  void minimum_area_bounding_box()
  {
    // apex_test_tools::memory_test::start();
    box = autoware::common::geometry::bounding_box::minimum_area_bounding_box(points);
    // apex_test_tools::memory_test::stop();
  }

  void minimum_perimeter_bounding_box()
  {
    // apex_test_tools::memory_test::start();
    box = autoware::common::geometry::bounding_box::minimum_perimeter_bounding_box(points);
    // apex_test_tools::memory_test::stop();
  }
  template <typename IT>
  void eigenbox(const IT begin, const IT end)
  {
    // apex_test_tools::memory_test::start();
    box = autoware::common::geometry::bounding_box::eigenbox_2d(begin, end);
    // apex_test_tools::memory_test::stop();
  }
  template <typename IT>
  void lfit_bounding_box_2d(const IT begin, const IT end)
  {
    // apex_test_tools::memory_test::start();
    box = autoware::common::geometry::bounding_box::lfit_bounding_box_2d(begin, end);
    // apex_test_tools::memory_test::stop();
  }

  PointT make(const float x, const float y)
  {
    PointT ret;
    xr_(ret) = x;
    yr_(ret) = y;
    return ret;
  }

  void check(const float cx, const float cy, const float sx, const float sy, const float val) const
  {
    constexpr float32_t TOL = 1.0E-4F;
    ASSERT_LT(fabsf(box.size.x - sx), TOL);
    ASSERT_LT(fabsf(box.size.y - sy), TOL);
    ASSERT_LT(fabsf(box.value - val), TOL) << box.value;

    ASSERT_LT(fabsf(box.centroid.x - cx), TOL);
    ASSERT_LT(fabsf(box.centroid.y - cy), TOL);
  }

  void test_corners(const std::vector<PointT> & expect, const float TOL = 1.0E-6F) const
  {
    for (uint32_t idx = 0U; idx < 4U; ++idx) {
      bool found = false;
      for (auto & p : expect) {
        if (fabsf(x_(p) - box.corners[idx].x) < TOL && fabsf(y_(p) - box.corners[idx].y) < TOL) {
          found = true;
          break;
        }
      }
      ASSERT_TRUE(found) << idx << ": " << box.corners[idx].x << ", " << box.corners[idx].y;
    }
  }

  /// \brief th_deg - phi_deg, normalized to +/- 180 deg
  float32_t angle_distance_deg(const float th_deg, const float phi_deg) const
  {
    return fmodf((th_deg - phi_deg) + 540.0F, 360.0F) - 180.0F;
  }

  /// \brief converts a radian value to a degree value
  float32_t rad2deg(const float rad_val) const { return rad_val * 57.2958F; }

  void test_orientation(const float ref_angle_deg, const float TOL = 1.0E-4F) const
  {
    bool found = false;
    const float angle_deg = rad2deg(2.0F * asinf(box.orientation.z));
    found |= fabsf(angle_distance_deg(angle_deg, ref_angle_deg)) < TOL;
    found |= fabsf(angle_distance_deg(angle_deg, ref_angle_deg + 90.0F)) < TOL;
    found |= fabsf(angle_distance_deg(angle_deg, ref_angle_deg + 180.0F)) < TOL;
    found |= fabsf(angle_distance_deg(angle_deg, ref_angle_deg + 270.0F)) < TOL;
    ASSERT_TRUE(found) << angle_deg;
  }
};  // BoxTest

// Instantiate tests for given types, add more types here as they are used
using PointTypesBoundingBox =
  ::testing::Types<geometry_msgs::msg::Point32, autoware::common::types::PointXYZIF>;
// cppcheck-suppress syntaxError
TYPED_TEST_SUITE(BoxTest, PointTypesBoundingBox, );
/// NOTE: This is the older version due to 1.8.0 of GTest. v1.8.1 uses TYPED_TEST_SUITE

// TODO(c.ho) consider typed and paremterized tests:
// https://stackoverflow.com/questions/3258230/passing-a-typename-and-string-to-parameterized-test-using-google-test

///////////////////////////////////////////
TYPED_TEST(BoxTest, PointSegmentDistance)
{
  using autoware::common::geometry::closest_segment_point_2d;
  using autoware::common::geometry::point_line_segment_distance_2d;
  // normal case
  TypeParam p = this->make(-1.0F, 0.0F);
  TypeParam q = this->make(-1.0F, 2.0F);
  TypeParam r = this->make(1.0F, 1.0F);
  TypeParam t = closest_segment_point_2d(p, q, r);
  ASSERT_FLOAT_EQ(x_(t), -1.0F);
  ASSERT_FLOAT_EQ(y_(t), 1.0F);
  ASSERT_FLOAT_EQ(point_line_segment_distance_2d(p, q, r), 2.0F);
  // boundary case
  p = this->make(1.0F, 0.0F);
  q = this->make(-2.0F, 0.0F);
  r = this->make(-5.0F, 0.0F);
  t = closest_segment_point_2d(p, q, r);
  ASSERT_FLOAT_EQ(x_(t), -2.0F);
  ASSERT_NEAR(y_(t), 0.0F, std::numeric_limits<float32_t>::epsilon());
  ASSERT_FLOAT_EQ(point_line_segment_distance_2d(p, q, r), 3.0F);
  // singular case
  p = this->make(1.0F, 5.0F);
  q = this->make(1.0F, 5.0F);
  r = this->make(1.0F, 1.0F);
  t = closest_segment_point_2d(p, q, r);
  ASSERT_FLOAT_EQ(x_(t), 1.0F);
  ASSERT_FLOAT_EQ(y_(t), 5.0F);
  ASSERT_FLOAT_EQ(point_line_segment_distance_2d(p, q, r), 4.0F);
}

TYPED_TEST(BoxTest, ClosestPointOnLine)
{
  using autoware::common::geometry::closest_line_point_2d;
  // normal case
  TypeParam p = this->make(-1.0F, 0.0F);
  TypeParam q = this->make(-1.0F, 2.0F);
  TypeParam r = this->make(1.0F, 1.0F);
  TypeParam t = closest_line_point_2d(p, q, r);
  ASSERT_FLOAT_EQ(x_(t), -1.0F);
  ASSERT_FLOAT_EQ(y_(t), 1.0F);
  // out-of-boundary case
  p = this->make(1.0F, 0.0F);
  q = this->make(-2.0F, 0.0F);
  r = this->make(-5.0F, 0.0F);
  t = closest_line_point_2d(p, q, r);
  ASSERT_FLOAT_EQ(x_(t), -5.0F);
  ASSERT_NEAR(y_(t), 0.0F, std::numeric_limits<float32_t>::epsilon());
  // singular case
  p = this->make(1.0F, 5.0F);
  q = this->make(1.0F, 5.0F);
  r = this->make(1.0F, 1.0F);
  EXPECT_THROW(t = closest_line_point_2d(p, q, r), std::runtime_error);
}

TYPED_TEST(BoxTest, Basic)
{
  const std::vector<TypeParam> data{
    {this->make(0, 0), this->make(1, 0), this->make(1, 1), this->make(0, 1)}};
  this->points.insert(this->points.begin(), data.begin(), data.end());

  this->minimum_area_bounding_box();

  this->check(0.5F, 0.5F, 1.0F, 1.0F, 1.0F);
  this->test_corners(data);
  this->test_orientation(0.0F);

  this->minimum_perimeter_bounding_box();

  this->check(0.5F, 0.5F, 1.0F, 1.0F, 2.0F);
  this->test_corners(data);
  this->test_orientation(0.0F);
}

//
TYPED_TEST(BoxTest, OrientedTriangle)
{
  this->points.insert(this->points.begin(), {this->make(5, 5), this->make(7, 7), this->make(5, 7)});

  this->minimum_area_bounding_box();

  this->check(5.5F, 6.5F, sqrtf(2.0F), 2.0F * sqrtf(2.0F), 4.0F);
  this->test_corners({this->make(5, 5), this->make(7, 7), this->make(4, 6), this->make(6, 8)});
  this->test_orientation(45.0F);

  this->minimum_perimeter_bounding_box();

  this->check(6.0F, 6.0F, 2.0F, 2.0F, 4.0F);
  this->test_corners({this->make(5, 5), this->make(7, 7), this->make(5, 7), this->make(7, 5)});
  this->test_orientation(0.0F);
}
//
TYPED_TEST(BoxTest, Hull)
{
  const uint32_t FUZZ_SIZE = 1024U;
  const float dx = 9.0F;
  const float dy = 15.0F;
  const float rx = 10.0F;
  const float ry = 5.0F;
  const float dth = 0.0F;

  ASSERT_EQ(FUZZ_SIZE % 4U, 0U);

  // fuzz part 1
  for (uint32_t idx = 0U; idx < FUZZ_SIZE; ++idx) {
    const float th = ((idx * autoware::common::types::TAU) / FUZZ_SIZE) + dth;
    this->points.push_back(this->make(rx * cosf(th) + dx, ry * sinf(th) + dy));
  }

  this->minimum_area_bounding_box();

  // allow 10cm = 2% of size for tolerance
  const float TOL_M = 0.1F;
  ASSERT_LT(fabsf(this->box.centroid.x - dx), TOL_M);
  ASSERT_LT(fabsf(this->box.centroid.y - dy), TOL_M);

  this->test_corners(
    {this->make(dx + rx, dy + ry), this->make(dx - rx, dy + ry), this->make(dx - rx, dy - ry),
     this->make(dx + rx, dy - ry)},
    TOL_M);

  this->test_orientation(this->rad2deg(dth), 1.0F);
  // allow 1 degree of tolerance

  ASSERT_LT(fabsf(this->box.size.y - 2.0F * rx), TOL_M);
  ASSERT_LT(fabsf(this->box.size.x - 2.0F * ry), TOL_M);
  ASSERT_FLOAT_EQ(this->box.value, this->box.size.x * this->box.size.y);
}

//
TYPED_TEST(BoxTest, Collinear)
{
  this->points.insert(
    this->points.begin(), {this->make(-2, -2), this->make(-3, -2), this->make(-4, -2),
                           this->make(-2, -4), this->make(-3, -4), this->make(-4, -4),
                           this->make(-2, -3), this->make(-2, -3), this->make(-2, -4)});

  this->minimum_area_bounding_box();

  this->check(-3.0F, -3.0F, 2.0F, 2.0F, 4.0F);
  this->test_corners(
    {this->make(-2, -2), this->make(-2, -4), this->make(-4, -4), this->make(-4, -2)});
  this->test_orientation(0.0F);
}

//
TYPED_TEST(BoxTest, Line1)
{
  this->points.insert(
    this->points.begin(), {this->make(-4, 3), this->make(-8, 6), this->make(-12, 9),
                           this->make(-16, 12), this->make(-20, 15), this->make(-24, 18),
                           this->make(-28, 21), this->make(-32, 24), this->make(-36, 27)});

  this->minimum_area_bounding_box();

  this->check(-20.0F, 15.0F, 1.0E-6F, 40.0F, 4.0E-5F);
  this->test_orientation(this->rad2deg(atan2f(3, -4)));
  this->test_corners(
    {this->make(-4, 3), this->make(-30, 27), this->make(-4, 3), this->make(-36, 27)});

  this->minimum_perimeter_bounding_box();

  this->check(-20.0F, 15.0F, 1.0E-6F, 40.0F, 40.00001F);
  this->test_orientation(this->rad2deg(atan2f(3, -4)));
  this->test_corners(
    {this->make(-4, 3), this->make(-30, 27), this->make(-4, 3), this->make(-36, 27)});
}

//
TYPED_TEST(BoxTest, Line2)
{
  this->points.insert(
    this->points.begin(),
    {this->make(4, 0), this->make(8, 0), this->make(12, 0), this->make(16, 0), this->make(20, 0),
     this->make(24, 0), this->make(28, 0), this->make(32, 0), this->make(36, 0)});

  this->minimum_area_bounding_box();

  this->check(20.0F, 0.0F, 1.0E-6F, 32.0F, 3.2E-5F);
  this->test_orientation(0.0F);
  this->test_corners({this->make(4, 0), this->make(36, 0), this->make(4, 0), this->make(36, 0)});
}

//
TYPED_TEST(BoxTest, Line3)
{
  this->points.insert(
    this->points.begin(),
    {this->make(4, 3), this->make(8, 6), this->make(12, 9), this->make(16, 12), this->make(20, 15),
     this->make(24, 18), this->make(28, 21), this->make(32, 24), this->make(36, 27)});

  this->minimum_area_bounding_box();

  this->check(20.0F, 15.0F, 1.0E-6F, 40.0F, 4.0E-5F);
  this->test_orientation(this->rad2deg(atan2f(3, 4)));
  this->test_corners({this->make(4, 3), this->make(36, 27), this->make(4, 3), this->make(36, 27)});
}

/*   _
    /_/ <-- first guess is suboptimal

*/
TYPED_TEST(BoxTest, SuboptInit)
{
  this->points.insert(
    this->points.begin(),
    {this->make(8, 15), this->make(17, 0), this->make(0, 0), this->make(25, 15)});

  this->minimum_area_bounding_box();

  this->check(12.5F, 7.5F, 15.0F, 25.0F, 375.0F);
  this->test_orientation(this->rad2deg(atan2f(15, 8)));
  // these are approximate.
  this->test_corners(
    {this->make(0, 0), this->make(25, 15), this->make(11.7647F, 22.0588F),
     this->make(13.2353F, -7.05882F)},
    0.1F);
}

//
TYPED_TEST(BoxTest, Centered)
{
  this->points.insert(
    this->points.begin(),
    {this->make(-1, 0), this->make(1, 0), this->make(0, -1), this->make(0, 1)});

  this->minimum_area_bounding_box();

  this->check(0.0F, 0.0F, sqrtf(2.0F), sqrtf(2.0F), 2.0F);
  this->test_orientation(45.0F);
  this->test_corners({this->make(-1, 0), this->make(1, 0), this->make(0, 1), this->make(0, -1)});
}

// convex_hull is imperfect in this case, check if this can save the day
TYPED_TEST(BoxTest, OverlappingPoints)
{
  this->points.insert(
    this->points.begin(), {this->make(0, 0), this->make(1, 0), this->make(1, 1), this->make(0, 1),
                           this->make(0, 0), this->make(1, 0), this->make(1, 1), this->make(0, 1)});

  this->minimum_area_bounding_box();

  this->check(0.5F, 0.5F, 1.0F, 1.0F, 1.0F);
  this->test_orientation(0.0F);
  this->test_corners({this->make(0, 0), this->make(1, 0), this->make(0, 1), this->make(1, 1)});
}

// check that minimum perimeter box is different from minimum area box
TYPED_TEST(BoxTest, Perimeter)
{
  this->points.insert(
    this->points.begin(), {this->make(0, 0), this->make(0, 1), this->make(0, 2), this->make(0, 3),
                           this->make(0, 4), this->make(1, -0.1),  // small fudge to force diagonal
                           this->make(2, 0), this->make(3, 0)});

  this->minimum_area_bounding_box();

  this->check(0.54F, 1.28F, 5.0F, 12.0F / 5.0F, 12.0F);
  this->test_orientation(-53.13F, 0.001F);
  this->test_corners(
    {this->make(3, 0), this->make(0, 4), this->make(-1.92, 2.56), this->make(1.08, -1.44)});

  // eigenbox should produce AABB TODO(c.ho)
  // compute minimum perimeter box
  this->minimum_perimeter_bounding_box();
  this->check(1.5F, 1.95F, 4.1F, 3.0F, 7.1F);
  // perimeter for first box would be 14.8
  this->test_orientation(0.0F, 0.001F);
  this->test_corners(
    {this->make(3, -0.1), this->make(0, 4), this->make(3, 4), this->make(0, -0.1)});
}

// bounding box is diagonal on an L
TYPED_TEST(BoxTest, Eigenbox1)
{
  std::vector<TypeParam> v{this->make(0, 0),  this->make(0, 1),
                           this->make(-1, 2),  // small fudge to force diagonal
                           this->make(0, 3),  this->make(0, 4),
                           this->make(1, 0),  this->make(2, -1),  // small fudge to force diagonal
                           this->make(3, 0),  this->make(4, 0)};
  this->points.insert(this->points.begin(), v.begin(), v.end());

  // rotating calipers should produce a diagonal box
  this->minimum_area_bounding_box();
  const float r = 4.0F;

  this->check(1.0F, 1.0F, r / sqrtf(2.0F), sqrtf(2.0F) * r, r * r);
  const std::vector<TypeParam> diag_corners{
    this->make(4, 0), this->make(0, 4), this->make(-2, 2), this->make(2, -2)};
  this->test_corners(diag_corners);
  this->test_orientation(45.0F, 0.001F);

  //// perimeter should also produce diagonal ////
  this->minimum_perimeter_bounding_box();
  this->check(
    1.0F, 1.0F, r / sqrtf(2.0F), sqrtf(2.0F) * r, r * (sqrtf(2.0F) + (1.0F / sqrtf(2.0F))));
  this->test_corners(diag_corners);
  this->test_orientation(45.0F, 0.001F);
  //// eigenbox should also produce aabb ////
  this->eigenbox(v.begin(), v.end());
  // TODO(c.ho) don't care about value..
  this->check(1.0F, 1.0F, r * sqrtf(2.0F), r / sqrtf(2.0F), 4.375F);
  this->test_corners(diag_corners);
  this->test_orientation(45.0F, 0.001F);
  //// Lfit should produce aabb ////
  this->lfit_bounding_box_2d(v.begin(), v.end());

  this->test_corners(
    {this->make(4, -1), this->make(-1, 4), this->make(4, 4), this->make(-1, -1)}, 0.25F);
  this->test_orientation(0.0F, 3.0F);
}

// same as above test, just rotated
TYPED_TEST(BoxTest, Eigenbox2)
{
  std::vector<TypeParam> v{this->make(0, 0),  this->make(1, 1),
                           this->make(1, 2),  // small fudge to force diagonal
                           this->make(3, 3),  this->make(4, 4),
                           this->make(1, -1), this->make(1, -2),  // small fudge to force diagonal
                           this->make(3, -3), this->make(4, -4)};
  this->points.insert(this->points.begin(), v.begin(), v.end());

  const std::vector<TypeParam> diag_corners{
    this->make(4, 4), this->make(0, 4), this->make(0, -4), this->make(4, -4)};
  // rotating calipers should produce a aabb
  this->minimum_area_bounding_box();
  this->check(2.0F, 0.0F, 8, 4, 32);
  this->test_corners(diag_corners);
  this->test_orientation(0.0F, 0.001F);

  //// perimeter should also produce aabb ////
  this->minimum_perimeter_bounding_box();
  this->check(2.0F, 0.0F, 8, 4, 12);
  this->test_corners(diag_corners);

  //// eigenbox should also produce obb ////
  this->eigenbox(v.begin(), v.end());
  this->test_orientation(0.0F, 0.001F);
  this->test_corners(diag_corners);
  //// Lfit should produce obb ////
  this->lfit_bounding_box_2d(v.begin(), v.end());
  this->test_corners(
    {this->make(3.5, 4.5), this->make(-1, 0), this->make(3.5, -4.5), this->make(8, 0)}, 0.2F);
  this->test_orientation(45.0F, 2.0F);
}
// line case for eigenbox
TYPED_TEST(BoxTest, Eigenbox3)
{
  // horizontal line with some noise
  std::vector<TypeParam> v{this->make(0, 0.00),   this->make(1, -0.01),  this->make(3, 0.02),
                           this->make(3, 0.03),   this->make(4, -0.04),  this->make(-1, 0.01),
                           this->make(-2, -0.02), this->make(-3, -0.03), this->make(-4, 0.04)};
  this->lfit_bounding_box_2d(v.begin(), v.end());
  this->test_corners(
    {this->make(-4, -0.04), this->make(-4, 0.04), this->make(4, -0.04), this->make(4, 0.04)}, 0.2F);
  this->test_orientation(0.0F, 1.0F);
}

// bad case: causes intersection2d to fail
// See https://gitlab.apex.ai/ApexAI/grand_central/issues/2862#note_156875 for more failure cases
TYPED_TEST(BoxTest, IntersectFail)
{
  std::vector<TypeParam> vals{
    this->make(-13.9, 0.100006), this->make(-14.1, 0.100006), this->make(-13.9, 0.300003),
    this->make(-14.1, 0.300003), this->make(-13.9, 0.5),      this->make(-14.1, 0.5),
    this->make(-13.9, 0.699997), this->make(-14.1, 0.699997), this->make(-14.3, 0.699997)};
  EXPECT_NO_THROW(this->lfit_bounding_box_2d(vals.begin(), vals.end()));
  vals = {this->make(-2.1, 10.1),     this->make(-1.89999, 10.1),  this->make(-1.89999, 10.3),
          this->make(-1.7, 10.3),     this->make(-1.5, 10.3),      this->make(-1.3, 10.3),
          this->make(-0.5, 10.3),     this->make(-0.300003, 10.3), this->make(-0.0999908, 10.3),
          this->make(0.699997, 10.3), this->make(0.900009, 10.3),  this->make(1.3, 10.3),
          this->make(1.7, 10.3)};
  EXPECT_NO_THROW(this->lfit_bounding_box_2d(vals.begin(), vals.end()));
  vals = {this->make(2.7, -5.5), this->make(2.7, -5.3), this->make(2.7, -5.1),
          this->make(2.7, -4.9), this->make(2.7, -4.7), this->make(2.7, -4.5),
          this->make(2.7, -4.3), this->make(2.7, -4.1), this->make(2.7, -3.9)};
  EXPECT_NO_THROW(this->lfit_bounding_box_2d(vals.begin(), vals.end()));
}
/// Handle slight floating point underflow case
// Note: raw discriminant checks are disabled because they don't work. I suspect this is due to
// tiny differences in floating point math when using our compile flags
TYPED_TEST(BoxTest, EigUnderflow)
{
  using autoware::common::geometry::bounding_box::details::Covariance2d;
  // auto discriminant = [](const Covariance2d cov) -> float32_t {
  //     // duplicated raw math
  //     const float32_t tr_2 = (cov.xx + cov.yy) * 0.5F;
  //     const float32_t det = (cov.xx * cov.yy) - (cov.xy * cov.xy);
  //     return (tr_2 * tr_2) - det;
  //   };
  TypeParam u, v;
  const Covariance2d c1{0.0300002, 0.0300002, 5.46677e-08, 0U};
  // EXPECT_LT(discriminant(c1), 0.0F);
  EXPECT_NO_THROW(autoware::common::geometry::bounding_box::details::eig_2d(c1, u, v));
  const Covariance2d c2{0.034286, 0.0342847, 2.12874e-09, 0U};
  // EXPECT_LT(discriminant(c2), 0.0F);
  EXPECT_NO_THROW(autoware::common::geometry::bounding_box::details::eig_2d(c2, u, v));
  const Covariance2d c3{0.0300002, 0.0300002, -2.84987e-08, 0U};
  // EXPECT_LT(discriminant(c3), 0.0F);
  EXPECT_NO_THROW(autoware::common::geometry::bounding_box::details::eig_2d(c3, u, v));
  const Covariance2d c4{0.0300004, 0.0300002, 3.84156e-08, 0U};
  // EXPECT_LT(discriminant(c4), 0.0F);
  EXPECT_NO_THROW(autoware::common::geometry::bounding_box::details::eig_2d(c4, u, v));
  const Covariance2d c5{0.0300014, 0.0300014, -7.45058e-09, 0U};
  // EXPECT_LT(discriminant(c5), 0.0F);
  EXPECT_NO_THROW(autoware::common::geometry::bounding_box::details::eig_2d(c5, u, v));
  const Covariance2d c6{0.0400004, 0.0400002, 3.28503e-08, 0U};
  // EXPECT_LT(discriminant(c6), 0.0F);
  EXPECT_NO_THROW(autoware::common::geometry::bounding_box::details::eig_2d(c6, u, v));
}

////////////////////////////////////////////////

#endif  // TEST_BOUNDING_BOX_HPP_
