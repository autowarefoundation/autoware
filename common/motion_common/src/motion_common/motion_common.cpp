// Copyright 2019-2021 the Autoware Foundation
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

#include "motion_common/motion_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "helper_functions/angle_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace motion
{
namespace motion_common
{
////////////////////////////////////////////////////////////////////////////////
bool is_past_point(const Point & state, const Point & pt) noexcept
{
  const auto w = static_cast<Real>(pt.pose.orientation.w);
  const auto z = static_cast<Real>(pt.pose.orientation.z);
  // double angle rule
  const auto c = (w + z) * (w - z);
  const auto s = 2.0F * w * z;

  return is_past_point(state, pt, c, s);
}

////////////////////////////////////////////////////////////////////////////////
bool is_past_point(
  const Point & state,
  const Point & current_pt,
  const Point & next_pt) noexcept
{
  const auto nx = next_pt.pose.position.x - current_pt.pose.position.x;
  const auto ny = next_pt.pose.position.y - current_pt.pose.position.y;

  return is_past_point(state, next_pt, nx, ny);
}

////////////////////////////////////////////////////////////////////////////////
bool is_past_point(
  const Point & state,
  const Point & pt,
  const double nx,
  const double ny) noexcept
{
  const auto dx = (state.pose.position.x - pt.pose.position.x);
  const auto dy = (state.pose.position.y - pt.pose.position.y);

  // Check if state is past last_pt when projected onto the ray defined by heading
  return ((nx * dx) + (ny * dy)) >= -std::numeric_limits<double>::epsilon();
}

////////////////////////////////////////////////////////////////////////////////
bool is_aligned(const Heading a, const Heading b, const Real dot_threshold)
{
  if (dot_threshold < Real{}) {
    throw std::domain_error{"Dot product threshold cannot be negative"};
  }
  const auto dot = (a.real * b.real) + (a.imag * b.imag);
  const auto amag = std::sqrt((a.real * a.real) + (a.imag * a.imag));
  const auto bmag = std::sqrt((b.real * b.real) + (b.imag * b.imag));
  const auto thresh = std::min(dot_threshold, Real{1.0});
  return (dot / (amag * bmag)) > thresh;
}

////////////////////////////////////////////////////////////////////////////////
bool heading_ok(const Trajectory & traj)
{
  const auto bad_heading = [](const auto & pt) -> bool {
      const auto real2 = static_cast<Real>(pt.pose.orientation.w * pt.pose.orientation.w);
      const auto imag2 = static_cast<Real>(pt.pose.orientation.z * pt.pose.orientation.z);
      constexpr auto TOL = 1.0E-3F;
      return std::fabs(1.0F - (real2 + imag2)) > TOL;
    };
  const auto bad_it = std::find_if(traj.points.begin(), traj.points.end(), bad_heading);
  // True if there is no bad point
  return bad_it == traj.points.end();
}

////////////////////////////////////////////////////////////////////////////////
void doTransform(
  const Point & t_in,
  Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept
{
  geometry_msgs::msg::PoseStamped p_in;
  p_in.pose = t_in.pose;
  p_in.header.stamp = transform.header.stamp;
  geometry_msgs::msg::PoseStamped p_out;
  tf2::doTransform(p_in, p_out, transform);
  t_out.pose = p_out.pose;
}

////////////////////////////////////////////////////////////////////////////////
void doTransform(
  const State & t_in,
  State & t_out,
  const geometry_msgs::msg::TransformStamped & transform) noexcept
{
  doTransform(t_in.state, t_out.state, transform);
  t_out.header.frame_id = transform.header.frame_id;
}

////////////////////////////////////////////////////////////////////////////////
Real to_angle(Heading heading) noexcept
{
  const auto mag2 = (heading.real * heading.real) + (heading.imag * heading.imag);
  if (std::abs(mag2 - 1.0F) > std::numeric_limits<Real>::epsilon()) {
    const auto imag = Real{1} / std::sqrt(mag2);
    heading.real *= imag;
    heading.imag *= imag;
    // Don't need to touch imaginary/z part
  }
  // See:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const auto y = Real{2.0} *heading.real * heading.imag;
  const auto x = Real{1} - (Real{2.0} *heading.imag * heading.imag);
  // TODO(c.ho) fast atan2
  return std::atan2(y, x);
}

////////////////////////////////////////////////////////////////////////////////
Real to_angle(Orientation orientation) noexcept
{
  return static_cast<Real>(tf2::getYaw(orientation));
}

////////////////////////////////////////////////////////////////////////////////
Heading nlerp(Heading a, Heading b, Real t)
{
  // Could technically use none, but I get basically nothing from that
  Heading ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  // check dot product: if negative, reflect one quaternion (360 deg rotation)
  {
    const auto dot = (a.real * b.real) + (a.imag * b.imag);
    if (dot < Real{}) {  // zero initialization
      b.real = -b.real;
      b.imag = -b.imag;
    }
  }
  // Linearly interpolate
  ret.real = interpolate(a.real, b.real, t);
  ret.imag = interpolate(a.imag, b.imag, t);
  // Normalize
  const auto s = 1.0F / std::sqrt((ret.real * ret.real) + (ret.imag * ret.imag));
  ret.real *= s;
  ret.imag *= s;
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Orientation slerp(const Orientation & a, const Orientation & b, const Real t)
{
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a.slerp(quat_b, t));
}

////////////////////////////////////////////////////////////////////////////////
Point interpolate(Point a, Point b, Real t)
{
  return interpolate(a, b, t, slerp);
}

////////////////////////////////////////////////////////////////////////////////
void sample(const Trajectory & in, Trajectory & out, std::chrono::nanoseconds period)
{
  sample(in, out, period, slerp);
}

////////////////////////////////////////////////////////////////////////////////
void error(const Point & state, const Point & ref, Diagnostic & out) noexcept
{
  {
    // compute heading normal of reference point
    const auto & q = ref.pose.orientation;
    const auto nx = (q.w * q.w) - (q.z * q.z);
    const auto ny = decltype(nx) {2.0} *q.w * q.z;
    // project state onto reference basis
    const auto dx = state.pose.position.x - ref.pose.position.x;
    const auto dy = state.pose.position.y - ref.pose.position.y;
    // normals rotated +90 deg
    out.lateral_error_m = static_cast<Real>((dx * (-ny)) + (dy * nx));
    out.longitudinal_error_m = static_cast<Real>((dx * nx) + (dy * ny));
  }
  out.velocity_error_mps = state.longitudinal_velocity_mps - ref.longitudinal_velocity_mps;
  out.acceleration_error_mps2 = state.acceleration_mps2 - ref.acceleration_mps2;

  using autoware::common::helper_functions::wrap_angle;
  out.yaw_error_rad = wrap_angle(to_angle(state.pose.orientation) - to_angle(ref.pose.orientation));
  out.yaw_rate_error_rps = state.heading_rate_rps - ref.heading_rate_rps;
}
}  // namespace motion_common
}  // namespace motion
namespace autoware_auto_geometry_msgs
{
namespace msg
{
Complex32 operator+(Complex32 a, Complex32 b) noexcept
{
  // Could technically use none, but I get basically nothing from that
  Complex32 ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  // check dot product: if negative, reflect one quaternion (360 deg rotation)
  {
    const auto dot = (a.real * b.real) + (a.imag * b.imag);
    if (dot < decltype(b.real) {}) {  // zero initialization
      b.real = -b.real;
      b.imag = -b.imag;
    }
  }
  ret.real = (a.real * b.real) - (a.imag * b.imag);
  ret.imag = (a.real * b.imag) + (a.imag * b.real);
  return ret;
}
Complex32 operator-(Complex32 a) noexcept
{
  a.real = -a.real;
  return a;
}
Complex32 operator-(Complex32 a, Complex32 b) noexcept
{
  return a + (-b);
}
}  // namespace msg
}  // namespace autoware_auto_msgs

namespace geometry_msgs
{
namespace msg
{
Quaternion operator+(Quaternion a, Quaternion b) noexcept
{
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a + quat_b);
}
Quaternion operator-(Quaternion a) noexcept
{
  tf2::Quaternion quat_a;
  tf2::fromMsg(a, quat_a);
  return tf2::toMsg(quat_a * -1.0);
}
Quaternion operator-(Quaternion a, Quaternion b) noexcept
{
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a * quat_b.inverse());
}
}  // namespace msg
}  // namespace geometry_msgs
