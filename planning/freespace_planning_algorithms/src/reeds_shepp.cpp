// Copyright 2021 Tier IV, Inc. All rights reserved.
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

// Software License Agreement (BSD License)
//
// Copyright (c) 2016, Guan-Horng Liu.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author:  Guan-Horng Liu

// Software License Agreement (BSD License)
//
// Copyright (c) 2010, Rice University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Rice University nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "freespace_planning_algorithms/reeds_shepp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.
using freespace_planning_algorithms::ReedsSheppStateSpace;

const double pi = M_PI;
const double twopi = 2. * pi;
const double RS_EPS = 1e-6;
const double ZERO = 10 * std::numeric_limits<double>::epsilon();

inline double mod2pi(double x)
{
  double v = std::fmod(x, twopi);
  if (v < -pi) {
    v += twopi;
  } else if (v > pi) {
    v -= twopi;
  }
  return v;
}
inline void polar(double x, double y, double & r, double & theta)
{
  r = std::sqrt(x * x + y * y);
  theta = std::atan2(y, x);
}
inline void tauOmega(
  double u, double v, double xi, double eta, double phi, double & tau, double & omega)
{
  double delta = mod2pi(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.;
  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2. * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3;
  tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

// formula 8.1 in Reeds-Shepp paper
inline bool LpSpLp(double x, double y, double phi, double & t, double & u, double & v)
{
  polar(x - std::sin(phi), y - 1. + std::cos(phi), u, t);
  if (t >= -ZERO) {
    v = mod2pi(phi - t);
    if (v >= -ZERO) {
      assert(std::abs(u * std::cos(t) + std::sin(phi) - x) < RS_EPS);
      assert(std::abs(u * std::sin(t) - std::cos(phi) + 1 - y) < RS_EPS);
      assert(std::abs(mod2pi(t + v - phi)) < RS_EPS);
      return true;
    }
  }
  return false;
}
// formula 8.2
inline bool LpSpRp(double x, double y, double phi, double & t, double & u, double & v)
{
  double t1, u1;
  polar(x + std::sin(phi), y - 1. - std::cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.) {
    double theta;
    u = sqrt(u1 - 4.);
    theta = std::atan2(2., u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    assert(std::abs(2 * std::sin(t) + u * std::cos(t) - std::sin(phi) - x) < RS_EPS);
    assert(std::abs(-2 * std::cos(t) + u * std::sin(t) + std::cos(phi) + 1 - y) < RS_EPS);
    assert(std::abs(mod2pi(t - v - phi)) < RS_EPS);
    return t >= -ZERO && v >= -ZERO;
  }
  return false;
}
void CSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath & path)
{
  double t, u, v, L_min = path.length(), L;
  if (LpSpLp(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[14], t, u, v);
    L_min = L;
  }
  if (
    LpSpLp(-x, y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[14], -t, -u, -v);
    L_min = L;
  }
  if (
    LpSpLp(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[15], t, u, v);
    L_min = L;
  }
  if (
    LpSpLp(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[15], -t, -u, -v);
    L_min = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[12], t, u, v);
    L_min = L;
  }
  if (
    LpSpRp(-x, y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[12], -t, -u, -v);
    L_min = L;
  }
  if (
    LpSpRp(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[13], t, u, v);
    L_min = L;
  }
  if (
    LpSpRp(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[13], -t, -u, -v);
  }
}
// formula 8.3 / 8.4  *** TYPO IN PAPER ***
inline bool LpRmL(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x - std::sin(phi), eta = y - 1. + std::cos(phi), u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.) {
    u = -2. * std::asin(.25 * u1);
    t = mod2pi(theta + .5 * u + pi);
    v = mod2pi(phi - t + u);
    assert(std::abs(2 * (sin(t) - std::sin(t - u)) + std::sin(phi) - x) < RS_EPS);
    assert(std::abs(2 * (-cos(t) + std::cos(t - u)) - std::cos(phi) + 1 - y) < RS_EPS);
    assert(std::abs(mod2pi(t - u + v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO;
  }
  return false;
}
void CCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath & path)
{
  double t, u, v, L_min = path.length(), L;
  if (LpRmL(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], t, u, v);
    L_min = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    // time flip
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], -t, -u, -v);
    L_min = L;
  }
  if (
    LpRmL(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], t, u, v);
    L_min = L;
  }
  if (
    LpRmL(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], -t, -u, -v);
    L_min = L;
  }

  // backwards
  double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], v, u, t);
    L_min = L;
  }
  if (
    LpRmL(-xb, yb, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[0], -v, -u, -t);
    L_min = L;
  }
  if (
    LpRmL(xb, -yb, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], v, u, t);
    L_min = L;
  }
  if (
    LpRmL(-xb, -yb, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[1], -v, -u, -t);
  }
}
// formula 8.7
inline bool LpRupLumRm(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi),
         rho = .25 * (2. + sqrt(xi * xi + eta * eta));
  if (rho <= 1.) {
    u = acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    assert(
      std::abs(2 * (sin(t) - std::sin(t - u) + std::sin(t - 2 * u)) - std::sin(phi) - x) < RS_EPS);
    assert(
      std::abs(2 * (-cos(t) + std::cos(t - u) - std::cos(t - 2 * u)) + std::cos(phi) + 1 - y) <
      RS_EPS);
    assert(std::abs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
    return t >= -ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.8
inline bool LpRumLumRp(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi),
         rho = (20. - xi * xi - eta * eta) / 16.;
  if (rho >= 0 && rho <= 1) {
    u = -acos(rho);
    if (u >= -.5 * pi) {
      tauOmega(u, u, xi, eta, phi, t, v);
      assert(std::abs(4 * std::sin(t) - 2 * std::sin(t - u) - std::sin(phi) - x) < RS_EPS);
      assert(std::abs(-4 * std::cos(t) + 2 * std::cos(t - u) + std::cos(phi) + 1 - y) < RS_EPS);
      assert(std::abs(mod2pi(t - v - phi)) < RS_EPS);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
void CCCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath & path)
{
  double t, u, v, L_min = path.length(), L;
  if (
    LpRupLumRm(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v))) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[2], t, u, -u, v);
    L_min = L;
  }
  if (
    LpRupLumRm(-x, y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, u, -v);
    L_min = L;
  }
  if (
    LpRupLumRm(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[3], t, u, -u, v);
    L_min = L;
  }
  if (
    LpRupLumRm(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, u, -v);
    L_min = L;
  }

  if (
    LpRumLumRp(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v))) {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[2], t, u, u, v);
    L_min = L;
  }
  if (
    LpRumLumRp(-x, y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, -u, -v);
    L_min = L;
  }
  if (
    LpRumLumRp(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v)))  // reflect
  {
    path =
      ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reedsSheppPathType[3], t, u, u, v);
    L_min = L;
  }
  if (
    LpRumLumRp(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + 2. * std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, -u, -v);
  }
}
// formula 8.9
inline bool LpRmSmLm(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x - std::sin(phi), eta = y - 1. + std::cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    double r = sqrt(rho * rho - 4.);
    u = 2. - r;
    t = mod2pi(theta + std::atan2(r, -2.));
    v = mod2pi(phi - .5 * pi - t);
    assert(std::abs(2 * (sin(t) - std::cos(t)) - u * std::sin(t) + std::sin(phi) - x) < RS_EPS);
    assert(
      std::abs(-2 * (sin(t) + std::cos(t)) + u * std::cos(t) - std::cos(phi) + 1 - y) < RS_EPS);
    assert(std::abs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.10
inline bool LpRmSmRm(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi), rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.) {
    t = theta;
    u = 2. - rho;
    v = mod2pi(t + .5 * pi - phi);
    assert(std::abs(2 * std::sin(t) - std::cos(t - v) - u * std::sin(t) - x) < RS_EPS);
    assert(std::abs(-2 * std::cos(t) - std::sin(t - v) + u * std::cos(t) + 1 - y) < RS_EPS);
    assert(std::abs(mod2pi(t + pi / 2 - v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
void CCSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath & path)
{
  double t, u, v, L_min = path.length() - .5 * pi, L;
  if (LpRmSmLm(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[4], t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmLm(-x, y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[4], -t, .5 * pi, -u, -v);
    L_min = L;
  }
  if (
    LpRmSmLm(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[5], t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmLm(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[5], -t, .5 * pi, -u, -v);
    L_min = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[8], t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmRm(-x, y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[8], -t, .5 * pi, -u, -v);
    L_min = L;
  }
  if (
    LpRmSmRm(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[9], t, -.5 * pi, u, v);
    L_min = L;
  }
  if (
    LpRmSmRm(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[9], -t, .5 * pi, -u, -v);
    L_min = L;
  }

  // backwards
  double xb = x * std::cos(phi) + y * std::sin(phi), yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[6], v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmLm(-xb, yb, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[6], -v, -u, .5 * pi, -t);
    L_min = L;
  }
  if (
    LpRmSmLm(xb, -yb, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[7], v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmLm(-xb, -yb, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[7], -v, -u, .5 * pi, -t);
    L_min = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[10], v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmRm(-xb, yb, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[10], -v, -u, .5 * pi, -t);
    L_min = L;
  }
  if (
    LpRmSmRm(xb, -yb, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[11], v, u, -.5 * pi, t);
    L_min = L;
  }
  if (
    LpRmSmRm(-xb, -yb, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[11], -v, -u, .5 * pi, -t);
  }
}
// formula 8.11 *** TYPO IN PAPER ***
inline bool LpRmSLmRp(double x, double y, double phi, double & t, double & u, double & v)
{
  double xi = x + std::sin(phi), eta = y - 1. - std::cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    u = 4. - sqrt(rho * rho - 4.);
    if (u <= ZERO) {
      t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      v = mod2pi(t - phi);
      assert(
        std::abs(4 * std::sin(t) - 2 * std::cos(t) - u * std::sin(t) - std::sin(phi) - x) < RS_EPS);
      assert(
        std::abs(-4 * std::cos(t) - 2 * std::sin(t) + u * std::cos(t) + std::cos(phi) + 1 - y) <
        RS_EPS);
      assert(std::abs(mod2pi(t - v - phi)) < RS_EPS);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
void CCSCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath & path)
{
  double t, u, v, L_min = path.length() - pi, L;
  if (LpRmSLmRp(x, y, phi, t, u, v) && L_min > (L = std::abs(t) + std::abs(u) + std::abs(v))) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[16], t, -.5 * pi, u, -.5 * pi, v);
    L_min = L;
  }
  if (
    LpRmSLmRp(-x, y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[16], -t, .5 * pi, -u, .5 * pi, -v);
    L_min = L;
  }
  if (
    LpRmSLmRp(x, -y, -phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[17], t, -.5 * pi, u, -.5 * pi, v);
    L_min = L;
  }
  if (
    LpRmSLmRp(-x, -y, phi, t, u, v) &&
    L_min > (L = std::abs(t) + std::abs(u) + std::abs(v)))  // time flip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
      ReedsSheppStateSpace::reedsSheppPathType[17], -t, .5 * pi, -u, .5 * pi, -v);
  }
}

ReedsSheppStateSpace::ReedsSheppPath reedsShepp(double x, double y, double phi)
{
  ReedsSheppStateSpace::ReedsSheppPath path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  return path;
}
}  // namespace

namespace freespace_planning_algorithms
{
const ReedsSheppStateSpace::ReedsSheppPathSegmentType
  ReedsSheppStateSpace::reedsSheppPathType[18][5] = {
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         // 0
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        // 1
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       // 2
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       // 3
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 4
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 5
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    // 6
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 7
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 8
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 9
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 10
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    // 11
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     // 12
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 13
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      // 14
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 15
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  // 16
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   // 17
};

ReedsSheppStateSpace::ReedsSheppPath::ReedsSheppPath(
  const ReedsSheppPathSegmentType * type, double t, double u, double v, double w, double x)
: type_(type)
{
  length_[0] = t;
  length_[1] = u;
  length_[2] = v;
  length_[3] = w;
  length_[4] = x;
  totalLength_ = std::abs(t) + std::abs(u) + std::abs(v) + std::abs(w) + std::abs(x);
}

double ReedsSheppStateSpace::distance(const StateXYT & s0, const StateXYT & s1) const
{
  return rho_ * reedsShepp(s0, s1).length();
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::reedsShepp(
  const StateXYT & s0, const StateXYT & s1) const
{
  double dx = s1.x - s0.x, dy = s1.y - s0.y, dth = s1.yaw - s0.yaw;
  double c = std::cos(s0.yaw), s = std::sin(s0.yaw);
  double x = c * dx + s * dy, y = -s * dx + c * dy;
  return ::reedsShepp(x / rho_, y / rho_, dth);
}

ReedsSheppStateSpace::StateXYT ReedsSheppStateSpace::interpolate(
  const StateXYT & s0, ReedsSheppPath & path, double seg) const
{
  if (seg < 0.0) {
    seg = 0.0;
  }
  if (seg > path.length()) {
    seg = path.length();
  }

  StateXYT s_out;
  double phi, v;

  s_out.x = s_out.y = 0.0;
  s_out.yaw = s0.yaw;

  for (unsigned int i = 0; i < 5 && seg > 0; ++i) {
    if (path.length_[i] < 0) {
      v = std::max(-seg, path.length_[i]);
      seg += v;
    } else {
      v = std::min(seg, path.length_[i]);
      seg -= v;
    }
    phi = s_out.yaw;
    switch (path.type_[i]) {
      case RS_LEFT:
        s_out.x += (sin(phi + v) - std::sin(phi));
        s_out.y += (-cos(phi + v) + std::cos(phi));
        s_out.yaw = phi + v;
        break;
      case RS_RIGHT:
        s_out.x += (-sin(phi - v) + std::sin(phi));
        s_out.y += (cos(phi - v) - std::cos(phi));
        s_out.yaw = phi - v;
        break;
      case RS_STRAIGHT:
        s_out.x += (v * std::cos(phi));
        s_out.y += (v * std::sin(phi));
        break;
      case RS_NOP:
        break;
    }
  }

  s_out.x = s_out.x * rho_ + s0.x;
  s_out.y = s_out.y * rho_ + s0.y;
  return s_out;
}
}  // namespace freespace_planning_algorithms
