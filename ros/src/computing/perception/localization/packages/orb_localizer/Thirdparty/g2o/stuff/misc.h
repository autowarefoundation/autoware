// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_STUFF_MISC_H
#define G2O_STUFF_MISC_H

#include <cmath>
#include "g2o/stuff/macros.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** @addtogroup utils **/
// @{

/** \file misc.h
 * \brief some general case utility functions
 *
 *  This file specifies some general case utility functions
 **/

namespace g2o {

/**
 * return the square value
 */
template <typename T>
inline T square(T x)
{
  return x*x;
}

/**
 * return the hypot of x and y
 */
template <typename T>
inline T hypot(T x, T y)
{
  return (T) (sqrt(x*x + y*y));
}

/**
 * return the squared hypot of x and y
 */
template <typename T>
inline T hypot_sqr(T x, T y)
{
  return x*x + y*y;
}

/**
 * convert from degree to radian
 */
inline double deg2rad(double degree)
{
  return degree * 0.01745329251994329576;
}

/**
 * convert from radian to degree
 */
inline double rad2deg(double rad)
{
  return rad * 57.29577951308232087721;
}

/**
 * normalize the angle
 */
inline double normalize_theta(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;
  
  double multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}

/**
 * inverse of an angle, i.e., +180 degree
 */
inline double inverse_theta(double th)
{
  return normalize_theta(th + M_PI);
}

/**
 * average two angles
 */
inline double average_angle(double theta1, double theta2)
{
  double x, y;

  x = cos(theta1) + cos(theta2);
  y = sin(theta1) + sin(theta2);
  if(x == 0 && y == 0)
    return 0;
  else
    return std::atan2(y, x);
}

/**
 * sign function.
 * @return the sign of x. +1 for x > 0, -1 for x < 0, 0 for x == 0
 */
template <typename T>
inline int sign(T x)
{
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}

/**
 * clamp x to the interval [l, u]
 */
template <typename T>
inline T clamp(T l, T x, T u) 
{
  if (x < l)
    return l;
  if (x > u)
    return u;
  return x;
}

/**
 * wrap x to be in the interval [l, u]
 */
template <typename T>
inline T wrap(T l, T x, T u) 
{
  T intervalWidth = u - l;
  while (x < l)
    x += intervalWidth;
  while (x > u)
    x -= intervalWidth;
  return x;
}

/**
 * tests whether there is a NaN in the array
 */
inline bool arrayHasNaN(const double* array, int size, int* nanIndex = 0)
{
  for (int i = 0; i < size; ++i)
    if (g2o_isnan(array[i])) {
      if (nanIndex)
        *nanIndex = i;
      return true;
    }
  return false;
}

/**
 * The following two functions are used to force linkage with static libraries.
 */
extern "C"
{
    typedef void (* ForceLinkFunction) (void);
}

struct ForceLinker
{
    ForceLinker(ForceLinkFunction function) { (function)(); }
};


} // end namespace

// @}

#endif
