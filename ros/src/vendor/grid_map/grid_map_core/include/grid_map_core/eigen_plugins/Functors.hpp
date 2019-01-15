/*
 * Functors.hpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

namespace grid_map {

template<typename Scalar>
struct Clamp
{
  Clamp(const Scalar& min, const Scalar& max)
      : min_(min),
        max_(max)
  {
  }
  const Scalar operator()(const Scalar& x) const
  {
    return x < min_ ? min_ : (x > max_ ? max_ : x);
  }
  Scalar min_, max_;
};

}
