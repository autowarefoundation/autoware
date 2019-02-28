#pragma once

#include <cmath>

namespace amathutils
{
const double G_MPSS = 9.80665; // m/s^2

inline double mps2kmph(const double _mpsval)
{
    return (_mpsval * 3.6); // mps * 60sec * 60minutes / 1000m
}

inline double kmph2mps(const double _kmphval)
{
    return (_kmphval * 1000.0 / 60.0 / 60.0); // kmph * 1000m / 60sec / 60sec
}

inline double rad2deg(const double _radval)
{
    return (_radval * (180.0 / M_PI));
}

inline double ded2rad(const double _degval)
{
    return (_degval * (M_PI / 180.0));
}

} // namespace amathutils