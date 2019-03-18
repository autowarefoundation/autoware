#pragma once
#include <cmath>
#include <vector>
#include <algorithm>

class Butterworth2dFilter
{
private:
  double y1_, y2_, u1_, u2_, a0_, a1_, a2_, b0_, b1_, b2_; // weight parameters set by dt & cutoff_hz

public:
  Butterworth2dFilter(double dt = 0.1, double f_cutoff_hz = 10.0);
  ~Butterworth2dFilter();
  void initialize(const double &dt, const double &f_cutoff_hz);
  double filter(double &u0);
  void filt_vector(const std::vector<double> &t, std::vector<double> &u);
  void filtfilt_vector(const std::vector<double> &t, std::vector<double> &u); // filtering forward and backward direction
};

class MoveAverageFilter
{
public:
  MoveAverageFilter();
  ~MoveAverageFilter();
  static bool filt_vector(const int num, std::vector<double> &u);
};