#include "mpc_follower/lowpass_filter.h"

Butterworth2dFilter::Butterworth2dFilter(double dt, double f_cutoff_hz)
{
  initialize(dt, f_cutoff_hz);
};

Butterworth2dFilter::~Butterworth2dFilter(){};

void Butterworth2dFilter::initialize(const double &dt, const double &f_cutoff_hz)
{
  y1_ = 0.0;
  y2_ = 0.0;
  u2_ = 0.0;
  u1_ = 0.0;

  /* 2d butterworth lowpass filter with bi-linear transformation */
  double wc = 2.0 * M_PI * f_cutoff_hz;
  double n = 2 / dt;
  a0_ = n * n + sqrt(2) * wc * n + wc * wc;
  a1_ = 2 * wc * wc - 2 * n * n;
  a2_ = n * n - sqrt(2) * wc * n + wc * wc;
  b0_ = wc * wc;
  b1_ = 2 * b0_;
  b2_ = b0_;
}

double Butterworth2dFilter::filter(double &u0)
{
  double y0 = (b2_ * u2_ + b1_ * u1_ + b0_ * u0 - a2_ * y2_ - a1_ * y1_) / a0_;
  y2_ = y1_;
  y1_ = y0;
  u2_ = u1_;
  u1_ = u0;
  return y0;
}

void Butterworth2dFilter::filt_vector(const std::vector<double> &t, std::vector<double> &u)
{
  double y1 = u.at(0);
  double y2 = u.at(0);
  double u2 = u.at(0);
  double u1 = u.at(0);
  double y0 = 0.0;
  double u0 = 0.0;
  for (unsigned int i = 0; i < u.size(); ++i)
  {
    u0 = u.at(i);
    y0 = (b2_ * u2 + b1_ * u1 + b0_ * u0 - a2_ * y2 - a1_ * y1) / a0_;
    y2 = y1;
    y1 = y0;
    u2 = u1;
    u1 = u0;
    u.at(i) = y0;
  }
}

// filtering forward and backward direction
void Butterworth2dFilter::filtfilt_vector(const std::vector<double> &t, std::vector<double> &u)
{
  std::vector<double> u_rev(u);

  // forward filtering
  filt_vector(t, u);

  // backward filtering
  std::reverse(u_rev.begin(), u_rev.end());
  filt_vector(t, u_rev);
  std::reverse(u_rev.begin(), u_rev.end());

  // merge
  for (unsigned int i = 0; i < u.size(); ++i)
  {
    u[i] = (u[i] + u_rev[i]) * 0.5;
  }
}

bool MoveAverageFilter::filt_vector(const int num, std::vector<double> &u)
{

  if ((int)u.size() < num)
  {
    printf("[MovingAverageFilter] vector size is low than moving average number\n");
    return false;
  }
  std::vector<double> filtered_u(u);
  for (unsigned int i = 0; i < u.size(); ++i)
  {
    double tmp = 0.0;
    int count = 0;
    for (int j = -num; j < num + 1; ++j)
    {
      if (i + j > -0.5 && i + j < u.size() - 0.5)
      {
        tmp += u[i + j];
        ++count;
      }
    }
    filtered_u[i] = tmp / count;
  }
  u = filtered_u;
  return true;
}