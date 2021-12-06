// Copyright 2018 Forrest
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CUBIC_SPLINE_HPP_
#define CUBIC_SPLINE_HPP_

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static std::vector<double> vec_diff(const std::vector<double> & input)
{
  std::vector<double> output;
  for (unsigned int i = 1; i < input.size(); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
}

static std::vector<double> cum_sum(const std::vector<double> & input)
{
  std::vector<double> output;
  double temp = 0;
  for (unsigned int i = 0; i < input.size(); i++) {
    temp += input[i];
    output.push_back(temp);
  }
  return output;
}

class Spline
{
public:
  std::vector<double> x;
  std::vector<double> y;
  int nx;
  std::vector<double> h;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  // Eigen::VectorXf c;
  std::vector<double> d;

  Spline() {}
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(const std::vector<double> & x_, const std::vector<double> & y_)
  : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_)
  {
    Eigen::MatrixXd A = calc_A();
    Eigen::VectorXd B = calc_B();
    // std::cerr << "A det " << A.determinant() << std::endl;
    // std::cerr << "A QR" << A.colPivHouseholderQr().solve(B) << std::endl;
    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
    // std::cerr << "c eigen " << c_eigen << std::endl;
    double * c_pointer = c_eigen.data();
    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (int i = 0; i < nx - 1; i++) {
      d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
      b.push_back((a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
  }

  double calc(double t)
  {
    if (t < x.front() || t > x.back()) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
  }

  double calc(double t, double s)
  {
    if (t < 0 || t > s) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
  }

  double calc_d(double t)
  {
    if (t < x.front() || t > x.back()) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double calc_d(double t, double s)
  {
    if (t < 0 || t > s) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double calc_dd(double t)
  {
    if (t < x.front() || t > x.back()) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

  double calc_dd(double t, double s)
  {
    if (t < 0.0 || t > s) {
      std::cout << "Dangerous" << std::endl;
      std::cout << t << std::endl;
      throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

private:
  Eigen::MatrixXd calc_A()
  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
    A(0, 0) = 1;
    for (int i = 0; i < nx - 1; i++) {
      if (i != nx - 2) {
        A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
      }
      A(i + 1, i) = h[i];
      A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;
    return A;
  }
  Eigen::VectorXd calc_B()
  {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for (int i = 0; i < nx - 2; i++) {
      B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
  }

  int bisect(double t, int start, int end)
  {
    int mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1) {
      return mid;
    } else if (t > x[mid]) {
      return bisect(t, mid, end);
    } else {
      return bisect(t, start, mid);
    }
  }
};

class Spline2D
{
public:
  Spline sx;
  Spline sy;
  std::vector<double> s;
  double max_s_value_;

  Spline2D(const std::vector<double> & x, const std::vector<double> & y)
  {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
    max_s_value_ = *std::max_element(s.begin(), s.end());
  }

  std::array<double, 2> calc_position(double s_t)
  {
    double x = sx.calc(s_t, max_s_value_);
    double y = sy.calc(s_t, max_s_value_);
    return {{x, y}};
  }

  double calc_curvature(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double ddx = sx.calc_dd(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    double ddy = sy.calc_dd(s_t, max_s_value_);
    return (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
  }

  double calc_yaw(double s_t)
  {
    double dx = sx.calc_d(s_t, max_s_value_);
    double dy = sy.calc_d(s_t, max_s_value_);
    return std::atan2(dy, dx);
  }

private:
  std::vector<double> calc_s(const std::vector<double> & x, const std::vector<double> & y)
  {
    std::vector<double> ds;
    std::vector<double> out_s{0};
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    std::vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  }
};

#endif  // CUBIC_SPLINE_HPP_
