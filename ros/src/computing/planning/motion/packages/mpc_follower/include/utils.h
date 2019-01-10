#pragma once
#include <cmath>
#include <vector>

// set into [-pi to pi]
double intoSemicircle(const double a) {
  double b = fmod(a, 2.0 * M_PI);
  b -= 2.0 * M_PI * ((b > M_PI) - (b < -M_PI));
  return b;
}


void convertEulerAngleToMonotonic(std::vector<double> &a) {

  for (uint i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + intoSemicircle(da);
  }
}


void fill_increase(std::vector<double>::iterator first,
                   std::vector<double>::iterator last, double init,
                   double diff) {
  double value = init;
  while (first != last) {
    *first++ = value;
    value += diff;
  }
}

void filteringMovingAverate(std::vector<double> &u, const int num) {

  if (u.size() < num) {
    printf("[MovingAverageFilter] vector size is low than moving average "
           "number\n");
    return;
  }

  std::vector<double> filtered_u(u);

  for (uint i = 0; i < u.size(); ++i) {
    double tmp = 0.0;
    int count = 0;
    for (int j = -num; j < num + 1; ++j) {
      if (i + j > -0.5 && i + j < u.size() - 0.5) {
        tmp += u[i + j];
        ++count;
      }
    }
    filtered_u[i] = tmp / (2 * num + 1);
  }
  u = filtered_u;
}

// 1D interpolation
bool interp1d(const std::vector<double> &index,
              const std::vector<double> &values, const double &ref,
              double &ret) {

  ret = 0.0;

  if (!(index.size() == values.size())) {
    printf("index and values must have same size, return false.\n");
    return false;
  }

  if (index.size() == 1) {
    printf("index size is 1, too short. return false.\n");
    return false;
  }

  if (ref < index.front()) {
    ret = index.front();
    printf("ref point is out of index (low), return false.\n");
    return false;
  }

  if (index.back() < ref) {
    ret = index.back();
    printf("ref point is out of index (high), return false.\n");
    return false;
  }

  for (uint i = 1; i < index.size(); ++i) {
    if (!(index[i] > index[i - 1])) {
      printf("index must be monotonically increasing, return false.\n");
      return false;
    }
  }

  uint i = 1;
  while (ref > index[i]) {
    ++i;
  }

  const double a = ref - index[i - 1];
  const double d_index = index[i] - index[i - 1];
  ret = ((d_index - a) * values[i - 1] + a * values[i]) / d_index;

  // printf("i = %d, ref = %f, index.front() = %f, index.back() = %f, a = %f,
  // d_index = %f, ret = %f\n",i,ref, index.front(), index.back(), a, d_index,
  // ret);
  return true;
}


bool interp1d(const Eigen::VectorXd &index,
              const Eigen::VectorXd &values, const double &ref,
              double &ret) {

  ret = 0.0;

  if (!(index.size() == values.size())) {
    printf("index and values must have same size, return false.\n");
    return false;
  }

  if (index.size() == 1) {
    printf("index size is 1, too short. return false.\n");
    return false;
  }

  uint end = index.size() - 1;

  if (ref < index[0]) {
    ret = index[0];
    printf("ref point is out of index (low), return false.\n");
    return false;
  }

  if (index[end] < ref) {
    ret = index[end];
    printf("ref point is out of index (high), return false.\n");
    return false;
  }

  for (uint i = 1; i < index.size(); ++i) {
    if (!(index[i] > index[i - 1])) {
      printf("index must be monotonically increasing, return false.\n");
      return false;
    }
  }

  uint i = 1;
  while (ref > index[i]) {
    ++i;
  }

  const double a = ref - index[i - 1];
  const double d_index = index[i] - index[i - 1];
  ret = ((d_index - a) * values[i - 1] + a * values[i]) / d_index;

  // printf("i = %d, ref = %f, index.front() = %f, index.back() = %f, a = %f,
  // d_index = %f, ret = %f\n",i,ref, index.front(), index.back(), a, d_index,
  // ret);
  return true;
}
