#pragma once

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace amathutils
{
class Vector3d
{
protected:
  Eigen::Vector3d vec_;
  Vector3d(const Eigen::Vector3d &vec);

public:
  Vector3d();
  Vector3d(const double _x, const double _y, const double _z);
  virtual ~Vector3d(){};
  static Vector3d Zero();
  static Vector3d Identity();
  static Vector3d UnitX();
  static Vector3d UnitY();
  static Vector3d UnitZ();
  static double getDistance(const Vector3d &_a, const Vector3d &_b);
  double getDistance(const Vector3d &_a) const;
  static double dot(const Vector3d &_a, const Vector3d &_b);
  double dot(const Vector3d &_a) const;
  static Vector3d cross(const Vector3d &_a, const Vector3d &_b);
  Vector3d cross(const Vector3d &_a) const;
  static Vector3d normalized(const Vector3d &_a);
  void normalize();
  double getL1norm() const;
  double getL2norm() const;
  double getX() const;
  double getY() const;
  double getZ() const;
  double &x();
  double &y();
  double &z();
  Vector3d operator-(const Vector3d &other) const;
  Vector3d operator+(const Vector3d &other) const;
  Vector3d operator*(const double ratio) const;
  Vector3d operator/(const double ratio) const;
  Vector3d &operator+=(const Vector3d &other);
  Vector3d &operator-=(const Vector3d &other);
  Vector3d &operator*=(const double ratio);
  Vector3d &operator/=(const double ratio);
  bool operator==(const Vector3d &other) const;
  bool operator!=(const Vector3d &other) const;
};

} // namespace amathutils