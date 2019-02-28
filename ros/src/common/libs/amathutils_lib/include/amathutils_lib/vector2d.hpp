#pragma once

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace amathutils
{
class Vector2d
{
protected:
  Eigen::Vector2d vec_;
  Vector2d(const Eigen::Vector2d &vec);

public:
  Vector2d();
  Vector2d(const double _x, const double _y);
  virtual ~Vector2d(){};
  static Vector2d Zero();
  static Vector2d Identity();
  static Vector2d UnitX();
  static Vector2d UnitY();
  static Vector2d UnitFromAngle(const double angle);
  static double getDistance(const Vector2d &_a, const Vector2d &_b);
  double getDistance(const Vector2d &_a) const;
  static double dot(const Vector2d &_a, const Vector2d &_b);
  double dot(const Vector2d &_a) const;
  static double cross(const Vector2d &_a, const Vector2d &_b);
  double cross(const Vector2d &_a) const;
  static Vector2d normalized(const Vector2d &_a);
  void normalize();
  Vector2d rotate(const double angle) const;
  double getAngle() const;
  double getL1norm() const;
  double getL2norm() const;
  double getX() const;
  double getY() const;
  double& x();
  double& y();
  Vector2d operator-(const Vector2d &other) const;
  Vector2d operator+(const Vector2d &other) const;
  Vector2d operator*(const double ratio) const;
  Vector2d operator/(const double ratio) const;
  Vector2d &operator+=(const Vector2d &other);
  Vector2d &operator-=(const Vector2d &other);
  Vector2d &operator*=(const double ratio);
  Vector2d &operator/=(const double ratio);
  bool operator==(const Vector2d &other) const;
};

} // namespace amathutils