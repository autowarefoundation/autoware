#include <amathutils_lib/vector3d.hpp>
#include <iostream>

namespace amathutils
{
Vector3d::Vector3d() { vec_ = Eigen::Vector3d::Zero(); }
Vector3d::Vector3d(const double _x, const double _y, const double _z) { vec_ << _x, _y, _z; }
Vector3d::Vector3d(const Eigen::Vector3d &vec) : vec_(vec){};

Vector3d Vector3d::Zero() { return Vector3d(); }
Vector3d Vector3d::Identity() { return Vector3d(1.0, 1.0, 1.0); }
Vector3d Vector3d::UnitX() { return Vector3d(1.0, 0.0, 0.0); }
Vector3d Vector3d::UnitY() { return Vector3d(0.0, 1.0, 0.0); }
Vector3d Vector3d::UnitZ() { return Vector3d(0.0, 0.0, 1.0); }

double Vector3d::getDistance(const Vector3d &_a, const Vector3d &_b)
{
    return (_a - _b).getL2norm();
}

double Vector3d::getDistance(const Vector3d &_a) const
{
    return getDistance(*this, _a);
}

double Vector3d::getL1norm() const { return getX() + getY() + getZ(); }
double Vector3d::getL2norm() const { return vec_.norm(); }
double Vector3d::getX() const { return vec_.x(); }
double Vector3d::getY() const { return vec_.y(); }
double Vector3d::getZ() const { return vec_.z(); }
double &Vector3d::x() { return vec_.x(); }
double &Vector3d::y() { return vec_.y(); }
double &Vector3d::z() { return vec_.z(); }

double Vector3d::dot(const Vector3d &_a, const Vector3d &_b)
{
    Eigen::Vector3d vec1;
    vec1 << _a.getX(), _a.getY(), _a.getZ();
    Eigen::Vector3d vec2;
    vec2 << _b.getX(), _b.getY(), _b.getZ();
    return vec1.dot(vec2);
}

double Vector3d::dot(const Vector3d &_a) const
{
    return dot(*this, _a);
}

Vector3d Vector3d::cross(const Vector3d &_a, const Vector3d &_b)
{
    Eigen::Vector3d vec1;
    vec1 << _a.getX(), _a.getY(), _a.getZ();
    Eigen::Vector3d vec2;
    vec2 << _b.getX(), _b.getY(), _b.getZ();
    return vec1.cross(vec2);
}

Vector3d Vector3d::cross(const Vector3d &_a) const
{
    return cross(*this, _a);
}

Vector3d Vector3d::operator-(const Vector3d &other) const
{
    return Vector3d(getX() - other.getX(), getY() - other.getY(), getZ() - other.getZ());
}

Vector3d Vector3d::operator+(const Vector3d &other) const
{
    return Vector3d(getX() + other.getX(), getY() + other.getY(), getZ() + other.getZ());
}

Vector3d Vector3d::operator*(const double ratio) const
{
    return Vector3d(getX() * ratio, getY() * ratio, getZ() * ratio);
}

Vector3d Vector3d::operator/(const double ratio) const
{
    constexpr double ep = 10e-10;
    if (std::abs(ratio) < ep)
    {
        std::cerr << __FILE__ << "(" << __LINE__ << ")"
                  << ":"
                  << "too small ratio" << std::endl;
    }
    return Vector3d(getX() / ratio, getY() / ratio, getZ() / ratio);
}

Vector3d &Vector3d::operator+=(const Vector3d &other)
{
    x() += other.getX();
    y() += other.getY();
    z() += other.getZ();
    return *this;
}

Vector3d &Vector3d::operator-=(const Vector3d &other)
{
    x() -= other.getX();
    y() -= other.getY();
    z() -= other.getZ();
    return *this;
}

Vector3d &Vector3d::operator*=(const double ratio)
{
    x() *= ratio;
    y() *= ratio;
    z() *= ratio;
    return *this;
}

Vector3d &Vector3d::operator/=(const double ratio)
{
    constexpr double ep = 10e-10;
    if (std::abs(ratio) < ep)
    {
        std::cerr << __FILE__ << "(" << __LINE__ << ")"
                  << ":"
                  << "too small ratio" << std::endl;
    }
    x() /= ratio;
    y() /= ratio;
    z() /= ratio;
    return *this;
}

bool Vector3d::operator==(const Vector3d &other) const
{
    constexpr double ep = 10e-10;
    return (std::abs(getX() - other.getX()) < ep &&
            std::abs(getY() - other.getY()) < ep &&
            std::abs(getZ() - other.getZ()) < ep);
}

} // namespace amathutils