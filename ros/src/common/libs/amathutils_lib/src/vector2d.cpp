#include <amathutils_lib/vector2d.hpp>
#include <iostream>

namespace amathutils
{
Vector2d::Vector2d() { vec_ = Eigen::Vector2d::Zero(); }
Vector2d::Vector2d(const double _x, const double _y) { vec_ << _x, _y; }
Vector2d::Vector2d(const Eigen::Vector2d &vec) : vec_(vec){};

Vector2d Vector2d::Zero() { return Vector2d(); }
Vector2d Vector2d::Identity() { return Vector2d(1.0, 1.0); }
Vector2d Vector2d::UnitX() { return Vector2d(1.0, 0.0); }
Vector2d Vector2d::UnitY() { return Vector2d(0.0, 1.0); }
Vector2d Vector2d::UnitFromAngle(const double angle)
{
    return Vector2d(std::cos(angle), std::sin(angle));
}

double Vector2d::getDistance(const Vector2d &_a, const Vector2d &_b)
{
    return (_a - _b).getL2norm();
}

double Vector2d::getDistance(const Vector2d &_a) const
{
    return getDistance(*this, _a);
}

double Vector2d::getAngle() const
{
    return std::atan2(getY(), getX());
}

double Vector2d::getL1norm() const { return getX() + getY(); }
double Vector2d::getL2norm() const { return vec_.norm(); }
double Vector2d::getX() const { return vec_.x(); }
double Vector2d::getY() const { return vec_.y(); }
double &Vector2d::x() { return vec_.x(); }
double &Vector2d::y() { return vec_.y(); }

double Vector2d::dot(const Vector2d &_a, const Vector2d &_b)
{
    Eigen::Vector2d vec1;
    vec1 << _a.getX(), _a.getY();
    Eigen::Vector2d vec2;
    vec2 << _b.getX(), _b.getY();
    return vec1.dot(vec2);
}

double Vector2d::dot(const Vector2d &_a) const
{
    return dot(*this, _a);
}

double Vector2d::cross(const Vector2d &_a, const Vector2d &_b)
{
    return _a.getX() * _b.getY() - _a.getY() * _b.getX();
}

double Vector2d::cross(const Vector2d &_a) const
{
    return cross(*this, _a);
}

Vector2d Vector2d::normalized(const Vector2d &_a)
{
    Eigen::Vector2d vec;
    vec << _a.getX(), _a.getY();
    return Vector2d(vec.normalized());
}

void Vector2d::normalize()
{
    return vec_.normalize();
}

Vector2d Vector2d::rotate(const double angle) const
{
    return Vector2d(getX() * std::cos(angle) - getY() * std::sin(angle),
                    getX() * std::sin(angle) + getY() * std::cos(angle));
}

Vector2d Vector2d::operator-(const Vector2d &other) const
{
    return Vector2d(getX() - other.getX(), getY() - other.getY());
}

Vector2d Vector2d::operator+(const Vector2d &other) const
{
    return Vector2d(getX() + other.getX(), getY() + other.getY());
}

Vector2d Vector2d::operator*(const double ratio) const
{
    return Vector2d(getX() * ratio, getY() * ratio);
}

Vector2d Vector2d::operator/(const double ratio) const
{
    constexpr double ep = 10e-10;
    if (std::abs(ratio) < ep)
    {
        std::cerr << __FILE__ << "(" << __LINE__ << ")"
                  << ":"
                  << "too small ratio" << std::endl;
    }
    return Vector2d(getX() / ratio, getY() / ratio);
}

Vector2d &Vector2d::operator+=(const Vector2d &other)
{
    x() += other.getX();
    y() += other.getY();
    return *this;
}

Vector2d &Vector2d::operator-=(const Vector2d &other)
{
    x() -= other.getX();
    y() -= other.getY();
    return *this;
}

Vector2d &Vector2d::operator*=(const double ratio)
{
    x() *= ratio;
    y() *= ratio;
    return *this;
}

Vector2d &Vector2d::operator/=(const double ratio)
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
    return *this;
}

bool Vector2d::operator==(const Vector2d &other) const
{
    constexpr double ep = 10e-10;
    return (std::abs(getX() - other.getX()) < ep &&
            std::abs(getY() - other.getY()) < ep);
}

} // namespace amathutils