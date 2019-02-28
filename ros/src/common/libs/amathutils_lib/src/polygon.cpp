#include <amathutils_lib/polygon.hpp>
#include <iostream>

namespace amathutils
{
Polygon2d::Polygon2d() {}
Polygon2d::Polygon2d(const std::vector<Vector2d> &_v_point) : v_point_(_v_point)
{
    if (v_point_.front() == v_point_.back())
    {
        v_point_.push_back(v_point_.front());
    }
}

void Polygon2d::setPolygon(const std::vector<Vector2d> &v_point)
{
    v_point_ = v_point;
    if (v_point_.front() == v_point_.back())
    {
        // std::cerr << __FILE__ << "(" << __LINE__ << ")"
        //           << ":"
        //           << "First and last must match. Forced to match." << std::endl;
        v_point_.push_back(v_point_.front());
    }
}

bool Polygon2d::isInPolygon(const std::vector<Vector2d> &v_point, const Vector2d &point)
{
    // http://sampleyy.hatenablog.com/entry/2015/03/28/110152
    double signed_angle_sum = 0;
    for (size_t i = 0; i < v_point.size() - 1; ++i)
    {
        Vector2d vec1(v_point.at(i).getX() - point.getX(), v_point.at(i).getY() - point.getY());
        Vector2d vec2(v_point.at(i+1).getX() - point.getX(), v_point.at(i+1).getY() - point.getY());
        const double sined_angle = std::atan2(vec1.dot(vec2), vec1.cross(vec2));
        signed_angle_sum += sined_angle;
    }
    constexpr double ep = 0.001;
    if (fabs(2 * M_PI - fabs(signed_angle_sum)) < ep)
        return true;
    else
        return false;
}

bool Polygon2d::isInPolygon(const Vector2d &point)
{
    return isInPolygon(v_point_, point);
}
} // namespace amathutils