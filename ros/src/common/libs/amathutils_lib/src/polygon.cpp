#include <amathutils_lib/polygon.hpp>
#include <iostream>

namespace amathutils
{
Polygon2d::Polygon2d() {}
Polygon2d::Polygon2d(const std::vector<Vector2d> &_v_point) : v_point_(_v_point)
{
}

void Polygon2d::setPolygon(const std::vector<Vector2d> &v_point)
{
    v_point_ = v_point;
}

bool Polygon2d::isInPolygon(const std::vector<Vector2d> &v_point, const Vector2d &point, const bool boundary)
{
    if (v_point.size() < 3)
    {
        std::cerr << __FILE__ << "(" << __LINE__ << ")"
                  << ":"
                  << "Not enough polygon points." << std::endl;
        return false;
    }
    // http://sampleyy.hatenablog.com/entry/2015/03/28/110152
    double signed_angle_sum = 0;
    for (size_t i = 0; i < v_point.size(); ++i)
    {
        Vector2d vec1(v_point.at(i).getX() - point.getX(), v_point.at(i).getY() - point.getY());
        Vector2d vec2(v_point.at((i + 1) % v_point.size()).getX() - point.getX(), v_point.at((i + 1) % v_point.size()).getY() - point.getY());
        const double dot = vec1.dot(vec2);
        const double cross = vec1.cross(vec2);
        double signed_angle = std::atan2(cross, dot);
        constexpr double ep = 1.0e-10;
        // check boundary
        if (std::abs(std::abs(signed_angle) - M_PI) < ep)
        {
            return boundary;
            if (boundary)
                signed_angle = std::abs(signed_angle);
            else
                signed_angle = -std::abs(signed_angle);
        }
        // check corner
        if (std::abs(cross) < ep && std::abs(dot) < ep)
        {
            return boundary;
        }
        signed_angle_sum += signed_angle;
    }
    constexpr double ep = 1.0e-5;
    if (fabs(2 * M_PI - fabs(signed_angle_sum)) < ep)
        return true;
    else
        return false;
}

bool Polygon2d::isInPolygon(const Vector2d &point, const bool boundary)
{
    return isInPolygon(v_point_, point, boundary);
}
} // namespace amathutils