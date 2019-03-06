#include "amathutils_lib/kdtree_search.hpp"
#include <iostream>

namespace amathutils
{
KdTreeSearcher::KdTreeSearcher() {}
KdTreeSearcher::KdTreeSearcher(const std::vector<Vector2d> &_v_vec)
{
    setSource(_v_vec);
}

KdTreeSearcher::KdTreeSearcher(const std::vector<Vector3d> &_v_vec)
{
    setSource(_v_vec);
}

void KdTreeSearcher::setSource(const std::vector<Vector2d> &v_vec)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < v_vec.size(); ++i)
    {
        const pcl::PointXYZ point(v_vec.at(i).getX(), v_vec.at(i).getY(), 0.0);
        point_cloud_ptr->push_back(point);
    }
    kdtree_.setInputCloud(point_cloud_ptr);
}

void KdTreeSearcher::setSource(const std::vector<Vector3d> &v_vec)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < v_vec.size(); ++i)
    {
        const pcl::PointXYZ point(v_vec.at(i).getX(), v_vec.at(i).getY(), v_vec.at(i).getZ());
        point_cloud_ptr->push_back(point);
    }
    kdtree_.setInputCloud(point_cloud_ptr);
}

bool KdTreeSearcher::searchNearestNeighbor(const Vector2d &vec, int &index) const
{
    Vector3d vec3d(vec.getX(), vec.getY(), 0.0);
    return searchNearestNeighbor(vec3d, index);
}

bool KdTreeSearcher::searchNearestNeighbor(const Vector3d &vec, int &index) const
{
    constexpr int k = 1;
    std::vector<int> v_index;
    if (searchNearestK(vec, k, v_index))
    {
        index = v_index.at(0);
        return true;
    }
    return false;
}

bool KdTreeSearcher::searchNearestK(const Vector2d &vec, const int k, std::vector<int> &v_index) const
{
    Vector3d vec3d(vec.getX(), vec.getY(), 0.0);
    return searchNearestK(vec3d, k, v_index);
}

bool KdTreeSearcher::searchNearestK(const Vector3d &vec, const int k, std::vector<int> &v_index) const
{
    std::vector<float> v_squared_dist;
    const pcl::PointXYZ point(vec.getX(), vec.getY(), vec.getZ());

    if (kdtree_.nearestKSearch(point, k, v_index, v_squared_dist) > 0)
    {
        return true;
    }
    return false;
}

bool KdTreeSearcher::searchRadius(const Vector2d &vec, const double radius, std::vector<int> &v_index) const
{
    Vector3d vec3d(vec.getX(), vec.getY(), 0.0);
    return searchRadius(vec3d, radius, v_index);
}

bool KdTreeSearcher::searchRadius(const Vector3d &vec, const double radius, std::vector<int> &v_index) const
{
    std::vector<float> v_squared_dist;
    const pcl::PointXYZ point(vec.getX(), vec.getY(), vec.getZ());

    if (kdtree_.radiusSearch(point, radius, v_index, v_squared_dist) > 0)
    {
        return true;
    }
    return false;
}

} // namespace amathutils