#pragma once

#include <amathutils_lib/vector2d.hpp>
#include <amathutils_lib/vector3d.hpp>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace amathutils
{
class KdTreeSearcher
{
  protected:
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

  public:
    KdTreeSearcher();
    KdTreeSearcher(const std::vector<Vector2d> &_v_vec);
    KdTreeSearcher(const std::vector<Vector3d> &_v_vec);
    virtual ~KdTreeSearcher(){};
    void setSource(const std::vector<Vector2d> &v_vec);
    void setSource(const std::vector<Vector3d> &v_vec);

    bool searchNearestNeighbor(const Vector2d &vec, int &index) const;
    bool searchNearestNeighbor(const Vector3d &vec, int &index) const;
    bool searchNearestK(const Vector2d &vec, const int k, std::vector<int> &v_index) const;
    bool searchNearestK(const Vector3d &vec, const int k, std::vector<int> &v_index) const;
    bool searchRadius(const Vector2d &vec, const double radius, std::vector<int> &v_index) const;
    bool searchRadius(const Vector3d &vec, const double radius, std::vector<int> &v_index) const;
};

} // namespace amathutils