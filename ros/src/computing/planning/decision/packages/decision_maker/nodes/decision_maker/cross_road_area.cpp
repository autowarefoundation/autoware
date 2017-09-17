#include <cross_road_area.hpp>
#include <euclidean_space.hpp>

namespace decision_maker
{
#define TARGET_WAYPOINTS_NUM 15  // need to change rosparam
CrossRoadArea *CrossRoadArea::findClosestCrossRoad(const autoware_msgs::lane &_finalwaypoints,
                                                   std::vector<CrossRoadArea> &intersects)
{
  CrossRoadArea *_area = nullptr;

  euclidean_space::point _pa;
  euclidean_space::point _pb;

  double _min_distance = DBL_MAX;

  int _label = 1;

  if (!_finalwaypoints.waypoints.empty())
  {
    _pa.x = _finalwaypoints.waypoints[TARGET_WAYPOINTS_NUM].pose.pose.position.x;
    _pa.y = _finalwaypoints.waypoints[TARGET_WAYPOINTS_NUM].pose.pose.position.y;
    _pa.z = 0.0;
  }

  for (size_t i = 0; i < intersects.size(); i++)
  {
    _pb.x = intersects[i].bbox.pose.position.x;
    _pb.y = intersects[i].bbox.pose.position.y;

    _pb.z = 0.0;

    double __temp_dis = euclidean_space::EuclideanSpace::find_distance(&_pa, &_pb);

    intersects[i].bbox.label = 0;
    if (_min_distance >= __temp_dis)
    {
      _area = &intersects[i];
      _min_distance = __temp_dis;  //
    }
  }

  if (_area)
  {
    _area->bbox.label = 3;
  }

  return _area;
}
}
