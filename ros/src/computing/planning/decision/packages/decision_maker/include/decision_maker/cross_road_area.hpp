#ifndef __CROSS_ROAD_AREA_HPP
#define __CROSS_ROAD_AREA_HPP

#include <vector>

#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>

#include <autoware_msgs/Lane.h>

namespace decision_maker
{
class CrossRoadArea
{
public:
  int id;
  int area_id;
  std::vector<geometry_msgs::Point> points;
  jsk_recognition_msgs::BoundingBox bbox;

  std::vector<autoware_msgs::Lane> insideLanes;
  std::vector<geometry_msgs::Point> insideWaypoint_points;

  CrossRoadArea(void)
  {
    id = 0;
    area_id = 0;
    points.clear();
    insideLanes.clear();
    insideWaypoint_points.clear();
  }

  static CrossRoadArea* findClosestCrossRoad(const autoware_msgs::Lane& _finalwaypoints,
                                             std::vector<CrossRoadArea>& intersects);
  static bool isInsideArea(const CrossRoadArea* _TargetArea, geometry_msgs::Point pt);

  static CrossRoadArea* getCrossRoadArea(std::vector<CrossRoadArea>& areas, int aid)
  {
    CrossRoadArea* ret = nullptr;
    for (auto& area : areas)
    {
      if (area.area_id == aid)
      {
        ret = &area;
        break;
      }
    }
    return ret;
  }
};
}

#endif
