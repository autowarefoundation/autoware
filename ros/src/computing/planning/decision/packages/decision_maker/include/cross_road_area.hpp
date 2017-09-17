#ifndef __CROSS_ROAD_AREA_HPP
#define __CROSS_ROAD_AREA_HPP

#include <vector>

#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>

#include <autoware_msgs/lane.h>

namespace decision_maker
{
#define CROSS_ROAD 3232
class CrossRoadArea
{
public:
  int id;
  int area_id;
  std::vector<geometry_msgs::Point> points;

  jsk_recognition_msgs::BoundingBox bbox;

  CrossRoadArea(void)
  {
    id = 0;
    area_id = 0;
    points.clear();
  }

  static CrossRoadArea *findClosestCrossRoad(const autoware_msgs::lane &_finalwaypoints,
                                             std::vector<CrossRoadArea> &intersects);
};
}

#endif
