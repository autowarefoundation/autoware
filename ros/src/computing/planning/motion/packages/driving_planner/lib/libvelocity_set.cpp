#include "libvelocity_set.h"



// extract edge points from zebra zone
std::vector<geometry_msgs::Point> RemoveNeedlessPoints(std::vector<geometry_msgs::Point> &area_points)
{
  area_points.push_back(area_points.front());
  std::map<double, int> length_index;
  for (unsigned int i = 0; i < area_points.size()-1; i++)
    length_index[CalcSquareOfLength(area_points[i], area_points[i+1])] = i;

  std::vector<geometry_msgs::Point> new_points;
  auto it = length_index.end();
  int first = (--it)->second;
  int second = (--it)->second;
  new_points.push_back(area_points[first]);
  new_points.push_back(area_points[first+1]);
  new_points.push_back(area_points[second]);
  new_points.push_back(area_points[second+1]);

  return new_points;
}


void CrossWalk::CrossWalkCallback(const map_file::CrossWalkArray &msg)
{

  crosswalk_ = msg;

  loaded_crosswalk = true;
  if (loaded_crosswalk && loaded_areaclass &&
      loaded_lineclass && loaded_pointclass) {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }

}

void CrossWalk::AreaclassCallback(const map_file::AreaClassArray &msg)
{

  areaclass_ = msg;

  loaded_areaclass = true;
  if (loaded_crosswalk && loaded_areaclass &&
      loaded_lineclass && loaded_pointclass) {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }
}

void CrossWalk::LineclassCallback(const map_file::LineClassArray &msg)
{

  lineclass_ = msg;

  loaded_lineclass = true;
  if (loaded_crosswalk && loaded_areaclass &&
      loaded_lineclass && loaded_pointclass) {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }

}

void CrossWalk::PointclassCallback(const map_file::PointClassArray &msg)
{

  pointclass_ = msg;

  loaded_pointclass = true;
  if (loaded_crosswalk && loaded_areaclass &&
      loaded_lineclass && loaded_pointclass) {
    loaded_all = true;
    ROS_INFO("All VectorMap loaded");
  }

}


geometry_msgs::Point CrossWalk::getPoint(const int &pid) const
{
  geometry_msgs::Point point;
  for (const auto &p : pointclass_.point_classes) {
    if (p.pid == pid) {
      point.x = p.ly;
      point.y = p.bx;
      point.z = p.h;
      return point;
    }
  }

  ROS_ERROR("can't find a point of pid %d", pid);
  return point;
}

geometry_msgs::Point CrossWalk::calcCenterofGravity(const int &aid) const
{
  int search_lid = -1;
  for (const auto &area : areaclass_.area_classes)
    if (area.aid == aid) {
      search_lid = area.slid;
      break;
    }

  std::vector<geometry_msgs::Point> area_points;
  while (search_lid) {
    for (const auto &line : lineclass_.line_classes) {
      if (line.lid == search_lid) {
	area_points.push_back(getPoint(line.bpid));
	search_lid = line.flid;
      }
    }
  }

  geometry_msgs::Point point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  if (area_points.size() > 4) {
    std::vector<geometry_msgs::Point> filterd_points = RemoveNeedlessPoints(area_points);
    for (const auto &p : filterd_points) {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
  } else {
    for (const auto &p : area_points) {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
  }

  point.x /= 4;
  point.y /= 4;
  point.z /= 4;
  return point;
}

double CrossWalk::calcCrossWalkWidth(const int &aid) const
{
  int search_lid = -1;
  for (const auto &area : areaclass_.area_classes)
    if (area.aid == aid) {
      search_lid = area.slid;
      break;
    }

  std::vector<geometry_msgs::Point> area_points;
  while (search_lid) {
    for (const auto &line : lineclass_.line_classes) {
      if (line.lid == search_lid) {
	area_points.push_back(getPoint(line.bpid));
	//_points.push_back(area_points.back());///
	search_lid = line.flid;
      }
    }
  }

  area_points.push_back(area_points.front());
  double max_length = CalcSquareOfLength(area_points[0], area_points[1]);
  for (unsigned int i = 1; i < area_points.size()-1; i++) {
    if (CalcSquareOfLength(area_points[i], area_points[i+1]) > max_length)
      max_length = CalcSquareOfLength(area_points[i], area_points[i+1]);
  }

  return sqrt(max_length);
}

// count the number of crosswalks
int CrossWalk::countAreaSize() const
{
  int count = 0;
  for (const auto &x : crosswalk_.cross_walks)
    if (x.bdid == 0)
      count++;

  return count;
}

void CrossWalk::getAID(std::vector< std::vector<int> > &aid_crosswalk) const
{
  for (const auto &x : crosswalk_.cross_walks)
    if (x.bdid > 0) // if it is not outer frame
      aid_crosswalk[x.bdid].push_back(x.aid); // save area id
}

void CrossWalk::calcDetectionArea(const std::vector< std::vector<int> > &aid_crosswalk)
{
  for (unsigned int bdid = 1; bdid < aid_crosswalk.size(); bdid++) {
    double width = 0.0;
    for (const auto &aid : aid_crosswalk[bdid]) {
      detection_points_[bdid].points.push_back(calcCenterofGravity(aid));
      width += calcCrossWalkWidth(aid);
    }
    width /= aid_crosswalk[bdid].size();
    detection_points_[bdid].width = width;
  }
}

void CrossWalk::calcCenterPoints()
{
  for (unsigned int i = 1; i < detection_points_.size(); i++) {
    geometry_msgs::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = 0.0;
    for (const auto &p : detection_points_[i].points) {
      center.x += p.x;
      center.y += p.y;
      center.z += p.z;
    }
    center.x /= detection_points_[i].points.size();
    center.y /= detection_points_[i].points.size();
    center.z /= detection_points_[i].points.size();
    detection_points_[i].center = center;
  }
}


void CrossWalk::setCrossWalkPoints()
{
  int crosswalk_size = countAreaSize();

  // aid_crosswalk[BDID] has AIDs of its zebra zone
  std::vector< std::vector<int> > aid_crosswalk(crosswalk_size+1);
  getAID(aid_crosswalk);

  detection_points_.resize(crosswalk_size+1);
  calcDetectionArea(aid_crosswalk);
  calcCenterPoints();

  ROS_INFO("Set cross walk detection points");
  set_points = true;
}


geometry_msgs::Point ObstaclePoints::getObstaclePoint(const EControl &kind)
{
    geometry_msgs::Point point;
    decided_ = false;

    if (kind == STOP) {
      for (const auto &p : stop_points_) {
	point.x += p.x;
	point.y += p.y;
	point.z += p.z;
      }
      point.x /= stop_points_.size();
      point.y /= stop_points_.size();
      point.z /= stop_points_.size();


      previous_detection_ = point;
      return point;
    } else if (kind == DECELERATE) {
      for (const auto &p : decelerate_points_) {
	point.x += p.x;
	point.y += p.y;
	point.z += p.z;
      }
      point.x /= decelerate_points_.size();
      point.y /= decelerate_points_.size();
      point.z /= decelerate_points_.size();

      previous_detection_ = point;
      return point;
    } else {
      return previous_detection_;
    }
}
