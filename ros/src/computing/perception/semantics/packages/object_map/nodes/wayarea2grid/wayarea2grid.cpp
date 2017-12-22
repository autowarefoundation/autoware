#include "wayarea2grid.h"
#include "wayarea2grid_util.h"

namespace object_map
{

// Constructor
WayareaToGrid::WayareaToGrid() :
    nh_("~"),
    private_nh_("~")
{
  initForROS();
  initForVMap();
}

// Destructor
WayareaToGrid::~WayareaToGrid()
{
}

void WayareaToGrid::initForROS()
{
  // rosparam setting
  private_nh_.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<double>("map_resolution", map_resolution_, 0.2);
  private_nh_.param<double>("map_length_x", map_length_x_, 80);
  private_nh_.param<double>("map_length_y", map_length_y_, 30);
  private_nh_.param<double>("map_position_x", map_position_x_, 20);
  private_nh_.param<double>("map_position_y", map_position_y_, 0);

  // setup publisher
  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("wayarea_grid_map", 1, true);
}

// subscribe vector map infomation and create wayarea points
void WayareaToGrid::initForVMap()
{
  // subscribe vector map data
  // non blocking
  vector_map::VectorMap vmap;
  vmap.subscribe(nh_, vector_map::Category::POINT | vector_map::Category::LINE | vector_map::Category::AREA | vector_map::Category::WAY_AREA, ros::Duration(3.0));

  // all true -> all data
  std::vector<vector_map_msgs::WayArea> way_areas =
      vmap.findByFilter([](const vector_map_msgs::WayArea& way_area){return true;});

  if (way_areas.empty())
  {
    ROS_WARN_STREAM("No WayArea...");
    return;
  }

  for (const auto& way_area : way_areas)
  {
    vector_map_msgs::Area area = vmap.findByKey(vector_map::Key<vector_map::Area>(way_area.aid));
    area_points_.emplace_back(searchAreaPoints(area, vmap));
  }

}

void WayareaToGrid::run() const
{
  bool set_map = false;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    static grid_map::GridMap map({"wayarea"});
    if (!set_map)
    {
      map.setFrameId(sensor_frame_);
      map.setGeometry(grid_map::Length(map_length_x_, map_length_y_),
                      map_resolution_,
                      grid_map::Position(map_position_x_, map_position_y_));
      set_map = true;
    }

    // timer start
    auto start = std::chrono::system_clock::now();

    if (!area_points_.empty())
    {
      fillPolygonCV(map, area_points_);
    }

    publishMap(map, grid_map_pub_);

    // timer end
    auto end = std::chrono::system_clock::now();
    auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

    loop_rate.sleep();
  }
}


// convert vector map area points to ros points
std::vector<geometry_msgs::Point> WayareaToGrid::searchAreaPoints(const vector_map::Area& area, const vector_map::VectorMap& vmap) const
{
  std::vector<geometry_msgs::Point> area_points;
  std::vector<geometry_msgs::Point> area_points_empty;

  if (area.aid == 0)
    return area_points_empty;

  vector_map_msgs::Line line = vmap.findByKey(vector_map::Key<vector_map_msgs::Line>(area.slid));
  // must set beginning line
  if (line.lid == 0 || line.blid != 0)
    return area_points_empty;

  // Search all lines in area
  while (line.flid != 0)
  {
    vector_map_msgs::Point bp = vmap.findByKey(vector_map::Key<vector_map_msgs::Point>(line.bpid));
    if (bp.pid == 0)
      return area_points_empty;

    vector_map_msgs::Point fp = vmap.findByKey(vector_map::Key<vector_map_msgs::Point>(line.fpid));
    if (fp.pid == 0)
      return area_points_empty;

    // 2 points of line
    area_points.push_back(vector_map::convertPointToGeomPoint(bp));
    area_points.push_back(vector_map::convertPointToGeomPoint(fp));

    line = vmap.findByKey(vector_map::Key<vector_map_msgs::Line>(line.flid));
    if (line.lid == 0)
      return area_points_empty;
  }

  vector_map_msgs::Point bp = vmap.findByKey(vector_map::Key<vector_map_msgs::Point>(line.bpid));
  vector_map_msgs::Point fp = vmap.findByKey(vector_map::Key<vector_map_msgs::Point>(line.fpid));
  if (bp.pid == 0 || fp.pid == 0)
    return area_points_empty;

  area_points.push_back(vector_map::convertPointToGeomPoint(bp));
  area_points.push_back(vector_map::convertPointToGeomPoint(fp));

  return area_points;
}

// fill polygon area of OccupancyGrid in "layer"
void WayareaToGrid::fillPolygonCV(grid_map::GridMap& map, const std::vector<std::vector<geometry_msgs::Point>>& area_points_vec) const
{
  map["wayarea"].setConstant(100);

  cv::Mat original_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "wayarea", CV_8UC1, 0, 100, original_image);

  // deep copy
  cv::Mat filled_image = original_image.clone();

  // for obtaining vector map coordinate in velodyne frame
  tf::StampedTransform tf = findTransform(map.getFrameId(), map_frame_, tf_listener_);

  // calculate map position
  grid_map::Position map_pos = map.getPosition();
  double origin_x_offset = map.getLength().x() / 2.0 - map_pos.x();
  double origin_y_offset = map.getLength().y() / 2.0 - map_pos.y();

  for (const auto& points : area_points_vec)
  {
    std::vector<cv::Point> cv_points;

    for (const auto& p : points)
    {
      // transform to GridMap coordinate
      geometry_msgs::Point tf_point = transformPoint(p, tf);

      // coordinate conversion for cv image
      double cv_x = (map.getLength().y() - origin_y_offset - tf_point.y) / map.getResolution();
      double cv_y = (map.getLength().x() - origin_x_offset - tf_point.x) / map.getResolution();
      cv_points.emplace_back(cv::Point(cv_x, cv_y));
    }

    // fill polygon
    // NOTE: cv::Scalar(255, 255, 255) is white, and white means 100 in Occupancy Grid of ROS
    //       cv::Scalar(  0,   0,   0) -> 0
    //       cv::Scalar(0) is Okay... (1 channel)
    cv::fillConvexPoly(filled_image, cv_points.data(), cv_points.size(), cv::Scalar(0));
  }

  // convert to ROS msg
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(filled_image, "wayarea", map, 0, 100);
}


}  // namespace object_map
