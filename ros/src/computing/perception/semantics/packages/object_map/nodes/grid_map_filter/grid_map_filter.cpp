#include "grid_map_filter.h"
#include "grid_map_filter_util.h"

namespace object_map
{

// Constructor
GridMapFilter::GridMapFilter() :
    private_nh_("~")
{
  initForROS();
  initForVMap();
}

// Destructor
GridMapFilter::~GridMapFilter()
{
}

void GridMapFilter::initForROS()
{
  // rosparam setting
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("map_topic", map_topic_, "/realtime_cost_map");
  private_nh_.param<double>("dist_transform_distance", dist_transform_distance_, 3.0);
  private_nh_.param<bool>("use_dist_transform", use_dist_transform_, false);
  private_nh_.param<bool>("use_wayarea", use_wayarea_, false);
  private_nh_.param<bool>("use_fill_circle", use_fill_circle_, false);
  private_nh_.param<int>("fill_circle_cost_threshold", fill_circle_cost_thresh_, 20);
  private_nh_.param<double>("circle_radius", circle_radius_, 1.7);

  // setup subscriber
  occupancy_grid_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic_, 10, &GridMapFilter::callbackFromOccupancyGrid, this);

  // setup publisher
  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("filtered_grid_map", 1, true);
}

// subscribe vector map infomation and create wayarea points
void GridMapFilter::initForVMap()
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

void GridMapFilter::run() const
{
  ros::spin();
}

void GridMapFilter::callbackFromOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg) const
{
  // timer start
  auto start = std::chrono::system_clock::now();

  ROS_INFO_STREAM("Subscribed Occupancy Grid Map");

  std::string original_layer = "original";
  static grid_map::GridMap map({original_layer, "distance_transform", "wayarea", "dist_wayarea", "circle"});
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "original", map);

  // apply distance transform to OccupancyGrid
  if (use_dist_transform_)
  {
    distanceTransform(map, original_layer);
  }

  // fill polygon
  if (!area_points_.empty() && use_wayarea_)
  {
    fillPolygonCV(map, area_points_);
    map["dist_wayarea"] = map["distance_transform"] + map["wayarea"];
  }

  // fill circle
  if (use_fill_circle_)
  {
    // OccupancyGrid (0~100) -> cv image (0~255)
    int cost_threshold = fill_circle_cost_thresh_ * (255.0 / 100.0);
    //double vehicle_width = 2.4;
    //int radius = (vehicle_width * std::sqrt(2.0) / 2.0) / map.getResolution();
    // convert to cv image size
    int radius = circle_radius_ / map.getResolution();
    fillCircleCV(map, original_layer, cost_threshold, radius);
  }

  // publish grid map as ROS message
  publishMap(map, grid_map_pub_);

  // timer end
  auto end = std::chrono::system_clock::now();
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;
}

void GridMapFilter::distanceTransform(grid_map::GridMap& map, const std::string& layer) const
{
  cv::Mat original_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, layer, CV_8UC1, original_image);

  // binalize
  cv::Mat binary_image;
  cv::threshold(original_image, binary_image, 20, 100, cv::THRESH_BINARY_INV);

  // distance transform
  // 3: fast
  // 5: slow but accurate
  cv::Mat dt_image;
  cv::distanceTransform(binary_image, dt_image, CV_DIST_L2, 5);

  // Convert to int...
  cv::Mat dt_int_image(dt_image.size(), CV_8UC1);
  cv::Mat dt_int_inv_image(dt_image.size(), CV_8UC1);

  // max distance for cost propagation
  double max_dist = dist_transform_distance_; // meter
  double resolution = map.getResolution();

  for (int y = 0; y < dt_image.rows; y++)
  {
    for (int x = 0; x < dt_image.cols; x++)
    {
      // actual distance [meter]
      double dist = dt_image.at<float>(y, x) * resolution;
      if (dist > max_dist)
        dist = max_dist;

      // Make value range 0 ~ 255
      int round_dist = dist / max_dist * 255;
      int inv_round_dist = 255 - round_dist;

      dt_int_image.at<unsigned char>(y, x)     = round_dist;
      dt_int_inv_image.at<unsigned char>(y, x) = inv_round_dist;
    }
  }

  // convert to ROS msg
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(dt_int_inv_image, "distance_transform", map, 0, 100);
  //grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(dt_int_inv_image, "dist_elevation", map, 0, 1.5);
}

// convert vector map area points to ros points
std::vector<geometry_msgs::Point> GridMapFilter::searchAreaPoints(const vector_map::Area& area, const vector_map::VectorMap& vmap) const
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
void GridMapFilter::fillPolygonCV(grid_map::GridMap& map, const std::vector<std::vector<geometry_msgs::Point>>& area_points_vec) const
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

// create configuration space from layer
void GridMapFilter::fillCircleCV(grid_map::GridMap& map, const std::string& layer, const double cost_threshold, const double radius) const
{
  map["circle"] = map[layer];
  cv::Mat original_image;

  // 0 ~ 100 -> 0 ~ 255
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "circle", CV_8UC1, 0, 100, original_image);

  cv::Mat filled_image = original_image.clone();

  for (int y = 0; y < original_image.rows; y++)
  {
    for (int x = 0; x < original_image.cols; x++)
    {
      // uchar -> int
      int data = original_image.at<unsigned char>(y, x);

      if (data > cost_threshold)
      {
        cv::circle(filled_image, cv::Point(x, y), radius, cv::Scalar(255), -1, CV_AA);
      }
    }
  }

  // convert to ROS msg
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(filled_image, "circle", map, 0, 100);
}


}  // namespace object_map
