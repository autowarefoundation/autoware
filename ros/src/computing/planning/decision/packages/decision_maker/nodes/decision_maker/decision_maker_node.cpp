#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tf/transform_listener.h>

// lib
#include <euclidean_space.hpp>
#include <state.hpp>
#include <state_context.hpp>

#include <decision_maker_node.hpp>
//#include <vector_map/vector_map.h>

#include <autoware_msgs/lane.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// for dynamic reconfigure
//#include <dynamic_reconfigure/server.h>
//#include <decision_maker/decision_makerConfig.h>

namespace decision_maker
{
#define DOUBLE_MAX 1.7976931348623158e308
#define TARGET_WAYPOINTS_NUM 14

CrossRoadArea *DecisionMakerNode::findClosestCrossRoad(void)
{
  CrossRoadArea *_area = nullptr;

  euclidean_space::point _pa;
  euclidean_space::point _pb;

  double _min_distance = DBL_MAX;

  int _label = 1;

  if (!current_finalwaypoints_.waypoints.empty())
  {
    _pa.x = current_finalwaypoints_.waypoints[TARGET_WAYPOINTS_NUM].pose.pose.position.x;
    _pa.y = current_finalwaypoints_.waypoints[TARGET_WAYPOINTS_NUM].pose.pose.position.y;
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

bool DecisionMakerNode::isInsideArea(geometry_msgs::Point pt)
{
  // simply implementation
  //
  if (ClosestArea_ != nullptr)
  {
    double x1 = ClosestArea_->bbox.pose.position.x - (ClosestArea_->bbox.dimensions.x / 2);
    double x2 = ClosestArea_->bbox.pose.position.x + (ClosestArea_->bbox.dimensions.x / 2);

    double y1 = ClosestArea_->bbox.pose.position.y - (ClosestArea_->bbox.dimensions.y / 2);
    double y2 = ClosestArea_->bbox.pose.position.y + (ClosestArea_->bbox.dimensions.y / 2);

    if ((x1 <= pt.x && pt.x <= x2))
    {
      if (y1 <= pt.y && pt.y <= y2)
      {
        return true;
      }
    }
  }
  return false;
}

void DecisionMakerNode::initStateMsgs(void)
{
}

void DecisionMakerNode::initROS(int argc, char **argv)
{
  // status subscriber
  Subs["current_pose"] = nh_.subscribe("current_pose", 20, &DecisionMakerNode::callbackFromCurrentPose, this);
  Subs["current_velocity"] =
      nh_.subscribe("current_velocity", 20, &DecisionMakerNode::callbackFromCurrentVelocity, this);
  Subs["light_color"] = nh_.subscribe("light_color", 10, &DecisionMakerNode::callbackFromLightColor, this);
  Subs["points_raw"] = nh_.subscribe("filtered_points", 1, &DecisionMakerNode::callbackFromPointsRaw, this);
  Subs["final_waypoints"] = nh_.subscribe("final_waypoints", 100, &DecisionMakerNode::callbackFromFinalWaypoint, this);
  Subs["twist_cmd"] = nh_.subscribe("twist_cmd", 10, &DecisionMakerNode::callbackFromTwistCmd, this);

  // vector map subscriber
  Subs["vector_map_area"] =
      nh_.subscribe("/vector_map_info/area", 1, &DecisionMakerNode::callbackFromVectorMapArea, this);
  Subs["vector_map_point"] =
      nh_.subscribe("/vector_map_info/point", 1, &DecisionMakerNode::callbackFromVectorMapPoint, this);
  Subs["vector_map_line"] =
      nh_.subscribe("/vector_map_info/line", 1, &DecisionMakerNode::callbackFromVectorMapLine, this);
  Subs["vector_map_crossroad"] =
      nh_.subscribe("/vector_map_info/cross_road", 1, &DecisionMakerNode::callbackFromVectorMapCrossRoad, this);

  // pub
  Pubs["state"] = nh_.advertise<std_msgs::String>("state", 1);
  Pubs["state_overlay"] = nh_.advertise<jsk_rviz_plugins::OverlayText>("/state/overlay_text", 1);

  Pubs["state_local_diffdistance"] = nh_.advertise<std_msgs::Float64>("/state/val_diff_distance", 1);

  Pubs["crossroad_visual"] = nh_.advertise<visualization_msgs::MarkerArray>("/state/cross_road_marker", 1);
  Pubs["crossroad_inside_visual"] = nh_.advertise<visualization_msgs::Marker>("/state/cross_inside_marker", 1);
  Pubs["crossroad_bbox"] = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/state/bbox", 10);

#if 0
  // dynamic reconfigure
  dynamic_reconfigure::Server<decision_maker::decision_makerConfig> dr_server;
  dynamic_reconfigure::Server<decision_maker::decision_makerConfig>::CallbackType dr_server_f;

  dr_server_f = boost::bind(&callbackFromDynamicReconfigure, _1, _2);
  dr_server.setCallback(dr_server_f);
#endif
  // message setup
  state_text_msg.text_size = 18;
  state_text_msg.line_width = 0;
  state_text_msg.font = "DejaVu Sans Mono";
  state_text_msg.width = 500;
  state_text_msg.height = 50;
  state_text_msg.top = 10;
  state_text_msg.left = 10;
  state_text_msg.text = "UNDEFINED";

  // initial publishing state message
  update_msgs();

  // to move initial state from start state
  // this part confirm broadcasting tf(map to world)
  {
    std::cout << "wait for tf of map to world" << std::endl;
    tf::TransformListener tf;
    tf.waitForTransform("map", "world", ros::Time(), ros::Duration(999));

    if (!ctx->TFInitialized())
      std::cerr << "failed initialization " << std::endl;
  }
  {
    initVectorMapClient();
    displayMarker();
  }
}

void DecisionMakerNode::run(void)
{
  ros::Rate loop_rate(0.3);

  // for subscribe callback function
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // ros::MultiThreadedSpinner spinner(2);
  // spinner.spin();
  while (ros::ok())
  {
    update();
    displayMarker();
    loop_rate.sleep();
  }
}

void DecisionMakerNode::update_pubsub(void)
{
  // if state machine require to re-subscribe topic,
  // this function will re-definition subscriber.
}

void DecisionMakerNode::initVectorMap(void)
{
  if (!vector_map_init && vMap_Areas_flag & vMap_Points_flag & vMap_Lines_flag & vMap_CrossRoads_flag)
  {
    vector_map_init = true;

    int _index = 0;

    for (const auto &cross_road : vMap_CrossRoads.data)
    {
      for (const auto &area : vMap_Areas.data)
      {
        if (cross_road.aid == area.aid)
        {
          CrossRoadArea carea;
          carea.id = _index++;
          carea.area_id = area.aid;

          double x_avg = 0.0, x_min = 0.0, x_max = 0.0;
          double y_avg = 0.0, y_min = 0.0, y_max = 0.0;
          double z = 0.0;

          int points_count = 0;
          for (const auto &line : vMap_Lines.data)
          {
            if (area.slid <= line.lid && line.lid <= area.elid)
            {
              for (const auto &point : vMap_Points.data)
              {
                if (line.fpid <= point.pid && point.pid <= line.fpid)
                {
                  geometry_msgs::Point _point;
                  _point.x = point.ly;
                  _point.y = point.bx;
                  _point.z = point.h;

                  x_avg += _point.x;
                  y_avg += _point.y;

                  x_min = (x_min == 0.0) ? _point.x : std::min(_point.x, x_min);
                  x_max = (x_max == 0.0) ? _point.x : std::max(_point.x, x_max);
                  y_min = (y_min == 0.0) ? _point.y : std::min(_point.y, y_min);
                  y_max = (y_max == 0.0) ? _point.y : std::max(_point.y, y_max);
                  z = _point.z;
                  points_count++;

                  carea.points.push_back(_point);
                }  // if pid
              }    // points iter
            }      // if lid
          }        // line iter
          carea.bbox.pose.position.x = x_avg / (double)points_count;
          carea.bbox.pose.position.y = y_avg / (double)points_count;
          carea.bbox.pose.position.z = z;
          carea.bbox.dimensions.x = x_max - x_min;
          carea.bbox.dimensions.y = y_max - y_min;
          carea.bbox.dimensions.z = 2;
          carea.bbox.label = 1;
          intersects.push_back(carea);
        }
      }
    }
  }
}

void DecisionMakerNode::displayMarker(void)
{
  // vector_map init
  // parse vectormap
  initVectorMap();

  jsk_recognition_msgs::BoundingBoxArray bbox_array;

  static visualization_msgs::MarkerArray marker_array;
  static visualization_msgs::Marker crossroad_marker;
  static visualization_msgs::Marker inside_marker;

  crossroad_marker.header.frame_id = "/map";
  crossroad_marker.header.stamp = ros::Time();
  crossroad_marker.id = 1;
  crossroad_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crossroad_marker.action = visualization_msgs::Marker::ADD;
  crossroad_marker.ns = "crossroad";

  double scale = 3.0;
  crossroad_marker.scale.x = scale;
  crossroad_marker.scale.y = scale;
  crossroad_marker.scale.z = 0.5;
  crossroad_marker.color.a = 0.15;
  crossroad_marker.color.r = 1.0;
  crossroad_marker.color.g = 0.0;
  /// www.sinet.ad.jp/aboutsinettd::cout << "x: "<< _point.x << std::endl;
  crossroad_marker.color.b = 0.0;
  crossroad_marker.frame_locked = true;
  crossroad_marker.lifetime = ros::Duration(0.3);

  inside_marker = crossroad_marker;
  inside_marker.color.a = 0.5;
  inside_marker.color.r = 1.0;
  inside_marker.color.g = 1.0;
  inside_marker.color.b = 0.0;
  inside_marker.ns = "inside";
  inside_marker.lifetime = ros::Duration();

  bbox_array.header = crossroad_marker.header;

  for (auto &area : intersects)
  {
    for (const auto &p : area.points)
    {
      // if(isInsideArea(p))
      // inside_marker.points.push_back(p);
      crossroad_marker.points.push_back(p);
    }
    area.bbox.header = crossroad_marker.header;
    bbox_array.boxes.push_back(area.bbox);
  }

  Pubs["crossroad_bbox"].publish(bbox_array);
  bbox_array.boxes.clear();

  // marker_array.markers.push_back(inside_marker);
  marker_array.markers.push_back(crossroad_marker);

  Pubs["crossroad_visual"].publish(marker_array);

  for (const auto &p : inside_points_)
    inside_marker.points.push_back(p);

  Pubs["crossroad_inside_visual"].publish(inside_marker);

  marker_array.markers.clear();
}

void DecisionMakerNode::update_msgs(void)
{
  if (ctx)
  {
    static std::string prevStateName;
    CurrentStateName = *ctx->getCurrentStateName();

    if (prevStateName != CurrentStateName)
    {
      prevStateName = CurrentStateName;
      update_pubsub();
    }

    state_string_msg.data = CurrentStateName;
    state_text_msg.text = CurrentStateName + "\n" + TextOffset;

    Pubs["state"].publish(state_string_msg);
    Pubs["state_overlay"].publish(state_text_msg);
  }
  else
    std::cerr << "ctx is not found " << std::endl;
}

bool DecisionMakerNode::initVectorMapClient()
{
#ifdef USE_VMAP_SERVER  // This is not successfully run due to failing vmap
  // server

  vector_map::VectorMap vmap;
  vmap.subscribe(nh_, vector_map::Category::AREA, ros::Duration(0));

  cross_road_srv.request.pose = current_pose_;
  cross_road_srv.request.waypoints = current_finalwaypoints_;

  cross_road_cli = nh_.serviceClient<vector_map_server::GetCrossRoad>("vector_map_server/get_cross_road");

  return cross_road_cli.call(cross_road_srv);
#endif
}

bool DecisionMakerNode::isCrossRoadByVectorMapServer(const autoware_msgs::lane &lane_msg,
                                                    const geometry_msgs::PoseStamped &pose_msg)
{
#ifdef USE_VMAP_SERVER  // this is not successfully run
  cross_road_srv.request.pose = pose_msg;
  cross_road_srv.request.waypoints.waypoints.clear();
  std::cout << "test" << std::endl;

  for (int i = 0; i < 50; i++)
  {
    cross_road_srv.request.waypoints.waypoints.push_back(lane_msg.waypoints[i]);
  }
  for (const auto &wayp : lane_msg.waypoints)
    cross_road_srv.request.waypoints.waypoints.push_back(wayp);

  cross_road_cli.call(cross_road_srv);

  for (const auto &cross_road_d : cross_road_srv.response.objects.data)
  {
    std::cout << "EEEEEEEEE" << cross_road_d.linkid << std::endl;
  }
#else
#endif
}

double DecisionMakerNode::calcIntersectWayAngle(const autoware_msgs::lane &lane_msg,
                                               const geometry_msgs::PoseStamped &pose_msg)
{
  if (vMap_CrossRoads_flag)
  {
    int FirstPoint = 0;
    int EndPoint = 0;
    int index = 0;
    int PrevPoint = 0;
    double diff = 0.0;
    inside_points_.clear();
    for (int index = 0; index < lane_msg.waypoints.size(); index++)
    {
      if (isInsideArea(lane_msg.waypoints[index].pose.pose.position))
      {
        if (!FirstPoint)
          FirstPoint = index;

        inside_points_.push_back(lane_msg.waypoints[index].pose.pose.position);
      }
      else if (FirstPoint && !EndPoint)
      {
        EndPoint = PrevPoint;
        break;
      }
      PrevPoint = index;
    }

    if (EndPoint == 0)
    {
      std::cerr << "Not inside Cross Road" << std::endl;
    }
    else
    {
      geometry_msgs::Pose _end_point;
      _end_point = lane_msg.waypoints[EndPoint].pose.pose;

      double r, p, y, _y;

      tf::Quaternion quat_end(_end_point.orientation.x, _end_point.orientation.y, _end_point.orientation.z,
                              _end_point.orientation.w);
      tf::Quaternion quat_in(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z,
                             pose_msg.pose.orientation.w);

      tf::Matrix3x3(quat_end).getRPY(r, p, y);
      tf::Matrix3x3(quat_in).getRPY(r, p, _y);

      diff = std::floor(_y - y) * 180.0 / M_PI;
#if 1
      /* DEBUG */
      std::cout << "Yaw:" << _y << "-" << y << ":" << _y - y << std::endl;
      if (diff > 50)
      {
        std::cout << "Right Turn!!!!!!" << diff << std::endl;
      }
      else if (diff < -50)
      {
        std::cout << "Left Turn!!!!!!" << diff << std::endl;
      }
      else
      {
        std::cout << "Straight" << diff << std::endl;
      }
      std::cout << "Size:" << lane_msg.waypoints.size() << ":"
                << "First Point = " << FirstPoint << "/ End Point = " << EndPoint << std::endl;
#endif
    }

    return diff;
  }
  else
  {
    return 0.0;
  }
}

void DecisionMakerNode::update(void)
{
  update_msgs();
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "decision_maker");
  decision_maker::DecisionMakerNode smn(argc, argv);
  smn.run();

  return 0;
}
