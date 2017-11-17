#include <float.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include "autoware_msgs/obj_label.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include <math.h>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector_map/vector_map.h>
#include <vector_map_server/GetLane.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using vector_map::Node;
using vector_map::Point;
using vector_map::Key;

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1;
static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1;
static constexpr bool ADVERTISE_LATCH = false;
static constexpr double LOOP_RATE = 15.0;

ros::Publisher obj_pose_pub;
ros::Publisher obj_pose_timestamp_pub;
ros::Publisher cluster_class_pub;
ros::Publisher marker_array_pub;

static std::string object_type;
static std::vector<geometry_msgs::Point> centroids;
static std_msgs::Header sensor_header;
static std::vector<autoware_msgs::CloudCluster> v_cloud_cluster;
static ros::Time obj_pose_timestamp;
static double threshold_min_dist;

static vector_map::VectorMap vmap;
static ros::ServiceClient vmap_server;
static bool use_vmap;
static double vmap_threshold;

struct obj_label_t
{
  std::vector<geometry_msgs::Point> reprojected_positions;
  std::vector<int> obj_id;
};
obj_label_t obj_label;

enum ObjLabel
{
  Car,
  Person,
  Unknown
};

static double euclid_distance(const geometry_msgs::Point pos1, const geometry_msgs::Point pos2)
{
  return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) + pow(pos1.z - pos2.z, 2));

} /* static double distance() */

/* fusion reprojected position and pointcloud centroids */
void fusion_cb(const autoware_msgs::obj_label::ConstPtr &obj_label_msg,
               const autoware_msgs::CloudClusterArray::ConstPtr &in_cloud_cluster_array_ptr)
{
  tf::StampedTransform tform;
  tf::TransformListener tflistener;
  try
  {
    ros::Time latest_time = ros::Time(0);
    tflistener.waitForTransform("/map", "/velodyne", latest_time, ros::Duration(10));
    tflistener.lookupTransform("/map", "/velodyne", latest_time, tform);
  }
  catch (tf::TransformException ex)
  {
    ROS_INFO("%s: %s", __FUNCTION__, ex.what());
    return;
  }

  obj_label_t obj_label;
  object_type = obj_label_msg->type;
  obj_pose_timestamp = obj_label_msg->header.stamp;

  for (unsigned int i = 0; i < obj_label_msg->obj_id.size(); ++i)
  {
    obj_label.reprojected_positions.push_back(obj_label_msg->reprojected_pos.at(i));
    obj_label.obj_id.push_back(obj_label_msg->obj_id.at(i));
  }

  std::vector<autoware_msgs::CloudCluster> v_cloud_cluster;
  std_msgs::Header header = sensor_header;
  std::vector<geometry_msgs::Point> centroids;

  for (int i(0); i < (int)in_cloud_cluster_array_ptr->clusters.size(); ++i)
  {
    autoware_msgs::CloudCluster cloud_cluster = in_cloud_cluster_array_ptr->clusters.at(i);
    /* convert centroids coodinate from velodyne frame to map frame */
    tf::Vector3 pt(cloud_cluster.centroid_point.point.x, cloud_cluster.centroid_point.point.y,
                   cloud_cluster.centroid_point.point.z);
    tf::Vector3 converted = tform * pt;
    sensor_header = cloud_cluster.header;
    v_cloud_cluster.push_back(cloud_cluster);
    geometry_msgs::Point point_in_map;
    point_in_map.x = converted.x();
    point_in_map.y = converted.y();
    point_in_map.z = converted.z();

    centroids.push_back(point_in_map);
  }

  if (centroids.empty() || obj_label.reprojected_positions.empty() || obj_label.obj_id.empty())
  {
    jsk_recognition_msgs::BoundingBoxArray pub_msg;
    pub_msg.header = header;
    std_msgs::Time time;
    obj_pose_pub.publish(pub_msg);
    autoware_msgs::CloudClusterArray cloud_clusters_msg;
    cloud_clusters_msg.header = header;
    cluster_class_pub.publish(cloud_clusters_msg);
    visualization_msgs::MarkerArray marker_array_msg;
    marker_array_pub.publish(marker_array_msg);
    time.data = obj_pose_timestamp;
    obj_pose_timestamp_pub.publish(time);
    return;
  }

  std::vector<int> obj_indices;
  for (unsigned int i = 0; i < obj_label.obj_id.size(); ++i)
  {
    unsigned int min_idx = 0;
    double min_distance = DBL_MAX;

    /* calculate each euclid distance between reprojected position and centroids
     */
    for (unsigned int j = 0; j < centroids.size(); ++j)
    {
      double distance = euclid_distance(obj_label.reprojected_positions.at(i), centroids.at(j));

      /* Nearest centroid correspond to this reprojected object */
      if (distance < min_distance)
      {
        min_distance = distance;
        min_idx = j;
      }
    }
    if (min_distance < threshold_min_dist)
    {
      obj_indices.push_back(min_idx);
    }
    else
    {
      obj_indices.push_back(-1);
    }
  }

  /* Publish marker with centroids coordinates */
  jsk_recognition_msgs::BoundingBoxArray pub_msg;
  pub_msg.header = header;
  autoware_msgs::CloudClusterArray cloud_clusters_msg;
  cloud_clusters_msg.header = header;
  visualization_msgs::MarkerArray marker_array_msg;
  int marker_id = 0;

  for (unsigned int i = 0; i < obj_label.obj_id.size(); ++i)
  {
    if (obj_indices.at(i) == -1)
    {
      continue;
    }

    v_cloud_cluster.at(obj_indices.at(i)).label = object_type;
    if (object_type == "car")
    {
      v_cloud_cluster.at(obj_indices.at(i)).bounding_box.label = Car;
    }
    else if (object_type == "person")
    {
      v_cloud_cluster.at(obj_indices.at(i)).bounding_box.label = Person;
    }
    else
    {
      v_cloud_cluster.at(obj_indices.at(i)).bounding_box.label = Unknown;
      v_cloud_cluster.at(obj_indices.at(i)).label = "unknown";
    }

    jsk_recognition_msgs::BoundingBox bounding_box;
    bounding_box = v_cloud_cluster.at(obj_indices.at(i)).bounding_box;

    /* adjust object rotation using lane in vector_map */
    tf::Quaternion q_obj(bounding_box.pose.orientation.x, bounding_box.pose.orientation.y,
                         bounding_box.pose.orientation.z, bounding_box.pose.orientation.w);
    bool fixed_rotation = false;
    if (use_vmap)
    {
      int rot_n = 0;  // yaw' = yaw + n*pi/2
      vector_map_server::GetLane get_lane;
      get_lane.request.pose.pose = bounding_box.pose;
      tf::Vector3 orgpt(bounding_box.pose.position.x, bounding_box.pose.position.y, bounding_box.pose.position.z);
      tf::Vector3 convpt = tform * orgpt;
      get_lane.request.pose.pose.position.x = convpt.x();
      get_lane.request.pose.pose.position.y = convpt.y();
      get_lane.request.pose.pose.position.z = convpt.z();
      ROS_INFO("pos x=%f y=%f z=%f", get_lane.request.pose.pose.position.x, get_lane.request.pose.pose.position.y,
               get_lane.request.pose.pose.position.z);
      if (vmap_server.call(get_lane))
      {
        for (const auto &lane : get_lane.response.objects.data)
        {
          Node bn = vmap.findByKey(Key<Node>(lane.bnid));
          Point bp = vmap.findByKey(Key<Point>(bn.pid));
          Node fn = vmap.findByKey(Key<Node>(lane.fnid));
          Point fp = vmap.findByKey(Key<Point>(fn.pid));
          ROS_INFO(" lane bn=(%f,%f) fn=(%f,%f)", bp.ly, bp.bx, fp.ly, fp.bx);
          double mx = get_lane.request.pose.pose.position.x;
          double my = get_lane.request.pose.pose.position.y;
          if ((mx - fp.ly) * (mx - fp.ly) + (my - fp.bx) * (my - fp.bx) < vmap_threshold)
          {
            fixed_rotation = true;
            tf::Quaternion qm_lane;                                     // map-cood
            qm_lane.setRPY(0, 0, atan2(fp.bx - bp.bx, fp.ly - bp.ly));  // y,x
            tf::Quaternion q_step;                                      // 90 deg
            q_step.setRPY(0, 0, M_PI / 2);
            double r_max = M_PI;
            // search in 0,90,180,270-degree
            tf::Quaternion qr_obj = q_obj;  // rotated
            for (int i = 0; i < 4; ++i)
            {
              tf::Quaternion qrm_obj = tform * qr_obj;  // map-cood
              double r = qm_lane.angle(qrm_obj);
              r = (r >= M_PI / 2) ? (r - M_PI) : r;
              if (fabs(r) < r_max)
              {
                r_max = fabs(r);
                rot_n = i;
              }
              qr_obj *= q_step;
            }
            double roll, pitch, yaw;
            tf::Matrix3x3(q_obj).getRPY(roll, pitch, yaw);
            ROS_INFO(" %d roll=%f pitch=%f yaw=%f", rot_n * 90, roll, pitch, yaw);
            break;
          }
        }
      }
      else
      {
        ROS_INFO("%s: VectorMap Server call failed.", __FUNCTION__);
      }
      // determine rotation
      tf::Quaternion dq_obj;
      dq_obj.setRPY(0, 0, M_PI * rot_n / 2);
      q_obj *= dq_obj;
      // bounding_box
      bounding_box.pose.orientation.x = q_obj.x();
      bounding_box.pose.orientation.y = q_obj.y();
      bounding_box.pose.orientation.z = q_obj.z();
      bounding_box.pose.orientation.w = q_obj.w();
      if (rot_n % 2 == 1) // swap x-y at 90,270 deg
      {
        std::swap(bounding_box.dimensions.x, bounding_box.dimensions.y);
      }
      // cloud clusters
      v_cloud_cluster.at(obj_indices.at(i)).bounding_box = bounding_box;
    }

    // x-axis
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.id = marker_id++;
    marker.lifetime = ros::Duration(0.1);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose.position = bounding_box.pose.position;
    marker.pose.orientation = bounding_box.pose.orientation;
    marker.scale.x = 2.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker_array_msg.markers.push_back(marker);

    // y-axis
    tf::Quaternion qy;
    qy.setRPY(0, 0, M_PI / 2);
    q_obj *= qy;
    marker.id = marker_id++;
    marker.pose.orientation.x = q_obj.x();
    marker.pose.orientation.y = q_obj.y();
    marker.pose.orientation.z = q_obj.z();
    marker.pose.orientation.w = q_obj.w();
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker_array_msg.markers.push_back(marker);

    // z-axis
    tf::Quaternion qz;
    qz.setRPY(0, -M_PI / 2, 0);
    q_obj *= qz;
    marker.id = marker_id++;
    marker.pose.orientation.x = q_obj.x();
    marker.pose.orientation.y = q_obj.y();
    marker.pose.orientation.z = q_obj.z();
    marker.pose.orientation.w = q_obj.w();
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker_array_msg.markers.push_back(marker);

    // rotated by lane
    if (fixed_rotation)
    {
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker_array_msg.markers.push_back(marker);
    }

    v_cloud_cluster.at(obj_indices.at(i)).bounding_box.value = obj_label.obj_id.at(i);
    pub_msg.boxes.push_back(bounding_box);
    cloud_clusters_msg.clusters.push_back(v_cloud_cluster.at(obj_indices.at(i)));
  }

  marker_array_pub.publish(marker_array_msg);
  obj_pose_pub.publish(pub_msg);
  cluster_class_pub.publish(cloud_clusters_msg);
  std_msgs::Time time;
  time.data = obj_pose_timestamp;
  obj_pose_timestamp_pub.publish(time);
}

int main(int argc, char *argv[])
{
  /* ROS initialization */
  ros::init(argc, argv, "obj_fusion");

  ros::NodeHandle n;
  ros::NodeHandle private_n("~");

  private_n.param("min_dist", threshold_min_dist, 2.0);
  private_n.param("use_vmap", use_vmap, true);
  private_n.param("vmap_threshold", vmap_threshold, 5.0);
  vmap_threshold *= vmap_threshold;  // squared

  typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::obj_label, autoware_msgs::CloudClusterArray>
      SyncPolicy;
  message_filters::Subscriber<autoware_msgs::obj_label> obj_label_sub(n, "obj_label", SUBSCRIBE_QUEUE_SIZE);
  message_filters::Subscriber<autoware_msgs::CloudClusterArray> cluster_centroids_sub(n, "/cloud_clusters",
                                                                                      SUBSCRIBE_QUEUE_SIZE);
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(SUBSCRIBE_QUEUE_SIZE), obj_label_sub,
                                                 cluster_centroids_sub);
  sync.registerCallback(boost::bind(&fusion_cb, _1, _2));

  obj_pose_pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("obj_pose", ADVERTISE_QUEUE_SIZE, ADVERTISE_LATCH);
  cluster_class_pub = n.advertise<autoware_msgs::CloudClusterArray>("/cloud_clusters_class", ADVERTISE_QUEUE_SIZE);
  obj_pose_timestamp_pub = n.advertise<std_msgs::Time>("obj_pose_timestamp", ADVERTISE_QUEUE_SIZE);
  marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("obj_pose_arrow", 1, true);
  vmap_server = n.serviceClient<vector_map_server::GetLane>("/vector_map_server/get_lane");
  vmap.subscribe(n, vector_map::Category::POINT | vector_map::Category::NODE, ros::Duration(0));  // non-blocking

  ros::spin();

  return 0;
}
