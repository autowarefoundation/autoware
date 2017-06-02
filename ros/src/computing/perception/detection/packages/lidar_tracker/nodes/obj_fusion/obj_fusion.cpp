#include <cv_tracker_msgs/obj_label.h>
#include <float.h>
#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>
#include <math.h>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

/* flag for comfirming whether multiple topics are received */
static bool isReady_obj_label;
static bool isReady_cluster_centroids;

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 100;
static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool ADVERTISE_LATCH = false;
static constexpr double LOOP_RATE = 15.0;

ros::Publisher obj_pose_pub;
ros::Publisher obj_pose_timestamp_pub;
ros::Publisher cluster_class_pub;

static std::string object_type;
static std::vector<geometry_msgs::Point> centroids;
static std_msgs::Header sensor_header;
static std::vector<lidar_tracker::CloudCluster> v_cloud_cluster;
static ros::Time obj_pose_timestamp;
static double threshold_min_dist;
static tf::StampedTransform transform;

struct obj_label_t {
  std::vector<geometry_msgs::Point> reprojected_positions;
  std::vector<int> obj_id;
};

obj_label_t obj_label;

/* mutex to handle objects from within multi thread safely */
std::mutex mtx_flag_obj_label;
std::mutex mtx_flag_cluster_centroids;
std::mutex mtx_reprojected_positions;
std::mutex mtx_centroids;
#define LOCK(mtx) (mtx).lock()
#define UNLOCK(mtx) (mtx).unlock()

static double euclid_distance(const geometry_msgs::Point pos1,
                              const geometry_msgs::Point pos2) {
  return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) +
              pow(pos1.z - pos2.z, 2));

} /* static double distance() */

/* fusion reprojected position and pointcloud centroids */
static void fusion_objects(void) {
  obj_label_t obj_label_current;
  std::vector<lidar_tracker::CloudCluster> v_cloud_cluster_current;
  std_msgs::Header header = sensor_header;
  std::vector<geometry_msgs::Point> centroids_current;

  LOCK(mtx_reprojected_positions);
  copy(obj_label.reprojected_positions.begin(),
       obj_label.reprojected_positions.end(),
       back_inserter(obj_label_current.reprojected_positions));
  copy(obj_label.obj_id.begin(), obj_label.obj_id.end(),
       back_inserter(obj_label_current.obj_id));
  UNLOCK(mtx_reprojected_positions);

  LOCK(mtx_centroids);
  copy(centroids.begin(), centroids.end(), back_inserter(centroids_current));
  copy(v_cloud_cluster.begin(), v_cloud_cluster.end(),
       back_inserter(v_cloud_cluster_current));
  UNLOCK(mtx_centroids);

  if (centroids_current.empty() ||
      obj_label_current.reprojected_positions.empty() ||
      obj_label_current.obj_id.empty()) {
    jsk_recognition_msgs::BoundingBoxArray pub_msg;
    pub_msg.header = header;
    std_msgs::Time time;
    obj_pose_pub.publish(pub_msg);
    lidar_tracker::CloudClusterArray cloud_clusters_msg;
    cloud_clusters_msg.header = header;
    cluster_class_pub.publish(cloud_clusters_msg);

    time.data = obj_pose_timestamp;
    obj_pose_timestamp_pub.publish(time);
    return;
  }

  std::vector<int> obj_indices;

  for (unsigned int i = 0; i < obj_label_current.obj_id.size(); ++i) {
    unsigned int min_idx = 0;
    double min_distance = DBL_MAX;

    /* calculate each euclid distance between reprojected position and centroids
     */
    for (unsigned int j = 0; j < centroids_current.size(); j++) {
      double distance =
          euclid_distance(obj_label_current.reprojected_positions.at(i),
                          centroids_current.at(j));

      /* Nearest centroid correspond to this reprojected object */
      if (distance < min_distance) {
        min_distance = distance;
        min_idx = j;
      }
    }
    if (min_distance < threshold_min_dist) {
      obj_indices.push_back(min_idx);
    } else {
      obj_indices.push_back(-1);
    }
  }

  /* Publish marker with centroids coordinates */
  jsk_recognition_msgs::BoundingBoxArray pub_msg;
  pub_msg.header = header;
  lidar_tracker::CloudClusterArray cloud_clusters_msg;
  cloud_clusters_msg.header = header;

  for (unsigned int i = 0; i < obj_label_current.obj_id.size(); ++i) {
    jsk_recognition_msgs::BoundingBox bounding_box;
    if (obj_indices.at(i) == -1)
      continue;

    v_cloud_cluster_current.at(obj_indices.at(i)).label = object_type;

    if (object_type == "car") {
      v_cloud_cluster_current.at(obj_indices.at(i)).bounding_box.label = 0;
    } else if (object_type == "person") {
      v_cloud_cluster_current.at(obj_indices.at(i)).bounding_box.label = 1;
    } else {
      v_cloud_cluster_current.at(obj_indices.at(i)).bounding_box.label = 2;
      v_cloud_cluster_current.at(obj_indices.at(i)).label = "unknown";
    }
    v_cloud_cluster_current.at(obj_indices.at(i)).bounding_box.value =
        obj_label_current.obj_id.at(i);
    bounding_box = v_cloud_cluster_current.at(obj_indices.at(i)).bounding_box;
    pub_msg.boxes.push_back(bounding_box);
    cloud_clusters_msg.clusters.push_back(
        v_cloud_cluster_current.at(obj_indices.at(i)));
  }

  obj_pose_pub.publish(pub_msg);
  cluster_class_pub.publish(cloud_clusters_msg);
  std_msgs::Time time;
  time.data = obj_pose_timestamp;
  obj_pose_timestamp_pub.publish(time);
}

void obj_label_cb(const cv_tracker_msgs::obj_label &obj_label_msg) {
  object_type = obj_label_msg.type;
  obj_pose_timestamp = obj_label_msg.header.stamp;

  LOCK(mtx_reprojected_positions);
  obj_label.reprojected_positions.clear();
  obj_label.obj_id.clear();
  UNLOCK(mtx_reprojected_positions);

  LOCK(mtx_reprojected_positions);
  for (unsigned int i = 0; i < obj_label_msg.obj_id.size(); ++i) {
    obj_label.reprojected_positions.push_back(
        obj_label_msg.reprojected_pos.at(i));
    obj_label.obj_id.push_back(obj_label_msg.obj_id.at(i));
  }
  UNLOCK(mtx_reprojected_positions);

  /* confirm obj_label is subscribed */
  LOCK(mtx_flag_obj_label);
  isReady_obj_label = true;
  UNLOCK(mtx_flag_obj_label);

  /* Publish fusion result if both of topics are ready */
  if (isReady_obj_label && isReady_cluster_centroids) {
    fusion_objects();

    LOCK(mtx_flag_obj_label);
    isReady_obj_label = false;
    UNLOCK(mtx_flag_obj_label);

    LOCK(mtx_flag_cluster_centroids);
    isReady_cluster_centroids = false;
    UNLOCK(mtx_flag_cluster_centroids);
  }

} /* void obj_label_cb() */

void cluster_centroids_cb(
    const lidar_tracker::CloudClusterArray::Ptr &in_cloud_cluster_array_ptr) {
  LOCK(mtx_centroids);
  centroids.clear();
  v_cloud_cluster.clear();
  UNLOCK(mtx_centroids);

  LOCK(mtx_centroids);
  static tf::TransformListener trf_listener;
  try {
    trf_listener.lookupTransform("map", "velodyne", ros::Time(0), transform);
    for (int i(0); i < (int)in_cloud_cluster_array_ptr->clusters.size(); ++i) {
      lidar_tracker::CloudCluster cloud_cluster =
          in_cloud_cluster_array_ptr->clusters.at(i);
      /* convert centroids coodinate from velodyne frame to map frame */
      tf::Vector3 pt(cloud_cluster.centroid_point.point.x,
                     cloud_cluster.centroid_point.point.y,
                     cloud_cluster.centroid_point.point.z);
      tf::Vector3 converted = transform * pt;
      sensor_header = cloud_cluster.header;
      v_cloud_cluster.push_back(cloud_cluster);
      geometry_msgs::Point point_in_map;
      point_in_map.x = converted.x();
      point_in_map.y = converted.y();
      point_in_map.z = converted.z();

      centroids.push_back(point_in_map);
    }
  } catch (tf::TransformException ex) {
    ROS_INFO("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  UNLOCK(mtx_centroids);

  LOCK(mtx_flag_cluster_centroids);
  isReady_cluster_centroids = true;
  UNLOCK(mtx_flag_cluster_centroids);

  /* Publish fusion result if both of topics are ready */
  if (isReady_obj_label && isReady_cluster_centroids) {
    fusion_objects();

    LOCK(mtx_flag_obj_label);
    isReady_obj_label = false;
    UNLOCK(mtx_flag_obj_label);

    LOCK(mtx_flag_cluster_centroids);
    isReady_cluster_centroids = false;
    UNLOCK(mtx_flag_cluster_centroids);
  }

} /* void cluster_centroids_cb() */

int main(int argc, char *argv[]) {
  /* ROS initialization */
  ros::init(argc, argv, "obj_fusion");

  ros::NodeHandle n;
  ros::NodeHandle private_n("~");

  if (!private_n.getParam("min_dist", threshold_min_dist)) {
    threshold_min_dist = 2.0;
  }
  /* Initialize flags */
  isReady_obj_label = false;
  isReady_cluster_centroids = false;

  ros::Subscriber obj_label_sub =
      n.subscribe("obj_label", SUBSCRIBE_QUEUE_SIZE, obj_label_cb);
  ros::Subscriber cluster_centroids_sub = n.subscribe(
      "/cloud_clusters", SUBSCRIBE_QUEUE_SIZE, cluster_centroids_cb);
  obj_pose_pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>(
      "obj_pose", ADVERTISE_QUEUE_SIZE, ADVERTISE_LATCH);
  cluster_class_pub = n.advertise<lidar_tracker::CloudClusterArray>(
      "/cloud_clusters_class", ADVERTISE_QUEUE_SIZE);
  obj_pose_timestamp_pub =
      n.advertise<std_msgs::Time>("obj_pose_timestamp", ADVERTISE_QUEUE_SIZE);
  ros::spin();

  return 0;
}
