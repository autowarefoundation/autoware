#include <ros/ros.h>
#include <cv_tracker/obj_label.h>
#include <lidar_tracker/centroids.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <float.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <std_msgs/Time.h>

/* flag for comfirming whether multiple topics are received */
static bool isReady_obj_label;
static bool isReady_cluster_centroids;

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 100;
static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool ADVERTISE_LATCH = false;
static constexpr double LOOP_RATE = 15.0;

ros::Publisher obj_pose_pub;
ros::Publisher obj_pose_timestamp_pub;

static std::vector<geometry_msgs::Point> reprojected_positions;
static std::string object_type;
static std::vector<geometry_msgs::Point> centroids;
static ros::Time obj_pose_timestamp;

static tf::StampedTransform transform;

/* mutex to handle objects from within multi thread safely */
std::mutex mtx_flag_obj_label;
std::mutex mtx_flag_cluster_centroids;
std::mutex mtx_reprojected_positions;
std::mutex mtx_centroids;
#define LOCK(mtx) (mtx).lock()
#define UNLOCK(mtx) (mtx).unlock()

static double euclid_distance(const geometry_msgs::Point pos1,
                              const geometry_msgs::Point pos2)
{
    return sqrt(pow(pos1.x - pos2.x, 2) +
                pow(pos1.y - pos2.y, 2) +
                pow(pos1.z - pos2.z, 2));

} /* static double distance() */


/* fusion reprojected position and pointcloud centroids */
static void fusion_objects(void)
{
    std::vector<geometry_msgs::Point> reprojected_positions_current;
    std::vector<geometry_msgs::Point> centroids_current;

    LOCK(mtx_reprojected_positions);
    copy(reprojected_positions.begin(), reprojected_positions.end(), back_inserter(reprojected_positions_current));
    UNLOCK(mtx_reprojected_positions);

    LOCK(mtx_centroids);
    copy(centroids.begin(), centroids.end(), back_inserter(centroids_current));
    UNLOCK(mtx_centroids);

    if (centroids_current.empty() || reprojected_positions_current.empty()) {
        visualization_msgs::MarkerArray pub_msg;
        obj_pose_pub.publish(pub_msg);
        obj_pose_timestamp_pub.publish(obj_pose_timestamp);
        return;
    }

    std::vector<unsigned int> obj_indices;
    for(const auto& reproj_pos : reprojected_positions_current)
        {
            unsigned int min_idx      = 0;
            double       min_distance = DBL_MAX;

            /* calculate each euclid distance between reprojected position and centroids */
            for (unsigned int i=0; i<centroids_current.size(); i++)
                {
                    double distance = euclid_distance(reproj_pos, centroids_current.at(i));

                    /* Nearest centroid correspond to this reprojected object */
                    if (distance < min_distance)
                        {
                            min_distance = distance;
                            min_idx      = i;
                        }
                }
            obj_indices.push_back(min_idx);
        }

    /* Publish marker with centroids coordinates */
    visualization_msgs::MarkerArray pub_msg;

    std_msgs::ColorRGBA color_red;
    color_red.r = 1.0f;
    color_red.g = 0.0f;
    color_red.b = 0.0f;
    color_red.a = 1.0f;

    std_msgs::ColorRGBA color_blue;
    color_blue.r = 0.0f;
    color_blue.g = 0.0f;
    color_blue.b = 1.0f;
    color_blue.a = 1.0f;

    std_msgs::ColorRGBA color_green;
    color_green.r = 0.0f;
    color_green.g = 1.0f;
    color_green.b = 0.0f;
    color_green.a = 1.0f;

    for (const auto& idx : obj_indices)
        {
            visualization_msgs::Marker marker;

            /*Set the frame ID */
            marker.header.frame_id = "map";

            /* Set the namespace and id for this marker */
            marker.ns = object_type;
            marker.id = idx;

            /* Set the marker type */
            marker.type = visualization_msgs::Marker::SPHERE;

            /* Set the pose of the marker */
            marker.pose.position = centroids_current.at(idx);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.w = 0.0;

            /* Set the scale of the marker -- We assume object as 1.5m sphere */
            marker.scale.x = (double)1.5;
            marker.scale.y = (double)1.5;
            marker.scale.z = (double)1.5;

            /* Set the color */
            if (object_type == "car") {
                marker.color = color_blue;
            }
            else if (object_type == "person") {
                marker.color = color_green;
            }
            else {
                marker.color = color_red;
            }

            marker.lifetime = ros::Duration(0.3);

            pub_msg.markers.push_back(marker);
        }

        obj_pose_pub.publish(pub_msg);
        obj_pose_timestamp_pub.publish(obj_pose_timestamp);
}


void obj_label_cb(const cv_tracker::obj_label& obj_label_msg)
{
    object_type = obj_label_msg.type;
    obj_pose_timestamp = obj_label_msg.header.stamp;

    LOCK(mtx_reprojected_positions);
    reprojected_positions.clear();
    UNLOCK(mtx_reprojected_positions);

    LOCK(mtx_reprojected_positions);
    for (const auto& point : obj_label_msg.reprojected_pos)
        {
            reprojected_positions.push_back(point);
        }
    UNLOCK(mtx_reprojected_positions);

    /* confirm obj_label is subscribed */
    LOCK(mtx_flag_obj_label);
    isReady_obj_label = true;
    UNLOCK(mtx_flag_obj_label);

    /* Publish fusion result if both of topics are ready */
   if (isReady_obj_label && isReady_cluster_centroids)
        {
            fusion_objects();

            LOCK(mtx_flag_obj_label);
            isReady_obj_label = false;
            UNLOCK(mtx_flag_obj_label);

            LOCK(mtx_flag_cluster_centroids);
            isReady_cluster_centroids = false;
            UNLOCK(mtx_flag_cluster_centroids);
        }

} /* void obj_label_cb() */


void cluster_centroids_cb(const lidar_tracker::centroids& cluster_centroids_msg)
{
    LOCK(mtx_centroids);
    centroids.clear();
    UNLOCK(mtx_centroids);

    LOCK(mtx_centroids);
    for (const auto& point : cluster_centroids_msg.points)
        {
            /* convert centroids coodinate from velodyne frame to map frame */
            tf::Vector3 pt(point.x, point.y, point.z);
            tf::Vector3 converted = transform * pt;

            geometry_msgs::Point point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();

            centroids.push_back(point_in_map);
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


int main(int argc, char* argv[])
{
    /* ROS initialization */
    ros::init(argc, argv, "obj_fusion");

    ros::NodeHandle n;

    /* Initialize flags */
    isReady_obj_label         = false;
    isReady_cluster_centroids = false;

    ros::Subscriber obj_label_sub         = n.subscribe("obj_label", SUBSCRIBE_QUEUE_SIZE, obj_label_cb);
    ros::Subscriber cluster_centroids_sub = n.subscribe("/cluster_centroids", SUBSCRIBE_QUEUE_SIZE, cluster_centroids_cb);
    obj_pose_pub = n.advertise<visualization_msgs::MarkerArray>("obj_pose", ADVERTISE_QUEUE_SIZE, ADVERTISE_LATCH);
    obj_pose_timestamp_pub = n.advertise<std_msgs::Time>("obj_pose_timestamp", ADVERTISE_QUEUE_SIZE);

    tf::TransformListener trf_listener;
    ros::Rate loop_rate(LOOP_RATE);  // Try to loop in "LOOP_RATE" [Hz]
    while (n.ok())
        {
            try {
                trf_listener.lookupTransform("map", "velodyne", ros::Time(0), transform);
            }
            catch (tf::TransformException ex) {
                ROS_INFO("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }

    return 0;
}
