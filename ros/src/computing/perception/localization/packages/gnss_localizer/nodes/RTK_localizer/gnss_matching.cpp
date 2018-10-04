#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "gnss_matching_history.h"
#include <autoware_msgs/gnss_surface_speed.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, autoware_msgs::gnss_surface_speed>
    GnssSync;

//pthread_mutex_t mutex;//スレッドのロックに使用する予定

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

class gnss_matching_class
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    gnss_matching::Gnss_Matching_History gmh;

    Eigen::Matrix4f tf_gbtol;

    ros::Publisher localizer_pose_pub;
    ros::Publisher estimate_twist_pub;
    ros::Publisher gnss_pose_vehicle_pub;

    tf::TransformBroadcaster gnss_tb;

    double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
    {
      double diff_rad = lhs_rad - rhs_rad;
      if(diff_rad >= M_PI)
         diff_rad = diff_rad - 2*M_PI;
      else if(diff_rad < -M_PI)
         diff_rad = diff_rad + 2*M_PI;
      return diff_rad;
    }

    void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                           const autoware_msgs::gnss_surface_speed::ConstPtr& speed_msg)
    {
        //std::cout<<std::setprecision(16)<<pose_msg->pose.position.x<<std::endl;
        ros::Time current_gnss_time = pose_msg->header.stamp;

        {
            std::cout<<std::setprecision(16)<<"yaw : "<<pose_msg->pose.orientation.z<<std::endl;
            tf::Quaternion gnss_q(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y,
                                  pose_msg->pose.orientation.z, pose_msg->pose.orientation.w);
            tf::Matrix3x3 gnss_m(gnss_q);
            Eigen::Matrix4f t_gnss(Eigen::Matrix4f::Identity());   // base_link
            for(int i=0;i<3;i++)
            {
                for(int j=0;j<3;j++) t_gnss(i,j)=gnss_m[i][j];
            }
            t_gnss(0,3)=pose_msg->pose.position.x; t_gnss(1,3)=pose_msg->pose.position.y; t_gnss(2,3)=pose_msg->pose.position.z;

            publish_localizer(t_gnss,current_gnss_time);

            /*pose current_gnss_pose;
            current_gnss_pose.x = pose_msg->pose.position.x;
            current_gnss_pose.y = pose_msg->pose.position.y;
            current_gnss_pose.z = pose_msg->pose.position.z;
            gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);*/

            Eigen::Matrix4f t2_gnss(Eigen::Matrix4f::Identity());  // localizer
            t2_gnss = t_gnss * tf_gbtol.inverse();

            // Update gnss_pose2
            tf::Matrix3x3 mat_b_gnss;  // base_link
            mat_b_gnss.setValue(static_cast<double>(t2_gnss(0, 0)), static_cast<double>(t2_gnss(0, 1)), static_cast<double>(t2_gnss(0, 2)),
                                static_cast<double>(t2_gnss(1, 0)), static_cast<double>(t2_gnss(1, 1)), static_cast<double>(t2_gnss(1, 2)),
                                static_cast<double>(t2_gnss(2, 0)), static_cast<double>(t2_gnss(2, 1)), static_cast<double>(t2_gnss(2, 2)));

            pose gnss_pose;
            gnss_pose.x = t2_gnss(0, 3);
            gnss_pose.y = t2_gnss(1, 3);
            gnss_pose.z = t2_gnss(2, 3);
            mat_b_gnss.getRPY(gnss_pose.roll, gnss_pose.pitch, gnss_pose.yaw, 1);
            //gnss_pose.yaw += M_PI;
            //if(gnss_pose.yaw >= M_PI) gnss_pose.yaw -= M_PI*2;
            gnss_q.setRPY(gnss_pose.roll, gnss_pose.pitch, gnss_pose.yaw);
            tf::Vector3 v(gnss_pose.x, gnss_pose.y, gnss_pose.z);
            tf::Transform transform(gnss_q, v);
            geometry_msgs::PoseStamped gnss_pose_msg;
            gnss_pose_msg.header.frame_id = "/map";
            gnss_pose_msg.header.stamp = current_gnss_time;
            gnss_pose_msg.pose.position.x = transform.getOrigin().getX();
            gnss_pose_msg.pose.position.y = transform.getOrigin().getY();
            gnss_pose_msg.pose.position.z = transform.getOrigin().getZ();
            gnss_pose_msg.pose.orientation.x = transform.getRotation().x();
            gnss_pose_msg.pose.orientation.y = transform.getRotation().y();
            gnss_pose_msg.pose.orientation.z = transform.getRotation().z();
            gnss_pose_msg.pose.orientation.w = transform.getRotation().w();
            gnss_pose_vehicle_pub.publish(gnss_pose_msg);

            tf::Quaternion current_q;
            transform.setOrigin(tf::Vector3(gnss_pose.x, gnss_pose.y, gnss_pose.z));
            current_q.setRPY(gnss_pose.roll, gnss_pose.pitch, gnss_pose.yaw);
            transform.setRotation(current_q);
            gnss_tb.sendTransform(tf::StampedTransform(transform, current_gnss_time, "/map", "/base_link"));

            if(gmh.get_data_size() > 0)
            {
                Eigen::Matrix4f t2_gnss_prev = gmh.get_transform(0);
                pose gnss_pose_prev;
                gnss_pose_prev.x = t2_gnss_prev(0, 3);
                gnss_pose_prev.y = t2_gnss_prev(1, 3);
                gnss_pose_prev.z = t2_gnss_prev(2, 3);
                mat_b_gnss.getRPY(gnss_pose_prev.roll, gnss_pose_prev.pitch, gnss_pose_prev.yaw, 1);
                gnss_q.setRPY(gnss_pose_prev.roll, gnss_pose_prev.pitch, gnss_pose_prev.yaw);
                geometry_msgs::PoseStamped prev_stamp = gmh.get_pose(0);
                publish_estimate_twist(gnss_pose, gnss_pose_prev, speed_msg->surface_speed, current_gnss_time, prev_stamp.header.stamp);
            }

            gmh.push_gnss_pose(*pose_msg,*speed_msg,t2_gnss);
        }
    }

    void publish_estimate_twist(pose current, pose prev, double gnss_speed, ros::Time current_time, ros::Time prev_time)
    {
        double diff_time = (current_time - prev_time).toSec();
	//if(diff_time == 0) return;

        //std::cout<<std::setprecision(16)<<diff_time<<std::endl;
        // Compute the velocity and acceleration
        double diff_x = current.x - prev.x;
        double diff_y = current.y - prev.y;
        double diff_z = current.z - prev.z;
        double diff_yaw = calcDiffForRadian(current.yaw, prev.yaw);
        double diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

        double velocity = (diff_time > 0) ? (diff / diff_time) : 0;
        double angular_velocity   = (diff_time > 0) ? (diff_yaw / diff_time) : 0;
        //std::cout<<std::setprecision(20)<<"yaw : "<<diff_yaw<<std::endl;

        geometry_msgs::TwistStamped estimate_twist_msg;
        // Set values for /estimate_twist
        estimate_twist_msg.header.stamp = current_time;
        estimate_twist_msg.header.frame_id = "/base_link";
        //estimate_twist_msg.twist.linear.x = speed_msg->surface_speed;
        estimate_twist_msg.twist.linear.x = gnss_speed/3.6;//velocity;
        estimate_twist_msg.twist.linear.y = 0.0;
        estimate_twist_msg.twist.linear.z = 0.0;
        estimate_twist_msg.twist.angular.x = 0.0;
        estimate_twist_msg.twist.angular.y = 0.0;
        estimate_twist_msg.twist.angular.z = angular_velocity;
        std::cout<<std::setprecision(16)<<"diff : "<<diff<<std::endl;
	std::cout<<std::setprecision(16)<<"gnss : "<<gnss_speed/3.6<<std::endl;
        estimate_twist_pub.publish(estimate_twist_msg);
    }

    void publish_localizer(Eigen::Matrix4f t_gnss,ros::Time updateTime)
    {
        tf::Matrix3x3 mat_l;  // localizer
        mat_l.setValue(static_cast<double>(t_gnss(0, 0)), static_cast<double>(t_gnss(0, 1)), static_cast<double>(t_gnss(0, 2)),
                       static_cast<double>(t_gnss(1, 0)), static_cast<double>(t_gnss(1, 1)), static_cast<double>(t_gnss(1, 2)),
                       static_cast<double>(t_gnss(2, 0)), static_cast<double>(t_gnss(2, 1)), static_cast<double>(t_gnss(2, 2)));

        // Update localizer_pose
        pose localizer_pose;
        localizer_pose.x = t_gnss(0, 3);
        localizer_pose.y = t_gnss(1, 3);
        localizer_pose.z = t_gnss(2, 3);
        mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);
        //localizer_pose.yaw += M_PI;
        //if(localizer_pose.yaw >= M_PI) localizer_pose.yaw -= M_PI*2;
        tf::Quaternion localizer_q;
        localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);

        geometry_msgs::PoseStamped localizer_pose_msg;
        localizer_pose_msg.header.frame_id = "/map";
        localizer_pose_msg.header.stamp = updateTime;
        localizer_pose_msg.pose.position.x = localizer_pose.x;
        localizer_pose_msg.pose.position.y = localizer_pose.y;
        localizer_pose_msg.pose.position.z = localizer_pose.z;
        localizer_pose_msg.pose.orientation.x = localizer_q.x();
        localizer_pose_msg.pose.orientation.y = localizer_q.y();
        localizer_pose_msg.pose.orientation.z = localizer_q.z();
        localizer_pose_msg.pose.orientation.w = localizer_q.w();
        localizer_pose_pub.publish(localizer_pose_msg);
    }
public:
    gnss_matching_class(ros::NodeHandle nh_, ros::NodeHandle private_nh_)
    {
        nh = nh_; private_nh = private_nh_;

        const unsigned int max_his_size=10;
        gmh.set_max_history_size(max_his_size);

        double _tf_gx, _tf_gy, _tf_gz, _tf_groll, _tf_gpitch, _tf_gyaw;
        if (nh.getParam("gx", _tf_gx) == false)
        {
          std::cout << "gx is not set." << std::endl;
          return;
        }
        if (nh.getParam("gy", _tf_gy) == false)
        {
          std::cout << "gy is not set." << std::endl;
          return;
        }
        if (nh.getParam("gz", _tf_gz) == false)
        {
          std::cout << "gz is not set." << std::endl;
          return;
        }
        if (nh.getParam("ax", _tf_groll) == false)
        {
          std::cout << "ax is not set." << std::endl;
          return;
        }
        if (nh.getParam("ay", _tf_gpitch) == false)
        {
          std::cout << "ay is not set." << std::endl;
          return;
        }
        if (nh.getParam("az", _tf_gyaw) == false)
        {
          std::cout << "az is not set." << std::endl;
          return;
        }
        Eigen::Translation3f tl_gbtol(_tf_gx, _tf_gy, _tf_gz);                 // tl: translation
        Eigen::AngleAxisf rot_x_gbtol(_tf_groll, Eigen::Vector3f::UnitX());  // rot: rotation
        Eigen::AngleAxisf rot_y_gbtol(_tf_gpitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z_gbtol(_tf_gyaw, Eigen::Vector3f::UnitZ());
        tf_gbtol = (tl_gbtol * rot_z_gbtol * rot_y_gbtol * rot_x_gbtol).matrix();

        localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
        estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist_gnss", 10);
        gnss_pose_vehicle_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_pose_matching", 10);

        message_filters::Subscriber<geometry_msgs::PoseStamped> sub_gnss_pose(nh, "gnss_pose", 10);
        message_filters::Subscriber<autoware_msgs::gnss_surface_speed> sub_surface_speed(nh, "gnss_surface_speed", 10);
        message_filters::Synchronizer<GnssSync> sync_gnss(GnssSync(10),sub_gnss_pose, sub_surface_speed);
        sync_gnss.registerCallback(boost::bind(&gnss_matching_class::gnss_callback, this, _1, _2));
        ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_matching");
//    pthread_mutex_init(&mutex, NULL);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    gnss_matching_class gmc(nh,private_nh);
    return 0;
}
