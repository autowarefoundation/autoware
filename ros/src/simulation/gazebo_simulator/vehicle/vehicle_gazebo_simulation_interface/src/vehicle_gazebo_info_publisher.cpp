#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/GetLinkState.h>
#include <sensor_msgs/JointState.h>
#include <autoware_msgs/VehicleStatus.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <cmath>

class VehicleGazeboInfoPublisher
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceClient client_;
    ros::Publisher vehicle_pose_pub_;
    ros::Publisher vehicle_twist_pub_;
    ros::Publisher vehicle_vel_pub_;
    ros::Publisher steering_angle_pub_;
    ros::Publisher vehicle_status_pub_;
    ros::Timer publish_timer_; // publish timer
    ros::Subscriber odom_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double wheel_radius_;
    double wheel_base_;
    std::string ns_;
    bool enable_base_link_tf_;
    void publishTimerCallback(const ros::TimerEvent &e);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &input_msg);

  public:
    VehicleGazeboInfoPublisher();
    ~VehicleGazeboInfoPublisher(){};
};

VehicleGazeboInfoPublisher::VehicleGazeboInfoPublisher() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
    client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    vehicle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/gazebo_vehicle/pose", 1);
    vehicle_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/gazebo_vehicle/twist", 1);
    vehicle_vel_pub_ = nh_.advertise<std_msgs::Float64>("/gazebo_vehicle/velocity", 1);
    steering_angle_pub_ = nh_.advertise<std_msgs::Float64>("/gazebo_vehicle/steering_angle", 1);
    vehicle_status_pub_ = nh_.advertise<autoware_msgs::VehicleStatus>("/vehicle_status", 1);
    double publish_pose_rate;
    pnh_.param<double>("publish_pose_rate", publish_pose_rate, double(10.0));
    pnh_.param<double>("/vehicle_info/wheel_radius", wheel_radius_, 0.341);
    pnh_.param("/vehicle_info/wheel_base", wheel_base_, 2.95);
    pnh_.param("ns", ns_, std::string("autoware_gazebo"));
    pnh_.param("enable_base_link_tf", enable_base_link_tf_, false);

    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_pose_rate), &VehicleGazeboInfoPublisher::publishTimerCallback, this);
    odom_sub_ = nh_.subscribe("joint_states", 1, &VehicleGazeboInfoPublisher::jointStateCallback, this);
}

void VehicleGazeboInfoPublisher::publishTimerCallback(const ros::TimerEvent &e)
{
    gazebo_msgs::GetLinkState base_link_srv;
    base_link_srv.request.link_name = ns_ + "::base_link";
    base_link_srv.request.reference_frame = "";
    ros::Time current_time = ros::Time::now();
    client_.call(base_link_srv);

    geometry_msgs::PoseStamped output_pose;
    output_pose.header.frame_id = "world";
    output_pose.header.stamp = current_time;
    output_pose.pose = base_link_srv.response.link_state.pose;

    vehicle_pose_pub_.publish(output_pose);

    if (enable_base_link_tf_)
    {
        tf2::Transform tf_world2map, tf_world2base_link, tf_map2base_link;

        try
        {
            geometry_msgs::TransformStamped ros_world2map;
            ros_world2map = tf_buffer_.lookupTransform("world", "map", ros::Time(0));
            tf2::fromMsg(ros_world2map.transform, tf_world2map);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        {
            geometry_msgs::Pose ros_world2base_link;
            ros_world2base_link.position.x = output_pose.pose.position.x;
            ros_world2base_link.position.y = output_pose.pose.position.y;
            ros_world2base_link.position.z = output_pose.pose.position.z;
            ros_world2base_link.orientation.x = output_pose.pose.orientation.x;
            ros_world2base_link.orientation.y = output_pose.pose.orientation.y;
            ros_world2base_link.orientation.z = output_pose.pose.orientation.z;
            ros_world2base_link.orientation.w = output_pose.pose.orientation.w;
            tf2::fromMsg(ros_world2base_link, tf_world2base_link);
        }

        tf_map2base_link = tf_world2map.inverse() * tf_world2base_link;

        geometry_msgs::TransformStamped ros_map2base_link;
        ros_map2base_link.header.frame_id = "map";
        ros_map2base_link.child_frame_id = "base_link";
        ros_map2base_link.header.stamp = current_time;
        ros_map2base_link.transform = tf2::toMsg(tf_map2base_link);
        tf_broadcaster_.sendTransform(ros_map2base_link);
    }
}

void VehicleGazeboInfoPublisher::jointStateCallback(const sensor_msgs::JointState::ConstPtr &input_msg)
{
    std_msgs::Float64 output_vel, output_steering_angle;
    geometry_msgs::TwistStamped output_twiststamped;
    autoware_msgs::VehicleStatus output_vehicle_status;
    double steering_right_front_angle = 0;
    double steering_left_front_angle = 0;
    double wheel_right_rear_vel = 0;
    double wheel_left_rear_vel = 0;
    for (size_t i = 0; i < input_msg->name.size(); ++i)
    {
        if (input_msg->name.at(i) == std::string("steering_right_front_joint"))
            steering_right_front_angle = input_msg->position.at(i);
        if (input_msg->name.at(i) == std::string("steering_left_front_joint"))
            steering_left_front_angle = input_msg->position.at(i);
        if (input_msg->name.at(i) == std::string("wheel_right_rear_joint"))
            wheel_right_rear_vel = input_msg->velocity.at(i);
        if (input_msg->name.at(i) == std::string("wheel_left_rear_joint"))
            wheel_left_rear_vel = input_msg->velocity.at(i);
    }
    ros::Time current_time = ros::Time::now();
    output_vel.data = wheel_radius_ * (wheel_left_rear_vel + wheel_right_rear_vel) / 2.0;
    output_steering_angle.data = (steering_right_front_angle + steering_left_front_angle) / 2.0;
    output_vehicle_status.header.stamp = current_time;
    output_vehicle_status.header.frame_id = "base_link";
    output_vehicle_status.speed = output_vel.data * 3.6; //km/h
    output_vehicle_status.angle = output_steering_angle.data * 180.0 / M_PI; //degree 

    output_twiststamped.header.stamp = current_time;
    output_twiststamped.header.frame_id = "base_link";
    output_twiststamped.twist.linear.x = output_vel.data;
    output_twiststamped.twist.angular.z = std::tan(output_steering_angle.data) * output_vel.data / wheel_base_;
    vehicle_twist_pub_.publish(output_twiststamped);
    vehicle_vel_pub_.publish(output_vel);
    steering_angle_pub_.publish(output_steering_angle);
    vehicle_status_pub_.publish(output_vehicle_status);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vehicle_gazebo_info_publisher");

    VehicleGazeboInfoPublisher node;
    ros::spin();

    return 0;
}
