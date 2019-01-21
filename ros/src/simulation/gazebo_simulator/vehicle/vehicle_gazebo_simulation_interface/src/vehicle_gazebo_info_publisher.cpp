#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetLinkState.h>
#include <sensor_msgs/JointState.h>
#include <autoware_msgs/VehicleStatus.h>
#include <string>
#include <cmath>

class VehicleGazeboInfoPublisher
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceClient client_;
    ros::Publisher vehicle_pose_pub_;
    ros::Publisher vehicle_vel_pub_;
    ros::Publisher steering_angle_pub_;
    ros::Publisher vehicle_status_pub_;
    ros::Timer publish_timer_; // publish timer
    ros::Subscriber odom_sub_;
    double wheel_radius_;
    std::string ns_;
    void publishTimerCallback(const ros::TimerEvent &e);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &input_msg);

  public:
    VehicleGazeboInfoPublisher();
    ~VehicleGazeboInfoPublisher(){};
};

VehicleGazeboInfoPublisher::VehicleGazeboInfoPublisher() : nh_(""), pnh_("~")
{
    client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    vehicle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vehicle_info/pose", 1, true);
    vehicle_vel_pub_ = nh_.advertise<std_msgs::Float64>("/vehicle_info/velocity", 1, true);
    steering_angle_pub_ = nh_.advertise<std_msgs::Float64>("/vehicle_info/steering_angle", 1, true);
    vehicle_status_pub_ = nh_.advertise<autoware_msgs::VehicleStatus>("/vehicle_status", 1, true);
    double publish_pose_rate;
    pnh_.param<double>("publish_pose_rate", publish_pose_rate, double(10.0));
    pnh_.param<double>("wheel_radius", wheel_radius_, 0.341);
    pnh_.param("ns", ns_, std::string("autoware_gazebo"));
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
    output_pose.header.frame_id = "base_link";
    output_pose.header.stamp = current_time;
    output_pose.pose = base_link_srv.response.link_state.pose;

    vehicle_pose_pub_.publish(output_pose);
}

void VehicleGazeboInfoPublisher::jointStateCallback(const sensor_msgs::JointState::ConstPtr &input_msg)
{
    std_msgs::Float64 output_vel, output_steering_angle;
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
    output_vel.data = wheel_radius_ * (wheel_left_rear_vel + wheel_right_rear_vel) / 2.0;
    output_steering_angle.data = (steering_right_front_angle + steering_left_front_angle) / 2.0;
    output_vehicle_status.header.stamp = ros::Time::now();
    output_vehicle_status.header.frame_id = "base_link";
    output_vehicle_status.speed = output_vel.data;
    output_vehicle_status.angle = output_steering_angle.data;

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
