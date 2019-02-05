#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <autoware_msgs/ControlCommand.h>
#include <autoware_msgs/VehicleCmd.h>

class VehicleGazeboInputSubscriber
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher wheel_right_rear_pub_;
    ros::Publisher wheel_left_rear_pub_;
    ros::Publisher steering_right_front_pub_;
    ros::Publisher steering_left_front_pub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber steering_angle_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber vehicle_cmd_sub_;

    void twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg);
    void sterringAngleCallback(const std_msgs::Float64::ConstPtr &input_steering_angle_msg);
    void velocityCallback(const std_msgs::Float64::ConstPtr &input_velocity_msg);
    void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr &input_msg);
    void publishControlCommandStamped2Gazebo(const autoware_msgs::ControlCommand &input_msg);
    void publishTwistStamped2Gazebo(const geometry_msgs::TwistStamped &input_twist_msg);
    double wheel_base_;
    double wheel_tread_;
    double wheel_radius_;
    bool twiststamped_;
    bool ctrl_cmd_;

  public:
    VehicleGazeboInputSubscriber();
    ~VehicleGazeboInputSubscriber(){};
};

VehicleGazeboInputSubscriber::VehicleGazeboInputSubscriber() : nh_(""), pnh_("~")
{

    pnh_.param("/vehicle_info/wheel_base", wheel_base_, 2.95);
    pnh_.param("/vehicle_info/wheel_radius", wheel_radius_, 0.341);
    pnh_.param("/vehicle_info/wheel_tread", wheel_tread_, 1.55);
    pnh_.param("twiststamped", twiststamped_, true);
    pnh_.param("ctrl_cmd", ctrl_cmd_, false);
    wheel_right_rear_pub_ = nh_.advertise<std_msgs::Float64>("wheel_right_rear_velocity_controller/command", 1, true);
    wheel_left_rear_pub_ = nh_.advertise<std_msgs::Float64>("wheel_left_rear_velocity_controller/command", 1, true);
    steering_right_front_pub_ = nh_.advertise<std_msgs::Float64>("steering_right_front_position_controller/command", 1, true);
    steering_left_front_pub_ = nh_.advertise<std_msgs::Float64>("steering_left_front_position_controller/command", 1, true);

    bool twist_sub, steering_angle_sub, velocity_sub;
    pnh_.param("twist_sub", twist_sub, true);
    pnh_.param("steering_angle_sub", steering_angle_sub, false);
    pnh_.param("velocity_sub", velocity_sub, false);
    if (twist_sub)
        twist_sub_ = nh_.subscribe("/cmd_vel", 1, &VehicleGazeboInputSubscriber::twistCallback, this);
    if (steering_angle_sub)
        steering_angle_sub_ = nh_.subscribe("/steering_angle", 1, &VehicleGazeboInputSubscriber::sterringAngleCallback, this);
    if (velocity_sub)
        velocity_sub_ = nh_.subscribe("/velocity", 1, &VehicleGazeboInputSubscriber::velocityCallback, this);
    vehicle_cmd_sub_ = nh_.subscribe("/vehicle_cmd", 1, &VehicleGazeboInputSubscriber::vehicleCmdCallback, this);
}

void VehicleGazeboInputSubscriber::twistCallback(const geometry_msgs::Twist::ConstPtr &input_twist_msg)
{
    std_msgs::Float64 output_wheel_rear, output_steering_right_front, output_steering_left_front;
    output_wheel_rear.data = input_twist_msg->linear.x / wheel_radius_;

    double vref_rear = input_twist_msg->linear.x;
    constexpr double min_vref_rear = 0.01;
    if (std::fabs(vref_rear) < min_vref_rear) // Prevent zero division when calculating ackerman steering
    {
        vref_rear = 0.0 < vref_rear ? min_vref_rear : -min_vref_rear;
    }

    double delta_ref = std::atan(input_twist_msg->angular.z * wheel_base_ / vref_rear);
    delta_ref = 0.0 < vref_rear ? delta_ref : -delta_ref;
    constexpr double max_delta_ref = M_PI / 4.0;
    if (max_delta_ref < std::fabs(delta_ref)) // It is a constraint that the theory does not turn more than 90 degrees
    {
        delta_ref = 0.0 < delta_ref ? max_delta_ref : -max_delta_ref;
    }

    output_steering_right_front.data = std::atan(std::tan(delta_ref) / (1.0 + (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));
    output_steering_left_front.data = std::atan(std::tan(delta_ref) / (1.0 - (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));

    wheel_right_rear_pub_.publish(output_wheel_rear);
    wheel_left_rear_pub_.publish(output_wheel_rear);
    steering_right_front_pub_.publish(output_steering_right_front);
    steering_left_front_pub_.publish(output_steering_left_front);
}

void VehicleGazeboInputSubscriber::sterringAngleCallback(const std_msgs::Float64::ConstPtr &input_steering_angle_msg)
{
    std_msgs::Float64 output_steering_right_front, output_steering_left_front;

    double delta_ref = input_steering_angle_msg->data;
    constexpr double max_delta_ref = M_PI / 4.0;
    if (max_delta_ref < std::fabs(delta_ref)) // It is a constraint that the theory does not turn more than 90 degrees
    {
        delta_ref = 0.0 < delta_ref ? max_delta_ref : -max_delta_ref;
    }
    output_steering_right_front.data = std::atan(std::tan(delta_ref) / (1.0 + (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));
    output_steering_left_front.data = std::atan(std::tan(delta_ref) / (1.0 - (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));

    steering_right_front_pub_.publish(output_steering_right_front);
    steering_left_front_pub_.publish(output_steering_left_front);
}

void VehicleGazeboInputSubscriber::velocityCallback(const std_msgs::Float64::ConstPtr &input_velocity_msg)
{
    std_msgs::Float64 output_wheel_rear;
    output_wheel_rear.data = input_velocity_msg->data / wheel_radius_;
    wheel_right_rear_pub_.publish(output_wheel_rear);
    wheel_left_rear_pub_.publish(output_wheel_rear);
}

void VehicleGazeboInputSubscriber::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr &input_msg)
{
    if (twiststamped_)
        publishTwistStamped2Gazebo(input_msg->twist_cmd);
    if (ctrl_cmd_)
        publishControlCommandStamped2Gazebo(input_msg->ctrl_cmd);
}

void VehicleGazeboInputSubscriber::publishControlCommandStamped2Gazebo(const autoware_msgs::ControlCommand &input_msg)
{
    std_msgs::Float64 output_wheel_rear, output_steering_right_front, output_steering_left_front;

    double delta_ref = input_msg.steering_angle;
    constexpr double max_delta_ref = M_PI / 4.0;
    if (max_delta_ref < std::fabs(delta_ref)) // It is a constraint that the theory does not turn more than 90 degrees
    {
        delta_ref = 0.0 < delta_ref ? max_delta_ref : -max_delta_ref;
    }
    output_steering_right_front.data = std::atan(std::tan(delta_ref) / (1.0 + (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));
    output_steering_left_front.data = std::atan(std::tan(delta_ref) / (1.0 - (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));
    output_wheel_rear.data = input_msg.linear_velocity / wheel_radius_;
    wheel_right_rear_pub_.publish(output_wheel_rear);
    wheel_left_rear_pub_.publish(output_wheel_rear);
    steering_right_front_pub_.publish(output_steering_right_front);
    steering_left_front_pub_.publish(output_steering_left_front);
}

void VehicleGazeboInputSubscriber::publishTwistStamped2Gazebo(const geometry_msgs::TwistStamped &input_twist_msg)
{
    std_msgs::Float64 output_wheel_rear, output_steering_right_front, output_steering_left_front;
    output_wheel_rear.data = input_twist_msg.twist.linear.x / wheel_radius_;

    double vref_rear = input_twist_msg.twist.linear.x;
    constexpr double min_vref_rear = 0.01;
    if (std::fabs(vref_rear) < min_vref_rear) // Prevent zero division when calculating ackerman steering
    {
        vref_rear = 0.0 < vref_rear ? min_vref_rear : -min_vref_rear;
    }

    double delta_ref = std::atan(input_twist_msg.twist.angular.z * wheel_base_ / vref_rear);
    delta_ref = 0.0 < vref_rear ? delta_ref : -delta_ref;
    constexpr double max_delta_ref = M_PI / 4.0;
    if (max_delta_ref < std::fabs(delta_ref)) // It is a constraint that the theory does not turn more than 90 degrees
    {
        delta_ref = 0.0 < delta_ref ? max_delta_ref : -max_delta_ref;
    }

    output_steering_right_front.data = std::atan(std::tan(delta_ref) / (1.0 + (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));
    output_steering_left_front.data = std::atan(std::tan(delta_ref) / (1.0 - (wheel_tread_ / (2.0 * wheel_base_)) * std::tan(delta_ref)));

    wheel_right_rear_pub_.publish(output_wheel_rear);
    wheel_left_rear_pub_.publish(output_wheel_rear);
    steering_right_front_pub_.publish(output_steering_right_front);
    steering_left_front_pub_.publish(output_steering_left_front);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vehicle_gazebo_input_subscriber");

    VehicleGazeboInputSubscriber node;
    ros::spin();

    return 0;
}
