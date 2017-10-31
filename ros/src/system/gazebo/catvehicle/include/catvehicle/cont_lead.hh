#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/physics/Joint.hh>
#include <stdlib.h>
//#include <unistd.h>


namespace gazebo
{
class LeadSteering : public ModelPlugin
{
public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	void OnUpdate(const common::UpdateInfo & /*_info*/);
//	void SteeringSmooth();
private:
        void LeadVehicleSimROSThread();
        void Callback(const geometry_msgs::Twist::ConstPtr& msg);

        //ROS
	ros::Subscriber sub_;
        boost::thread ros_spinner_thread_;
        ros::NodeHandle* rosnode_;

	//variables
	double angle;
//	double difference;
//	double prev_angle;
//	double cur_angle;
	
	//Gazebo
	physics::JointPtr steering_joints[2];
//        ros::NodeHandle* rosnode_;
	physics::JointController *j_cont;
	event::ConnectionPtr updateConnection;
	physics::ModelPtr model;
};
}
