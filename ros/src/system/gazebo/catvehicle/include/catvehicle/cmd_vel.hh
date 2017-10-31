#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cstdio>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/physics/Joint.hh>
#include <cstdlib>
//#include <unistd.h>


namespace gazebo
{
class CmdVelController : public ModelPlugin
{
public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	void OnUpdate(const common::UpdateInfo & /*_info*/);
//	void SteeringSmooth();
private:
        void CatVehicleSimROSThread();
        void Callback(const geometry_msgs::Twist::ConstPtr& msg);

        //ROS
	ros::Subscriber sub_;
        boost::thread ros_spinner_thread_;
        ros::NodeHandle* rosnode_;

	//variables
	double u1; // commanded velocity
	double u2; // commanded tire angle
	double x1; // actual velocity
	double x2; // actual tire angle
//	double difference;
//	double prev_angle;
//	double cur_angle;
	
	//Gazebo
	physics::JointPtr steering_joints[2];
	physics::JointPtr velocity_joints[2];
//        ros::NodeHandle* rosnode_;
	physics::JointController *steering_cont;
	physics::JointVelocityController *rearWheel_cont;
	event::ConnectionPtr updateConnection;
	physics::ModelPtr model;
};
}
