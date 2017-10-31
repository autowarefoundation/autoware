#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <stdlib.h>


namespace gazebo
{
class CatGPS : public ModelPlugin
{
public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        void OnUpdate(const common::UpdateInfo & /*_info*/);
private:
        void CatVehicleSimROSThread();
        void Callback(const nav_msgs::Odometry::ConstPtr& msg);


        //ROS
        ros::Subscriber sub_;
        boost::thread ros_spinner_thread_;
        ros::NodeHandle* rosnode_;

	//gazebo
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;

	//variables
	int k;
	double x , y , z , x_ang , y_ang , z_ang , omega , x_new , y_new , offset_x , offset_y , car_height;
	math::Vector3 vector;
	math::Pose _pose;
	math::Quaternion quaternion;
};
}
