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
#include<gazebo_msgs/ModelStates.h>

namespace gazebo
{
    class CatSteering : public ModelPlugin
    {
        public:
            CatSteering();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            //void OnUpdate(const common::UpdateInfo & _info);

        private:
            void CatVehicleSimROSThread();
            void modelRead(const gazebo_msgs::ModelStates::ConstPtr& msg);

            physics::PhysicsEnginePtr physicsEngine;
            //to read the name space from urdf file
            std::string robotNamespace;
            //to read the name space from urdf file
            std::string tfScope;
            //Name of the speed topic being published
            std::string speedTopic;
    	    // Name of the tire angle topic being published
	        std::string tireTopic;
            // Name of the odometry topic being published
            std::string odomTopic;
            //ROS
            ros::Subscriber sub_;
            ros::Publisher ros_pub;
            ros::Publisher steering_pub;
            ros::Publisher odom_pub;
            boost::thread ros_spinner_thread_;
            ros::NodeHandle* rosnode_;
            
            //velocity vector to fetch velocity from model entity
            math::Vector3 linear_vel;
            math::Vector3 angular_vel;
            //Gazebo
            physics::JointPtr steering_joints[2];
            physics::JointController *j_cont;
            event::ConnectionPtr updateConnection;


            //Pointer to the model entity
            physics::ModelPtr model;
            //Pointer to the world in which the model exists
            physics::WorldPtr world;


            //rate at which to update the catsteering
            double updateRate;
            //Previous time when the catsteering was updated.
            ros::Time prevUpdateTime;
    };
}
