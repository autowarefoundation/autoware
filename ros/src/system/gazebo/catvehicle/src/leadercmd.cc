/* Author: Rahul Kumar Bhadani
 * This code applies velocity profile to the leader car
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <cstdio>
#include <cstdlib>


// this global var holds the distance
geometry_msgs::Twist leader_vel;



int main( int argc, char **argv )
{
	ros::init(argc, argv, "leadercmd");

	ros::NodeHandle n;
    
    std::string leader_InputVel_topic="/catvehicle/cmd_vel";
    

    ros::Publisher leader_vel_pub = n.advertise<geometry_msgs::Twist>(leader_InputVel_topic, 1);

    ros::Rate loop_rate(100);

    double bias = 3.0;
    double sinecomp = 0.0;
    double t = 0;
    double pi = 3.14159265359;
	while( ros::ok() )
	{
        if(t > 100)
        {
            t = 0.0;
        }
        sinecomp = sin(t*5/(2*pi));
        t += 0.005;


    	leader_vel.linear.x = bias + sinecomp;
            
        leader_vel_pub.publish(leader_vel);
		ros::spinOnce( );
		loop_rate.sleep( );

	}

	return EXIT_SUCCESS;
}

