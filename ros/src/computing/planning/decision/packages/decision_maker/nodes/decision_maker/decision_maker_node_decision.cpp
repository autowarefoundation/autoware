#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <autoware_msgs/lane.h>
#include <autoware_msgs/traffic_light.h>

#include <cross_road_area.hpp>
#include <decision_maker_node.hpp>
#include <euclidean_space.hpp>
#include <state.hpp>
#include <state_context.hpp>

namespace decision_maker
{
	double DecisionMakerNode::calcIntersectWayAngle(const CrossRoadArea &area)
	{
		double diff = 0.0;
		if(area.insideWaypoints.empty())
		{
			ROS_INFO("Not inside CrossRoad");
		}
		else
		{
			const geometry_msgs::Pose InPose = area.insideWaypoints.front().pose.pose;
			const geometry_msgs::Pose OutPose = area.insideWaypoints.back().pose.pose;
			double r, p, y, _y;

			tf::Quaternion quat_end(OutPose.orientation.x, OutPose.orientation.y, OutPose.orientation.z,
					OutPose.orientation.w);
			tf::Quaternion quat_in(InPose.orientation.x, InPose.orientation.y, InPose.orientation.z, InPose.orientation.w);
			tf::Matrix3x3(quat_in).getRPY(r, p, y);
			tf::Matrix3x3(quat_end).getRPY(r, p, _y);
			diff = y - _y;
			diff = diff > M_PI? 
				diff - 2 * M_PI : diff < -M_PI?
				2 * M_PI - diff : diff;
			diff = (int)std::floor(diff *180/M_PI);

#ifdef DEBUG_PRINT
			std::cout << "Yaw:" << _y << "-" << y << ":" << _y - y << std::endl;
			std::cout << "In:" << InPose.position.x << "," << InPose.position.y << std::endl;
			std::cout << "End:" << OutPose.position.x << "," << OutPose.position.y << std::endl;
			if (diff <= ANGLE_LEFT)
				std::cout << "LEFT :" << diff << std::endl;
			else if (diff >= ANGLE_RIGHT)
				std::cout << "RIGHT:" << diff << std::endl;
			else
				std::cout << "Straight:" << diff << std::endl;
#endif
		
		}

		return diff;
}

bool DecisionMakerNode::isLocalizationConvergence(double _x, double _y, double _z, double _roll, double _pitch,
		double _yaw)
{
	static int _init_count = 0;
	static euclidean_space::point *a = new euclidean_space::point();
	static euclidean_space::point *b = new euclidean_space::point();

	static std::vector<double> distances;
	static int distances_count = 0;
	double avg_distances = 0.0;

	a->x = b->x;
	a->y = b->y;
	a->z = b->z;

	b->x = _x;
	b->y = _y;
	b->z = _z;

	distances.push_back(euclidean_space::EuclideanSpace::find_distance(a, b));
	if (++distances_count > param_convergence_count_)
	{
		distances.erase(distances.begin());
		distances_count--;
		avg_distances = std::accumulate(distances.begin(), distances.end(), 0) / distances.size();
		if (avg_distances <= param_convergence_threshold_)
			return ctx->setCurrentState(state_machine::DRIVE_STATE);
	}
	else
	{
		return false;
	}
}
}
