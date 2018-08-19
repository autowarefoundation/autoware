/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "op_perception_simulator_core.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>
#include "op_ros_helpers/op_RosHelpers.h"

#include "op_utility/UtilityH.h"
#include "math.h"
#include "op_planner/MatrixOperations.h"

namespace PerceptionSimulatorNS
{

typedef boost::mt19937 ENG;
typedef boost::normal_distribution<double> NormalDIST;
typedef boost::variate_generator<ENG, NormalDIST> VariatGEN;

constexpr double SIMU_OBSTACLE_WIDTH = 1.5;
constexpr double SIMU_OBSTACLE_LENGTH = 1.5;
constexpr double SIMU_OBSTACLE_HEIGHT = 1.4;
constexpr double SIMU_OBSTACLE_POINTS_NUM = 50;
constexpr int SIMU_OBSTACLE_ID = 100001;

OpenPlannerSimulatorPerception::OpenPlannerSimulatorPerception()
{
	m_bSetSimulatedObj = false;
	nh.getParam("/op_perception_simulator/simObjNumber" , m_DecParams.nSimuObjs);
	nh.getParam("/op_perception_simulator/GuassianErrorFactor" , m_DecParams.errFactor);
	nh.getParam("/op_perception_simulator/pointCloudPointsNumber" , m_DecParams.nPointsPerObj);

	pub_DetectedObjects = nh.advertise<autoware_msgs::CloudClusterArray>("cloud_clusters",1);

	sub_simulated_obstacle_pose_rviz = nh.subscribe("/clicked_point", 1, &OpenPlannerSimulatorPerception::callbackGetRvizPoint,	this);

	for(int i=1; i <= m_DecParams.nSimuObjs; i++)
	{
		std::ostringstream str_pose;
		//str_pose << "/op_car_simulator" << i << "/sim_box_pose_" << i;
		str_pose << "/sim_box_pose_" << i;
		std::cout << "Subscribe to Topic : " <<  str_pose.str() <<  std::endl;

		ros::Subscriber _sub;
		_sub =  nh.subscribe(str_pose.str(), 10, &OpenPlannerSimulatorPerception::callbackGetSimuData, this);
		sub_objs.push_back(_sub);
	}

	ros::Subscriber _sub;
	_sub =  nh.subscribe("/sim_box_pose_ego", 10, &OpenPlannerSimulatorPerception::callbackGetSimuData, this);
	sub_objs.push_back(_sub);

	std::cout << "OpenPlannerSimulatorPerception initialized successfully " << std::endl;

}

OpenPlannerSimulatorPerception::~OpenPlannerSimulatorPerception()
{
}


void OpenPlannerSimulatorPerception::callbackGetRvizPoint(const geometry_msgs::PointStampedConstPtr& msg)
{
	tf::StampedTransform transform;
	PlannerHNS::RosHelpers::GetTransformFromTF("map", "world", transform);

	geometry_msgs::Pose point;
	point.position.x = msg->point.x + transform.getOrigin().x();
	point.position.y = msg->point.y + transform.getOrigin().y();
	point.position.z = msg->point.z + transform.getOrigin().z();
	point.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

	m_SimulatedCluter = GenerateSimulatedObstacleCluster(SIMU_OBSTACLE_WIDTH , SIMU_OBSTACLE_LENGTH, SIMU_OBSTACLE_HEIGHT, SIMU_OBSTACLE_POINTS_NUM, point);
	m_SimulatedCluter.id = SIMU_OBSTACLE_ID;
	m_SimulatedCluter.score = 0; //zero velocity
	m_SimulatedCluter.indicator_state = 3; // default indicator value

	m_bSetSimulatedObj = true;
}

void OpenPlannerSimulatorPerception::callbackGetSimuData(const geometry_msgs::PoseArray &msg)
{
	int obj_id = -1;
	double actual_speed = 0;
	double actual_steering  = 0;
	int indicator = 3;

	if(msg.poses.size() > 0 )
	{
		obj_id = msg.poses.at(0).position.x;
		actual_speed = msg.poses.at(0).position.y;
		actual_steering = msg.poses.at(0).position.z;
	}

	if(msg.poses.size() == 4)
	{
		indicator = msg.poses.at(3).orientation.w;
	}

	if(obj_id < 0)
		return;

	int index = -1;
	for(unsigned int i = 0; i < m_ObjClustersArray.clusters.size() ; i++ )
	{
		if((int)m_ObjClustersArray.clusters.at(i).id == obj_id)
		{
			index = i;
			break;
		}
	}

	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);
	int nPoints = m_DecParams.nPointsPerObj + (rand()%POINT_CLOUD_ADDTIONAL_ERR_NUM - POINT_CLOUD_ADDTIONAL_ERR_NUM/2);

	autoware_msgs::CloudCluster c = GenerateSimulatedObstacleCluster(msg.poses.at(2).position.y, msg.poses.at(2).position.x, msg.poses.at(2).position.z, nPoints, msg.poses.at(1));
	c.id = obj_id;
	c.score = actual_speed;
	c.indicator_state = indicator;

	if(index >= 0) // update existing
	{
		m_ObjClustersArray.clusters.at(index) = c;
		m_keepTime.at(index).second = OBJECT_KEEP_TIME;
	}
	else
	{
		m_ObjClustersArray.clusters.push_back(c);
		m_keepTime.push_back(std::make_pair(c.id, OBJECT_KEEP_TIME));
	}
}


autoware_msgs::CloudCluster OpenPlannerSimulatorPerception::GenerateSimulatedObstacleCluster(const double& width, const double& length, const double& height, const int& nPoints, const geometry_msgs::Pose& centerPose)
{
	autoware_msgs::CloudCluster cluster;

	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);

	ENG eng(t.tv_nsec);
	NormalDIST dist_x(0, m_DecParams.errFactor);
	VariatGEN gen_x(eng, dist_x);

	cluster.centroid_point.point.x = centerPose.position.x + gen_x();
	cluster.centroid_point.point.y = centerPose.position.y + gen_x();
	cluster.centroid_point.point.z = centerPose.position.z;

	cluster.avg_point.point.x = centerPose.position.x;
	cluster.avg_point.point.y = centerPose.position.y;
	cluster.avg_point.point.z = centerPose.position.z;

	double yaw_angle = tf::getYaw(centerPose.orientation);
	cluster.estimated_angle = yaw_angle;

	cluster.dimensions.x = width;
	cluster.dimensions.y = length;
	cluster.dimensions.z = height;
	pcl::PointCloud<pcl::PointXYZI> point_cloud;

	PlannerHNS::Mat3 rotationMat(yaw_angle);
	PlannerHNS::Mat3 translationMat(cluster.avg_point.point.x, cluster.avg_point.point.y);

	for(int i=1; i < nPoints; i++)
	{
		UtilityHNS::UtilityH::GetTickCount(t);
		PlannerHNS::WayPoint center_p;
		srand(t.tv_nsec);

		center_p.pos.x = ((double)(rand()%100)/100.0 - CONTOUR_DISTANCE_ERROR);
		center_p.pos.x *= width;

		srand(t.tv_nsec/i);
		center_p.pos.y = ((double)(rand()%100)/100.0 - CONTOUR_DISTANCE_ERROR);
		center_p.pos.y *= length;

		srand(t.tv_nsec/i*i);
		center_p.pos.z = ((double)(rand()%100)/100.0 - CONTOUR_DISTANCE_ERROR)* height;

		center_p.pos = rotationMat*center_p.pos;
		center_p.pos = translationMat*center_p.pos;

		pcl::PointXYZI p;
		p.x = center_p.pos.x;
		p.y = center_p.pos.y;
		p.z = center_p.pos.z;

		point_cloud.points.push_back(p);
	}

	pcl::toROSMsg(point_cloud, cluster.cloud);

	return cluster;
}

void OpenPlannerSimulatorPerception::MainLoop()
{

	ros::Rate loop_rate(15);

	while (ros::ok())
	{
		ros::spinOnce();

		//clean old data
		for(unsigned int i = 0 ; i < m_keepTime.size(); i++)
		{
			if(m_keepTime.at(i).second <= 0)
			{
				m_keepTime.erase(m_keepTime.begin()+i);
				m_ObjClustersArray.clusters.erase(m_ObjClustersArray.clusters.begin()+i);
				i--;
			}
			else
				m_keepTime.at(i).second -= 1;
		}

		if(m_bSetSimulatedObj)
		{
			m_AllObjClustersArray = m_ObjClustersArray;
			m_AllObjClustersArray.clusters.push_back(m_SimulatedCluter);
			pub_DetectedObjects.publish(m_AllObjClustersArray);
		}
		else
		{
			pub_DetectedObjects.publish(m_ObjClustersArray);
		}

		loop_rate.sleep();
	}
}

}
