/*
 *  Copyright (c) 2017, Nagoya University
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
#include "../include/OpenPlannerSimulatorPerception_core.h"

#include "op_utility/UtilityH.h"
#include "math.h"
#include "op_planner/MatrixOperations.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

namespace OpenPlannerSimulatorPerceptionNS
{

OpenPlannerSimulatorPerception::OpenPlannerSimulatorPerception()
{
	nh.getParam("/op_simulator_perception/simObjNumber" 			, m_DecParams.nSimuObjs);
	nh.getParam("/op_simulator_perception/GuassianErrorFactor" 	, m_DecParams.errFactor);

	pub_DetectedObjects 	= nh.advertise<autoware_msgs::CloudClusterArray>("cloud_clusters",1);

	for(int i=1; i <= m_DecParams.nSimuObjs; i++)
	{
		std::ostringstream str_pose;
		str_pose << "/op_simulator" << i << "/sim_box_pose_" << i;
		std::cout << "Subscribe to Topic : " <<  str_pose.str() <<  std::endl;

		ros::Subscriber _sub;
		_sub =  nh.subscribe(str_pose.str(), 10, &OpenPlannerSimulatorPerception::callbackGetSimuData, 		this);
		sub_objs.push_back(_sub);
	}

	ros::Subscriber _sub;
	_sub =  nh.subscribe("/sim_box_pose_ego", 10, &OpenPlannerSimulatorPerception::callbackGetSimuData, 		this);
	sub_objs.push_back(_sub);

	std::cout << "OpenPlannerSimulatorPerception initialized successfully " << std::endl;

}

OpenPlannerSimulatorPerception::~OpenPlannerSimulatorPerception()
{
}

void OpenPlannerSimulatorPerception::callbackGetSimuData(const geometry_msgs::PoseArray &msg)
{
	int obj_id = -1;
	if(msg.poses.size() > 0 )
	{
		obj_id = msg.poses.at(0).position.x;

	}

//	ROS_INFO("Obj ID = %d", obj_id);

	if(obj_id < 0)
		return;

	int index = -1;
	for(int i = 0; i < m_ObjClustersArray.clusters.size() ; i++ )
	{
		if(m_ObjClustersArray.clusters.at(i).id == obj_id)
		{
			index = i;
			break;
		}
	}

	autoware_msgs::CloudCluster c = GenerateSimulatedObstacleCluster(msg.poses.at(2).position.y, msg.poses.at(2).position.x, msg.poses.at(2).position.y, 50, msg.poses.at(1));
	c.id = obj_id;

	if(index >= 0) // update existing
	{
		m_ObjClustersArray.clusters.at(index) = c;
		m_keepTime.at(index).second = 10;
	//	ROS_INFO("Update Obj ID = %d", c.id);
	}
	else
	{
		m_ObjClustersArray.clusters.push_back(c);
		m_keepTime.push_back(make_pair(c.id, 10));
	//	ROS_INFO("Insert Obj ID = %d", c.id);
	}

//	geometry_msgs::Pose p;
//	p.position.x  = msg->pose.position.x + m_OriginPos.position.x;
//	p.position.y  = msg->pose.position.y + m_OriginPos.position.y;
//	p.position.z  = msg->pose.position.z + m_OriginPos.position.z;
//	p.orientation = msg->pose.orientation;

	//m_SimParams.startPose =  PlannerHNS::WayPoint(p.position.x, p.position.y, p.position.z , tf::getYaw(p.orientation));

}

autoware_msgs::CloudCluster OpenPlannerSimulatorPerception::GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::Pose& centerPose)
{
	autoware_msgs::CloudCluster cluster;

	cluster.centroid_point.point.x = centerPose.position.x;
	cluster.centroid_point.point.y = centerPose.position.y;
	cluster.centroid_point.point.z = centerPose.position.z;

	cluster.dimensions.x = x_rand;
	cluster.dimensions.y = y_rand;
	cluster.dimensions.z = z_rand;
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	timespec t;
	for(int i=1; i < nPoints; i++)
	{
		UtilityHNS::UtilityH::GetTickCount(t);
		pcl::PointXYZ p;
		srand(t.tv_nsec/i);
		double x = (double)(rand()%100)/100.0 - 0.5;

		srand(t.tv_nsec/i*i);
		double y = (double)(rand()%100)/100.0 - 0.5;

		srand(t.tv_nsec);
		double z = (double)(rand()%100)/100.0 - 0.5;

		p.x = centerPose.position.x + x*x_rand;
		p.y = centerPose.position.y + y*y_rand;
		p.z = centerPose.position.z + z*z_rand;
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

		if(m_ObjClustersArray.clusters.size()>0)
			pub_DetectedObjects.publish(m_ObjClustersArray);

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

		std::cout << "Number of Obstacles: (" << m_keepTime.size() << ", " << m_ObjClustersArray.clusters.size() << ") "<< std::endl;

		loop_rate.sleep();
	}
}

}
