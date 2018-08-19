
/// \file  PolygonGenerator.h
/// \brief Generate convex hull from point cloud cluster of detected object
/// \author Hatem Darweesh
/// \date Nov 2, 2016

/*
 *  Copyright (c) 2016, Nagoya University
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

#ifndef OP_POLYGONGENERATOR_H_
#define OP_POLYGONGENERATOR_H_

#include "op_planner/RoadNetwork.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace ContourTrackerNS
{

class QuarterView
{
public:
	int id;
	int min_ang;
	int max_ang;
	PlannerHNS::WayPoint max_from_center;
	bool bFirst;

	QuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
		bFirst = true;
	}

	void InitQuarterView(const int& min_a, const int& max_a, const int& index)
	{
		min_ang = min_a;
		max_ang = max_a;
		id = index;
		bFirst = true;
	}

	void ResetQuarterView()
	{
		bFirst = true;
	}

	bool UpdateQuarterView(const PlannerHNS::WayPoint& v)
	{
		if(v.pos.a <= min_ang || v.pos.a > max_ang)
			return false;

		if(bFirst)
		{
			max_from_center = v;
			bFirst = false;
		}
		else if(v.cost > max_from_center.cost)
			max_from_center = v;

		return true;
	}

	bool GetMaxPoint(PlannerHNS::WayPoint& maxPoint)
	{
		if(bFirst)
			return false;
		else
			maxPoint = max_from_center;

		return true;
	}
};

class PolygonGenerator
{

public:

	PlannerHNS::GPSPoint m_Centroid;
	std::vector<QuarterView> m_Quarters;
	std::vector<PlannerHNS::GPSPoint> m_Polygon;

	PolygonGenerator(int nQuarters);
	virtual ~PolygonGenerator();
	std::vector<QuarterView> CreateQuarterViews(const int& nResolution);
	std::vector<PlannerHNS::GPSPoint> EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const PlannerHNS::GPSPoint& original_centroid, PlannerHNS::GPSPoint& new_centroid, const double& polygon_resolution = 1.0);
};

} /* namespace PlannerXNS */

#endif /* OP_POLYGONGENERATOR_H_ */
