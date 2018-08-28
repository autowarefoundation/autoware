
/// \file  PolygonGenerator.cpp
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

#include "PolygonGenerator.h"
#include "op_planner/PlanningHelpers.h"

namespace ContourTrackerNS
{

PolygonGenerator::PolygonGenerator(int nQuarters)
{
	m_Quarters = CreateQuarterViews(nQuarters);
}

PolygonGenerator::~PolygonGenerator()
{
}

std::vector<PlannerHNS::GPSPoint> PolygonGenerator::EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const PlannerHNS::GPSPoint& original_centroid, PlannerHNS::GPSPoint& new_centroid, const double& polygon_resolution)
{
	for(unsigned int i=0; i < m_Quarters.size(); i++)
			m_Quarters.at(i).ResetQuarterView();

	PlannerHNS::WayPoint p;
	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		p.pos.x = cluster.points.at(i).x;
		p.pos.y = cluster.points.at(i).y;
		p.pos.z = original_centroid.z;

		PlannerHNS::GPSPoint v(p.pos.x - original_centroid.x , p.pos.y - original_centroid.y, 0, 0);
		p.cost = pointNorm(v);
		p.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);

		for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
		{
			if(m_Quarters.at(j).UpdateQuarterView(p))
				break;
		}
	}

	m_Polygon.clear();
	PlannerHNS::WayPoint wp;
	for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
	{
		if(m_Quarters.at(j).GetMaxPoint(wp))
			m_Polygon.push_back(wp.pos);
	}

//	//Fix Resolution:
	bool bChange = true;
	while (bChange && m_Polygon.size()>1)
	{
		bChange = false;
		PlannerHNS::GPSPoint p1 =  m_Polygon.at(m_Polygon.size()-1);
		for(unsigned int i=0; i< m_Polygon.size(); i++)
		{
			PlannerHNS::GPSPoint p2 = m_Polygon.at(i);
			double d = hypot(p2.y- p1.y, p2.x - p1.x);
			if(d > polygon_resolution)
			{
				PlannerHNS::GPSPoint center_p = p1;
				center_p.x = (p2.x + p1.x)/2.0;
				center_p.y = (p2.y + p1.y)/2.0;
				m_Polygon.insert(m_Polygon.begin()+i, center_p);
				bChange = true;
				break;
			}

			p1 = p2;
		}
	}
	PlannerHNS::GPSPoint sum_p;
	for(unsigned int i = 0 ; i< m_Polygon.size(); i++)
	{
		sum_p.x += m_Polygon.at(i).x;
		sum_p.y += m_Polygon.at(i).y;
	}

	new_centroid = original_centroid;

	if(m_Polygon.size() > 0)
	{
		new_centroid.x = sum_p.x / (double)m_Polygon.size();
		new_centroid.y = sum_p.y / (double)m_Polygon.size();
	}

	return m_Polygon;

}

std::vector<QuarterView> PolygonGenerator::CreateQuarterViews(const int& nResolution)
{
	std::vector<QuarterView> quarters;
	if(nResolution <= 0)
		return quarters;

	double range = 360.0 / nResolution;
	double angle = 0;
	for(int i = 0; i < nResolution; i++)
	{
		QuarterView q(angle, angle+range, i);
		quarters.push_back(q);
		angle+=range;
	}

	return quarters;
}

} /* namespace PlannerXNS */
