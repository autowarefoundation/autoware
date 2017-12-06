/*
 * PolygonGenerator.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: ai-driver
 */

#include "PolygonGenerator.h"

namespace PlannerXNS
{


PolygonGenerator::PolygonGenerator() {
	// TODO Auto-generated constructor stub

}

PolygonGenerator::~PolygonGenerator() {
	// TODO Auto-generated destructor stub
}

PlannerHNS::GPSPoint PolygonGenerator::CalculateCentroid(const pcl::PointCloud<pcl::PointXYZ>& cluster)
{
	PlannerHNS::GPSPoint c;

	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		c.x += cluster.points.at(i).x;
		c.y += cluster.points.at(i).y;
	}

	c.x = c.x/cluster.points.size();
	c.y = c.y/cluster.points.size();

	return c;
}

std::vector<PlannerHNS::GPSPoint> PolygonGenerator::EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const PlannerHNS::GPSPoint& original_centroid )
{
	std::vector<QuarterView> quarters = CreateQuarterViews(QUARTERS_NUMBER);


	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		PlannerHNS::WayPoint p;
		p.pos.x = cluster.points.at(i).x;
		p.pos.y = cluster.points.at(i).y;
		p.pos.z = original_centroid.z;

		PlannerHNS::GPSPoint v(p.pos.x - original_centroid.x , p.pos.y - original_centroid.y,p.pos.z,0);
		p.cost = pointNorm(v);
		p.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);

		for(unsigned int j = 0 ; j < quarters.size(); j++)
		{
			if(quarters.at(j).UpdateQuarterView(p))
				break;
		}
	}

	std::vector<PlannerHNS::GPSPoint> polygon;

	for(unsigned int j = 0 ; j < quarters.size(); j++)
	{

		PlannerHNS::WayPoint wp;
		int nPoints = quarters.at(j).GetMaxPoint(wp);
		if(nPoints >= MIN_POINTS_PER_QUARTER)
		{
			polygon.push_back(wp.pos);
		}
	}

//	//Fix Resolution:
//	bool bChange = true;
//	while (bChange && polygon.size()>1)
//	{
//		bChange = false;
//		GPSPoint p1 =  polygon.at(polygon.size()-1);
//		for(unsigned int i=0; i< polygon.size(); i++)
//		{
//			GPSPoint p2 = polygon.at(i);
//			double d = hypot(p2.y- p1.y, p2.x - p1.x);
//			if(d > MIN_DISTANCE_BETWEEN_CORNERS)
//			{
//				GPSPoint center_p = p1;
//				center_p.x = (p2.x + p1.x)/2.0;
//				center_p.y = (p2.y + p1.y)/2.0;
//				polygon.insert(polygon.begin()+i, center_p);
//				bChange = true;
//				break;
//			}
//
//			p1 = p2;
//		}
//	}

	return polygon;

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

void CheckConvexPoligon(std::vector<PlannerHNS::WayPoint>& polygon)
{

//	if(polygon.size() <= 3)
//		return;
//
//	WayPoint p1 = polygon.at(0);
//	WayPoint p3;
//	WayPoint p2;
//
//	for(int i=1; i< polygon.size()-1; i++)
//	{
//		p1 = polygon.at(i-1);
//		if(i+2 == polygon.size())
//		{
//			p2 = polygon.at(polygon.size()-1);
//			p3 = polygon.at(0);
//		}
//		else
//		{
//			p2 = polygon.at(i);
//			p3 = polygon.at(i+1);
//		}
//
//	}
}

} /* namespace PlannerXNS */
