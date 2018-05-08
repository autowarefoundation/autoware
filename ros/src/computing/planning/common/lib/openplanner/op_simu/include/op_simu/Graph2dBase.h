/*
 * Graph2dBase.h
 *
 *  Created on: Oct 8, 2016
 *      Author: hatem
 */

#ifndef GRAPH2DBASE_H_
#define GRAPH2DBASE_H_

#include "DrawingHelpers.h"
#include "op_planner/RoadNetwork.h"
#include <vector>

namespace Graphics {

class Graph2dBase {
public:
	void ReInitGraphResolution(double width, double height, int nMaxPoints, double a_color[], double g_color[]);
	Graph2dBase(double width, double height, int nMaxPoints, double max_y, double min_y, std::string str_title, std::string str_x, std::string str_y, double a_color[], double g_color[]);
	void UpdateComment(const std::string& str_com);
	virtual ~Graph2dBase();
	double DrawGraph();
	void InsertPoint(const double& x, const double& y);
	void InsertPointTimeStamp(const timespec& tStamp, const double& y);
	void InsertPointsList(const std::vector<PlannerHNS::GPSPoint>& points);


protected:
	double w,h;
	double nPoints;
	std::vector<PlannerHNS::GPSPoint> xy_arr;
	std::vector<PlannerHNS::GPSPoint> xy_arr_original;

	double axes_color[3];
	double graph_color[3];
	std::string graph_title;
	std::string x_name;
	std::string y_name;
	std::string str_comment;
	PlannerHNS::GPSPoint max_point;
	PlannerHNS::GPSPoint min_point;

	timespec m_PrevTimeStamp;
};

} /* namespace Graphics */

#endif /* GRAPH2DBASE_H_ */
