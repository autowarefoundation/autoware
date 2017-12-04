/*
 * Graph2dBase.cpp
 *
 *  Created on: Oct 8, 2016
 *      Author: hatem
 */

#include "Graph2dBase.h"
#include "UtilityH.h"

using namespace PlannerHNS;


namespace Graphics
{

#define BORDER_MARGIN 20
#define AXE_WIDTH 2

Graph2dBase::Graph2dBase(double width, double height, int nMaxPoints, double max_y, double min_y, std::string str_title, std::string str_x, std::string str_y, double a_color[], double g_color[]) {

	w = width;
	h = height;
	nPoints = nMaxPoints;
	graph_title = str_title;
	x_name = str_x;
	y_name = str_y;


	axes_color[0] = a_color[0];
	axes_color[1] = a_color[1];
	axes_color[2] = a_color[2];

	graph_color[0] = g_color[0];
	graph_color[1] = g_color[1];
	graph_color[2] = g_color[2];

	m_PrevTimeStamp.tv_nsec = 0;
	m_PrevTimeStamp.tv_sec = 0;
	max_point.y = max_y;
	min_point.y = min_y;


}

void Graph2dBase::ReInitGraphResolution(double width, double height, int nMaxPoints, double a_color[], double g_color[])
{
	w = width;
	h = height;
	nPoints = nMaxPoints;
	axes_color[0] = a_color[0];
	axes_color[1] = a_color[1];
	axes_color[2] = a_color[2];

	graph_color[0] = g_color[0];
	graph_color[1] = g_color[1];
	graph_color[2] = g_color[2];
}

Graph2dBase::~Graph2dBase()
{

}


double Graph2dBase::DrawGraph()
{

	glDisable(GL_LIGHTING);

	glBegin(GL_POLYGON);
		glColor3f(0.9,0.9,0.9);
		glNormal3f(0.1, 0.1, 0.1);
		glVertex3f(0, 0, 0);
		glVertex3f(0, h, 0);
		glVertex3f(w, h, 0);
		glVertex3f(w, 0, 0);
	glEnd();


	GPSPoint p_origin, x_end, y_end;
	p_origin.x = BORDER_MARGIN;
	p_origin.y = h - BORDER_MARGIN*2;

	x_end.x = w - BORDER_MARGIN*2;
	x_end.y = h - BORDER_MARGIN*2;

	y_end.x = BORDER_MARGIN;
	y_end.y = BORDER_MARGIN;


	glColor3f(axes_color[0],axes_color[1],axes_color[2]);
	DrawingHelpers::DrawLinePoygonline(p_origin, x_end, AXE_WIDTH);
	DrawingHelpers::DrawLinePoygonline(p_origin, y_end, AXE_WIDTH);

	glColor3f(graph_color[0],graph_color[1],graph_color[2]);
	DrawingHelpers::DrawString(x_end.x, x_end.y, GLUT_BITMAP_HELVETICA_18, (char*)x_name.c_str());
	DrawingHelpers::DrawString(y_end.x, y_end.y, GLUT_BITMAP_HELVETICA_18, (char*)y_name.c_str());
	DrawingHelpers::DrawString(BORDER_MARGIN, p_origin.y+BORDER_MARGIN, GLUT_BITMAP_HELVETICA_18, (char*)graph_title.c_str());


	glPointSize(4);
	glBegin(GL_POINTS);
		for(unsigned int i = 1 ; i < xy_arr.size(); i++)
		{
			//glVertex3f(xy_arr.at(i-1).x, xy_arr.at(i-1).y, 0);
			glVertex3f(xy_arr.at(i).x, xy_arr.at(i).y, 0);
		}
	glEnd();
	glBegin(GL_LINES);
		for(unsigned int i = 1 ; i < xy_arr.size(); i++)
		{
			glVertex3f(xy_arr.at(i-1).x, xy_arr.at(i-1).y, 0);
			glVertex3f(xy_arr.at(i).x, xy_arr.at(i).y, 0);
		}
	glEnd();


	glEnable(GL_LIGHTING);
}

void Graph2dBase::InsertPointTimeStamp(const timespec& tStamp, const double& y)
{
	GPSPoint p(0,y,0,0);
	if(xy_arr_original.size() == 0)
	{
		xy_arr_original.push_back(p);
	}
	else
	{
		if(tStamp.tv_nsec == 0 && tStamp.tv_sec == 0)
		{
			p.x = xy_arr_original.at(xy_arr_original.size()-1).x+1;
		}
		else
		{
			double t = UtilityHNS::UtilityH::GetTimeDiff(m_PrevTimeStamp, tStamp);
			p.x = xy_arr_original.at(xy_arr_original.size()-1).x+t;
		}
	}



	double initial_x = xy_arr_original.at(0).x;

//	if(xy_arr_original.size() > nPoints)
//	{
//		initial_x = xy_arr_original.at(0).x;
//		xy_arr_original.erase(xy_arr_original.begin()+0);
//	}



	xy_arr_original.push_back(p);

	xy_arr.clear();


	for(unsigned int i=0; i<xy_arr_original.size(); i++ )
	{
		//p.x = BORDER_MARGIN + ((xy_arr_original.at(i).x - avg_x) / (max_point.x - min_point.x) * 10.0);
		//p.y = h - BORDER_MARGIN*2 - ((xy_arr_original.at(i).y - avg_y) / (max_point.y - min_point.y) * 10.0);
		double y_val = (h - BORDER_MARGIN*3) * ((xy_arr_original.at(i).y - min_point.y) / (max_point.y - min_point.y));
		double x_val = (xy_arr_original.at(i).x - initial_x)*10.0;

		p.x = BORDER_MARGIN + x_val;
		p.y = (h - BORDER_MARGIN*2) - y_val;
		xy_arr.push_back(p);
	}

	double total_x = 0;

	for(int i=xy_arr.size()-1; i>0; i-- )
	{
		total_x += (xy_arr.at(i).x - xy_arr.at(i-1).x);
		if(total_x > (w - BORDER_MARGIN*3))
		{
			xy_arr_original.erase(xy_arr_original.begin(), xy_arr_original.begin()+i);
			break;
		}
	}

	m_PrevTimeStamp = tStamp;
}

void Graph2dBase::InsertPoint(const double& x, const double& y)
{

}

void Graph2dBase::InsertPointsList(const std::vector<PlannerHNS::GPSPoint>& points)
{

}

void Graph2dBase::UpdateComment(const std::string& str_com)
{
	str_comment = str_com;
}

} /* namespace Graphics */
