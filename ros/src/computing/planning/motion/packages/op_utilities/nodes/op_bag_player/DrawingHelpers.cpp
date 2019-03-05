/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "DrawingHelpers.h"
#include <stdarg.h>
#include <stdio.h>
#include <cmath>
#include "op_utility/UtilityH.h"
#include "op_planner/PlanningHelpers.h"
#include <GL/freeglut.h>

using namespace std;
using namespace PlannerHNS;
using namespace UtilityHNS;

namespace OP_TESTING_NS
{

DrawingHelpers::DrawingHelpers()
{
}

DrawingHelpers::~DrawingHelpers()
{
}

void DrawingHelpers::DrawString(float x, float y, GLvoid* font_style, char* format, ...)
{
	glDisable(GL_LIGHTING);

	va_list args;
	char buffer[1000], *s;

	va_start(args, format);
	vsprintf(buffer, format, args);
	va_end(args);
	//GLuint ox = x;
	GLuint oy = y;

	glRasterPos2f(x, y);
	for (s = buffer; *s; s++)
	{
		if(*s == ',')
		{
			x += 220;
			y = oy;
			glRasterPos2f(x, y);
			continue;
		}
		else if(*s == '\n')
		{
			y+=12;
			glRasterPos2f(x, y);
			continue;
		}

		glutBitmapCharacter(font_style, *s);
	}
	glEnable(GL_LIGHTING);
}

void DrawingHelpers::DrawGrid(const double& x, const double& y, const double& w, const double& h, const double& cell_l)
{
	glPushMatrix();
	int nVerticalLisne   = floor(w/cell_l);
	int nHorizontalLines = floor(h/cell_l);

	glBegin(GL_LINES);
	glColor3ub(210,210,210);
	double incr = y;
	for(int r=0; r<= nHorizontalLines; r++)
	{
		glNormal3f(1.0, 1.0, 1.0);
		glVertex3f(x, incr, 0);
		glVertex3f(x+w, incr, 0);
		incr+=cell_l;
	}

	double incc = x;
	for(int r=0; r<= nVerticalLisne; r++)
	{
		glNormal3f(1.0, 1.0, 1.0);
		glVertex3f(incc, y,  0);
		glVertex3f(incc, y + h, 0);
		incc+=cell_l;
	}
	glEnd();

	glPopMatrix();
}

void DrawingHelpers::DrawArrow(const double& x, const double& y, const double& a)
{
	const int nSlicesStacks = 50;
	const double percent = 20.0;
	const double innerPercent = 15.0;
	double half_length = 10/2.0;

	glPushMatrix();
	//Draw one cylender and cone
	glTranslated(x, y, 0.5);
	glRotated(a*RAD2DEG, 0,0,1);

	//X Axis
	glPushMatrix();
	glColor3ub(200,200,200);
	glRotated(90, 0,1,0);
	glutSolidCylinder(half_length/percent, half_length,nSlicesStacks,nSlicesStacks);
	glTranslated(0,0,half_length);
	glColor3f(1,1,0);
	glutSolidCone(half_length/innerPercent, half_length/innerPercent,nSlicesStacks,nSlicesStacks);
	glPopMatrix();

	glPopMatrix();
}

void DrawingHelpers::DrawCustomOrigin(const double& x, const double& y, const double& z, const int& yaw, const int& roll, const int& pitch, const double& length)
{
	const int nSlicesStacks = 50;
	const double percent = 20.0;
	const double innerPercent = 15.0;
	double half_length = length/2.0;

	glPushMatrix();
	//Draw one cylender and cone
	glTranslated(x, y, z);
	glRotated(yaw, 0,0,1);
	glRotated(roll, 1,0,0);
	glRotated(pitch, 0,1,0);

	//Z Axis
	glPushMatrix();
	glColor3f(0.65,0.65,0.65);
	glutSolidCylinder(half_length/percent, half_length,nSlicesStacks,nSlicesStacks);
	glTranslated(0,0,half_length);
	glColor3f(0,0,1);
	glutSolidCone(half_length/innerPercent, half_length/innerPercent,nSlicesStacks,nSlicesStacks);
	glPopMatrix();

	//X Axis
	glPushMatrix();
	glColor3f(0.65,0.65,0.65);
	glRotated(90, 0,1,0);
	glutSolidCylinder(half_length/percent, half_length,nSlicesStacks,nSlicesStacks);
	glTranslated(0,0,half_length);
	glColor3f(1,1,0);
	glutSolidCone(half_length/innerPercent, half_length/innerPercent,nSlicesStacks,nSlicesStacks);
	glPopMatrix();

//	//Y Axis
	glPushMatrix();
	glColor3f(0.65,0.65,0.65);
	glRotated(90, 1,0,0);
	glutSolidCylinder(half_length/percent, half_length, nSlicesStacks, nSlicesStacks);
	glTranslated(0,0,half_length);
	glColor3f(1,0,0);
	glutSolidCone(half_length/innerPercent, half_length/innerPercent, nSlicesStacks,nSlicesStacks);
	glPopMatrix();

	//glDisable(GL_LIGHTING);
	glPopMatrix();

}

vector<vector<float> > DrawingHelpers::PreparePathForDrawing(const std::vector<PlannerHNS::WayPoint>& path,
		std::vector<std::vector<PlannerHNS::WayPoint> >& redyForDraw, double w, double resolution)
{
	vector<vector<float> > colorProfiles;
	if(path.size() < 2) return colorProfiles;
	int size = path.size();
	WayPoint p1 = path[0];
	WayPoint p2 =p1;
	WayPoint prev_point = p1;
	WayPoint center, prev_center ,pa, pb, pc, pd;
	double a = 0;
	double prev_angle = 0;
	vector<WayPoint> four_temp;
	vector<float> color_vector;

	for(int i=0; i < size ; i++)
	{

		color_vector.clear();
		four_temp.clear();

		pa = p2 = path[i];

		color_vector.push_back(p1.v/12.0);
		color_vector.push_back(p1.v/12.0);
		color_vector.push_back(p1.v/12.0);
		colorProfiles.push_back(color_vector);

		if(distance2points(p1.pos, p2.pos) < resolution)
		  continue;

		center.pos.x = p1.pos.x + (p2.pos.x-p1.pos.x)/2.0;
		center.pos.y = p1.pos.y + (p2.pos.y-p1.pos.y)/2.0;

		a = atan2(p2.pos.y- p1.pos.y, p2.pos.x- p1.pos.x);

		pa.pos.x = p1.pos.x - w * cos(a - M_PI/2.0);
		pa.pos.y = p1.pos.y - w * sin(a - M_PI/2.0);
		pa.pos.z = p1.pos.z;

		pb.pos.x = p1.pos.x + w * cos(a - M_PI/2.0);
		pb.pos.y = p1.pos.y + w * sin(a - M_PI/2.0);
		pb.pos.z = p1.pos.z;


		pc.pos.x = p2.pos.x + w * cos(a - M_PI/2.0);
		pc.pos.y = p2.pos.y + w * sin(a - M_PI/2.0);
		pc.pos.z = p2.pos.z;

		pd.pos.x = p2.pos.x - w * cos(a - M_PI/2.0);
		pd.pos.y = p2.pos.y - w * sin(a - M_PI/2.0);
		pd.pos.z = p2.pos.z;

		if(!(prev_point.pos.x == p1.pos.x &&  prev_point.pos.y == p1.pos.y))
		{
			prev_angle = atan2(p1.pos.y- prev_point.pos.y, p1.pos.x- prev_point.pos.x);

			pa.pos.x = p1.pos.x - w * cos(prev_angle - M_PI/2.0);
			pa.pos.y = p1.pos.y - w * sin(prev_angle - M_PI/2.0);

			pb.pos.x = p1.pos.x + w * cos(prev_angle - M_PI/2.0);
			pb.pos.y = p1.pos.y + w * sin(prev_angle - M_PI/2.0);
		}

	  	four_temp.push_back(pa);
	  	four_temp.push_back(pb);
	  	four_temp.push_back(pc);
	  	four_temp.push_back(pd);

	  	redyForDraw.push_back(four_temp);

		prev_point = p1;
		p1 = p2;
	}
	  return colorProfiles;
}

void DrawingHelpers::DrawPrePreparedPolygons(std::vector<std::vector<PlannerHNS::WayPoint> >& path,
		double z, float color[3],int nSkipPoints, const std::vector<std::vector<float> >* colorProfile)
{


	if(!colorProfile)
		glColor3f(color[0], color[1], color[2]);

	for(unsigned int i=0; i< path.size(); i+=nSkipPoints)
	{
		if(path[i].size() == 4)
		{
			if(path[i][0].pLane && (path[i][0].pLane->pRightLane || path[i][0].pLane->pLeftLane))
				glColor3f(1, 0, 0);
			else if(colorProfile)
			{
				glColor3f(color[0]*(*colorProfile)[i][0], color[1] * (*colorProfile)[i][1], color[2] * (*colorProfile)[i][2]);
			}

			 glBegin(GL_POLYGON);
				  glNormal3f(0.0, 0.0, 0.1);
//				  glVertex3f(path[i][0].p.x, path[i][0].p.y,path[i][0].p.z+z);
//				  glVertex3f(path[i][1].p.x, path[i][1].p.y,path[i][1].p.z+z);
//				  glVertex3f(path[i][2].p.x, path[i][2].p.y,path[i][2].p.z+z);
//				  glVertex3f(path[i][3].p.x, path[i][3].p.y,path[i][3].p.z+z);
				  glVertex3f(path[i][0].pos.x, path[i][0].pos.y,z);
				  glVertex3f(path[i][1].pos.x, path[i][1].pos.y,z);
				  glVertex3f(path[i][2].pos.x, path[i][2].pos.y,z);
				  //glVertex3f((path[i][2].p.x+path[i][1].p.x)/2.0, (path[i][2].p.y+path[i][1].p.y)/2.0,z);
				  glVertex3f(path[i][3].pos.x, path[i][3].pos.y,z);
			  glEnd();
		}
	}


}

void DrawingHelpers::DrawCostPath(const std::vector<PlannerHNS::WayPoint*>& path_points, const double& z, const double& width)
{
	if(path_points.size()==0) return;

	WayPoint p1 = *path_points[0];
	float color[3] = {0,0,0};

	double max_cost = 0;
	for(unsigned int i=0; i < path_points.size(); i++)
	{
		if(path_points.at(i)->cost > max_cost)
			max_cost = path_points.at(i)->cost;
	}

	int size = path_points.size();

	for(int i=0; i < size; i++)
	{
		p1 = *path_points[i];
		double norm_cost = path_points.at(i)->cost / max_cost * 2.0;
		if(norm_cost <= 1.0)
		{
			color[0] = norm_cost;
			color[1] = 1.0;
		}
		else if(norm_cost > 1.0)
		{
			color[0] = 1.0;
			color[1] = 2.0 - norm_cost;
		}

		glColor3f(color[0], color[1], color[2]);

		//DrawLinePoygonFromCenterX(p1, z, p2, z, width, 0, prev_point);
		DrawWideEllipse(p1.pos.x, p1.pos.y, z, 0.5, 0.5, 0.25, color);
	}
}

void DrawingHelpers::DrawWidePath(const std::vector<PlannerHNS::WayPoint>& path_points, const double& z, const double& width, float color[3], bool bGadient)
{
	if(path_points.size()==0) return;

	WayPoint p1 = path_points[0];
	WayPoint p2 = p1;

	float localColor[3] = {color[0],color[1],color[2]};

	int size = path_points.size();
	WayPoint prev_point = p1;

	for(int i=1; i < size; i+=2)
	{
		p2 = path_points[i];
		if(bGadient)
		{
			localColor[0] = color[0] * (float)(i+20)*3/(float)size;
			localColor[1] = color[1] * (float)(i+20)*3/(float)size;
			localColor[2] = color[2] * (float)(i+20)*3/(float)size;
		}

		if(p2.bDir == BACKWARD_DIR)
			glColor3f(1,0, 0);
		else
			glColor3f(localColor[0],localColor[1],localColor[2]);

		DrawLinePoygonFromCenterX(p1, z, p2, z, width, 0, prev_point);

		prev_point = p1;

		p1 = p2;
	}
}

void DrawingHelpers::DrawLinePoygonline(const PlannerHNS::GPSPoint& p1, const PlannerHNS::GPSPoint& p2, const double& w)
{
	GPSPoint center, prev_center ,pa, pb, pc, pd, prev_pa,prev_pb;
	double a = 0;

	center.x = p1.x + (p2.x-p1.x)/2.0;
	center.y = p1.y + (p2.y-p1.y)/2.0;

	 a = atan2(p2.y- p1.y, p2.x- p1.x);

	pa.x = p1.x - w * cos(a - M_PI/2.0);
	pa.y = p1.y - w * sin(a - M_PI/2.0);

	pb.x = p1.x + w * cos(a - M_PI/2.0);
	pb.y = p1.y + w * sin(a - M_PI/2.0);


	pc.x = p2.x + w * cos(a - M_PI/2.0);
	pc.y = p2.y + w * sin(a - M_PI/2.0);

	pd.x = p2.x - w * cos(a - M_PI/2.0);
	pd.y = p2.y - w * sin(a - M_PI/2.0);

	glBegin(GL_POLYGON);
	  glNormal3f(0.1, 0.1, 0.1);
	  glVertex3f(pa.x, pa.y, p1.z);
	  glVertex3f(pb.x, pb.y, p1.z);
	  glVertex3f(pc.x, pc.y, p2.z);
	  glVertex3f(pd.x, pd.y, p2.z);
	glEnd();
}

void DrawingHelpers::DrawLinePoygonFromCenterX(const PlannerHNS::WayPoint& p1, const double& z,
		const PlannerHNS::WayPoint& p2, const double& z2, const double& w, const double& h,
		PlannerHNS::WayPoint& prev_point)
{
	GPSPoint center, prev_center ,pa, pb, pc, pd, prev_pa,prev_pb;
	double a = 0;
	double prev_angle = 0;

	center.x = p1.pos.x + (p2.pos.x-p1.pos.x)/2.0;
	center.y = p1.pos.y + (p2.pos.y-p1.pos.y)/2.0;

	 a = atan2(p2.pos.y- p1.pos.y, p2.pos.x- p1.pos.x);

	pa.x = p1.pos.x - w * cos(a - M_PI/2.0);
	pa.y = p1.pos.y - w * sin(a - M_PI/2.0);

	pb.x = p1.pos.x + w * cos(a - M_PI/2.0);
	pb.y = p1.pos.y + w * sin(a - M_PI/2.0);


	pc.x = p2.pos.x + w * cos(a - M_PI/2.0);
	pc.y = p2.pos.y + w * sin(a - M_PI/2.0);

	pd.x = p2.pos.x - w * cos(a - M_PI/2.0);
	pd.y = p2.pos.y - w * sin(a - M_PI/2.0);

	if(!(prev_point.pos.x == p1.pos.x &&  prev_point.pos.y == p1.pos.y))
	{
		prev_angle = atan2(p1.pos.y- prev_point.pos.y, p1.pos.x- prev_point.pos.x);

		pa.x = p1.pos.x - w * cos(prev_angle - M_PI/2.0);
		pa.y = p1.pos.y - w * sin(prev_angle - M_PI/2.0);

		pb.x = p1.pos.x + w * cos(prev_angle - M_PI/2.0);
		pb.y = p1.pos.y + w * sin(prev_angle - M_PI/2.0);

	}

	  glBegin(GL_POLYGON);
		  glNormal3f(0.1, 0.1, 0.1);
		  glVertex3f(pa.x, pa.y,z);
		  glVertex3f(pb.x, pb.y, z);
		  glVertex3f(pc.x, pc.y,z);
		  glVertex3f(pd.x, pd.y, z);
	  glEnd();

}

void DrawingHelpers::DrawCustomCarModel(const PlannerHNS::WayPoint& pose,const double& steeringAngle, const std::vector<PlannerHNS::GPSPoint>& carPoints,float color[3], const double& angleFix)
{
	if(carPoints.size() == 4)
	{
		double z_margin = 0.05;

		glPushMatrix();
		glTranslated(pose.pos.x, pose.pos.y, pose.pos.z);
		glRotated(pose.pos.a*RAD2DEG + angleFix, 0,0,1);
		for(unsigned  int i = 0; i < 4; i++)
		{
			glBegin(GL_LINE_STRIP);
			//glColor3f(0,1,1);
			glColor3f(color[0],color[1],color[2]);
			glVertex3f(carPoints[i].x, carPoints[i].y, carPoints[i].z);
			glVertex3f(carPoints[i].x, carPoints[i].y, carPoints[i].z+1);

			glEnd();
		}

		glBegin(GL_POLYGON);
		//glColor3f(0,0,1);
		glColor3f(color[0],color[0],color[2]);
			glVertex3f(carPoints[0].x, carPoints[0].y, carPoints[0].z+z_margin);
			glVertex3f(carPoints[1].x, carPoints[1].y, carPoints[1].z+z_margin);
			glVertex3f(carPoints[2].x, carPoints[2].y, carPoints[2].z+z_margin);
			glVertex3f(carPoints[3].x, carPoints[3].y, carPoints[3].z+z_margin);
			glVertex3f(carPoints[0].x, carPoints[0].y, carPoints[0].z+z_margin);

			glVertex3f(carPoints[0].x, carPoints[0].y, carPoints[0].z+z_margin);
			glVertex3f(carPoints[2].x, carPoints[2].y, carPoints[2].z+z_margin);

			glVertex3f(carPoints[1].x, carPoints[1].y, carPoints[1].z+z_margin);
			glVertex3f(carPoints[3].x, carPoints[3].y, carPoints[3].z+z_margin);

		glEnd();

		glBegin(GL_LINE_LOOP);
		glColor3f(color[0],color[0],color[2]);
			glVertex3f(carPoints[0].x, carPoints[0].y, carPoints[0].z+1);
			glVertex3f(carPoints[1].x, carPoints[1].y, carPoints[1].z+1);
			glVertex3f(carPoints[2].x, carPoints[2].y, carPoints[2].z+1);
			glVertex3f(carPoints[3].x, carPoints[3].y, carPoints[3].z+1);
			glVertex3f(carPoints[0].x, carPoints[0].y, carPoints[0].z+1);

			glVertex3f(carPoints[0].x, carPoints[0].y, carPoints[0].z+1);
			glVertex3f(carPoints[2].x, carPoints[2].y, carPoints[2].z+1);

			glVertex3f(carPoints[1].x, carPoints[1].y, carPoints[1].z+1);
			glVertex3f(carPoints[3].x, carPoints[3].y, carPoints[3].z+1);

		glEnd();


		double width = fabs(carPoints[0].x - carPoints[2].x);
		double length = fabs(carPoints[0].y - carPoints[2].y);
		double innerRad = 1.0;
		double scale_factor = 0.1;
		glColor3f(0.05,0.05,0.05);

		glPushMatrix();
		glTranslated(width/2.0,length/2.0 - 0.5,0);
		glScaled(scale_factor, scale_factor, scale_factor);
		glRotated(90, 0,0,1);
		glRotated(90, 1,0,0);
		glutSolidTorus(innerRad, 3.0, 20, 20);
		glPopMatrix();

		glPushMatrix();
		glTranslated(-width/2.0,length/2.0 - 0.5,0);
		glScaled(scale_factor, scale_factor, scale_factor);
		glRotated(90, 0,0,1);
		glRotated(90, 1,0,0);
		glutSolidTorus(innerRad, 3.0, 20, 20);
		glPopMatrix();

		glPushMatrix();
		glTranslated(width/2.0,-length/2.0 + 0.5,0);
		glScaled(scale_factor, scale_factor, scale_factor);
		glRotated(90+steeringAngle*RAD2DEG, 0,0,1);
		glRotated(90, 1,0,0);
		glutSolidTorus(innerRad, 3.0, 20, 20);
		glPopMatrix();

		glPushMatrix();
		glTranslated(-width/2.0,-length/2.0 + 0.5,0);
		glScaled(scale_factor, scale_factor, scale_factor);
		glRotated(90+steeringAngle*RAD2DEG, 0,0,1);
		glRotated(90, 1,0,0);
		glutSolidTorus(innerRad, 3.0, 20, 20);
		glPopMatrix();




		glPopMatrix();
	}

	DrawCustomOrigin(pose.pos.x, pose.pos.y, pose.pos.z, pose.pos.a*RAD2DEG, 0,0, 2);
}

void DrawingHelpers::DrawFilledEllipse(float x, float y, float z, float width, float height)
{
	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLE_FAN);
		//All triangles fan out starting with this point
		glVertex3f (x,y,z);
		for (float i = 0; i <=M_PI*2*RAD2DEG; i+=0.1)
		{
			glVertex3f(x + width*cos(i), y+height*sin(i), z);
		}
	glEnd();
	glEnable(GL_LIGHTING);
}

void DrawingHelpers::DrawWideEllipse(float x, float y, float z, float outer_width, float outer_height,
		float inner_width,float color[3])
{
	//std::vector<WayPoint> ellipse_points;
	glColor3f(color[0], color[1], color[2]);
	GPSPoint p1 = GPSPoint(x + outer_width*cos(0),y + outer_height*sin(0),z,0);
	GPSPoint p2 = p1;
	for (float i = 0.1; i <= M_PI*2 + 0.1; i+=0.1)
	{
		//ellipse_points.push_back(WayPoint(x + outer_width*cos(i), y+outer_height*sin(i), z, 0));
		p2.x = x + outer_width*cos(i);
		p2.y = y + outer_height*sin(i);
		p2.z = z;

		DrawLinePoygonline(p1,p2, outer_width - inner_width);
		p1 = p2;

	}

	//DrawWidePath(ellipse_points, z, outer_width - inner_width,color);
}

void DrawingHelpers::DrawSimpleEllipse(float x, float y, float z, float outer_width, float outer_height)
{
	glBegin(GL_LINE_STRIP);
	for (float jj = 0; jj <=M_PI*2.0; jj+=0.1)
	{
		glVertex3f(x + outer_width*cos(jj), y+ outer_height*sin(jj),z);
	}
	glEnd();
}

void DrawingHelpers::DrawPedal(float x, float y, float z, float width, float height, float inner_height, float color[3])
{
	GPSPoint pa, pb, pc, pd;
	double w2 = width/2.0;
	double h2 = height/2.0;

	pa.x = x - w2;
	pa.y = y - h2;

	pb.x = x + w2;
	pb.y = y - h2;

	pc.x = x + w2;
	pc.y = y + h2;

	pd.x = x - w2;
	pd.y = y + h2;

	glBegin(GL_LINE_LOOP);
	  glVertex3f(pa.x, pa.y, z);
	  glVertex3f(pb.x, pb.y, z);
	  glVertex3f(pc.x, pc.y, z);
	  glVertex3f(pd.x, pd.y, z);
	glEnd();

	GPSPoint p1(x, y + h2, z, 0);
	GPSPoint p2(x, y + h2 - inner_height, z, 0);

	glColor3f(color[0], color[1], color[2]);
	DrawLinePoygonline(p1,p2,w2);

}

} /* namespace Graphics */
