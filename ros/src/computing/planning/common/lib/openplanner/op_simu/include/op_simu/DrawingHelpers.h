/*
 * DrawingHelpers.h
 *
 *  Created on: May 31, 2016
 *      Author: hatem
 */

#ifndef DRAWINGHELPERS_H_
#define DRAWINGHELPERS_H_

#include "glm.h"
#include "op_planner/RoadNetwork.h"
#include "glm.h"

namespace Graphics {

class DrawingHelpers {
public:
	DrawingHelpers();
	virtual ~DrawingHelpers();
	static void DrawString(float x, float y, GLvoid *font_style, char* format, ...);
	static void DrawGrid(const double& x, const double& y, const double& w, const double& h, const double& cell_l);
	static void DrawCustomOrigin(const double& x, const double& y, const double& z,
			const int& yaw, const int& roll, const int& pitch, const double& length);
	static void DrawArrow(const double& x, const double& y, const double& a);

	static std::vector<std::vector<float> > PreparePathForDrawing(const std::vector<PlannerHNS::WayPoint>& path,
			std::vector<std::vector<PlannerHNS::WayPoint> >& redyForDraw, double w, double resolution = 1);

	static void DrawPrePreparedPolygons(std::vector<std::vector<PlannerHNS::WayPoint> >& path,
			double z, float color[3],int nSkipPoints = 1, const std::vector<std::vector<float> >* colorProfile = 0);

	static void DrawWidePath(const std::vector<PlannerHNS::WayPoint>& path_points, const double& z,
			const double& width, float color[3], bool bGadient = true);

	static void DrawCostPath(const std::vector<PlannerHNS::WayPoint*>& path_points, const double& z, const double& width);

	static void DrawLinePoygonFromCenterX(const PlannerHNS::WayPoint& p1, const double& z,
			const PlannerHNS::WayPoint& p2, const double& z2, const double& w, const double& h,
			PlannerHNS::WayPoint& prev_point);

	static void DrawLinePoygonline(const PlannerHNS::GPSPoint& p1, const PlannerHNS::GPSPoint& p2, const double& w);

	static void DrawCustomCarModel(const PlannerHNS::WayPoint& pose, const double& steeringAngle, const std::vector<PlannerHNS::GPSPoint>& carPoints,float color[3], const double& angleFix);

	static GLMmodel* LoadModel(const char* fileName);

	static void DrawModel(GLMmodel* pmod,double length, double width, double height, double x, double y,double z, double heading, double pitch , double roll );

	static void DrawFilledEllipse(float x, float y, float z, float width, float height);

	static void DrawWideEllipse(float x, float y, float z, float outer_width, float outer_height, float inner_width,float color[3]);

	static void DrawSimpleEllipse(float x, float y, float z, float outer_width, float outer_height);

	static void DrawPedal(float x, float y, float z, float width, float height, float inner_height, float color[3]);

};

} /* namespace Graphics */


#endif
