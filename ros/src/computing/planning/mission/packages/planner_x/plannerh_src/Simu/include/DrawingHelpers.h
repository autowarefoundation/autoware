/*
 * DrawingHelpers.h
 *
 *  Created on: May 31, 2016
 *      Author: hatem
 */


#include "glm.h"
#include "RoadNetwork.h"
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

	static std::vector<std::vector<float> > PreparePathForDrawing(const std::vector<PlannerHNS::WayPoint>& path,
			std::vector<std::vector<PlannerHNS::WayPoint> >& redyForDraw, double w, int resolution = 1);

	static void DrawPrePreparedPolygons(std::vector<std::vector<PlannerHNS::WayPoint> >& path,
			double z, float color[3],int nSkipPoints = 1, const std::vector<std::vector<float> >* colorProfile = 0);

	static void DrawWidePath(const std::vector<PlannerHNS::WayPoint>& path_points, const double& z,
			const double& width, float color[3]);

	static void DrawLinePoygonFromCenterX(const PlannerHNS::WayPoint& p1, const double& z,
			const PlannerHNS::WayPoint& p2, const double& z2, const double& w, const double& h,
			PlannerHNS::WayPoint& prev_point);

	static void DrawCustomCarModel(const PlannerHNS::WayPoint& pose,const std::vector<PlannerHNS::GPSPoint>& carPoints,float color[3], const double& angleFix);

	static GLMmodel* LoadModel(const char* fileName);

	static void DrawModel(GLMmodel* pmod,double length, double width, double height, double x, double y,double z, double heading, double pitch , double roll );

	static void DrawFilledEllipse(float x, float y, float z, float width, float height);

};

} /* namespace Graphics */

