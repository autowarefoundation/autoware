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

#ifndef DRAWINGHELPERS_TEST
#define DRAWINGHELPERS_TEST

#include "GL/glut.h"
#include "op_planner/RoadNetwork.h"

namespace OP_TESTING_NS
{

class DrawingHelpers
{
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

	static void DrawFilledEllipse(float x, float y, float z, float width, float height);

	static void DrawWideEllipse(float x, float y, float z, float outer_width, float outer_height, float inner_width,float color[3]);

	static void DrawSimpleEllipse(float x, float y, float z, float outer_width, float outer_height);

	static void DrawPedal(float x, float y, float z, float width, float height, float inner_height, float color[3]);

};

}


#endif
