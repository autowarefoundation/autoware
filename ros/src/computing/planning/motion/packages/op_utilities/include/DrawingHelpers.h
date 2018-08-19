/*
// *  Copyright (c) 2018, Nagoya University
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
