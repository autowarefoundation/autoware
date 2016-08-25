/*
 * MatrixOperations.h
 *
 *  Created on: Jun 19, 2016
 *      Author: hatem
 */

#ifndef MATRIXOPERATIONS_H_
#define MATRIXOPERATIONS_H_

#include "RoadNetwork.h"
#include <math.h>


namespace PlannerHNS {


class Mat3
{
	double m11, m12, m13;
	double m21, m22, m23;
	double m31, m32, m33;

	double m[3][3];

public:
	Mat3()
	{
		//initialize Identity by default
		m11 = m22 = m33 = 1;
		m12 = m13 = m21 = m23 = m31 = m32 = 0;
	}

	Mat3(double angle, POINT2D trans)
	{
		//Initialize Rotation Matrix
		double c = cos(angle);
		double s = sin(angle);
		m11 = c;
		m12 = s;
		m21 = -s;
		m22 = c;
		m31 = trans.x;
		m32 = trans.y;
		m13 = m23= 0;
		m33 = 1;
	}

	Mat3(double transX, double transY, bool mirrorX, bool mirrorY )
	{
		m11 = m22 = m33 = 1;
		m12 = m13 = m21 = m23 = m31 = m32 = 0;
		m[0][0] = (mirrorX == true ) ? -1 : 1; m[0][1] =  0; m[0][2] =  transX;
		m[1][0] = 0; m[1][1] =  (mirrorY==true) ? -1 : 1; m[1][2] =  transY;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	Mat3(double transX, double transY)
	{
		m11 = m22 = m33 = 1;
		m12 = m13 = m21 = m23 = m31 = m32 = 0;
		m[0][0] = 1; m[0][1] =  0; m[0][2] =  transX;
		m[1][0] = 0; m[1][1] =  1; m[1][2] =  transY;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	Mat3(double rotation_angle)
	{
		m11 = m22 = m33 = 1;
		m12 = m13 = m21 = m23 = m31 = m32 = 0;
		double c = cos(rotation_angle);
		double s = sin(rotation_angle);
		m[0][0] = c; m[0][1] = -s; m[0][2] =  0;
		m[1][0] = s; m[1][1] =  c; m[1][2] =  0;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	Mat3(GPSPoint rotationCenter)
	{
		m11 = m22 = m33 = 1;
		m12 = m13 = m21 = m23 = m31 = m32 = 0;
		double c = cos(rotationCenter.a);
		double s = sin(rotationCenter.a);
		double u = rotationCenter.x;
		double v = rotationCenter.y;
		m[0][0] = c; m[0][1] = -s; m[0][2] = -u*c + v*s + u;
		m[1][0] = s; m[1][1] =  c; m[1][2] = -u*s - v*c + v;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}


	GPSPoint operator * (GPSPoint v)
	{
		GPSPoint _v = v;
		v.x = m[0][0]*_v.x + m[0][1]*_v.y + m[0][2]*1;
		v.y = m[1][0]*_v.x + m[1][1]*_v.y + m[1][2]*1;
		return v;
	}

	POINT2D operator * (POINT2D v)
	{
		Mat3 m = *this;
		POINT2D r;
		r.x = m.m11 * v.x + m.m21 * v.y + m.m31 * 1;
		r.y = m.m12 * v.x + m.m22 * v.y + m.m32 * 1;
		return r;
	}

	Mat3 operator *(Mat3 m2)
	{
		Mat3 m1 = *this;
		Mat3 r;
		r.m11 = m1.m11 * m2.m11 + m1.m12 * m2.m21 + m1.m13 * m2.m31;
		r.m12 = m1.m11 * m2.m12 + m1.m12 * m2.m22 + m1.m13 * m2.m32;
		r.m13 = m1.m11 * m2.m13 + m1.m12 * m2.m23 + m1.m13 * m2.m33;

		r.m21 = m1.m21 * m2.m11 + m1.m22 * m2.m21 + m1.m23 * m2.m31;
		r.m22 = m1.m21 * m2.m12 + m1.m22 * m2.m22 + m1.m23 * m2.m32;
		r.m23 = m1.m21 * m2.m13 + m1.m22 * m2.m23 + m1.m23 * m2.m33;

		r.m31 = m1.m31 * m2.m11 + m1.m32 * m2.m21 + m1.m33 * m2.m31;
		r.m32 = m1.m31 * m2.m12 + m1.m32 * m2.m22 + m1.m33 * m2.m32;
		r.m33 = m1.m31 * m2.m13 + m1.m32 * m2.m23 + m1.m33 * m2.m33;

		return r;
	}
};

} /* namespace PlannerHNS */

#endif /* MATRIXOPERATIONS_H_ */
