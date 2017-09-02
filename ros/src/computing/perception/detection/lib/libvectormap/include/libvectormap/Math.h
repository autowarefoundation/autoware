/*
 * Math.h
 *
 *  Created on: Apr 8, 2015
 *      Author: sujiwo
 */

#ifndef SRC_MATH_H_
#define SRC_MATH_H_

#include <Eigen/Eigen>
#include <math.h>


typedef Eigen::Vector2f Point2;
typedef Eigen::Vector3f Point3;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector4f Point4;
typedef Eigen::Vector4f Vector4;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Quaternionf Quaternion;
typedef Eigen::Matrix4f Matrix4;
typedef Eigen::Matrix2f Matrix2;


template<typename F> static F degreeToRadian (const F degree)
{ return degree * M_PI / 180; }

template<typename F> static F radianToDegree (const F radian)
{ return radian * 180 / M_PI; }


inline double distance (const Point2 &p1, const Point2 &p2)
{
	Vector2 p3 (p2.x()-p1.x(), p2.y()-p1.y());
	return sqrt (p3.squaredNorm());
}


inline double distance (const Point3 &p1, const Point3 &p2)
{
	Vector3 p3 (p2.x()-p1.x(), p2.y()-p1.y(), p2.z()-p1.z());
	return sqrt(p3.squaredNorm());
}


inline double distance (const Point4 &p1, const Point4 &p2)
{
	Vector4 p3 (p2.x()-p1.x(), p2.y()-p1.y(), p2.z()-p1.z(), p2.w()-p1.w());
	return sqrt(p3.squaredNorm());
}


inline double distance (const double &x1, const double &y1, const double &x2, const double &y2)
{
	Point2 p(x1, y1), q(x2, y2);
	return distance (p, q);
}


inline Vector2 perpendicular (const Vector2 &v)
{ return Vector2 (-v.y(), v.x()); }


inline void rotate (Vector2 &v, const float angleInDegree)
{
	Matrix2 rot;
	double c = cos (degreeToRadian(angleInDegree)),
		s = sin (degreeToRadian(angleInDegree));
	rot (0, 0) = c;
	rot (0, 1) = -s;
	rot (1, 0) = s;
	rot (1, 1) = c;
	v = rot * v;
}


#endif /* SRC_MATH_H_ */
