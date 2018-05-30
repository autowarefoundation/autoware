//
//
// Polygon3d.cpp
//

#include <cmath>

#include "Polygon3D.hpp"
#include "Polygon2D.hpp"
#include <algorithm> // for std::sort()
#include <iterator>
#include <sstream>


// ////////////////////////////////////////////////////////////
namespace datatypes
{

Polygon3D::Polygon3D()
{
}

Polygon3D::Polygon3D(const Point3D& p1)
{
	push_back(p1);		// Insert the element at the end of the list
}

Polygon3D::Polygon3D(const Point3D& p1, const Point3D& p2)
{
	push_back(p1);		// Insert the element at the end of the list
	push_back(p2);		// Insert the element at the end of the list
}
Polygon3D::Polygon3D(const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
	push_back(p1);		// Insert the element at the end of the list
	push_back(p2);		// Insert the element at the end of the list
	push_back(p3);		// Insert the element at the end of the list
}

//
// Initialize this polygon with an existing one
//
Polygon3D::Polygon3D(const base_class& other_vector)
	: base_class(other_vector)
{
}

/*
void Polygon3D::fromString(const std::string& polyString)
{
	clear();

	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	tokenizer listOfTokens(polyString, boost::char_separator<char>(Polygon2D::getSeparatorCharacters()));
	for (tokenizer::const_iterator iter = listOfTokens.begin();
		 iter != listOfTokens.end(); ++iter)
	{
		float x = boost::lexical_cast<float>(*iter);
		++iter;
		if (iter == listOfTokens.end())
			throw std::runtime_error("When parsing a Polygon3D string, the last point only has a x component.");
		float y = boost::lexical_cast<float>(*iter);
		++iter;
		if (iter == listOfTokens.end())
			throw std::runtime_error("When parsing a Polygon3D string, the last point only has a x and y component.");
		float z = boost::lexical_cast<float>(*iter);
		this->push_back(Point3D(x, y, z));
	}
}
*/


/// Conversion to string for debugging
std::string Polygon3D::toString() const
{
	std::ostringstream o;
	o << *this;
	return o.str();
}

/// Compare-function for the sort algorithm sortForAngleXYPlane().
static bool comparePointAngleXYPlane(const Point3D& p1, const Point3D& p2)
{
	return (p1.getAngleAroundZ() < p2.getAngleAroundZ());
}

/**
* Sort the points for incrementing angles of the projection to the X-Y plane
* Result is a list soted from left to right.
*/
void Polygon3D::sortForAngleXYPlane()
{
	std::sort(begin(), end(), comparePointAngleXYPlane);
}



/// Text output for debugging
std::ostream& operator<<(std::ostream& os, const Polygon3D& poly)
{
	os << "[ ";
	if (!poly.empty())
	{
		// Polygon contains at least one element
		std::copy (poly.begin(), poly.end() - 1,
				   std::ostream_iterator<Point3D>(os, ", "));
		os << poly.back();	// Last element of the polygon
	}
	os << " ]";
	return os;
}

// Important: this code exists in a very similar for in the Polygon2D class.
// Bugfixes should be applied there also.
Polygon3D Polygon3D::getMovedPath(double dist) const
{
	Polygon3D moved;

	// We need 2 points minimum to have a valid path
	if (size() < 2)
	{
		if (size() > 0)
		{
			// only 1 point, move only y-direction.
			moved.push_back(at(0) + Point3D(0.0, dist, 0.0));
		}
		return moved; // finished.
	}

	// Here we have at least 2 points.

	// We need 2 segments to calculate the shift direction.
	// Start with 90 deg angle.
	double lastAng, ang; // Angles of the path segments.
	Point3D lastPos = at(0);

	// The first segment is also handles in the loop. The needed but not existing preceeding segment
	// is assumed to have the same direction.
	lastAng = (at(1) - lastPos).getAngleAroundZ();
	double edgeDist; // path width in a corner, is >= dist (path width)
	Point3D shiftVec; // Vector from source edge to new edge.
	Point3D pt;
	size_t i;
	for (i = 1; i < size(); i++)
	{
		pt = at(i);
		ang = (pt - lastPos).getAngleAroundZ();
		double diffAng = normalizeRadians(lastAng - ang);
		double halfAngBetween = diffAng / 2.0;

		// Calculation of the length from edge to new edge
		// The direction to the new path point (new edge)  has the direction of the half angle between both segments.
		double cosHalfAngBetween = std::abs(cos(halfAngBetween));

		// Do not calculate moved point if path angle to small
		// This usually happens, if we have a zick-zack situation.
		if ((cosHalfAngBetween) > 0.195)  //  0.195 = cos( 1/2 * PI  +- 1/16 * PI)
		{
			edgeDist = dist / cosHalfAngBetween;
			shiftVec = Point3D(0.0, edgeDist, 0.0);
			shiftVec.rotateAroundZ(lastAng - halfAngBetween);
			Point3D np = lastPos + shiftVec;

			moved.push_back(np);
		}

		lastAng = ang;
		lastPos = pt;
	}
	// Add last point
	moved.push_back(lastPos + shiftVec);

	return moved;
}

}	// namespace datatypes

