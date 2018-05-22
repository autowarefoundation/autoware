//
// Fields.cpp
//
//  Created on: 30.08.2011
//      Author: wahnfla
//

#include "Fields.hpp"
#include "../tools/errorhandler.hpp"	// for print...

namespace datatypes
{


//
// ////////////////////////////  FieldSegmentated /////////////////////////////////////////
//

/// fills the polygon clockwise
void FieldSegmented::computePolygon()
{
	m_fieldPolygon.clear();

	Polygon2D startPoints;

	for (FieldSegmentedPoints::const_iterator p = m_points.begin(); p != m_points.end(); ++p)
	{
		double endDist = p->getEndDist();

		if (!::isNaN(endDist))
		{
			m_fieldPolygon.push_back(Point2D::fromPolar(endDist, p->getAngle()));
		}

		double startDist = p->getStartDist();
		double angle = p->getAngle();

		if (::isNaN(startDist) == false)
		{
			startPoints.push_back(Point2D::fromPolar(startDist, angle));
		}
	}

	if (startPoints.empty())
	{
		// add origin if empty
		m_fieldPolygon.push_back(Point2D(0., 0.));
	}
	else
	{
		for (Polygon2D::const_reverse_iterator r = startPoints.rbegin(); r != startPoints.rend(); ++r)
		{
			m_fieldPolygon.push_back(*r);
		}
	}

//	printInfoMessage("Field: " + toString(m_fieldPolygon) + ".", true);
}

//
// Returns the number of points.
//
UINT32 FieldSegmented::getNumberOfPoints()
{
	return m_points.size();
}

//
// Returns a copy of the points.
//
FieldSegmentedPoints FieldSegmented::getPoints()
{
	return m_points;
}

//
// //////////////////////////// FieldRectangle  /////////////////////////////////////////
//


/// fills the polygon clockwise
void FieldRectangle::computePolygon()
{
	m_fieldPolygon.clear();

	Point2D l = Point2D::fromPolar(m_length, m_rotAngle);
	Point2D w = Point2D::fromPolar(m_width, m_rotAngle - 90 * deg2rad);

	m_fieldPolygon.push_back(Point2D::fromPolar(m_refPointDist, m_refPointAngle));
	m_fieldPolygon.push_back(l + m_fieldPolygon[0]);
	m_fieldPolygon.push_back(w + l + m_fieldPolygon[0]);
	m_fieldPolygon.push_back(w + m_fieldPolygon[0]);

//	printInfoMessage("Field: " + ::toString(m_fieldPolygon) + ".", true);
}


double FieldRectangle::getLength() const
{
	return m_length;
}

double FieldRectangle::getRefPointAngle() const
{
	return m_refPointAngle;
}

double FieldRectangle::getRefPointDist() const
{
	return m_refPointDist;
}

double FieldRectangle::getRotAngle() const
{
	return m_rotAngle;
}

double FieldRectangle::getWidth() const
{
	return m_width;
}

//
// Sets the length (x-direction) of the rectangle.
//
void FieldRectangle::setLength(double length)
{
	if (length < 0.00001)
	{
		// Is this value mistakenly 0 or negative?
		printError("FieldRectangle::setLength: Length must not be 0 or negative - aborting!");
		return;
	}
	
	this->m_length = length;
}

void FieldRectangle::setRefPointAngle(double refPointAngle)
{
	this->m_refPointAngle = refPointAngle;
}

void FieldRectangle::setRefPointDist(double refPointDist)
{
	this->m_refPointDist = refPointDist;
}

//
// Set the rotation angle around the reference point.
// Positive angles turn counter-clockwise.
//
void FieldRectangle::setRotAngle(double rotAngle)
{
	if ((rotAngle < -PI) || (rotAngle > PI))
	{
		// Is this value mistakenly written in [deg] instead of [rad]?
		printWarning("FieldRectangle::setRotAngle: rotAngle is outside of the limit [-PI, PI], did you forget deg-2-rad conversion?");
	}
	this->m_rotAngle = rotAngle;
}


void FieldRectangle::setWidth(double width)
{
	if (width < 0.00001)
	{
		// Is this value mistakenly 0 or negative?
		printError("FieldRectangle::setWidth: Width must not be 0 or negative - aborting!");
		return;
	}
	
	this->m_width = width;
}

//
// ///////////////////////// FieldRadial  ////////////////////////////////////////////
//

void FieldRadial::computePolygon()
{
	printError("FieldRadial::computePolygon: The MRS does not support radial fields.");
}


UINT16 FieldRadial::getFirstAngle() const
{
	return m_firstAngle;
}

UINT16 FieldRadial::getLastAngle() const
{
	return m_lastAngle;
}

UINT32 FieldRadial::getMaxDist() const
{
	return m_maxDist;
}

UINT32 FieldRadial::getMinDist() const
{
	return m_minDist;
}

void FieldRadial::setFirstAngle(UINT16 firstAngle)
{
	this->m_firstAngle = firstAngle;
}

void FieldRadial::setLastAngle(UINT16 lastAngle)
{
	this->m_lastAngle = lastAngle;
}

void FieldRadial::setMaxDist(UINT32 maxDist)
{
	this->m_maxDist = maxDist;
}

void FieldRadial::setMinDist(UINT32 minDist)
{
	this->m_minDist = minDist;
}

//
// ///////////////////////////// FieldDynamic ////////////////////////////////////////
//


double FieldDynamic::getMaxLength() const
{
	return m_maxLength;
}

double FieldDynamic::getSpeedMax() const
{
	return m_speedMax;
}

void FieldDynamic::setMaxLength(double maxLength)
{
	this->m_maxLength = maxLength;
}

void FieldDynamic::setSpeedMax(double speedMax)
{
	this->m_speedMax = speedMax;
}

//
// /////////////////////////////  Fields ////////////////////////////////////////
//

const FieldParameter& Fields::getField(UINT16 fieldNumber) const
{
	FieldVector::const_iterator f;
	for (f = m_fields.begin(); f != m_fields.end(); ++f)
	{
		const FieldParameter* fp = *f;
		if (fp->getFieldNumber() == fieldNumber)
		{
			break;
		}
	}

	if (f == m_fields.end())
	{
		dieWithError("Fields::getField(): No field available with the given number.");
	}

	// return the field as reference
	return *(*f);
}

//
// Add a new field to the field vector.
//
void Fields::add(FieldParameter* field)
{
	m_fields.push_back(field);
}

const UINT32 Fields::getUsedMemory() const
{
	UINT32 sum = sizeof(*this);
	FieldVector::const_iterator iter;
	for (iter = m_fields.begin(); iter != m_fields.end(); iter++)
	{
//		sum += iter->getUsedMemory();
		sum += 1000;	// Panic! Failed to solve the const correctness issue.
	}
	return sum;
}

//
//
//
UINT16 Fields::getNumberOfValidFields()
{
	UINT16 numValid = 0;
	FieldVector::iterator iter;
	for (iter = m_fields.begin(); iter != m_fields.end(); iter++)
	{
		if ((*iter)->getFieldNumber() != 0)
		{
			numValid++;
		}
	}
	return numValid;
}


} // namespace datatypes
