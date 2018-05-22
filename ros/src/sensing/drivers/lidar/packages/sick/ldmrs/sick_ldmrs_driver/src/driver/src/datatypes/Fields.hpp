//
// Fields.hpp
//
//  Created on: 30.08.2011
//      Author: wahnfla
//

#ifndef FIELDS_HPP
#define FIELDS_HPP

#include "Polygon2D.hpp"
#include <vector>
#include "../BasicDatatypes.hpp"
#include "FieldDescription.hpp"
#include "FieldParameter.hpp"

namespace datatypes
{



//
//
// *******************************************************************************
//
//
class Fields : public BasicData
{
public:
	typedef std::vector<FieldParameter*> FieldVector;

	Fields() {m_datatype = Datatype_Fields;}

	virtual ~Fields() {}
	const UINT32 getUsedMemory() const;

	void add(FieldParameter* field);

	const FieldParameter& getField(UINT16 fieldNumber) const;
	
	UINT16 getNumberOfValidFields();

	const FieldVector& getFields() const
	{
		return m_fields;
	}

private:

	FieldVector m_fields;
};

//
//
// *******************************************************************************
//
// Helper class for a point of a segmented field.
// Note that as with all internal data structures, units
// should be [rad] and [m].
//
class FieldSegmentedPoint
{
public:
	FieldSegmentedPoint(double angle, double startDist, double endDist) :
		m_angle(angle),
		m_startDist(startDist),
		m_endDist(endDist)
	{
	}
	FieldSegmentedPoint()
	{
		m_angle = 0.0;
		m_startDist = 0.0;
		m_endDist = 0.0;
	}

	double getAngle() const { return m_angle; }
	double getStartDist() const { return m_startDist; }
	double getEndDist() const { return m_endDist; }
	void setAngle(double angle) { m_angle = angle; }
	void setStartDist(double startDist) { m_startDist = startDist; }
	void setEndDist(double endDist) { m_endDist = endDist; }

private:
	double m_angle; ///< [rad]
	double m_startDist; ///< [m]
	double m_endDist; ///< [m]
};

typedef std::vector<FieldSegmentedPoint> FieldSegmentedPoints;

//
//
// *******************************************************************************
//
//
class  FieldSegmented : public FieldDescription
{
public:
	FieldSegmented()
	{
		m_fieldType = FieldDescription::Segmented;
	}
	
	virtual ~FieldSegmented() {}

	void addPoint(const FieldSegmentedPoint& point)
	{
		m_points.push_back(point);
	}
	
	virtual const UINT32 getUsedMemory() const
	{
		return sizeof(*this) + (m_points.size() * sizeof(FieldSegmentedPoints));
	}

	UINT32 getNumberOfPoints();
	void computePolygon();
	FieldSegmentedPoints getPoints();

private:
	FieldSegmentedPoints m_points;
};

//
//
// *******************************************************************************
//
//
class FieldRectangle : public FieldDescription
{
public:
	FieldRectangle()
	{
		m_fieldType = FieldDescription::Rectangle;
	}

	virtual ~FieldRectangle()
	{
	}

	virtual const UINT32 getUsedMemory() const
	{
		return sizeof(*this);
	}
	
	void computePolygon();
	double getLength() const;
	double getRefPointAngle() const;
	double getRefPointDist() const;
	double getRotAngle() const;
	double getWidth() const;
	void setLength(double length);
	void setRefPointAngle(double refPointAngle);
	void setRefPointDist(double refPointDist);
	void setRotAngle(double rotAngle);
	void setWidth(double width);

private:
	double m_refPointAngle; ///< [rad]
	double m_refPointDist; ///< [m]
	double m_rotAngle; ///< [rad]
	double m_width;			///< [m]
	double m_length;		///< [m]
};

//
//
// *******************************************************************************
//
//
class FieldRadial : public FieldDescription
{
public:

	FieldRadial()
	{
		m_fieldType = FieldDescription::Radial;
	}

	virtual ~FieldRadial()
	{
	}

	virtual const UINT32 getUsedMemory() const
	{
		return sizeof(*this);
	}
	
	UINT16 getFirstAngle() const;
	UINT16 getLastAngle() const;
	UINT32 getMaxDist() const;
	UINT32 getMinDist() const;
	void setFirstAngle(UINT16 m_firstAngle);
	void setLastAngle(UINT16 m_lastAngle);
	void setMaxDist(UINT32 m_maxDist);
	void setMinDist(UINT32 m_minDist);

	void computePolygon();

private:
	UINT16 m_firstAngle; ///< index of start angle relative to AngleScale
	UINT16 m_lastAngle; ///< index of last angle relative to AngleScale
	UINT32 m_minDist; ///< [mm]
	UINT32 m_maxDist; ///< [mm]

};

//
//
// *******************************************************************************
//
//
class FieldDynamic : public FieldRectangle
{
public:
	FieldDynamic()
	{
		m_fieldType = FieldDescription::Dynamic;
	}

	virtual ~FieldDynamic()
	{
	}

	virtual const UINT32 getUsedMemory() const
	{
		return sizeof(*this);
	}
	
	double getMaxLength() const;
	double getSpeedMax() const;
	void setMaxLength(double maxLength);
	void setSpeedMax(double speedMax);

private:
	double m_maxLength; ///< [m] extension at maximum speed to direction of RotAngle+90° relative to DistScale
	double m_speedMax; ///< [m/s]
};

}	 // namespace datatypes

#endif // FIELDS
