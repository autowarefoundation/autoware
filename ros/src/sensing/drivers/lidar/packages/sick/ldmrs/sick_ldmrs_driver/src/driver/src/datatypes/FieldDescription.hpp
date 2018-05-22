//
// FieldDescription.hpp
//
//  Created on: 30.08.2011
//      Author: wahnfla
//

#ifndef FIELDDESCRIPTION_HPP
#define FIELDDESCRIPTION_HPP

#include "Polygon2D.hpp"
#include <vector>
#include "../BasicDatatypes.hpp"

namespace datatypes
{

//
// Interface for fields.
//
class FieldDescription : public BasicData
{
public:
	enum FieldType
	{
		Undefined = 0,
		Segmented = 1,
		Rectangle = 2,
		Radial = 3,
		Dynamic = 4
	};

	FieldDescription();

	virtual void computePolygon() = 0;

	const Polygon2D& getFieldAsPolygon() const
	{
		return m_fieldPolygon;
	}

	FieldType getFieldType() const
	{
		return m_fieldType;
	}

	static std::string fieldTypeToString(FieldType type);
protected:
	FieldType m_fieldType;
	Polygon2D m_fieldPolygon;
};


} // namespace parameters

#endif // FIELDDESCRIPTION_HPP
