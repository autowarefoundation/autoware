/*
 * FieldDescription.cpp
 *
 *  Created on: 30.08.2011
 *      Author: wahnfla
 */

#include "FieldDescription.hpp"
#include "../tools/errorhandler.hpp"

namespace datatypes
{

FieldDescription::FieldDescription() :
		m_fieldType(Undefined)
{
	m_datatype = Datatype_FieldDescription;
}

//
//
//
std::string FieldDescription::fieldTypeToString(FieldType type)
{
	switch (type)
	{
	case Segmented:
		return "Segmented";
	case Rectangle:
		return "Rectangle";
	case Radial:
		return "Radial";
	case Dynamic:
		return "Dynamic";
	default:
		return "undefined";
	}
}

	
} // namespace datatypes
