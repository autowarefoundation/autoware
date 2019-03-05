//
// FieldParameter.cpp
//
//  Created on: 30.08.2011
//      Author: wahnfla
//

#include "FieldParameter.hpp"
#include "../tools/errorhandler.hpp"	// for print...
//#include "StringToolbox.hpp"

namespace datatypes
{

FieldParameter::FieldParameter()
{
	m_datatype = Datatype_FieldParameter;
	
	m_layerFilterBitmap = 0;
	m_enableLayerFilter = false;
	m_field = NULL;
	m_fieldNumber = 0;		// 0 = invalid
	m_versionNumber = 1;	// Default is v1
}

FieldParameter::~FieldParameter()
{
}

const UINT32 FieldParameter::getUsedMemory() const
{
	UINT32 length = sizeof(*this)+
					m_fieldName.length() +
					m_comment.length();
	if (m_field != NULL)
	{
		length += m_field->getUsedMemory();
	}
	return length;
}

//
// Returns true if the Field Number is not 0. 
//
const bool FieldParameter::isValid() const
{
	if ((m_fieldNumber == 0) || (m_field == NULL))
	{
		// Invalid
		return false;
	}
	
	return true;
}


UINT32 FieldParameter::getAngleScaleFactor() const
{
	return m_angleScaleFactor;
}

INT32 FieldParameter::getAngleScaleOffset() const
{
	return m_angleScaleOffset;
}

double FieldParameter::getDistScaleFactor() const
{
	return m_distScaleFactor;
}

double FieldParameter::getDistScaleOffset() const
{
	return m_distScaleOffset;
}

FieldDescription* FieldParameter::getField() const
{
	return m_field;
}

FieldParameter::FieldTypeIntern FieldParameter::getFieldTypeIntern() const
{
	return m_fieldTypeIntern;
}

//
// Returns the internal field type as a readable string.
//
std::string FieldParameter::getFieldTypeInternAsString() const
{
	switch (m_fieldTypeIntern)
	{
		case FieldTypeIntern_RECTANGLE:
			return "Rectangle";
		case FieldTypeIntern_SEGMENTED:
			return "Segmented";
		case FieldTypeIntern_RADIAL:
			return "Radial (not supported)";
		case FieldTypeintern_DYNAMIC:
			return "Dynamic (not supported)";
		default:
			return "(unknown)";
	}
	
	return "(unknown)";
}


void FieldParameter::setAngleScaleFactor(UINT32 angleScaleFactor)
{
	this->m_angleScaleFactor = angleScaleFactor;
}

void FieldParameter::setAngleScaleOffset(INT32 angleScaleOffset)
{
	this->m_angleScaleOffset = angleScaleOffset;
}

void FieldParameter::setDistScaleFactor(double distScaleFactor)
{
	this->m_distScaleFactor = distScaleFactor;
}

void FieldParameter::setDistScaleOffset(double distScaleOffset)
{
	this->m_distScaleOffset = distScaleOffset;
}

void FieldParameter::setField(FieldDescription* field)
{
	this->m_field = field;
}

void FieldParameter::setFieldNumber(UINT16 fieldNumber)
{
	this->m_fieldNumber = fieldNumber;
}

//
// Sets the internal field type for the LD-MRS
//
// Taken from SRTSys.h:
// #define FieldParam_t_FieldHdr_eType_FD_RADIAL       0     
// #define FieldParam_t_FieldHdr_eType_FD_RECTANGLE    1     
// #define FieldParam_t_FieldHdr_eType_FD_SEGMENTATED  2     
// #define FieldParam_t_FieldHdr_eType_FD_DYNAMIC      3
//
void FieldParameter::setFieldTypeIntern(FieldTypeIntern fieldTypeIntern)
{
	this->m_fieldTypeIntern = fieldTypeIntern;
}

//
// Sets the internal field type for the LD-MRS
//
// Taken from SRTSys.h:
// #define FieldParam_t_FieldHdr_eType_FD_RADIAL       0     
// #define FieldParam_t_FieldHdr_eType_FD_RECTANGLE    1     
// #define FieldParam_t_FieldHdr_eType_FD_SEGMENTATED  2     
// #define FieldParam_t_FieldHdr_eType_FD_DYNAMIC      3
//
void FieldParameter::setFieldTypeIntern(UINT8 fieldTypeIntern)
{
	this->m_fieldTypeIntern = (FieldTypeIntern)fieldTypeIntern;
}


const std::string& FieldParameter::getComment() const
{
	return m_comment;
}

const std::string& FieldParameter::getFieldName() const
{
	return m_fieldName;
}

//
// Set an additional text comment. The comment can be up to 128 characters.
//
void FieldParameter::setComment(const std::string& comment)
{
	m_comment = comment;
	if (m_comment.length() > 128)
	{
		m_comment = m_comment.substr(0, 128);
	}
}

//
// Set the name of the field. The name can be up to 32 characters.
//
void FieldParameter::setFieldName(const std::string& fieldName)
{
	this->m_fieldName = fieldName;
	if (m_fieldName.length() > 32)
	{
		m_fieldName = m_fieldName.substr(0, 32);
	}

}

UINT8 FieldParameter::getVersionNumber() const
{
	return m_versionNumber;
}

void FieldParameter::setVersionNumber(UINT8 versionNumber)
{
	this->m_versionNumber = versionNumber;
}


FieldParameter::CaseResult FieldParameter::getLastKnownInfringementState() const
{
	return m_lastKnownInfringementState;
}

void FieldParameter::setLastKnownInfringementState(FieldParameter::CaseResult m_lastKnownInfringementState)
{
	this->m_lastKnownInfringementState = m_lastKnownInfringementState;
}

UINT8 FieldParameter::getLayerFilterBitmap() const
{
	return m_layerFilterBitmap;
}

bool FieldParameter::isLayerFilterEnabled() const
{
	return m_enableLayerFilter;
}

void FieldParameter::setEnableLayerFilter(bool m_enableLayerFilter)
{
	this->m_enableLayerFilter = m_enableLayerFilter;
}

void FieldParameter::setLayerFilterBitmap(UINT8 m_layerFilterBitmap)
{
	this->m_layerFilterBitmap = m_layerFilterBitmap;
}

//
// Returns the type of the field.
// The internal type and the added field should match!
//
const FieldDescription::FieldType FieldParameter::getFieldType() const
{
	if (m_field == NULL)
	{
		// There is no associated field data structure, so we do not have a field type.
		return FieldDescription::Undefined;
	}
	else
	{
		// Check if the internal datatype and the field type match
		if ((m_field->getFieldType() == datatypes::FieldDescription::Rectangle) &&
			(getFieldTypeIntern() == FieldTypeIntern_RECTANGLE))
		{
			return FieldDescription::Rectangle;
		}
		if ((m_field->getFieldType() == datatypes::FieldDescription::Segmented) &&
			(getFieldTypeIntern() == FieldTypeIntern_SEGMENTED))
		{
			return FieldDescription::Segmented;
		}
		
		// No match!
		printError("FieldParameter::getFieldType(): The internal field type and the associated field structure do no match, returning UNDEFINED!");
	}
	
	return FieldDescription::Undefined;
}

const UINT16 FieldParameter::getFieldNumber() const
{
	return m_fieldNumber;
}


} // namespace datatypes
