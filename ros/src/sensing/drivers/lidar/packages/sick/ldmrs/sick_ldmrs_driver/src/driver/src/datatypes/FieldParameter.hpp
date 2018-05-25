//
// FieldParameter.hpp
//
//  Created on: 30.08.2011
//      Author: wahnfla
//

#ifndef FIELDPARAMETER_HPP
#define FIELDPARAMETER_HPP

#include "../BasicDatatypes.hpp"
#include "FieldDescription.hpp"
#include "Polygon2D.hpp"
#include <vector>
#include <stdexcept>
//#include "Trace.hpp"

namespace datatypes
{

//
// Class that represents the structure of a field parameter of a scanner
//
class FieldParameter : public BasicData
{
public:
	enum CaseResult
	{
		ECR_DONT_CARE = 0,
		ECR_LOW = 1,
		ECR_HIGH = 2,
		ECR_DETECTING = 3
	};
	
	// These codes match the internal representation of the LD-MRS
	enum FieldTypeIntern
	{
		FieldTypeIntern_RADIAL = 0,
		FieldTypeIntern_RECTANGLE = 1,
		FieldTypeIntern_SEGMENTED = 2,
		FieldTypeintern_DYNAMIC = 3
	};

	FieldParameter();
	virtual ~FieldParameter();
	virtual const UINT32 getUsedMemory() const;
	UINT32 getAngleScaleFactor() const;
	INT32 getAngleScaleOffset() const;
	double getDistScaleFactor() const;
	double getDistScaleOffset() const;
	FieldDescription* getField() const;
	FieldTypeIntern getFieldTypeIntern() const;
	std::string getFieldTypeInternAsString() const;
	void setAngleScaleFactor(UINT32 angleScaleFactor);
	void setAngleScaleOffset(INT32 angleScaleOffset);
	void setDistScaleFactor(double distScaleFactor);
	void setDistScaleOffset(double distScaleOffset);
	void setField(FieldDescription* field);
	
	void setFieldNumber(UINT16 m_fieldNumber);
	const UINT16 getFieldNumber() const;
	const bool isValid() const;
	
	void setFieldTypeIntern(UINT8 fieldTypeIntern);
	void setFieldTypeIntern(FieldTypeIntern m_fieldTypeIntern);
	const std::string & getComment() const;
	const std::string & getFieldName() const;
	void setComment(const std::string& comment);
	void setFieldName(const std::string& fieldName);
	UINT8 getVersionNumber() const;
	void setVersionNumber(UINT8 m_versionNumber);
	CaseResult getLastKnownInfringementState() const;
	void setLastKnownInfringementState(CaseResult lastKnownInfringementState);

	bool isLayerFilterEnabled() const;
	void setEnableLayerFilter(bool enableLayerFilter);
	UINT8 getLayerFilterBitmap() const;
	void setLayerFilterBitmap(UINT8 layerFilterBitmap);

	bool empty() const
	{
		return !m_field;
	}

	const Polygon2D & getPolygon() const
	{
		if (empty())
		{
			throw std::runtime_error("FieldDescription::getPolygon(): No field available.");
		}
		return m_field->getFieldAsPolygon();
	}

	const FieldDescription::FieldType getFieldType() const;
	

private:
	// Header data (sensor specific)
	double m_distScaleFactor; ///< conversion factor
	double m_distScaleOffset; ///< [m]
	UINT32 m_angleScaleFactor; ///< internal parameter: sensor specific
	INT32 m_angleScaleOffset; ///< internal parameter: sensor specific
	FieldTypeIntern m_fieldTypeIntern;  ///< internal parameter: sensor specific
	UINT16 m_fieldNumber;
	UINT8 m_versionNumber;
	std::string m_fieldName;
	std::string m_comment;
	FieldDescription* m_field;
	CaseResult m_lastKnownInfringementState;
	bool m_enableLayerFilter;
	UINT8 m_layerFilterBitmap;
};

} // namespace datatypes

#endif // FIELDPARAMETER_HPP
