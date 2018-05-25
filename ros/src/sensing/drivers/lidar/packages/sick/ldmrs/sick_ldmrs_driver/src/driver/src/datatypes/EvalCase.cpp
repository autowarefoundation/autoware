//
// EvalCase.cpp
//
//  Created on: 05.09.2011
//      Author: wahnfla
//

#include "EvalCase.hpp"
#include "../tools/errorhandler.hpp"	// for print...
//#include "StringToolbox.hpp"
#include <bitset>

namespace datatypes
{

EvalCase::EvalCase()
{
	m_datatype = Datatype_EvalCase;
	
	m_versionNumberIntern = 1;
}

EvalCase::~EvalCase()
{
}

// Estimate the memory usage of this object.
const UINT32 EvalCase::getUsedMemory() const
{
	return sizeof(*this) +
			m_caseName.length() +
			m_comment.length();
}


double EvalCase::getBlankingSize() const
{
	return m_blankingSize;
}

const std::string& EvalCase::getCaseName() const
{
	return m_caseName;
}

UINT8 EvalCase::getCaseNumber() const
{
	return m_caseNumber;
}

const std::string& EvalCase::getComment() const
{
	return m_comment;
}

UINT16 EvalCase::getFieldNumber() const
{
	return m_fieldNumber;
}

EvalCase::FilterType EvalCase::getFilterType() const
{
	return m_filterType;
}

// const EvalCase::InputStateVector& EvalCase::getHardwareInputs() const
// {
// 	return m_hardwareInputs;
// }

const UINT8 EvalCase::getLogicalInputState_as_UINT8() const
{
	UINT8 state = (((UINT8)(m_logicalInputState[1])) << 2) + ((UINT8)(m_logicalInputState[0]));
	return state;
}

double EvalCase::getMaxRadialCorridor() const
{
	return m_maxRadialCorridor;
}

double EvalCase::getMinFieldExp() const
{
	return m_minFieldExp;
}

UINT8 EvalCase::getOutputNumber() const
{
	return m_outputNumber;
}

UINT32 EvalCase::getResponseTime() const
{
	return m_responseTime;
}

UINT32 EvalCase::getResponseTimeExtended() const
{
	return m_responseTimeExtended;
}

EvalCase::EvaluationStrategy EvalCase::getStrategy() const
{
	return m_strategy;
}

UINT32 EvalCase::getVersionNumberIntern() const
{
	return m_versionNumberIntern;
}

bool EvalCase::isDistDependent() const
{
	return m_distDependent;
}

bool EvalCase::isResultNegation() const
{
	return m_resultNegation;
}

void EvalCase::setBlankingSize(double blankingSize)
{
	this->m_blankingSize = blankingSize;
	
	// Consistency check
	if (m_blankingSize > 10.0)
	{
		printWarning("EvalCase::setBlankingSize: Blanking size is very large. Was it really written in [m]?");
	}
}

void EvalCase::setCaseName(const std::string& caseName)
{
	m_caseName = caseName;
	if (m_caseName.length() > 32)
	{
		m_caseName = m_caseName.substr(0, 32);
	}
}

void EvalCase::setCaseNumber(UINT8 caseNumber)
{
	this->m_caseNumber = caseNumber;
	if (m_caseNumber == 0)
	{
		printWarning("EvalCase::setCaseNumber: Case number must be >0 for a valid EvalCase!");
	}
}

void EvalCase::setComment(const std::string& comment)
{
	m_comment = comment;
	if (m_comment.length() > 128)
	{
		m_comment = m_comment.substr(0, 128);
	}
}

void EvalCase::setDistDependent(bool distDependent)
{
	this->m_distDependent = distDependent;
}

bool EvalCase::getDistDependent()
{
	return m_distDependent;
}

void EvalCase::setFieldNumber(UINT16 fieldNumber)
{
	this->m_fieldNumber = fieldNumber;
}

void EvalCase::setFilterType(FilterType filterType)
{
	this->m_filterType = filterType;
}

// void EvalCase::setHardwareInputs(const EvalCase::InputStateVector& hardwareInputs)
// {
// 	this->m_hardwareInputs = hardwareInputs;
// }

// void EvalCase::setLogicalInputs(const EvalCase::InputStateVector& logicalInputs)
// {
// 	this->m_logicalInputs = logicalInputs;
// }

void EvalCase::setMaxRadialCorridor(double maxRadialCorridor)
{
	this->m_maxRadialCorridor = maxRadialCorridor;
}

void EvalCase::setMinFieldExp(double minFieldExp)
{
	this->m_minFieldExp = minFieldExp;
}

void EvalCase::setOutputNumber(UINT8 outputNumber)
{
	this->m_outputNumber = outputNumber;
}

void EvalCase::setResponseTime(UINT32 responseTime)
{
	this->m_responseTime = responseTime;
}

void EvalCase::setResponseTimeExtended(UINT32 responseTimeExtended)
{
	this->m_responseTimeExtended = responseTimeExtended;
}

void EvalCase::setResultNegation(bool resultNegation)
{
	this->m_resultNegation = resultNegation;
}

bool EvalCase::getResultNegation()
{
	return this->m_resultNegation;
}

void EvalCase::setStrategy(EvaluationStrategy strategy)
{
	this->m_strategy = strategy;
}

//
// Sets the internal version number, as used by the MRS.
//
void EvalCase::setVersionNumber(UINT16 versionNumber)
{
	this->m_versionNumberIntern = versionNumber;
}

//
// Returns the internal version number, as used by the MRS.
//
UINT16 EvalCase::getVersionNumber()
{
	return (this->m_versionNumberIntern);
}

EvalCase::ManipulationPrevention EvalCase::getManipulationPrevention() const
{
	return m_manipulationPrevention;
}

void EvalCase::setManipulationPrevention(EvalCase::ManipulationPrevention manPrev)
{
	m_manipulationPrevention = manPrev;
}

//
// Decode the input state data from the sensor into this
// objects structure.
//
void EvalCase::setLogicalInputState_from_UINT8(UINT8 value)
{
	// Input 1
	UINT8 inputValue = value & 0x03;
	switch (inputValue)
	{
		case 0:
			m_logicalInputState[0] = INPUT_INACTIVE;
			break;
		case  1:
			m_logicalInputState[0] = INPUT_LOW_ACTIVE;
			break;
		case 2:
			m_logicalInputState[0] = INPUT_HIGH_ACTIVE;
			break;
		default:
			printError("EvalCase::setLogicalInputState_from_UINT8: Configuration of input 1 has an invalid value, aborting!");
			return;
	}

	// Input 2
	inputValue = (value >> 2) & 0x03;
	switch (inputValue)
	{
		case 0:
			m_logicalInputState[1] = INPUT_INACTIVE;
			break;
		case  1:
			m_logicalInputState[1] = INPUT_LOW_ACTIVE;
			break;
		case 2:
			m_logicalInputState[1] = INPUT_HIGH_ACTIVE;
			break;
		default:
			printError("EvalCase::setLogicalInputState_from_UINT8: Configuration of input 2 has an invalid value, aborting!");
			return;
	}
}


} // namespace datatypes
