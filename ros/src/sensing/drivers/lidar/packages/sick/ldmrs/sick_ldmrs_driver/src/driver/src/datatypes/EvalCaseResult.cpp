//
// EvalCaseResult.cpp
//
// A single EvalCaseResult.
//
//

#include "EvalCaseResult.hpp"
#include "EvalCases.hpp"
#include "../sopas/colaa.hpp"
#include "../tools/errorhandler.hpp"

#include <sstream>


namespace datatypes
{


EvalCaseResult::EvalCaseResult () :
	uiVersionNo(0),
	CaseHdr(),
	m_eCaseResult(),
	m_sCaseName("no name"),
	m_sComment("no comment"),
	aTimeBlock()
{
	m_datatype = Datatype_EvalCaseResult;
}



// Default destructor
EvalCaseResult::~EvalCaseResult()
{
}

// Estimate the memory usage of this object.
const UINT32 EvalCaseResult::getUsedMemory() const
{
	return sizeof(*this) +
			m_sCaseName.length() +
			m_sComment.length();
}



std::string EvalCaseResult::toString() const
{
	std::ostringstream os;
	os << "EvalCaseResult members" << std::endl
	   << "uiVersionNo:           " << uiVersionNo << std::endl
	   << "CaseHdr: usiNumber:    " << (UINT16)(CaseHdr.usiNumber) << std::endl
	   << "   udiSysCount:        " << CaseHdr.udiSysCount << std::endl
	   << "   dDistScaleFactor:   " << CaseHdr.dDistScaleFactor << std::endl
	   << "   dDistScaleOffset:   " << CaseHdr.dDistScaleOffset << std::endl
	   << "   uiAngleScaleFactor: " << CaseHdr.uiAngleScaleFactor << std::endl
	   << "   iAngleScaleOffset:  " << CaseHdr.iAngleScaleOffset << std::endl
	   << "eCaseResult:           " << (UINT16)(m_eCaseResult) << std::endl
	   << "aFieldInfringement:    --- not in use ---" << std::endl
	   << "sCaseName:             " << ">" << m_sCaseName << "<" << std::endl
	   << "sComment:              " << ">" << m_sComment  << "<" << std::endl
	   << "aTimeBlock:            " << (UINT16)(aTimeBlock.uiYear)  << ":" << (UINT16)(aTimeBlock.usiMonth)  << ":" << (UINT16)(aTimeBlock.usiDay) << ":"
	   << (UINT16)(aTimeBlock.usiHour) << ":" << (UINT16)(aTimeBlock.usiMinute) << ":" << (UINT16)(aTimeBlock.usiSec) << ":"
	   << (UINT32)(aTimeBlock.udiUSec) << std::endl;

	return os.str();
}



std::ostream& operator<< (std::ostream& os, const EvalCaseResult& result)
{
	return os << result.toString();
}



} // END namespace datatypes
