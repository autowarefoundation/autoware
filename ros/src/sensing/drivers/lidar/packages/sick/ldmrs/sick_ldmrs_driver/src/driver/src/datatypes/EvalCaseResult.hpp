//
// EvalCaseResult.hpp
//
// A single EvalCaseResult.
//


#ifndef EVALCASERESULT_HPP
#define EVALCASERESULT_HPP

#include <ostream>
#include <string>
#include <vector>
#include "../BasicDatatypes.hpp"

namespace datatypes
{
	
//
// The EvalCaseResult class is a container for the current status of the configured EvalCases.
// Some scanners (such as the LDMRS) send such a structure with every scan, others (like the LMSxxx)
// send them only upon a change in any parameter value, e.g. if a field infringement happens.
//
// Typically, the reception of this structure is requested as a event.
//
class EvalCaseResult : public BasicData
{
public:
	UINT16 uiVersionNo;            // Version number of this structure (EvalCaseResult_t)

	struct                         // Header
	{
		UINT8  usiNumber;          // Evaluation case number
		UINT32 udiSysCount;        // Current system time since power on
		double dDistScaleFactor;   // Scale factor of radial distance
		double dDistScaleOffset;   // Scale offset of radial distance
		UINT32 uiAngleScaleFactor; // Angle resolution
		INT32  iAngleScaleOffset;  // Start angle of measurement area
	} CaseHdr;

	typedef enum
	{
		ECR_DONT_CARE = 0,
		ECR_LOW       = 1,
		ECR_HIGH      = 2,
		ECR_DETECTING = 3,
		ECR_FALLING   = 4,
		ECR_RAISING   = 5,
		ECR_INVALID   = 6
	} CaseResult;

	UINT8 m_eCaseResult;        // case result

	// aFieldInfringement is currently not in use. maybe it'll be added later.

	std::string m_sCaseName;         // Name of evaluation case (optional)
	std::string m_sComment;          // Comment (optional)

	struct                         // Timestamp
	{
		UINT16  uiYear;
		UINT8   usiMonth;
		UINT8   usiDay;
		UINT8   usiHour;
		UINT8   usiMinute;
		UINT8   usiSec;
		UINT32  udiUSec;
	} aTimeBlock;

//protected:



public:
	EvalCaseResult();
	~EvalCaseResult();

	std::string toString() const;
	const UINT32 getUsedMemory() const;
};


std::ostream&  operator<< (std::ostream& os, const EvalCaseResult& result);


} // END namespace datatypes


#endif // EVALCASERESULT_HPP
