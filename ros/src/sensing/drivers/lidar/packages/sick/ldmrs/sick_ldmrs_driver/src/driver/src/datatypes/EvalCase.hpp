//
// EvalCase.hpp
//
//  Created on: 05.09.2011
//      Author: wahnfla
//

#ifndef EVALCASE_HPP
#define EVALCASE_HPP

#include "../BasicDatatypes.hpp"
#include <vector>

namespace datatypes
{

//
// The parameters of an EvalCase. This *is* an EvalCase.
//
class EvalCase : public BasicData
{
public:
	enum EvaluationStrategy
	{
		BLANKING = 0,
		PIXEL = 1,
		CONTOUR = 2,
		IO_MAPPING = 3,
		FOCALPOSITION = 4
	};

	static std::string strategyToString(EvaluationStrategy strategy)
	{
		switch (strategy)
		{
		case BLANKING:
			return "BLANKING";
		case PIXEL:
			return "PIXEL";
		case CONTOUR:
			return "CONTOUR";
		case IO_MAPPING:
			return "IO_MAPPING";
		case FOCALPOSITION:
			return "FOCALPOSITION";
		}
		return "";
	}

	enum InputState
	{
		INPUT_INACTIVE = 0, ///< input is not relevant for this evaluation case
		INPUT_LOW_ACTIVE = 1,
		INPUT_HIGH_ACTIVE = 2
	};

	/// Manipulation prevention. If active, shadowing of field parts may trigger an event.
	enum ManipulationPrevention
	{
		ECS_INACTIVE = 0,
		ECS_ACTIVE = 1
	};

	/// kind of filter which is connected to evaluation case
	enum FilterType
	{
		UNFILTERED = 0
	};

//	typedef std::vector<InputState> InputStateVector;

	EvalCase();

	virtual ~EvalCase();

	virtual const UINT32 getUsedMemory() const;
	double getBlankingSize() const;
	const std::string& getCaseName() const;
	UINT8 getCaseNumber() const;
	const std::string& getComment() const;
	UINT16 getFieldNumber() const;
	FilterType getFilterType() const;
//	const InputStateVector& getHardwareInputs() const;
	const UINT8 getLogicalInputState_as_UINT8() const;
	double getMaxRadialCorridor() const;
	double getMinFieldExp() const;
	UINT8 getOutputNumber() const;
	UINT32 getResponseTime() const;
	UINT32 getResponseTimeExtended() const;
	EvaluationStrategy getStrategy() const;
//	UINT8 getStrategy() const;
	UINT32 getVersionNumberIntern() const;
	bool isDistDependent() const;
	bool isResultNegation() const;
	void setBlankingSize(double blankingSize);
	void setCaseName(const std::string& caseName);
	void setCaseNumber(UINT8 caseNumber);
	void setComment(const std::string& comment);
	void setDistDependent(bool distDependent);
	bool getDistDependent();
	void setFieldNumber(UINT16 fieldNumber);
	void setFilterType(FilterType filterType);
//	void setHardwareInputs(const InputStateVector& hardwareInputs);
//	void setLogicalInputs(const InputStateVector& logicalInputs);
	void setMaxRadialCorridor(double maxRadialCorridor);
	void setMinFieldExp(double minFieldExp);
	void setOutputNumber(UINT8 outputNumber);
	void setResponseTime(UINT32 responseTime);
	void setResponseTimeExtended(UINT32 responseTimeExtended);
	void setResultNegation(bool resultNegation);
	bool getResultNegation();
	void setStrategy(EvaluationStrategy strategy);
	void setVersionNumber(UINT16 versionNumber);
	UINT16 getVersionNumber();

	void setLogicalInputState_from_UINT8(UINT8 value);
	ManipulationPrevention getManipulationPrevention() const;
	void setManipulationPrevention(ManipulationPrevention manPrev);

private:
	InputState m_logicalInputState[2];	// two inputs
	UINT16 m_versionNumberIntern;
	UINT8 m_caseNumber;
	EvaluationStrategy m_strategy;
	bool m_resultNegation; ///< negation of evaluation result
	UINT32 m_responseTime; ///< [µs]
	UINT32 m_responseTimeExtended; ///< [µs]
	UINT16 m_outputNumber; ///< output which belongs to this evaluation case
//	InputStateVector m_hardwareInputs;
//	InputStateVector m_logicalInputs;
	bool m_distDependent; ///< if TRUE field evaluation works distance dependent
	double m_maxRadialCorridor; ///< [m]; max radial corridor (only active if distance dependency is active)
	ManipulationPrevention m_manipulationPrevention;
	double m_blankingSize;	///< [m]; minimum object size
	double m_minFieldExp;	///< [m]; minimum radial distance between field start and end point
	UINT16 m_fieldNumber;	///< index of field which is connected to evaluation case
	FilterType m_filterType;
	std::string m_caseName; ///< name of evaluation case is optional
	std::string m_comment;	///< comment is optional
};


} // namespace datatypes

#endif // EVALCASE_HPP
