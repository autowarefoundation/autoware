//
// SensorStateInfo.hpp
//
//  Created on: 02.09.2011
//      Author: wahnfla
//

#ifndef SENSORSTATEINFO_HPP
#define SENSORSTATEINFO_HPP

#include "Measurement.hpp"
#include "FieldDescription.hpp"
#include "Fields.hpp"
//#include "Serializable.hpp"
#include "../BasicDatatypes.hpp"
#include "EvalCases.hpp"
#include "EvalCaseResults.hpp"
#include <vector>
#include <map>

namespace datatypes
{

class SensorStateInfo : public BasicData
{
public:
	enum IOState
	{
		OFF = 0,
		ON = 1,
		PASSIVE = 2
	};

	static std::string ioStateToString(IOState state)
	{
		switch (state)
		{
		case OFF:
			return "OFF";
		case ON:
			return "ON";
		case PASSIVE:
			return "PASSIVE";
		}
		return "";
	}

	typedef std::vector<IOState> StateVector;
	typedef std::map<std::string, bool> StateMap;

	SensorStateInfo();

	virtual ~SensorStateInfo();

	const EvalCases& getEvalCases() const;
	const Fields& getFields() const;
	const StateVector& getInputStates() const;
	const MeasurementList& getMeasurementList() const;
	const StateVector& getOutputStates() const;
	const StateMap& getStateMap() const;
	UINT32 getDeviceID() const;
	void setEvalCases(const EvalCases& evalCases);
	void setFields(const Fields& fields);
	void setInputStates(const StateVector& m_inputStates);
	void setMeasurementList(const MeasurementList& m_measurementList);
	void setOutputStates(const StateVector& m_outputStates);
	void setStateMap(const StateMap& stateMap);
	void setDeviceID(UINT32 deviceID);
	void setLastKnownEvalCaseResults(const EvalCaseResults& evalCaseResults);

	std::string toString() const;

	const UINT32 getUsedMemory() const;	// Return total memory usage


private:
	/// e.g. temperature
	MeasurementList m_measurementList;
	Fields m_fields;
	EvalCases m_evalCases;
	StateVector m_inputStates;
	StateVector m_outputStates;
	StateMap m_stateMap;
};

} // namespace datatypes

#endif // SENSORSTATEINFO_HPP
