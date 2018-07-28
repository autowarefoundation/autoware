//
// SensorStateInfo.cpp
//
//  Created on: 02.09.2011
//      Author: wahnfla
//

#include "SensorStateInfo.hpp"
#include "Measurement.hpp"
#include "../tools/errorhandler.hpp"
//#include "StringToolbox.hpp"

namespace datatypes
{

SensorStateInfo::SensorStateInfo()
{
	m_datatype = Datatype_SensorStateInfo;
}


SensorStateInfo::~SensorStateInfo()
{
}

/**
 * Returns an estimation of the current memory usage of this object.
 * The correct count is *very* difficult to get because of the 1001 different
 * configuration options...
 */
const UINT32 SensorStateInfo::getUsedMemory() const
{
	UINT32 size;

	// Quick hack: Factor 4 may be ok...
	size = (UINT32)(4.0 * ((double)sizeof(*this)));

	return size;
}

const EvalCases& SensorStateInfo::getEvalCases() const
{
	return m_evalCases;
}

const Fields& SensorStateInfo::getFields() const
{
	return m_fields;
}

const SensorStateInfo::StateVector& SensorStateInfo::getInputStates() const
{
	return m_inputStates;
}

const MeasurementList& SensorStateInfo::getMeasurementList() const
{
	return m_measurementList;
}

const SensorStateInfo::StateVector& SensorStateInfo::getOutputStates() const
{
	return m_outputStates;
}

void SensorStateInfo::setEvalCases(const EvalCases& m_evalCases)
{
	this->m_evalCases = m_evalCases;
}

void SensorStateInfo::setFields(const Fields& m_fields)
{
	this->m_fields = m_fields;
}

void SensorStateInfo::setInputStates(const SensorStateInfo::StateVector& m_inputStates)
{
	this->m_inputStates = m_inputStates;
}

void SensorStateInfo::setMeasurementList(const MeasurementList& m_measurementList)
{
	this->m_measurementList = m_measurementList;
}

void SensorStateInfo::setOutputStates(const SensorStateInfo::StateVector& m_outputStates)
{
	this->m_outputStates = m_outputStates;
}

const SensorStateInfo::StateMap& SensorStateInfo::getStateMap() const
{
	return m_stateMap;
}

void SensorStateInfo::setStateMap(const SensorStateInfo::StateMap& stateMap)
{
	m_stateMap = stateMap;
}

// UINT32 SensorStateInfo::getDeviceID() const
// {
// 	return m_deviceID;
// }
// 
// void SensorStateInfo::setDeviceID(UINT32 deviceID)
// {
// 	m_deviceID = deviceID;
// }

//
//
//
std::string SensorStateInfo::toString() const
{
	std::stringstream output;

	// Measurement list
	std::vector<Measurement>::const_iterator meas;
	for (meas = m_measurementList.m_list.begin(); meas != m_measurementList.m_list.end(); ++meas)
	{
		output << meas->getName() << "=" ;
		output << meas->valueToString() << " ";
	}
	output << std::endl;

	// Filterparameter
	StateMap::const_iterator mapIter;
	for (mapIter = m_stateMap.begin(); mapIter != m_stateMap.end(); ++mapIter)
	{
		// First = Name
		// Second = Bool-Wert
		output << mapIter->first << "=" << std::string(mapIter->second ? "true" : "false") << " ";	// << std::endl;
	}
	output << std::endl;

	// Zustand der Bit-Eingaenge
	output << "Inputs:" << std::endl << " ";
	StateVector::const_iterator state;
	UINT32 number = 0;
	for (state = m_inputStates.begin(); state != m_inputStates.end(); ++state)
	{
		output << " " << number << "=" << ioStateToString(*state);	//  << std::endl;
		++number;
	}
	output << std::endl;

	// Zustand der Bit-Ausgaenge
	output << "Outputs:" << std::endl << " ";
	number = 0;
	for (state = m_outputStates.begin(); state != m_outputStates.end(); ++state)
	{
		output << " " << number << "=" << ioStateToString(*state);	// << std::endl;
		++number;
	}
	output << std::endl;

	// Koordinaten der Felder
	output << "Fields: " << std::endl;
	Fields::FieldVector::const_iterator field;
	for (field = m_fields.getFields().begin(); field != m_fields.getFields().end(); ++field)
	{
		const FieldParameter& fp = *(*field);
		output << "  " << fp.getFieldNumber() << ": " << fp.getFieldName() << ": " << FieldDescription::fieldTypeToString(fp.getFieldType()) << std::endl;
		output << "  " << fp.getPolygon().toString() << std::endl;
	}

	// Auswertefaelle
	output << "Evalcases: " << std::endl;
	EvalCaseVector::const_iterator ecp;
	number = 0;
	for (ecp = m_evalCases.getEvalCases().begin(); ecp != m_evalCases.getEvalCases().end(); ++ecp)
	{
		const EvalCase& evalCase = *(*ecp);
		output << evalCase.getCaseNumber() << "=" << evalCase.getCaseName();	//  << std::endl;
		output << ": " << "Strat.: " << EvalCase::strategyToString(evalCase.getStrategy());	// std::endl;
		output << "; " << "Blnk. size: " << evalCase.getBlankingSize() << "m";	//  << std::endl;
		output << "; " << "Dist. dep.: " << std::string(evalCase.isDistDependent() ? "true" : "false");	// << std::endl;
		output << "; " << "FieldNumber: " << evalCase.getFieldNumber();

		UINT32 responseTime = evalCase.getResponseTime();
		if (responseTime < 100000)
		{
			// Normal: Microsekunden
			output << "; " << "Resp. time: " << responseTime << " us";	//  << std::endl;
		}
		else
		{
			// Skaliert: Millisekunden
			output << "; " << "Resp. time: " << (UINT32)(responseTime / 1000) << " ms";	//  << std::endl;
		}
		output << "; " << "Output nr: " << evalCase.getOutputNumber();				//  << std::endl;
		output << "; " << "Res. neg.: " << std::string(evalCase.isResultNegation() ? "true" : "false") << std::endl;
		++number;
	}

	return output.str();
}

FieldParameter::CaseResult isFieldInfringed(const SensorStateInfo& sensorStateInfo, const FieldParameter& field, const EvalCaseResults& results)
{
	UINT8 isInfringed = 0;

	// find eval case that belongs to the field
	EvalCaseVector::const_iterator evalIter;
	for (evalIter = sensorStateInfo.getEvalCases().getEvalCases().begin(); evalIter != sensorStateInfo.getEvalCases().getEvalCases().end(); ++evalIter)
	{
		const EvalCase& ec = *(*evalIter);
		UINT32 i;
		for (i = 0; i < results.size(); ++i)
		{
			if (ec.getCaseNumber() == results[i].CaseHdr.usiNumber)
			{
				if (ec.getFieldNumber() == field.getFieldNumber())
				{
					isInfringed = results[i].m_eCaseResult;
					break;
				}
			}
		}
	}

	return FieldParameter::CaseResult(isInfringed);
}

void SensorStateInfo::setLastKnownEvalCaseResults(const EvalCaseResults& evalCaseResults)
{
	Fields::FieldVector::const_iterator fieldIter;
	for (fieldIter = m_fields.getFields().begin(); fieldIter != m_fields.getFields().end(); ++fieldIter)
	{
		FieldParameter& f = const_cast<FieldParameter&>(*(*fieldIter));
		FieldParameter::CaseResult fieldInfringed = isFieldInfringed(*this, f, evalCaseResults);
		f.setLastKnownInfringementState(fieldInfringed);
	}
}


} // namespace datatypes
