//
// EvalCases.cpp
//
// Created on: 05.09.2011
//      Author: wahnfla
//

#include "EvalCases.hpp"
#include "../tools/errorhandler.hpp"

namespace datatypes
{

EvalCases::EvalCases()
{
	m_datatype = Datatype_EvalCases;
}

void EvalCases::clear()
{
	m_evalCases.clear();
}

//
// Add a new EvalCase to the list.
//
void EvalCases::add(EvalCase_ptr evalCase)
{
	if (m_evalCases.size() >= 16)
	{
		// Error
		printError("EvalCases::add: Cannot add this eval case. The MRS can only handle up to 16 eval cases!");
	}
	else
	{
		// Add the case
		m_evalCases.push_back(evalCase);
	}
}

const EvalCaseVector& EvalCases::getEvalCases() const
{
	return m_evalCases;
}

//
// Estimate the total memory usage of this object.
//
const UINT32 EvalCases::getUsedMemory() const
{
	UINT32 mem = 0;
	mem += sizeof(*this);
	for (UINT32 i = 0; i<m_evalCases.size(); i++)
	{
		EvalCase* ec = m_evalCases.at(i);
		mem += ec->getUsedMemory();
	}
	
	return mem;
}


} // namespace datatypes
