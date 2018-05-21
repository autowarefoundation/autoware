//
// EvalCaseResults.cpp
//

#include "EvalCaseResults.hpp"
#include <sstream>


namespace datatypes
{

//
EvalCaseResults::EvalCaseResults ()
{
	m_datatype = Datatype_EvalCaseResults;
}



EvalCaseResults::EvalCaseResults (UINT8 sourceId)
{
	m_datatype = Datatype_EvalCaseResults;
	m_sourceId = sourceId;
}



// Default destructor
EvalCaseResults::~EvalCaseResults()
{
}

// Estimate the memory usage of this object.
const UINT32 EvalCaseResults::getUsedMemory() const
{
	UINT32 sum = sizeof(*this);
	EvalCaseResultVector::const_iterator iter;
	for (iter = m_evalCases.begin(); iter != m_evalCases.end(); iter++)
	{
		sum += iter->getUsedMemory();
	}
	return sum;
}


std::string EvalCaseResults::toString() const
{
	std::ostringstream os;
	os << "There are " << size() << " EvalCaseResults:" << std::endl << std::endl;
	for (size_t i = 0; i < size(); i++)
	{
		os << at(i).toString() << std::endl;
	}

	return os.str();
}

//
//
//
bool EvalCaseResults::operator==(const EvalCaseResults& other)
{
	bool isTheSame = false;

	if (size() == other.size())
	{
		EvalCaseResultVector::const_iterator iter1, iter2;
		iter1 = m_evalCases.begin();
		iter2 = other.m_evalCases.begin();
		isTheSame = true;

		for (; iter1 != m_evalCases.end(); ++iter1, ++iter2)
		{
			if ((iter1->m_sCaseName != iter2->m_sCaseName) || (iter1->m_eCaseResult != iter2->m_eCaseResult))
			{
				isTheSame = false;
				break;
			}
		}
	}

	return isTheSame;
}


bool EvalCaseResults::operator!=(const EvalCaseResults& other)
{
	return ! operator==(other);
}

} // END namespace datatypes
