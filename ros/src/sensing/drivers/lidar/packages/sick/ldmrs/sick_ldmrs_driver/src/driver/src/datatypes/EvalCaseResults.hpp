//
// EvalCaseResults.hpp
// Container for eval case results
//


#ifndef EVALCASERESULTS_HPP
#define EVALCASERESULTS_HPP

#include <vector>
#include "../BasicDatatypes.hpp"
#include "EvalCaseResult.hpp"


namespace datatypes
{

//
//
//
//
class EvalCaseResults : public BasicData
{

protected:
	typedef std::vector<EvalCaseResult> EvalCaseResultVector;

	// List of EvalCaseResults
	EvalCaseResultVector m_evalCases;

public:
	EvalCaseResults();
	EvalCaseResults (UINT8 deviceID);
	~EvalCaseResults();
	const UINT32 getUsedMemory() const;

	std::string toString() const;

	// Returns the number of EventCaseResults.
	size_t size() const { return m_evalCases.size(); }

	// Returns true if EvalCases is empty.
	bool empty() const { return m_evalCases.empty(); }



	// Insert an EvalCaseResult.
	void add(const EvalCaseResult& newCase) { m_evalCases.push_back(newCase); }


	// Returns the n-th EventCaseResult
	const EvalCaseResult& at(UINT8 n) const { return m_evalCases.at(n); }

	const EvalCaseResult& operator[] (UINT32 n) const { return m_evalCases[n]; }

	bool operator==(const EvalCaseResults& other);

	bool operator!=(const EvalCaseResults& other);

};


} // END namespace datatypes


#endif // EVALCASERESULTS_HPP
