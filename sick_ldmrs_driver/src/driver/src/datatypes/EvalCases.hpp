//
// EvalCases.hpp
//
//  Created on: 05.09.2011
//      Author: wahnfla
//

#ifndef EVALCASES_HPP
#define EVALCASES_HPP

#include "../BasicDatatypes.hpp"
#include "EvalCase.hpp"
#include <vector>

namespace datatypes
{

typedef EvalCase* EvalCase_ptr;
typedef std::vector<EvalCase_ptr> EvalCaseVector;

class EvalCases : public BasicData
{
public:
	EvalCases();

	virtual ~EvalCases() {};
	virtual const UINT32 getUsedMemory() const;

	void clear();
	void add(EvalCase_ptr evalCase);
	const EvalCaseVector& getEvalCases() const;
	
private:
	EvalCaseVector m_evalCases;
};

} // namespace datatypes

#endif // EVALCASES_HPP
