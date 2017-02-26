#include <vector>
#include <iostream>
#include <limits>
#include <time.h>
// http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

//typedef std::vector<int> std::vector<int>;
//typedef std::vector<float> std::vector<float>;

class AssignmentProblemSolver
{
private:
	// --------------------------------------------------------------------------
	// Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
	// --------------------------------------------------------------------------
	void assignmentoptimal(std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, size_t nOfRows, size_t nOfColumns);
	void buildassignmentvector(std::vector<int>& assignment, std::vector<bool>& starMatrix, size_t nOfRows, size_t nOfColumns);
	void computeassignmentcost(const std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, size_t nOfRows);
	void step2a(std::vector<int>& assignment, std::vector<float>& distMatrix, std::vector<bool>& starMatrix, std::vector<bool>& newStarMatrix, std::vector<bool>& primeMatrix, std::vector<bool>& coveredColumns, std::vector<bool>& coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
	void step2b(std::vector<int>& assignment, std::vector<float>& distMatrix, std::vector<bool>& starMatrix, std::vector<bool>& newStarMatrix, std::vector<bool>& primeMatrix, std::vector<bool>& coveredColumns, std::vector<bool>& coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
	void step3(std::vector<int>& assignment, std::vector<float>& distMatrix, std::vector<bool>& starMatrix, std::vector<bool>& newStarMatrix, std::vector<bool>& primeMatrix, std::vector<bool>& coveredColumns, std::vector<bool>& coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
	void step4(std::vector<int>& assignment, std::vector<float>&distMatrix, std::vector<bool>&starMatrix, std::vector<bool>& newStarMatrix, std::vector<bool>& primeMatrix, std::vector<bool>& coveredColumns, std::vector<bool>& coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim, size_t row, size_t col);
	void step5(std::vector<int>& assignment, std::vector<float>& distMatrix, std::vector<bool>& starMatrix, std::vector<bool>& newStarMatrix, std::vector<bool>& primeMatrix, std::vector<bool>& coveredColumns, std::vector<bool>& coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
	// --------------------------------------------------------------------------
	// Computes a suboptimal solution. Good for cases with many forbidden assignments.
	// --------------------------------------------------------------------------
	void assignmentsuboptimal1(std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, size_t nOfRows, size_t nOfColumns);
	// --------------------------------------------------------------------------
	// Computes a suboptimal solution. Good for cases with many forbidden assignments.
	// --------------------------------------------------------------------------
	void assignmentsuboptimal2(std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, size_t nOfRows, size_t nOfColumns);

public:
	enum TMethod
	{
		optimal,
		many_forbidden_assignments,
		without_forbidden_assignments
	};

	AssignmentProblemSolver();
	~AssignmentProblemSolver();
	float Solve(const std::vector<float>& distMatrixIn, size_t nOfRows, size_t nOfColumns, std::vector<int>& assignment, TMethod Method = optimal);
};
