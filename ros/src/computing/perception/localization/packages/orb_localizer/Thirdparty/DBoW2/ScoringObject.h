/**
 * File: ScoringObject.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions to compute bow scores 
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_SCORING_OBJECT__
#define __D_T_SCORING_OBJECT__

#include "BowVector.h"

namespace DBoW2 {

/// Base class of scoring functions
class GeneralScoring
{
public:
  /**
   * Computes the score between two vectors. Vectors must be sorted and 
   * normalized if necessary
   * @param v (in/out)
   * @param w (in/out)
   * @return score
   */
  virtual double score(const BowVector &v, const BowVector &w) const = 0;

  /**
   * Returns whether a vector must be normalized before scoring according
   * to the scoring scheme
   * @param norm norm to use
   * @return true iff must normalize
   */
  virtual bool mustNormalize(LNorm &norm) const = 0;

  /// Log of epsilon
	static const double LOG_EPS; 
  // If you change the type of WordValue, make sure you change also the
	// epsilon value (this is needed by the KL method)

  virtual ~GeneralScoring() {} //!< Required for virtual base classes
	
};

/** 
 * Macro for defining Scoring classes
 * @param NAME name of class
 * @param MUSTNORMALIZE if vectors must be normalized to compute the score
 * @param NORM type of norm to use when MUSTNORMALIZE
 */
#define __SCORING_CLASS(NAME, MUSTNORMALIZE, NORM) \
  NAME: public GeneralScoring \
  { public: \
    /** \
     * Computes score between two vectors \
     * @param v \
     * @param w \
     * @return score between v and w \
     */ \
    virtual double score(const BowVector &v, const BowVector &w) const; \
    \
    /** \
     * Says if a vector must be normalized according to the scoring function \
     * @param norm (out) if true, norm to use
     * @return true iff vectors must be normalized \
     */ \
    virtual inline bool mustNormalize(LNorm &norm) const  \
      { norm = NORM; return MUSTNORMALIZE; } \
  }
  
/// L1 Scoring object
class __SCORING_CLASS(L1Scoring, true, L1);

/// L2 Scoring object
class __SCORING_CLASS(L2Scoring, true, L2);

/// Chi square Scoring object
class __SCORING_CLASS(ChiSquareScoring, true, L1);

/// KL divergence Scoring object
class __SCORING_CLASS(KLScoring, true, L1);

/// Bhattacharyya Scoring object
class __SCORING_CLASS(BhattacharyyaScoring, true, L1);

/// Dot product Scoring object
class __SCORING_CLASS(DotProductScoring, false, L1);

#undef __SCORING_CLASS
  
} // namespace DBoW2

#endif

