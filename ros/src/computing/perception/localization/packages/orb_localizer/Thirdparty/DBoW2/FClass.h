/**
 * File: FClass.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: generic FClass to instantiate templated classes
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_FCLASS__
#define __D_T_FCLASS__

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

namespace DBoW2 {

/// Generic class to encapsulate functions to manage descriptors.
/**
 * This class must be inherited. Derived classes can be used as the
 * parameter F when creating Templated structures
 * (TemplatedVocabulary, TemplatedDatabase, ...)
 */
class FClass
{
  class TDescriptor;
  typedef const TDescriptor *pDescriptor;
  
  /**
   * Calculates the mean value of a set of descriptors
   * @param descriptors
   * @param mean mean descriptor
   */
  virtual void meanValue(const std::vector<pDescriptor> &descriptors, 
    TDescriptor &mean) = 0;
  
  /**
   * Calculates the distance between two descriptors
   * @param a
   * @param b
   * @return distance
   */
  static double distance(const TDescriptor &a, const TDescriptor &b);
  
  /**
   * Returns a string version of the descriptor
   * @param a descriptor
   * @return string version
   */
  static std::string toString(const TDescriptor &a);
  
  /**
   * Returns a descriptor from a string
   * @param a descriptor
   * @param s string version
   */
  static void fromString(TDescriptor &a, const std::string &s);

  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors
   * @param mat (out) NxL 32F matrix
   */
  static void toMat32F(const std::vector<TDescriptor> &descriptors, 
    cv::Mat &mat);
};

} // namespace DBoW2

#endif
