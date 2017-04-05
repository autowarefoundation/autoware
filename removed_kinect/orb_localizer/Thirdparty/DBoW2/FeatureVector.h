/**
 * File: FeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_FEATURE_VECTOR__
#define __D_T_FEATURE_VECTOR__

#include <map>
#include <vector>
#include <iostream>
#include "BowVector.h"

namespace DBoW2 {

/// Vector of nodes with indexes of local features
class FeatureVector: 
  public std::map<NodeId, std::vector<unsigned int> >
{
public:

  /**
   * Constructor
   */
  FeatureVector(void);
  
  /**
   * Destructor
   */
  ~FeatureVector(void);
  
  /**
   * Adds a feature to an existing node, or adds a new node with an initial
   * feature
   * @param id node id to add or to modify
   * @param i_feature index of feature to add to the given node
   */
  void addFeature(NodeId id, unsigned int i_feature);

  /**
   * Sends a string versions of the feature vector through the stream
   * @param out stream
   * @param v feature vector
   */
  friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);
    
};

} // namespace DBoW2

#endif

