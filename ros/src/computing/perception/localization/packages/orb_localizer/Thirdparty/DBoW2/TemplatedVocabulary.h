/**
 * This is a modified version of TemplatedVocabulary.h from DBoW2 (see below).
 * Added functions: Save and Load from text files without using cv::FileStorage.
 * Date: August 2015
 * Raúl Mur-Artal
 */

/**
 * File: TemplatedVocabulary.h
 * Date: February 2011
 * Author: Dorian Galvez-Lopez
 * Description: templated vocabulary 
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_TEMPLATED_VOCABULARY__
#define __D_T_TEMPLATED_VOCABULARY__

#include <cassert>

#include <vector>
#include <numeric>
#include <fstream>
#include <string>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <limits>

#include "DUtils/Random.h"
#include "BowVector.h"
#include "FeatureVector.h"
#include "ScoringObject.h"

using namespace std;

namespace DBoW2 {

/// @param TDescriptor class of descriptor
/// @param F class of descriptor functions
template<class TDescriptor, class F>
/// Generic Vocabulary
class TemplatedVocabulary
{		
public:
  
  /**
   * Initiates an empty vocabulary
   * @param k branching factor
   * @param L depth levels
   * @param weighting weighting type
   * @param scoring scoring type
   */
  TemplatedVocabulary(int k = 10, int L = 5, 
    WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM);
  
  /**
   * Creates the vocabulary by loading a file
   * @param filename
   */
  TemplatedVocabulary(const std::string &filename);
  
  /**
   * Creates the vocabulary by loading a file
   * @param filename
   */
  TemplatedVocabulary(const char *filename);
  
  /** 
   * Copy constructor
   * @param voc
   */
  TemplatedVocabulary(const TemplatedVocabulary<TDescriptor, F> &voc);
  
  /**
   * Destructor
   */
  virtual ~TemplatedVocabulary();
  
  /** 
   * Assigns the given vocabulary to this by copying its data and removing
   * all the data contained by this vocabulary before
   * @param voc
   * @return reference to this vocabulary
   */
  TemplatedVocabulary<TDescriptor, F>& operator=(
    const TemplatedVocabulary<TDescriptor, F> &voc);
  
  /** 
   * Creates a vocabulary from the training features with the already
   * defined parameters
   * @param training_features
   */
  virtual void create
    (const std::vector<std::vector<TDescriptor> > &training_features);
  
  /**
   * Creates a vocabulary from the training features, setting the branching
   * factor and the depth levels of the tree
   * @param training_features
   * @param k branching factor
   * @param L depth levels
   */
  virtual void create
    (const std::vector<std::vector<TDescriptor> > &training_features, 
      int k, int L);

  /**
   * Creates a vocabulary from the training features, setting the branching
   * factor nad the depth levels of the tree, and the weighting and scoring
   * schemes
   */
  virtual void create
    (const std::vector<std::vector<TDescriptor> > &training_features,
      int k, int L, WeightingType weighting, ScoringType scoring);

  /**
   * Returns the number of words in the vocabulary
   * @return number of words
   */
  virtual inline unsigned int size() const;
  
  /**
   * Returns whether the vocabulary is empty (i.e. it has not been trained)
   * @return true iff the vocabulary is empty
   */
  virtual inline bool empty() const;

  /**
   * Transforms a set of descriptores into a bow vector
   * @param features
   * @param v (out) bow vector of weighted words
   */
  virtual void transform(const std::vector<TDescriptor>& features, BowVector &v) 
    const;
  
  /**
   * Transform a set of descriptors into a bow vector and a feature vector
   * @param features
   * @param v (out) bow vector
   * @param fv (out) feature vector of nodes and feature indexes
   * @param levelsup levels to go up the vocabulary tree to get the node index
   */
  virtual void transform(const std::vector<TDescriptor>& features,
    BowVector &v, FeatureVector &fv, int levelsup) const;

  /**
   * Transforms a single feature into a word (without weight)
   * @param feature
   * @return word id
   */
  virtual WordId transform(const TDescriptor& feature) const;
  
  /**
   * Returns the score of two vectors
   * @param a vector
   * @param b vector
   * @return score between vectors
   * @note the vectors must be already sorted and normalized if necessary
   */
  inline double score(const BowVector &a, const BowVector &b) const;
  
  /**
   * Returns the id of the node that is "levelsup" levels from the word given
   * @param wid word id
   * @param levelsup 0..L
   * @return node id. if levelsup is 0, returns the node id associated to the
   *   word id
   */
  virtual NodeId getParentNode(WordId wid, int levelsup) const;
  
  /**
   * Returns the ids of all the words that are under the given node id,
   * by traversing any of the branches that goes down from the node
   * @param nid starting node id
   * @param words ids of words
   */
  void getWordsFromNode(NodeId nid, std::vector<WordId> &words) const;
  
  /**
   * Returns the branching factor of the tree (k)
   * @return k
   */
  inline int getBranchingFactor() const { return m_k; }
  
  /** 
   * Returns the depth levels of the tree (L)
   * @return L
   */
  inline int getDepthLevels() const { return m_L; }
  
  /**
   * Returns the real depth levels of the tree on average
   * @return average of depth levels of leaves
   */
  float getEffectiveLevels() const;
  
  /**
   * Returns the descriptor of a word
   * @param wid word id
   * @return descriptor
   */
  virtual inline TDescriptor getWord(WordId wid) const;
  
  /**
   * Returns the weight of a word
   * @param wid word id
   * @return weight
   */
  virtual inline WordValue getWordWeight(WordId wid) const;
  
  /** 
   * Returns the weighting method
   * @return weighting method
   */
  inline WeightingType getWeightingType() const { return m_weighting; }
  
  /** 
   * Returns the scoring method
   * @return scoring method
   */
  inline ScoringType getScoringType() const { return m_scoring; }
  
  /**
   * Changes the weighting method
   * @param type new weighting type
   */
  inline void setWeightingType(WeightingType type);
  
  /**
   * Changes the scoring method
   * @param type new scoring type
   */
  void setScoringType(ScoringType type);

  /**
   * Loads the vocabulary from a text file
   * @param filename
   */
  bool loadFromTextFile(const std::string &filename);

  /**
   * Saves the vocabulary into a text file
   * @param filename
   */
  void saveToTextFile(const std::string &filename) const;  

  /**
   * Saves the vocabulary into a file
   * @param filename
   */
  void save(const std::string &filename) const;
  
  /**
   * Loads the vocabulary from a file
   * @param filename
   */
  void load(const std::string &filename);
  
  /** 
   * Saves the vocabulary to a file storage structure
   * @param fn node in file storage
   */
  virtual void save(cv::FileStorage &fs, 
    const std::string &name = "vocabulary") const;
  
  /**
   * Loads the vocabulary from a file storage node
   * @param fn first node
   * @param subname name of the child node of fn where the tree is stored.
   *   If not given, the fn node is used instead
   */  
  virtual void load(const cv::FileStorage &fs, 
    const std::string &name = "vocabulary");
  
  /** 
   * Stops those words whose weight is below minWeight.
   * Words are stopped by setting their weight to 0. There are not returned
   * later when transforming image features into vectors.
   * Note that when using IDF or TF_IDF, the weight is the idf part, which
   * is equivalent to -log(f), where f is the frequency of the word
   * (f = Ni/N, Ni: number of training images where the word is present, 
   * N: number of training images).
   * Note that the old weight is forgotten, and subsequent calls to this 
   * function with a lower minWeight have no effect.
   * @return number of words stopped now
   */
  virtual int stopWords(double minWeight);

protected:

  /// Pointer to descriptor
  typedef const TDescriptor *pDescriptor;

  /// Tree node
  struct Node 
  {
    /// Node id
    NodeId id;
    /// Weight if the node is a word
    WordValue weight;
    /// Children 
    vector<NodeId> children;
    /// Parent node (undefined in case of root)
    NodeId parent;
    /// Node descriptor
    TDescriptor descriptor;

    /// Word id if the node is a word
    WordId word_id;

    /**
     * Empty constructor
     */
    Node(): id(0), weight(0), parent(0), word_id(0){}
    
    /**
     * Constructor
     * @param _id node id
     */
    Node(NodeId _id): id(_id), weight(0), parent(0), word_id(0){}

    /**
     * Returns whether the node is a leaf node
     * @return true iff the node is a leaf
     */
    inline bool isLeaf() const { return children.empty(); }
  };

protected:

  /**
   * Creates an instance of the scoring object accoring to m_scoring
   */
  void createScoringObject();

  /** 
   * Returns a set of pointers to descriptores
   * @param training_features all the features
   * @param features (out) pointers to the training features
   */
  void getFeatures(
    const vector<vector<TDescriptor> > &training_features, 
    vector<pDescriptor> &features) const;

  /**
   * Returns the word id associated to a feature
   * @param feature
   * @param id (out) word id
   * @param weight (out) word weight
   * @param nid (out) if given, id of the node "levelsup" levels up
   * @param levelsup
   */
  virtual void transform(const TDescriptor &feature, 
    WordId &id, WordValue &weight, NodeId* nid = NULL, int levelsup = 0) const;

  /**
   * Returns the word id associated to a feature
   * @param feature
   * @param id (out) word id
   */
  virtual void transform(const TDescriptor &feature, WordId &id) const;
      
  /**
   * Creates a level in the tree, under the parent, by running kmeans with
   * a descriptor set, and recursively creates the subsequent levels too
   * @param parent_id id of parent node
   * @param descriptors descriptors to run the kmeans on
   * @param current_level current level in the tree
   */
  void HKmeansStep(NodeId parent_id, const vector<pDescriptor> &descriptors, 
    int current_level);

  /**
   * Creates k clusters from the given descriptors with some seeding algorithm.
   * @note In this class, kmeans++ is used, but this function should be
   *   overriden by inherited classes.
   */
  virtual void initiateClusters(const vector<pDescriptor> &descriptors,
    vector<TDescriptor> &clusters) const;
  
  /**
   * Creates k clusters from the given descriptor sets by running the
   * initial step of kmeans++
   * @param descriptors 
   * @param clusters resulting clusters
   */
  void initiateClustersKMpp(const vector<pDescriptor> &descriptors, 
    vector<TDescriptor> &clusters) const;
  
  /**
   * Create the words of the vocabulary once the tree has been built
   */
  void createWords();
  
  /**
   * Sets the weights of the nodes of tree according to the given features.
   * Before calling this function, the nodes and the words must be already
   * created (by calling HKmeansStep and createWords)
   * @param features
   */
  void setNodeWeights(const vector<vector<TDescriptor> > &features);
  
protected:

  /// Branching factor
  int m_k;
  
  /// Depth levels 
  int m_L;
  
  /// Weighting method
  WeightingType m_weighting;
  
  /// Scoring method
  ScoringType m_scoring;
  
  /// Object for computing scores
  GeneralScoring* m_scoring_object;
  
  /// Tree nodes
  std::vector<Node> m_nodes;
  
  /// Words of the vocabulary (tree leaves)
  /// this condition holds: m_words[wid]->word_id == wid
  std::vector<Node*> m_words;
  
};

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary
  (int k, int L, WeightingType weighting, ScoringType scoring)
  : m_k(k), m_L(L), m_weighting(weighting), m_scoring(scoring),
  m_scoring_object(NULL)
{
  createScoringObject();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary
  (const std::string &filename): m_scoring_object(NULL)
{
  load(filename);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary
  (const char *filename): m_scoring_object(NULL)
{
  load(filename);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::createScoringObject()
{
  delete m_scoring_object;
  m_scoring_object = NULL;
  
  switch(m_scoring)
  {
    case L1_NORM: 
      m_scoring_object = new L1Scoring;
      break;
      
    case L2_NORM:
      m_scoring_object = new L2Scoring;
      break;
    
    case CHI_SQUARE:
      m_scoring_object = new ChiSquareScoring;
      break;
      
    case KL:
      m_scoring_object = new KLScoring;
      break;
      
    case BHATTACHARYYA:
      m_scoring_object = new BhattacharyyaScoring;
      break;
      
    case DOT_PRODUCT:
      m_scoring_object = new DotProductScoring;
      break;
    
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setScoringType(ScoringType type)
{
  m_scoring = type;
  createScoringObject();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setWeightingType(WeightingType type)
{
  this->m_weighting = type;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary(
  const TemplatedVocabulary<TDescriptor, F> &voc)
  : m_scoring_object(NULL)
{
  *this = voc;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::~TemplatedVocabulary()
{
  delete m_scoring_object;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor, F>& 
TemplatedVocabulary<TDescriptor,F>::operator=
  (const TemplatedVocabulary<TDescriptor, F> &voc)
{  
  this->m_k = voc.m_k;
  this->m_L = voc.m_L;
  this->m_scoring = voc.m_scoring;
  this->m_weighting = voc.m_weighting;

  this->createScoringObject();
  
  this->m_nodes.clear();
  this->m_words.clear();
  
  this->m_nodes = voc.m_nodes;
  this->createWords();
  
  return *this;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::create(
  const std::vector<std::vector<TDescriptor> > &training_features)
{
  m_nodes.clear();
  m_words.clear();
  
  // expected_nodes = Sum_{i=0..L} ( k^i )
	int expected_nodes = 
		(int)((pow((double)m_k, (double)m_L + 1) - 1)/(m_k - 1));

  m_nodes.reserve(expected_nodes); // avoid allocations when creating the tree
  
  
  vector<pDescriptor> features;
  getFeatures(training_features, features);


  // create root  
  m_nodes.push_back(Node(0)); // root
  
  // create the tree
  HKmeansStep(0, features, 1);

  // create the words
  createWords();

  // and set the weight of each node of the tree
  setNodeWeights(training_features);
  
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::create(
  const std::vector<std::vector<TDescriptor> > &training_features,
  int k, int L)
{
  m_k = k;
  m_L = L;
  
  create(training_features);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::create(
  const std::vector<std::vector<TDescriptor> > &training_features,
  int k, int L, WeightingType weighting, ScoringType scoring)
{
  m_k = k;
  m_L = L;
  m_weighting = weighting;
  m_scoring = scoring;
  createScoringObject();
  
  create(training_features);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::getFeatures(
  const vector<vector<TDescriptor> > &training_features, 
  vector<pDescriptor> &features) const
{
  features.resize(0);
  
  typename vector<vector<TDescriptor> >::const_iterator vvit;
  typename vector<TDescriptor>::const_iterator vit;
  for(vvit = training_features.begin(); vvit != training_features.end(); ++vvit)
  {
    features.reserve(features.size() + vvit->size());
    for(vit = vvit->begin(); vit != vvit->end(); ++vit)
    {
      features.push_back(&(*vit));
    }
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::HKmeansStep(NodeId parent_id, 
  const vector<pDescriptor> &descriptors, int current_level)
{
  if(descriptors.empty()) return;
        
  // features associated to each cluster
  vector<TDescriptor> clusters;
	vector<vector<unsigned int> > groups; // groups[i] = [j1, j2, ...]
	// j1, j2, ... indices of descriptors associated to cluster i

  clusters.reserve(m_k);
	groups.reserve(m_k);
  
  //const int msizes[] = { m_k, descriptors.size() };
  //cv::SparseMat assoc(2, msizes, CV_8U);
  //cv::SparseMat last_assoc(2, msizes, CV_8U);  
  //// assoc.row(cluster_idx).col(descriptor_idx) = 1 iif associated
  
  if((int)descriptors.size() <= m_k)
  {
    // trivial case: one cluster per feature
    groups.resize(descriptors.size());

    for(unsigned int i = 0; i < descriptors.size(); i++)
    {
      groups[i].push_back(i);
      clusters.push_back(*descriptors[i]);
    }
  }
  else
  {
    // select clusters and groups with kmeans
    
    bool first_time = true;
    bool goon = true;
    
    // to check if clusters move after iterations
    vector<int> last_association, current_association;

    while(goon)
    {
      // 1. Calculate clusters

			if(first_time)
			{
        // random sample 
        initiateClusters(descriptors, clusters);
      }
      else
      {
        // calculate cluster centres

        for(unsigned int c = 0; c < clusters.size(); ++c)
        {
          vector<pDescriptor> cluster_descriptors;
          cluster_descriptors.reserve(groups[c].size());
          
          /*
          for(unsigned int d = 0; d < descriptors.size(); ++d)
          {
            if( assoc.find<unsigned char>(c, d) )
            {
              cluster_descriptors.push_back(descriptors[d]);
            }
          }
          */
          
          vector<unsigned int>::const_iterator vit;
          for(vit = groups[c].begin(); vit != groups[c].end(); ++vit)
          {
            cluster_descriptors.push_back(descriptors[*vit]);
          }
          
          
          F::meanValue(cluster_descriptors, clusters[c]);
        }
        
      } // if(!first_time)

      // 2. Associate features with clusters

      // calculate distances to cluster centers
      groups.clear();
      groups.resize(clusters.size(), vector<unsigned int>());
      current_association.resize(descriptors.size());

      //assoc.clear();

      typename vector<pDescriptor>::const_iterator fit;
      //unsigned int d = 0;
      for(fit = descriptors.begin(); fit != descriptors.end(); ++fit)//, ++d)
      {
        double best_dist = F::distance(*(*fit), clusters[0]);
        unsigned int icluster = 0;
        
        for(unsigned int c = 1; c < clusters.size(); ++c)
        {
          double dist = F::distance(*(*fit), clusters[c]);
          if(dist < best_dist)
          {
            best_dist = dist;
            icluster = c;
          }
        }

        //assoc.ref<unsigned char>(icluster, d) = 1;

        groups[icluster].push_back(fit - descriptors.begin());
        current_association[ fit - descriptors.begin() ] = icluster;
      }
      
      // kmeans++ ensures all the clusters has any feature associated with them

      // 3. check convergence
      if(first_time)
      {
        first_time = false;
      }
      else
      {
        //goon = !eqUChar(last_assoc, assoc);
        
        goon = false;
        for(unsigned int i = 0; i < current_association.size(); i++)
        {
          if(current_association[i] != last_association[i]){
            goon = true;
            break;
          }
        }
      }

			if(goon)
			{
				// copy last feature-cluster association
				last_association = current_association;
				//last_assoc = assoc.clone();
			}
			
		} // while(goon)
    
  } // if must run kmeans
  
  // create nodes
  for(unsigned int i = 0; i < clusters.size(); ++i)
  {
    NodeId id = m_nodes.size();
    m_nodes.push_back(Node(id));
    m_nodes.back().descriptor = clusters[i];
    m_nodes.back().parent = parent_id;
    m_nodes[parent_id].children.push_back(id);
  }
  
  // go on with the next level
  if(current_level < m_L)
  {
    // iterate again with the resulting clusters
    const vector<NodeId> &children_ids = m_nodes[parent_id].children;
    for(unsigned int i = 0; i < clusters.size(); ++i)
    {
      NodeId id = children_ids[i];

      vector<pDescriptor> child_features;
      child_features.reserve(groups[i].size());

      vector<unsigned int>::const_iterator vit;
      for(vit = groups[i].begin(); vit != groups[i].end(); ++vit)
      {
        child_features.push_back(descriptors[*vit]);
      }

      if(child_features.size() > 1)
      {
        HKmeansStep(id, child_features, current_level + 1);
      }
    }
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor, F>::initiateClusters
  (const vector<pDescriptor> &descriptors, vector<TDescriptor> &clusters) const
{
  initiateClustersKMpp(descriptors, clusters);  
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::initiateClustersKMpp(
  const vector<pDescriptor> &pfeatures, vector<TDescriptor> &clusters) const
{
  // Implements kmeans++ seeding algorithm
  // Algorithm:
  // 1. Choose one center uniformly at random from among the data points.
  // 2. For each data point x, compute D(x), the distance between x and the nearest 
  //    center that has already been chosen.
  // 3. Add one new data point as a center. Each point x is chosen with probability 
  //    proportional to D(x)^2.
  // 4. Repeat Steps 2 and 3 until k centers have been chosen.
  // 5. Now that the initial centers have been chosen, proceed using standard k-means 
  //    clustering.

  DUtils::Random::SeedRandOnce();

  clusters.resize(0);
  clusters.reserve(m_k);
  vector<double> min_dists(pfeatures.size(), std::numeric_limits<double>::max());
  
  // 1.
  
  int ifeature = DUtils::Random::RandomInt(0, pfeatures.size()-1);
  
  // create first cluster
  clusters.push_back(*pfeatures[ifeature]);

  // compute the initial distances
  typename vector<pDescriptor>::const_iterator fit;
  vector<double>::iterator dit;
  dit = min_dists.begin();
  for(fit = pfeatures.begin(); fit != pfeatures.end(); ++fit, ++dit)
  {
    *dit = F::distance(*(*fit), clusters.back());
  }  

  while((int)clusters.size() < m_k)
  {
    // 2.
    dit = min_dists.begin();
    for(fit = pfeatures.begin(); fit != pfeatures.end(); ++fit, ++dit)
    {
      if(*dit > 0)
      {
        double dist = F::distance(*(*fit), clusters.back());
        if(dist < *dit) *dit = dist;
      }
    }
    
    // 3.
    double dist_sum = std::accumulate(min_dists.begin(), min_dists.end(), 0.0);

    if(dist_sum > 0)
    {
      double cut_d;
      do
      {
        cut_d = DUtils::Random::RandomValue<double>(0, dist_sum);
      } while(cut_d == 0.0);

      double d_up_now = 0;
      for(dit = min_dists.begin(); dit != min_dists.end(); ++dit)
      {
        d_up_now += *dit;
        if(d_up_now >= cut_d) break;
      }
      
      if(dit == min_dists.end()) 
        ifeature = pfeatures.size()-1;
      else
        ifeature = dit - min_dists.begin();
      
      clusters.push_back(*pfeatures[ifeature]);

    } // if dist_sum > 0
    else
      break;
      
  } // while(used_clusters < m_k)

}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::createWords()
{
  m_words.resize(0);
  
  if(!m_nodes.empty())
  {
    m_words.reserve( (int)pow((double)m_k, (double)m_L) );

    typename vector<Node>::iterator nit;
    
    nit = m_nodes.begin(); // ignore root
    for(++nit; nit != m_nodes.end(); ++nit)
    {
      if(nit->isLeaf())
      {
        nit->word_id = m_words.size();
        m_words.push_back( &(*nit) );
      }
    }
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setNodeWeights
  (const vector<vector<TDescriptor> > &training_features)
{
  const unsigned int NWords = m_words.size();
  const unsigned int NDocs = training_features.size();

  if(m_weighting == TF || m_weighting == BINARY)
  {
    // idf part must be 1 always
    for(unsigned int i = 0; i < NWords; i++)
      m_words[i]->weight = 1;
  }
  else if(m_weighting == IDF || m_weighting == TF_IDF)
  {
    // IDF and TF-IDF: we calculte the idf path now

    // Note: this actually calculates the idf part of the tf-idf score.
    // The complete tf-idf score is calculated in ::transform

    vector<unsigned int> Ni(NWords, 0);
    vector<bool> counted(NWords, false);
    
    typename vector<vector<TDescriptor> >::const_iterator mit;
    typename vector<TDescriptor>::const_iterator fit;

    for(mit = training_features.begin(); mit != training_features.end(); ++mit)
    {
      fill(counted.begin(), counted.end(), false);

      for(fit = mit->begin(); fit < mit->end(); ++fit)
      {
        WordId word_id;
        transform(*fit, word_id);

        if(!counted[word_id])
        {
          Ni[word_id]++;
          counted[word_id] = true;
        }
      }
    }

    // set ln(N/Ni)
    for(unsigned int i = 0; i < NWords; i++)
    {
      if(Ni[i] > 0)
      {
        m_words[i]->weight = log((double)NDocs / (double)Ni[i]);
      }// else // This cannot occur if using kmeans++
    }
  
  }

}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline unsigned int TemplatedVocabulary<TDescriptor,F>::size() const
{
  return m_words.size();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline bool TemplatedVocabulary<TDescriptor,F>::empty() const
{
  return m_words.empty();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
float TemplatedVocabulary<TDescriptor,F>::getEffectiveLevels() const
{
  long sum = 0;
  typename std::vector<Node*>::const_iterator wit;
  for(wit = m_words.begin(); wit != m_words.end(); ++wit)
  {
    const Node *p = *wit;
    
    for(; p->id != 0; sum++) p = &m_nodes[p->parent];
  }
  
  return (float)((double)sum / (double)m_words.size());
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TDescriptor TemplatedVocabulary<TDescriptor,F>::getWord(WordId wid) const
{
  return m_words[wid]->descriptor;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
WordValue TemplatedVocabulary<TDescriptor, F>::getWordWeight(WordId wid) const
{
  return m_words[wid]->weight;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
WordId TemplatedVocabulary<TDescriptor, F>::transform
  (const TDescriptor& feature) const
{
  if(empty())
  {
    return 0;
  }
  
  WordId wid;
  transform(feature, wid);
  return wid;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(
  const std::vector<TDescriptor>& features, BowVector &v) const
{
  v.clear();
  
  if(empty())
  {
    return;
  }

  // normalize 
  LNorm norm;
  bool must = m_scoring_object->mustNormalize(norm);

	typename vector<TDescriptor>::const_iterator fit;

  if(m_weighting == TF || m_weighting == TF_IDF)
  {
    for(fit = features.begin(); fit < features.end(); ++fit)
    {
      WordId id;
      WordValue w; 
      // w is the idf value if TF_IDF, 1 if TF
      
      transform(*fit, id, w);
      
      // not stopped
      if(w > 0) v.addWeight(id, w);
    }
    
    if(!v.empty() && !must)
    {
      // unnecessary when normalizing
      const double nd = v.size();
      for(BowVector::iterator vit = v.begin(); vit != v.end(); vit++) 
        vit->second /= nd;
    }
    
  }
  else // IDF || BINARY
  {
    for(fit = features.begin(); fit < features.end(); ++fit)
    {
      WordId id;
      WordValue w;
      // w is idf if IDF, or 1 if BINARY
      
      transform(*fit, id, w);
      
      // not stopped
      if(w > 0) v.addIfNotExist(id, w);
      
    } // if add_features
  } // if m_weighting == ...
  
  if(must) v.normalize(norm);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F> 
void TemplatedVocabulary<TDescriptor,F>::transform(
  const std::vector<TDescriptor>& features,
  BowVector &v, FeatureVector &fv, int levelsup) const
{
  v.clear();
  fv.clear();
  
  if(empty()) // safe for subclasses
  {
    return;
  }
  
  // normalize 
  LNorm norm;
  bool must = m_scoring_object->mustNormalize(norm);
  
  typename vector<TDescriptor>::const_iterator fit;
  
  if(m_weighting == TF || m_weighting == TF_IDF)
  {
    unsigned int i_feature = 0;
    for(fit = features.begin(); fit < features.end(); ++fit, ++i_feature)
    {
      WordId id;
      NodeId nid;
      WordValue w; 
      // w is the idf value if TF_IDF, 1 if TF
      
      transform(*fit, id, w, &nid, levelsup);
      
      if(w > 0) // not stopped
      { 
        v.addWeight(id, w);
        fv.addFeature(nid, i_feature);
      }
    }
    
    if(!v.empty() && !must)
    {
      // unnecessary when normalizing
      const double nd = v.size();
      for(BowVector::iterator vit = v.begin(); vit != v.end(); vit++) 
        vit->second /= nd;
    }
  
  }
  else // IDF || BINARY
  {
    unsigned int i_feature = 0;
    for(fit = features.begin(); fit < features.end(); ++fit, ++i_feature)
    {
      WordId id;
      NodeId nid;
      WordValue w;
      // w is idf if IDF, or 1 if BINARY
      
      transform(*fit, id, w, &nid, levelsup);
      
      if(w > 0) // not stopped
      {
        v.addIfNotExist(id, w);
        fv.addFeature(nid, i_feature);
      }
    }
  } // if m_weighting == ...
  
  if(must) v.normalize(norm);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F> 
inline double TemplatedVocabulary<TDescriptor,F>::score
  (const BowVector &v1, const BowVector &v2) const
{
  return m_scoring_object->score(v1, v2);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform
  (const TDescriptor &feature, WordId &id) const
{
  WordValue weight;
  transform(feature, id, weight);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(const TDescriptor &feature, 
  WordId &word_id, WordValue &weight, NodeId *nid, int levelsup) const
{ 
  // propagate the feature down the tree
  vector<NodeId> nodes;
  typename vector<NodeId>::const_iterator nit;

  // level at which the node must be stored in nid, if given
  const int nid_level = m_L - levelsup;
  if(nid_level <= 0 && nid != NULL) *nid = 0; // root

  NodeId final_id = 0; // root
  int current_level = 0;

  do
  {
    ++current_level;
    nodes = m_nodes[final_id].children;
    final_id = nodes[0];
 
    double best_d = F::distance(feature, m_nodes[final_id].descriptor);

    for(nit = nodes.begin() + 1; nit != nodes.end(); ++nit)
    {
      NodeId id = *nit;
      double d = F::distance(feature, m_nodes[id].descriptor);
      if(d < best_d)
      {
        best_d = d;
        final_id = id;
      }
    }
    
    if(nid != NULL && current_level == nid_level)
      *nid = final_id;
    
  } while( !m_nodes[final_id].isLeaf() );

  // turn node id into word id
  word_id = m_nodes[final_id].word_id;
  weight = m_nodes[final_id].weight;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
NodeId TemplatedVocabulary<TDescriptor,F>::getParentNode
  (WordId wid, int levelsup) const
{
  NodeId ret = m_words[wid]->id; // node id
  while(levelsup > 0 && ret != 0) // ret == 0 --> root
  {
    --levelsup;
    ret = m_nodes[ret].parent;
  }
  return ret;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::getWordsFromNode
  (NodeId nid, std::vector<WordId> &words) const
{
  words.clear();
  
  if(m_nodes[nid].isLeaf())
  {
    words.push_back(m_nodes[nid].word_id);
  }
  else
  {
    words.reserve(m_k); // ^1, ^2, ...
    
    vector<NodeId> parents;
    parents.push_back(nid);
    
    while(!parents.empty())
    {
      NodeId parentid = parents.back();
      parents.pop_back();
      
      const vector<NodeId> &child_ids = m_nodes[parentid].children;
      vector<NodeId>::const_iterator cit;
      
      for(cit = child_ids.begin(); cit != child_ids.end(); ++cit)
      {
        const Node &child_node = m_nodes[*cit];
        
        if(child_node.isLeaf())
          words.push_back(child_node.word_id);
        else
          parents.push_back(*cit);
        
      } // for each child
    } // while !parents.empty
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
int TemplatedVocabulary<TDescriptor,F>::stopWords(double minWeight)
{
  int c = 0;
  typename vector<Node*>::iterator wit;
  for(wit = m_words.begin(); wit != m_words.end(); ++wit)
  {
    if((*wit)->weight < minWeight)
    {
      ++c;
      (*wit)->weight = 0;
    }
  }
  return c;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
bool TemplatedVocabulary<TDescriptor,F>::loadFromTextFile(const std::string &filename)
{
    ifstream f;
    f.open(filename.c_str());
	
    if(f.eof())
	return false;

    m_words.clear();
    m_nodes.clear();

    string s;
    getline(f,s);
    stringstream ss;
    ss << s;
    ss >> m_k;
    ss >> m_L;
    int n1, n2;
    ss >> n1;
    ss >> n2;

    if(m_k<0 || m_k>20 || m_L<1 || m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
    {
        std::cerr << "Vocabulary loading failure: This is not a correct text file!" << endl;
	return false;
    }
    
    m_scoring = (ScoringType)n1;
    m_weighting = (WeightingType)n2;
    createScoringObject();

    // nodes
    int expected_nodes =
    (int)((pow((double)m_k, (double)m_L + 1) - 1)/(m_k - 1));
    m_nodes.reserve(expected_nodes);

    m_words.reserve(pow((double)m_k, (double)m_L + 1));

    m_nodes.resize(1);
    m_nodes[0].id = 0;
    while(!f.eof())
    {
        string snode;
        getline(f,snode);
        stringstream ssnode;
        ssnode << snode;

        int nid = m_nodes.size();
        m_nodes.resize(m_nodes.size()+1);
	m_nodes[nid].id = nid;
	
        int pid ;
        ssnode >> pid;
        m_nodes[nid].parent = pid;
        m_nodes[pid].children.push_back(nid);

        int nIsLeaf;
        ssnode >> nIsLeaf;

        stringstream ssd;
        for(int iD=0;iD<F::L;iD++)
        {
            string sElement;
            ssnode >> sElement;
            ssd << sElement << " ";
	}
        F::fromString(m_nodes[nid].descriptor, ssd.str());

        ssnode >> m_nodes[nid].weight;

        if(nIsLeaf>0)
        {
            int wid = m_words.size();
            m_words.resize(wid+1);

            m_nodes[nid].word_id = wid;
            m_words[wid] = &m_nodes[nid];
        }
        else
        {
            m_nodes[nid].children.reserve(m_k);
        }
    }

    return true;

}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::saveToTextFile(const std::string &filename) const
{
    fstream f;
    f.open(filename.c_str(),ios_base::out);
    f << m_k << " " << m_L << " " << " " << m_scoring << " " << m_weighting << endl;

    for(size_t i=1; i<m_nodes.size();i++)
    {
        const Node& node = m_nodes[i];

        f << node.parent << " ";
        if(node.isLeaf())
            f << 1 << " ";
        else
            f << 0 << " ";

        f << F::toString(node.descriptor) << " " << (double)node.weight << endl;
    }

    f.close();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::save(const std::string &filename) const
{
  cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
  if(!fs.isOpened()) throw string("Could not open file ") + filename;
  
  save(fs);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::load(const std::string &filename)
{
  cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + filename;
  
  this->load(fs);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::save(cv::FileStorage &f,
  const std::string &name) const
{
  // Format YAML:
  // vocabulary 
  // {
  //   k:
  //   L:
  //   scoringType:
  //   weightingType:
  //   nodes 
  //   [
  //     {
  //       nodeId:
  //       parentId:
  //       weight:
  //       descriptor: 
  //     }
  //   ]
  //   words
  //   [
  //     {
  //       wordId:
  //       nodeId:
  //     }
  //   ]
  // }
  //
  // The root node (index 0) is not included in the node vector
  //
  
  f << name << "{";
  
  f << "k" << m_k;
  f << "L" << m_L;
  f << "scoringType" << m_scoring;
  f << "weightingType" << m_weighting;
  
  // tree
  f << "nodes" << "[";
  vector<NodeId> parents, children;
  vector<NodeId>::const_iterator pit;

  parents.push_back(0); // root

  while(!parents.empty())
  {
    NodeId pid = parents.back();
    parents.pop_back();

    const Node& parent = m_nodes[pid];
    children = parent.children;

    for(pit = children.begin(); pit != children.end(); pit++)
    {
      const Node& child = m_nodes[*pit];

      // save node data
      f << "{:";
      f << "nodeId" << (int)child.id;
      f << "parentId" << (int)pid;
      f << "weight" << (double)child.weight;
      f << "descriptor" << F::toString(child.descriptor);
      f << "}";
      
      // add to parent list
      if(!child.isLeaf())
      {
        parents.push_back(*pit);
      }
    }
  }
  
  f << "]"; // nodes

  // words
  f << "words" << "[";
  
  typename vector<Node*>::const_iterator wit;
  for(wit = m_words.begin(); wit != m_words.end(); wit++)
  {
    WordId id = wit - m_words.begin();
    f << "{:";
    f << "wordId" << (int)id;
    f << "nodeId" << (int)(*wit)->id;
    f << "}";
  }
  
  f << "]"; // words

  f << "}";

}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::load(const cv::FileStorage &fs,
  const std::string &name)
{
  m_words.clear();
  m_nodes.clear();
  
  cv::FileNode fvoc = fs[name];
  
  m_k = (int)fvoc["k"];
  m_L = (int)fvoc["L"];
  m_scoring = (ScoringType)((int)fvoc["scoringType"]);
  m_weighting = (WeightingType)((int)fvoc["weightingType"]);
  
  createScoringObject();

  // nodes
  cv::FileNode fn = fvoc["nodes"];

  m_nodes.resize(fn.size() + 1); // +1 to include root
  m_nodes[0].id = 0;

  for(unsigned int i = 0; i < fn.size(); ++i)
  {
    NodeId nid = (int)fn[i]["nodeId"];
    NodeId pid = (int)fn[i]["parentId"];
    WordValue weight = (WordValue)fn[i]["weight"];
    string d = (string)fn[i]["descriptor"];
    
    m_nodes[nid].id = nid;
    m_nodes[nid].parent = pid;
    m_nodes[nid].weight = weight;
    m_nodes[pid].children.push_back(nid);
    
    F::fromString(m_nodes[nid].descriptor, d);
  }
  
  // words
  fn = fvoc["words"];
  
  m_words.resize(fn.size());

  for(unsigned int i = 0; i < fn.size(); ++i)
  {
    NodeId wid = (int)fn[i]["wordId"];
    NodeId nid = (int)fn[i]["nodeId"];
    
    m_nodes[nid].word_id = wid;
    m_words[wid] = &m_nodes[nid];
  }
}

// --------------------------------------------------------------------------

/**
 * Writes printable information of the vocabulary
 * @param os stream to write to
 * @param voc
 */
template<class TDescriptor, class F>
std::ostream& operator<<(std::ostream &os, 
  const TemplatedVocabulary<TDescriptor,F> &voc)
{
  os << "Vocabulary: k = " << voc.getBranchingFactor() 
    << ", L = " << voc.getDepthLevels()
    << ", Weighting = ";

  switch(voc.getWeightingType())
  {
    case TF_IDF: os << "tf-idf"; break;
    case TF: os << "tf"; break;
    case IDF: os << "idf"; break;
    case BINARY: os << "binary"; break;
  }

  os << ", Scoring = ";
  switch(voc.getScoringType())
  {
    case L1_NORM: os << "L1-norm"; break;
    case L2_NORM: os << "L2-norm"; break;
    case CHI_SQUARE: os << "Chi square distance"; break;
    case KL: os << "KL-divergence"; break;
    case BHATTACHARYYA: os << "Bhattacharyya coefficient"; break;
    case DOT_PRODUCT: os << "Dot product"; break;
  }
  
  os << ", Number of words = " << voc.size();

  return os;
}

} // namespace DBoW2

#endif
