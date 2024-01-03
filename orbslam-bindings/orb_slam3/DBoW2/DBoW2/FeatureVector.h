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

#include "BowVector.h"
#include <map>
#include <vector>
#include <iostream>
#include <memory>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#include "../../../target/cxxbridge/rust/cxx.h"

namespace DBoW2 {


/// Vector of nodes with indexes of local features
class FeatureVector: 
  public std::map<NodeId, std::vector<unsigned int> >
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const int version)
    {
        ar & boost::serialization::base_object<std::map<NodeId, std::vector<unsigned int> > >(*this);
    }

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

    // For rust bindings...
    std::unique_ptr<FeatureVector> clone() const; 
    rust::Vec<uint32_t> get_all_nodes() const;

    // So we can iterate through each node's feature vector 
    // (the std::vector in this map... std::map<NodeId, std::vector<unsigned int> >)
    // without copying the entire vector into rust
    rust::Vec<uint32_t> get_feat_from_node(unsigned int node_id) const;
    uint32_t vec_size(unsigned int node_id) const;
    uint32_t vec_get(unsigned int node_id, uint32_t index) const;

};

std::unique_ptr<FeatureVector> new_feat_vec();

} // namespace DBoW2

#endif