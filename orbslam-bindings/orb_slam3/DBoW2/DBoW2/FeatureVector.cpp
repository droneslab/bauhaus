/**
 * File: FeatureVector.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#include "FeatureVector.h"
#include <map>
#include <vector>
#include <iostream>

namespace DBoW2 {

// ---------------------------------------------------------------------------

FeatureVector::FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

FeatureVector::~FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

void FeatureVector::addFeature(NodeId id, unsigned int i_feature)
{
  FeatureVector::iterator vit = this->lower_bound(id);
  
  if(vit != this->end() && vit->first == id)
  {
    vit->second.push_back(i_feature);
  }
  else
  {
    vit = this->insert(vit, FeatureVector::value_type(id, 
      std::vector<unsigned int>() ));
    vit->second.push_back(i_feature);
  }
}

// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &out, 
  const FeatureVector &v)
{
  if(!v.empty())
  {
    FeatureVector::const_iterator vit = v.begin();
    
    const std::vector<unsigned int>* f = &vit->second;

    out << "<" << vit->first << ": [";
    if(!f->empty()) out << (*f)[0];
    for(unsigned int i = 1; i < f->size(); ++i)
    {
      out << ", " << (*f)[i];
    }
    out << "]>";
    
    for(++vit; vit != v.end(); ++vit)
    {
      f = &vit->second;
      
      out << ", <" << vit->first << ": [";
      if(!f->empty()) out << (*f)[0];
      for(unsigned int i = 1; i < f->size(); ++i)
      {
        out << ", " << (*f)[i];
      }
      out << "]>";
    }
  }
  
  return out;  
}

// --------------------------------------------------------------------------
// For rust bindings...

std::unique_ptr<FeatureVector> FeatureVector::clone() const {
    std::unique_ptr<FeatureVector> vec = std::make_unique<FeatureVector>();
    *vec = *this;
    return vec;
}

rust::Vec<uint32_t> FeatureVector::get_all_nodes() const {
    rust::Vec<uint32_t> keys;
    for(auto it = this->begin(); it != this->end(); ++it) {
        keys.push_back(it->first);
    }
    return keys;
}

rust::Vec<uint32_t> FeatureVector::get_feat_from_node(unsigned int node_id) const {
    auto features = this->find(node_id);
    rust::Vec<uint32_t> features_copy;
    if (features != this->end()) {
        for(auto it = features->second.begin(); it != features->second.end(); ++it) {
            features_copy.push_back(*it);
        }
    }
    return features_copy;
}

std::unique_ptr<FeatureVector> new_feat_vec() {
    std::unique_ptr<FeatureVector> vec = std::make_unique<FeatureVector>();
    return vec;
}

} // namespace DBoW2

