
// --------------------------------------------------------------------------
// For rust bindings...
// Needs to be separate struct encompassing a TemplatedVocabulary because
// I don't think the rust bindings library allows us to use templates.
#include <cassert>

#include <vector>
#include <numeric>
#include <fstream>
#include <string>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <limits>


#include"FORB.h"

#include "ORBVocabulary.h"
#include "../../src/DVConvert.h"
#include "../../src/Converter.h"


struct DVKeyPoint;

namespace DBoW2 {

ORBVocabulary::ORBVocabulary() {}

std::unique_ptr<ORBVocabulary> load_vocabulary_from_text_file(const string &file) {
    std::unique_ptr<ORBVocabulary> vocabulary = std::make_unique<ORBVocabulary>();
    bool bVocLoad = vocabulary->vocabulary.loadFromTextFile(file);

    if(!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << file << endl;
        exit(-1);
    }
    return vocabulary;
}

void ORBVocabulary::transform(
    const orb_slam3::DVMat& desc1, 
    DBoW2::BowVector & bow_vector,
    DBoW2::FeatureVector & feature_vector,
    int32_t levelsup
) const {
    cv::Mat desc2 = orb_slam3::get_descriptor_const(desc1);
    vector<cv::Mat> desc3 = orb_slam3::Converter::toDescriptorVector(desc2);

    vocabulary.transform(desc3, bow_vector, feature_vector, levelsup);
}
}