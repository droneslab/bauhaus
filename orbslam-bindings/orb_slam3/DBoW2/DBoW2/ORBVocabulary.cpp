
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
#include "../../src/CVConvert.h"
#include "../../src/Converter.h"
#include "dvos3binding/src/lib.rs.h"


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

std::size_t ORBVocabulary::size() const {
    vocabulary.size();
}

void ORBVocabulary::transform(
    const orb_slam3::WrapBindCVMat& desc1, 
    DBoW2::BowVector & bow_vector,
    DBoW2::FeatureVector & feature_vector,
    int32_t levelsup
) const {
    vector<cv::Mat> desc2 = orb_slam3::Converter::toDescriptorVector(*desc1.mat_ptr);

    vocabulary.transform(desc2, bow_vector, feature_vector, levelsup);
}
}