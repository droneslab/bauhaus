
#include "FeatureVector.h"
#include "BowVector.h"
#include "ScoringObject.h"
#include "TemplatedVocabulary.h"

namespace DBoW2 {

class ORBVocabulary {
    public:
        TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> vocabulary;

        ORBVocabulary();
        void transform(
            const orb_slam3::DVMat& desc1, 
            DBoW2::BowVector & bow_vector,
            DBoW2::FeatureVector & feature_vector,
            int levelsup
        ) const;
};

std::unique_ptr<ORBVocabulary> load_vocabulary_from_text_file(const string &file);
}