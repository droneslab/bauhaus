use std::fmt;
use cxx::{UniquePtr, let_cxx_string};
use log::info;
use core::{matrix::DVMatrix,};
use crate::modules::module::VocabularyModule;

use crate::map::keyframe::KeyFrame;


pub struct DVVocabulary {
    vocabulary: UniquePtr<dvos3binding::ffi::ORBVocabulary>,
    filename: String,
}
impl VocabularyModule for DVVocabulary {
    type BoWModule = DVBoW;
    type Descriptors = DVMatrix;
    type ItemToScore = KeyFrame;

    fn access(&self) {}
    fn load(filename: String) -> Self {
        let_cxx_string!(file = filename.clone());
        info!("Loading vocabulary...");

        let vocab = Self {
            vocabulary: dvos3binding::ffi::load_vocabulary_from_text_file(&file),
            filename,
        };

        return vocab;
    }
    fn size(&self) -> usize {
        self.vocabulary.size()
    }

    fn transform(&self, descriptors: &DVMatrix, bow: & mut DVBoW) {
        let descriptors2: dvos3binding::ffi::WrapBindCVMat = descriptors.into();

        self.vocabulary.transform(
            & descriptors2,
            bow.bow_vec.pin_mut(),
            bow.feat_vec.pin_mut(),
            4
        );
    }

    fn score(&self, kf1: &KeyFrame, kf2: &KeyFrame) -> f32 {
        self.vocabulary.score(&kf1.bow.as_ref().unwrap().bow_vec, &kf2.bow.as_ref().unwrap().bow_vec)
    }
}


pub struct DVBoW {
    pub bow_vec: UniquePtr<dvos3binding::ffi::BowVector>, // mBowVec
    pub feat_vec: UniquePtr<dvos3binding::ffi::FeatureVector>, // mFeatVec
}
impl DVBoW {
    pub fn new() -> Self {
        Self {
            bow_vec: dvos3binding::ffi::new_bow_vec(),
            feat_vec: dvos3binding::ffi::new_feat_vec(),
        }
    }
    pub fn clone(&self) -> Self {
        Self {
            bow_vec: self.bow_vec.clone(),
            feat_vec: self.feat_vec.clone(),
        }
    }
}


// Note: need to implement functions that would typically be derived
// because we can't derive them for a UniquePtr
impl Clone for DVVocabulary {
    fn clone(&self) -> DVVocabulary {
        DVVocabulary::load(self.filename.clone())
    }
}
impl fmt::Debug for DVVocabulary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVVocabulary")
         .field("filename", &self.filename)
         .finish()
    }
}
impl Clone for DVBoW {
    fn clone(&self) -> DVBoW {
        self.clone()
    }
}
impl fmt::Debug for DVBoW {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BoW").finish()
    }
}
impl Default for DVBoW {
    fn default() -> Self { 
        DVBoW::new()
    }
}