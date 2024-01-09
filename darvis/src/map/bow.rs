use std::fmt;
use cxx::{UniquePtr, let_cxx_string};
use log::info;
use core::{matrix::DVMatrix, config::{SETTINGS, SYSTEM}};

lazy_static! {
    pub static ref VOCABULARY: DVVocabulary = {
        let filename = SETTINGS.get::<String>(SYSTEM, "vocabulary_file");
        DVVocabulary::load(filename)
    };
}

pub struct DVVocabulary {
    vocabulary: UniquePtr<dvos3binding::ffi::ORBVocabulary>,
    filename: String,
}
impl DVVocabulary {
    pub fn access(&self) {}
    pub fn load(filename: String) -> Self {
        let_cxx_string!(file = filename.clone());
        info!("Loading vocabulary...");

        Self {
            vocabulary: dvos3binding::ffi::load_vocabulary_from_text_file(&file),
            filename,
        }
    }
    pub fn size(&self) -> usize {
        self.vocabulary.size()
    }

    pub fn transform(&self, descriptors: &DVMatrix, bow: & mut BoW) {
        let descriptors2: dvos3binding::ffi::WrapBindCVMat = descriptors.into();

        self.vocabulary.transform(
            & descriptors2,
            bow.bow_vec.pin_mut(),
            bow.feat_vec.pin_mut(),
            4
        );
    }
}


pub struct BoW {
    pub bow_vec: UniquePtr<dvos3binding::ffi::BowVector>, // mBowVec
    pub feat_vec: UniquePtr<dvos3binding::ffi::FeatureVector>, // mFeatVec
}
impl BoW {
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
impl Clone for BoW {
    fn clone(&self) -> BoW {
        self.clone()
    }
}
impl fmt::Debug for BoW {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BoW").finish()
    }
}
impl Default for BoW {
    fn default() -> Self { 
        BoW::new()
    }
}