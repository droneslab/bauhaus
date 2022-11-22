use std::fmt;
use cxx::{UniquePtr, let_cxx_string};
use dvcore::{matrix::DVMatrix, config::{GLOBAL_PARAMS, SYSTEM_SETTINGS}};
use log::info;
use opencv::prelude::Boxed;

lazy_static! {
    pub static ref VOCABULARY: DVVocabulary = {
        let filename = GLOBAL_PARAMS.get::<String>(SYSTEM_SETTINGS, "vocabulary_file");
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
        info!("Loading vocabulary");

        Self {
            vocabulary: dvos3binding::ffi::load_vocabulary_from_text_file(&file),
            filename,
        }
    }
    pub fn transform(&self, descriptors: &DVMatrix, bow: & mut BoW) {
        unsafe {
            let descriptors = descriptors.mat().clone().into_raw() as *const dvos3binding::ffi::DVMat;

            self.vocabulary.transform(
                &*descriptors,
                bow.bow_vec.pin_mut(),
                bow.feat_vec.pin_mut(),
                4
            );
            bow.is_empty = false;
        }

    }
}


pub struct BoW {
    // BoW
    bow_vec: UniquePtr<dvos3binding::ffi::BowVector>, // mBowVec
    feat_vec: UniquePtr<dvos3binding::ffi::FeatureVector>, // mFeatVec
    pub is_empty: bool
}
impl BoW {
    pub fn new() -> Self {
        Self {
            bow_vec: dvos3binding::ffi::new_bow_vec(),
            feat_vec: dvos3binding::ffi::new_feat_vec(),
            is_empty: true
        }
    }
    pub fn clone(&self) -> Self {
        // Sofiya: I am not sure this is thread-safe or that it works...
        Self {
            bow_vec: self.bow_vec.clone(),
            feat_vec: self.feat_vec.clone(),
            is_empty: self.is_empty
        }
    }
    pub fn get_feat_vec_nodes(&self) -> Vec<u32> {
        self.feat_vec.get_all_nodes()
    }
    pub fn get_feat_from_node(&self, node: u32) -> Vec<u32> {
        self.feat_vec.get_feat_from_node(node)
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