use std::fmt;

use cxx::{UniquePtr, let_cxx_string};
use dvcore::{matrix::DVMatrix, global_params::{GLOBAL_PARAMS, SYSTEM_SETTINGS}};
use log::info;
use opencv::prelude::Boxed;

pub struct DVVocabulary {
    vocabulary: UniquePtr<dvos3binding::ffi::ORBVocabulary>,
    filename: String,
}
impl DVVocabulary {
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
        }

    }
    // pub fn transform_with_direct_idx(&self, features: &DVMatrix) -> (abow::BoW, abow::DirectIdx) {
    //     let mut v_desc = Vec::new();
    //     let desc_vec = features.mat().to_vec_2d::<u8>().unwrap();

    //     for j in 0..features.mat().rows() {
    //         let desc_j = desc_vec.get(j as usize).unwrap();
    //         let mut desc_val : Desc = [0; 32];
    //         for (&x, p) in desc_j.iter().zip(desc_val.iter_mut()) {
    //             *p = x;
    //         }
    //         v_desc.push(desc_val);

    //     }

    //     self.vocabulary.transform_with_direct_idx(&v_desc).unwrap()
    // }
    
}


pub struct BoW {
    // BoW
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
        // Sofiya: I am not sure this is thread-safe or that it works...
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
impl Default for DVVocabulary {
    fn default() -> Self { 
        let filename = GLOBAL_PARAMS.get::<String>(SYSTEM_SETTINGS, "vocabulary_file");
        let_cxx_string!(file = filename.clone());

        Self {
            vocabulary: dvos3binding::ffi::load_vocabulary_from_text_file(&file),
            filename,
        }
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