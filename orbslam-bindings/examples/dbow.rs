extern crate dvos3binding;
use dvos3binding::ffi::ORBVocabulary;
use nalgebra::{Isometry3};
use cxx::{let_cxx_string, CxxVector, SharedPtr, UniquePtr};

use opencv::core::{CV_8UC2, CV_8UC1};
use opencv::platform_types::size_t;
use opencv::prelude::*;
use opencv::types::VectorOfKeyPoint;

use std::pin::Pin;


fn main() {
    let_cxx_string!(filename = "/home/sofiya/darvis/vocabulary/ORBvoc.txt");

    let vocabulary = dvos3binding::ffi::load_vocabulary_from_text_file(&filename);
    dvos3binding::ffi::new_feat_vec();

}