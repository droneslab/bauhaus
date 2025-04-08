use core::{matrix::{DVMatrix, DVVectorOfKeyPoint}, system::Module};
use std::{fmt, fmt::Debug};

use opencv::{core::Mat, types::VectorOfPoint2f};


use super::module_definitions::FeatureExtractionModule;
use opencv::core::no_array;


pub struct GoodFeaturesExtractor { }
impl Module for GoodFeaturesExtractor { }
impl FeatureExtractionModule for GoodFeaturesExtractor {
    fn extract(&mut self, _image: &Mat) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {
        todo!("Not implemented")
    }

    fn extract_amount(&mut self, image: & Mat, max_features: i32, min_distance: f64) -> Result<VectorOfPoint2f, Box<dyn std::error::Error>> {
        let mut corners = opencv::types::VectorOfPoint2f::new();

        opencv::imgproc::good_features_to_track(
            image, &mut corners, max_features, 0.01, min_distance, & no_array(), 3, false, 0.00
        ).unwrap();

        Ok(corners)
    }

    fn extract_with_existing_points(&mut self, _image : &Mat, _points : &VectorOfPoint2f) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {
        todo !("Not implemented!")
    }
}

impl GoodFeaturesExtractor {
    pub fn new() -> Self {
        GoodFeaturesExtractor{}
    }
}
impl Clone for GoodFeaturesExtractor {
    fn clone(&self) -> Self {
        GoodFeaturesExtractor::new()
    }
}
impl Debug for GoodFeaturesExtractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVGoodFeaturesExtractor")
         .finish()
    }
}