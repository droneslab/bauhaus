use core::{config::SETTINGS, matrix::{DVMatrix, DVVectorOfKeyPoint}, system::Module};
use std::{fmt, fmt::Debug};

use opencv::{core::Mat, features2d::Feature2DTrait, types::VectorOfPoint2f};

use crate::registered_actors::FEATURE_DETECTION;

use super::module_definitions::FeatureExtractionModule;
use opencv::core::no_array;

pub struct FeatureExtractionSettings {
    pub max_features: i32,
    pub min_distance: f64,
}

pub struct GoodFeaturesExtractor { }
impl Module for GoodFeaturesExtractor { }
impl FeatureExtractionModule for GoodFeaturesExtractor {
    fn extract(&mut self, _image: &Mat) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {
        todo!("Not implemented")
    }

    fn extract_amount(&mut self, image: & Mat, max_features: i32, min_distance: f64) -> Result<(VectorOfPoint2f), Box<dyn std::error::Error>> {

        let mut descriptors = opencv::core::Mat::default();
        let mut corners = opencv::types::VectorOfPoint2f::new();

        opencv::imgproc::good_features_to_track(
            image, &mut corners, max_features, 0.01, min_distance, & no_array(), 3, false, 0.00
        ).unwrap();
        // cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);


        Ok(corners)
    }

    fn extract_with_existing_points(&mut self, image : &Mat, points : &VectorOfPoint2f) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {
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