use core::{config::SETTINGS, matrix::{DVMatrix, DVVectorOfKeyPoint}, system::Module};
use std::{fmt, fmt::Debug};

use opencv::{core::Mat, features2d::Feature2DTrait};

use crate::registered_actors::FEATURE_DETECTION;

use super::module_definitions::FeatureExtractionModule;


pub struct OpenCVExtractor {
    extractor: opencv::core::Ptr<opencv::features2d::ORB>,
    is_ini: bool,
}
impl Module for OpenCVExtractor { }
impl FeatureExtractionModule for OpenCVExtractor {
    fn extract(&mut self, image: & Mat) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {
        let mut descriptors = opencv::core::Mat::default();
        let mut keypoints= opencv::core::Vector::<opencv::core::KeyPoint>::new();

        self.extractor.detect(& *image, &mut keypoints, & opencv::core::no_array())?;
        self.extractor.compute(& *image, &mut keypoints, &mut descriptors)?;

        Ok((DVVectorOfKeyPoint::new(keypoints), DVMatrix::new(descriptors)))
    }

    fn extract_amount(&mut self, _image: & Mat, _max_features: i32, _min_distance: f64) -> Result<opencv::core::Vector<opencv::core::Point2f>, Box<dyn std::error::Error>> {
        todo!("Not implemented")
    }

    fn extract_with_existing_points(&mut self, _image : &Mat, _points : &opencv::core::Vector<opencv::core::Point2f>) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {
        todo!("Not implemented!")
    }

}

impl OpenCVExtractor {
    pub fn new(is_ini: bool) -> Self {
        let max_features = match is_ini {
            true => SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features") * 5,
            false => SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features")
        };
        let fast_threshold = match is_ini {
            true => SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
            false => SETTINGS.get::<i32>(FEATURE_DETECTION, "min_th_fast")
        };
        OpenCVExtractor{
            is_ini,
            extractor: opencv::features2d::ORB::create(
                max_features,
                SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                31,
                0,
                2,
                opencv::features2d::ORB_ScoreType::HARRIS_SCORE,
                31,
                fast_threshold
            ).unwrap()
        }
    }
}
impl Clone for OpenCVExtractor {
    fn clone(&self) -> Self {
        OpenCVExtractor::new(self.is_ini)
    }
}
impl Debug for OpenCVExtractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVOpenCVExtractor")
         .finish()
    }
}