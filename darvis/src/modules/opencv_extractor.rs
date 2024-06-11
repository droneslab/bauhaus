use core::{config::SETTINGS, matrix::{DVMatrix, DVVectorOfKeyPoint}, system::Module};
use std::{fmt, fmt::Debug};

use cxx::UniquePtr;
use opencv::features2d::Feature2DTrait;

use crate::registered_actors::{CAMERA, FEATURE_DETECTION};

use super::module::FeatureExtractionModule;


pub struct DVOpenCVExtractor {
    extractor: opencv::core::Ptr<opencv::features2d::ORB>,
    is_ini: bool,
}
impl Module for DVOpenCVExtractor { }
impl FeatureExtractionModule for DVOpenCVExtractor {
    fn extract(&mut self, image: opencv::core::Mat) -> Result<(opencv::types::VectorOfKeyPoint, opencv::core::Mat), Box<dyn std::error::Error>> {
        let mut descriptors = opencv::core::Mat::default();
        let mut keypoints= opencv::types::VectorOfKeyPoint::new();

        self.extractor.detect(& image, &mut  keypoints, & opencv::core::no_array())?;
        self.extractor.compute(& image, &mut keypoints, &mut descriptors)?;

        Ok((keypoints, descriptors))
    }
}

impl DVOpenCVExtractor {
    pub fn new(is_ini: bool) -> Self {
        let max_features = match is_ini {
            true => SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features") * 5,
            false => SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features")
        };
        let fast_threshold = match is_ini {
            true => SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
            false => SETTINGS.get::<i32>(FEATURE_DETECTION, "min_th_fast")
        };
        DVOpenCVExtractor{
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
impl Clone for DVOpenCVExtractor {
    fn clone(&self) -> Self {
        DVOpenCVExtractor::new(self.is_ini)
    }
}
impl Debug for DVOpenCVExtractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVOpenCVExtractor")
         .finish()
    }
}