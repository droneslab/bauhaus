use core::{config::SETTINGS, matrix::{DVMatrix, DVVectorOfKeyPoint}};
use std::{fmt, fmt::Debug};

use cxx::UniquePtr;
use opencv::features2d::Feature2DTrait;

use crate::registered_actors::{CAMERA, FEATURE_DETECTION};

use super::module::FeatureExtractionModule;


pub struct DVOpenCVExtractor {
    extractor: opencv::core::Ptr<opencv::features2d::ORB>,
    pub max_features: i32
}
impl FeatureExtractionModule for DVOpenCVExtractor {
    type Image = opencv::core::Mat;

    fn extract(&mut self, image: Self::Image) -> Result<(opencv::types::VectorOfKeyPoint, opencv::core::Mat), Box<dyn std::error::Error>> {
        let mut descriptors = opencv::core::Mat::default();
        let mut keypoints= opencv::types::VectorOfKeyPoint::new();

        self.extractor.detect(& image, &mut  keypoints, & opencv::core::no_array())?;
        self.extractor.compute(& image, &mut keypoints, &mut descriptors)?;

        Ok((keypoints, descriptors))
    }
}

impl DVOpenCVExtractor {
    pub fn new(max_features: i32) -> Self {
        DVOpenCVExtractor{
            max_features,
            extractor: opencv::features2d::ORB::create(
                2000,
                SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                31,
                0,
                2,
                opencv::features2d::ORB_ScoreType::HARRIS_SCORE,
                31,
                20
            ).unwrap()
        }
    }
}
impl Clone for DVOpenCVExtractor {
    fn clone(&self) -> Self {
        DVOpenCVExtractor::new(self.max_features)
    }
}
impl Debug for DVOpenCVExtractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVOpenCVExtractor")
         .finish()
    }
}