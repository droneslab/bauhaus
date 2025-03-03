use core::{config::SETTINGS, matrix::{mat_to_wrapbindcvmat, DVMatrix, DVVectorOfKeyPoint}, system::Module};
use std::{fmt, fmt::Debug};

use cxx::UniquePtr;
use opencv::{core::Mat, types::VectorOfPoint2f};

use crate::registered_actors::{CAMERA, FEATURE_DETECTION};

use super::module_definitions::FeatureExtractionModule;


pub struct ORBExtractor {
    extractor: UniquePtr<dvos3binding::ffi::ORBextractor>,
    is_ini: bool
}
impl Module for ORBExtractor { }
impl FeatureExtractionModule for ORBExtractor {
    fn extract(&mut self, image: & Mat) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {
        let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image.clone())).into();
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();

        let num_extracted = self.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);

        println !("Extracted {}", num_extracted);

        Ok((DVVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr), DVMatrix::new(descriptors.mat_ptr.mat_ptr)))
    }

    fn extract_amount(&mut self, image: & Mat, max_features: i32, min_distance: f64) -> Result<(VectorOfPoint2f), Box<dyn std::error::Error>> {
        todo!("Not implemented")
    }
}

impl ORBExtractor {
    pub fn new(is_ini: bool) -> Self {
        let max_features = match is_ini {
            true => {
                // sofiya orbslam2 loop closing
                SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features") * 5
            },
            false => SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features")
        };
        ORBExtractor{
            is_ini,
            extractor: dvos3binding::ffi::new_orb_extractor(
                max_features,
                SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "min_th_fast"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_begin"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_end")
            )
        }
    }
}
impl Clone for ORBExtractor {
    fn clone(&self) -> Self {
        ORBExtractor::new(self.is_ini)
    }
}
impl Debug for ORBExtractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVORBextractor")
         .finish()
    }
}