use core::{config::SETTINGS, matrix::{DVMatrix, DVVectorOfKeyPoint}};
use std::{fmt, fmt::Debug};

use cxx::UniquePtr;

use crate::registered_actors::{CAMERA, FEATURE_DETECTION};

use super::module::FeatureExtractionModule;


pub struct DVORBextractor {
    extractor: UniquePtr<dvos3binding::ffi::ORBextractor>,
    pub max_features: i32
}
impl FeatureExtractionModule for DVORBextractor {
    type Image = opencv::core::Mat;

    fn extract(&mut self, image: Self::Image) -> Result<(opencv::types::VectorOfKeyPoint, opencv::core::Mat), Box<dyn std::error::Error>> {
        let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image)).into();
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();

        self.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);

        Ok((keypoints.kp_ptr.kp_ptr, descriptors.mat_ptr.mat_ptr))
    }
}

impl DVORBextractor {
    pub fn new(max_features: i32) -> Self {
        DVORBextractor{
            max_features,
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
impl Clone for DVORBextractor {
    fn clone(&self) -> Self {
        DVORBextractor::new(self.max_features)
    }
}
impl Debug for DVORBextractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVORBextractor")
         .finish()
    }
}