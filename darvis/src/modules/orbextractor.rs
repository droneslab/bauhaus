use core::config::SETTINGS;
use std::{fmt, fmt::Debug};

use cxx::UniquePtr;

use crate::registered_actors::{CAMERA, FEATURE_DETECTION};


pub struct DVORBextractor {
    pub extractor: UniquePtr<dvos3binding::ffi::ORBextractor>,
    pub max_features: i32
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