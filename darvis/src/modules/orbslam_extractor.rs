use core::{config::SETTINGS, matrix::{DVMatrix, DVVectorOfKeyPoint}, system::Module};
use std::{fmt, fmt::Debug};

use cxx::UniquePtr;
use opencv::{core::Mat};

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

        let _num_extracted = self.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);

        Ok((DVVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr), DVMatrix::new(descriptors.mat_ptr.mat_ptr)))
    }

    fn extract_amount(&mut self, _image : &Mat, _max_features : i32, _min_distance : f64) -> Result<opencv::core::Vector<opencv::core::Point2f>, Box<dyn std::error::Error>>{
        todo !("Not implemented")}

    fn extract_with_existing_points(&mut self, image : &Mat, points : &opencv::core::Vector<opencv::core::Point2f>) -> Result<(DVVectorOfKeyPoint, DVMatrix), Box<dyn std::error::Error>> {

        let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image.clone())).into();
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();
        let points : dvos3binding::ffi::BindCVVectorOfPoint2f = dvos3binding::ffi::BindCVVectorOfPoint2f{
            vec_ptr : points.clone()
        };

        let _num_extracted = self.extractor.pin_mut().extract_with_existing_points(&image_dv, &points, &mut keypoints, &mut descriptors);

        Ok((DVVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr), DVMatrix::new(descriptors.mat_ptr.mat_ptr)))
    }
}

impl ORBExtractor {
    pub fn new(is_ini: bool) -> Self {
        let max_features = match is_ini {
            true => {
                SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_features")
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