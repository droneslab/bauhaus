use core::{
    config::SETTINGS,
    matrix::{BHMatrix, BHVectorOfKeyPoint},
    system::Module,
};
use std::{fmt, fmt::Debug};

use cxx::UniquePtr;
use opencv::core::Mat;

use crate::registered_actors::{CAMERA, FEATURE_DETECTION};

use super::module_definitions::FeatureExtractionModule;

pub struct ORBExtractor {
    extractor: UniquePtr<dvos3binding::ffi::ORBextractor>,
    is_ini: bool,
}
impl Module for ORBExtractor {}
impl FeatureExtractionModule for ORBExtractor {
    fn extract(
        &mut self,
        image: &Mat,
        mask: Option<Mat>,
    ) -> Result<(BHVectorOfKeyPoint, BHMatrix), Box<dyn std::error::Error>> {
        let image_dv: dvos3binding::ffi::WrapBindCVMat = (&BHMatrix::new(image.clone())).into();
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&BHMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints =
            BHVectorOfKeyPoint::empty().into();
        let mask: dvos3binding::ffi::WrapBindCVMat = match mask {
            Some(m) => (&BHMatrix::new(m)).into(),
            None => (&BHMatrix::default()).into(),
        };

        let _num_extracted =
            self.extractor
                .pin_mut()
                .extract(&image_dv, &mut keypoints, &mut descriptors, &mask);
        Ok((
            BHVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr),
            BHMatrix::new(descriptors.mat_ptr.mat_ptr),
        ))
    }

    // fn extract_with_existing_points(
    //     &mut self,
    //     image: &Mat,
    //     points: &VectorOfPoint2f,
    // ) -> Result<(BHVectorOfKeyPoint, BHMatrix), Box<dyn std::error::Error>> {
    //     let image_dv: dvos3binding::ffi::WrapBindCVMat = (&BHMatrix::new(image.clone())).into();
    //     let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&BHMatrix::default()).into();
    //     let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints =
    //         BHVectorOfKeyPoint::empty().into();
    //     let points: dvos3binding::ffi::BindCVVectorOfPoint2f =
    //         dvos3binding::ffi::BindCVVectorOfPoint2f {
    //             vec_ptr: points.clone(),
    //         };

    //     let _num_extracted = self.extractor.pin_mut().extract_with_existing_points(
    //         &image_dv,
    //         &points,
    //         &mut keypoints,
    //         &mut descriptors,
    //     );

    //     Ok((
    //         BHVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr),
    //         BHMatrix::new(descriptors.mat_ptr.mat_ptr),
    //     ))
    // }
}

impl ORBExtractor {
    pub fn new(is_ini: bool) -> Self {
        let max_features = match is_ini {
            true => SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_features"),
            false => SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features"),
        };
        ORBExtractor {
            is_ini,
            extractor: dvos3binding::ffi::new_orb_extractor(
                max_features,
                SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "min_th_fast"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_begin"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_end"),
            ),
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
        f.debug_struct("DVORBextractor").finish()
    }
}
