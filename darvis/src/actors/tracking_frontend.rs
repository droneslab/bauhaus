extern crate g2o;

use axiom::prelude::*;
use cxx::{UniquePtr, CxxVector};
use log::{error, warn, debug};
use std::pin::Pin;
use std::{sync::Arc, fmt};
use std::fmt::Debug;
use opencv::{prelude::*,features2d::{Feature2DTrait, ORB},types::{PtrOfORB, VectorOfKeyPoint},};
use dvcore::{matrix::*,lockwrap::ReadOnlyWrapper,plugin_functions::Function,config::*,};
use crate::{
    registered_modules::{TRACKING_BACKEND, FEATURE_DETECTION, CAMERA},
    actors::{messages::{ImageMsg, FeatureMsg,},},
    dvmap::{map::Map},
};

struct DVORBextractor (UniquePtr<dvos3binding::ffi::ORBextractor>);
impl DVORBextractor {
    pub fn new() -> Self {
        DVORBextractor(
            dvos3binding::ffi::new_orb_extractor(
                GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "max_features"),
                GLOBAL_PARAMS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
                GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "min_th_fast"),
                GLOBAL_PARAMS.get::<i32>(CAMERA, "stereo_overlapping_begin"),
                GLOBAL_PARAMS.get::<i32>(CAMERA, "stereo_overlapping_end")
            )
        )
    }
}
impl Clone for DVORBextractor {
    fn clone(&self) -> Self {
        DVORBextractor::new()
    }
}
impl Debug for DVORBextractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVORBextractor")
         .finish()
    }
}

#[derive(Debug, Clone)]
pub struct DarvisTrackingFront {
    map: ReadOnlyWrapper<Map>,
    orb_extractor: DVORBextractor,
}

impl DarvisTrackingFront {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisTrackingFront {
        DarvisTrackingFront { map, orb_extractor: DVORBextractor::new() }
    }

    pub fn tracking_frontend(&mut self, context: Context, message: Arc<ImageMsg>) {
        let image: Mat = (&message.frame).into(); // Taking ownership of message should not fail
        let (keypoints, descriptors) = self.extract_features(&image);
        self.send_message_to_backend(context, image, keypoints, descriptors);
    }

    fn extract_features(&mut self, image: &opencv::core::Mat) -> (VectorOfKeyPoint, Mat) {
        let mut keypoints = VectorOfKeyPoint::new();
        let mut descriptors = Mat::default();

        unsafe {
            let keypoints_cxx = keypoints.as_raw() as *mut CxxVector<dvos3binding::ffi::DVKeyPoint>;
            let descriptors_cxx = descriptors.as_raw() as *mut dvos3binding::ffi::DVMat;
            let image_dvmat = image.clone().into_raw() as *const dvos3binding::ffi::DVMat;

            self.orb_extractor.0.pin_mut().extract(
                &*image_dvmat, // should be unchanged
                // I think these two are changed:
                Pin::new_unchecked(keypoints_cxx.as_mut().unwrap()),
                Pin::new_unchecked(descriptors_cxx.as_mut().unwrap())
            );
        }

        // If descriptors are cloned into descriptors_cxx and then that object is modified in the C++ code, is descriptors empty here?
        (keypoints, descriptors)
    }

    pub fn send_message_to_backend(
        &mut self, context: Context,
        image: Mat, keypoints: VectorOfKeyPoint, descriptors: Mat
    ) {
        let align_id = context.system.find_aid_by_name(TRACKING_BACKEND).unwrap();
        let new_message = GLOBAL_PARAMS.get::<String>(TRACKING_BACKEND, "actor_message");
        //debug!("Keypoints {:?}", keypoints);
        //debug!("Descriptors {:?}", descriptors);
        match new_message.as_ref() {
            "FeatureMsg" => {
                align_id.send_new(FeatureMsg {
                    keypoints: DVVectorOfKeyPoint::new(keypoints),
                    descriptors: DVMatrix::new(descriptors),
                    image_width: image.cols(),
                    image_height: image.rows(),
                }).unwrap();
            },
            _ => {
                warn!("Invalid Message type: selecting FeatureMsg");
                align_id.send_new(FeatureMsg {
                    keypoints: DVVectorOfKeyPoint::new(keypoints),
                    descriptors: DVMatrix::new(descriptors),
                    image_width: image.cols(),
                    image_height: image.rows(),
                }).unwrap();
            },
        }
    }
}

impl Function for DarvisTrackingFront {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
        if let Some(image_msg) = message.content_as::<ImageMsg>() {
            self.tracking_frontend(context, image_msg);
        }
        Ok(Status::done(()))
    }
}
