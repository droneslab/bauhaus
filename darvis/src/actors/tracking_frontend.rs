extern crate g2o;

use axiom::prelude::*;
use log::{error, warn};
use std::sync::Arc;
use opencv::{prelude::*,features2d::{Feature2DTrait, ORB},types::{PtrOfORB, VectorOfKeyPoint},};
use dvcore::{matrix::*,lockwrap::ReadOnlyWrapper,plugin_functions::Function,global_params::*,};
use crate::{
    registered_modules::{TRACKING_BACKEND},
    actors::{messages::{ImageMsg, FeatureMsg,},},
    dvmap::{map::Map},
};

#[derive(Debug, Clone)]
pub struct DarvisTrackingFront {
    map: ReadOnlyWrapper<Map>,
}

impl DarvisTrackingFront {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisTrackingFront {
        DarvisTrackingFront { map }
    }

    pub fn tracking_frontend(&mut self, context: Context, message: Arc<ImageMsg>) {
        let image = message.frame.grayscale_to_cv_mat();
        let (keypoints, descriptors) = self.extract_features(&image);
        self.send_message_to_backend(context, image, keypoints, descriptors);
    }

    fn extract_features(&mut self, image: &opencv::core::Mat) -> (VectorOfKeyPoint, Mat) {
        let mut keypoints = VectorOfKeyPoint::new();
        let mut descriptors = Mat::default();

        let mut orb: PtrOfORB =  <dyn ORB>::default().unwrap();
        self.set_extractor_settings(&mut orb);
        orb.detect_and_compute(
            image,
            &Mat::default(), 
            &mut keypoints, 
            &mut descriptors, 
            false
        ).unwrap();

        (keypoints, descriptors)
    }

    pub fn send_message_to_backend(
        &mut self, context: Context,
        image: Mat, keypoints: VectorOfKeyPoint, descriptors: Mat
    ) {
        let align_id = context.system.find_aid_by_name(TRACKING_BACKEND).unwrap();
        let new_message = GLOBAL_PARAMS.get::<String>(TRACKING_BACKEND, "actor_message");
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

    fn set_extractor_settings(&mut self, orb: &mut PtrOfORB) {
        let max_features = GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "max_features");
        let scale_factor = GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "scale_factor");
        let n_levels = GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "n_levels");
        let fast_threshold = GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "fast_threshold");

        let res1 = orb.set_max_features(max_features);
        let res2 = orb.set_max_features(max_features);
        let res3 = orb.set_scale_factor(scale_factor);
        let res4 = orb.set_n_levels(n_levels);
        let res5 = orb.set_fast_threshold(fast_threshold);

        if res1.is_err() || res2.is_err() || res3.is_err() || res4.is_err() || res5.is_err() {
            panic!("tracking_frontend::set_extractor_settings;Error setting ORB extractor options from config");
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
