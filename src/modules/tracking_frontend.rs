use axiom::prelude::*;
use std::sync::Arc;
use opencv::{
    prelude::*,
    features2d::{Feature2DTrait, ORB},
    types::{PtrOfORB, VectorOfKeyPoint},
};
use darvis::{
    dvutils::*,
    map::{
        pose::Pose, map::Map, map_actor::MapWriteMsg, map_actor::MAP_ACTOR,
        keyframe::KeyFrame, frame::Frame, map::Id, misc::IMUBias, mappoint::MapPoint, orbmatcher::ORBmatcher, camera::DVCamera
    },
    lockwrap::ReadOnlyWrapper,
    plugin_functions::Function,
    global_params::*,
};
use crate::{
    registered_modules::{TRACKER},
    modules::{
        tracker_klt::*,
        messages::{
            image_msg::ImageMsg,
            feature_msg::FeatureMsg,
        },
    },
};

#[derive(Debug, Clone)]
pub struct DarvisTrackingFront {
    map: ReadOnlyWrapper<Map>
}

impl DarvisTrackingFront {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisTrackingFront {
        DarvisTrackingFront {
            map: map
        }
    }

    pub fn tracking_frontend(&mut self, _context: Context, message: Arc<ImageMsg>) {
        // Extract features
        let image = message.get_frame().grayscale_to_cv_mat();
        let (keypoints, descriptors) = self.extract_features(&image);

        self.send_message_to_backend(
            message, image, keypoints, descriptors
        );
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
        &mut self, message: Arc<ImageMsg>, 
        image: Mat, keypoints: VectorOfKeyPoint, descriptors: Mat
    ) {
        let align_id = message.get_actor_ids().get(TRACKER).unwrap();
        let new_message: String = GLOBAL_PARAMS.get(TRACKER, "actor_message");
        match new_message.as_ref() {
            "FeatureMsg" => {
                align_id.send_new(FeatureMsg::new(
                    keypoints.darvis_vector_of_keypoint(),
                    descriptors.grayscale_mat(),
                    image.cols(),
                    image.rows(),
                    message.get_actor_ids().clone()
                )).unwrap();
            },
            _ => {
                println!("Invalid Message type: selecting FeatureMsg");
                align_id.send_new(FeatureMsg::new(
                    keypoints.darvis_vector_of_keypoint(),
                    descriptors.grayscale_mat(),
                    image.cols(),
                    image.rows(),
                    message.get_actor_ids().clone()
                )).unwrap();
            },
        }
    }

    fn set_extractor_settings(&mut self, orb: &mut PtrOfORB) {
        let max_features: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "max_features");
        let scale_factor: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "scale_factor");
        let n_levels: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "n_levels");
        let fast_threshold: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "fast_threshold");

        let res1 = orb.set_max_features(max_features);
        let res2 = orb.set_max_features(max_features);
        let res3 = orb.set_scale_factor(scale_factor);
        let res4 = orb.set_n_levels(n_levels);
        let res5 = orb.set_fast_threshold(fast_threshold);

        if res1.is_err() || res2.is_err() || res3.is_err() || res4.is_err() || res5.is_err() {
            println!("Error setting ORB extractor options from config");
        }
    }
}

impl Function for DarvisTrackingFront {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
        if let Some(image_msg) = message.content_as::<ImageMsg>() {
            self.tracking_frontend(_context, image_msg);
        }
        Ok(Status::done(()))
    }
}
