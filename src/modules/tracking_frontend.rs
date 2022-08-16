use axiom::prelude::*;
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
            framepose_msg::FramePoseMsg,
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

    pub fn orb_extract(&mut self, _context: Context, message: Message) -> ActorResult<()> {
        let mut kp1 = VectorOfKeyPoint::new();
        let mut des1 = Mat::default();
        let img1 = msg.get_frame().grayscale_to_cv_mat();

        let mut orb: PtrOfORB =  <dyn ORB>::default().unwrap();
        set_extractor_settings(&mut orb);
        orb.detect_and_compute(&img1,&Mat::default(), &mut kp1, &mut des1, false).unwrap();
    }

    pub fn send_message_to_backend(&mut self) {
        let align_id = msg.get_actor_ids().get(TRACKER).unwrap();
        let tracker_msg: String = GLOBAL_PARAMS.get(TRACKER, "actor_message");
        match tracker_msg.as_ref() {
            "FramePoseMsg" => {
                align_id.send_new(FramePoseMsg::new(
                    kp1.darvis_vector_of_keypoint(),
                    des1.grayscale_mat(),
                    img1.cols(),
                    img1.rows(),
                    msg.get_actor_ids().clone()
                )).unwrap();
            },
            // "TrackerMsgKLT" => {
            //     align_id.send_new(TrackerMsgKLT::new(
            //         msg.get_frame().clone(),
            //         kp1.darvis_vector_of_keypoint(),
            //         des1.grayscale_mat(),
            //         msg.get_actor_ids().clone()
            //     )).unwrap();
            // },
            _ => {
                println!("Invalid Message type: selecting FramePoseMsg");
                align_id.send_new(FramePoseMsg::new(
                    kp1.darvis_vector_of_keypoint(),
                    des1.grayscale_mat(),
                    img1.cols(),
                    img1.rows(),
                    msg.get_actor_ids().clone()
                )).unwrap();
            },
        }
    }

}

fn set_extractor_settings(orb: &mut PtrOfORB) {
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

impl Function for DarvisTrackingFront {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
        if let Some(msg) = message.content_as::<ImageMsg>() {
            self.orb_extract(_context, message).unwrap();
            
        
            self.send_message_to_backend(_context, message);
        }
        Ok(Status::done(()))
    }
}
