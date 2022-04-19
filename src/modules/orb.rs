use axiom::prelude::*;
use opencv::{
    prelude::*,
    features2d::{Feature2DTrait, ORB},
    types::{PtrOfORB, VectorOfKeyPoint},
};
use darvis::{
    config::*,
    dvutils::*,
    plugin_functions::*
};
use crate::{
    registered_modules::{TRACKER},
    modules::{
        tracker_klt::*,
        messages::{
            frame_msg::FrameMsg,
            tracker_msg::TrackerMsg,
        },
    },
};

#[derive(Debug, Clone)]
pub struct DarvisOrb;

impl DarvisOrb {
    pub fn new() -> DarvisOrb {
        DarvisOrb {}
    }

    // This is the handler that will be used by the actor.
    pub fn orb_extract(&mut self, _context: Context, message: Message) -> ActorResult<()> {
        if let Some(msg) = message.content_as::<FrameMsg>() {
            let mut kp1 = VectorOfKeyPoint::new();
            let mut des1 = Mat::default();
            let img1 = msg.get_frame().grayscale_to_cv_mat();

            let mut orb: PtrOfORB =  <dyn ORB>::default().unwrap();
            set_extractor_settings(&mut orb);
            orb.detect_and_compute(&img1,&Mat::default(), &mut kp1, &mut des1, false).unwrap();

            let align_id = msg.get_actor_ids().get(TRACKER).unwrap();
            let tracker_msg: String = GLOBAL_PARAMS.get(TRACKER.to_string(), "actor_message".to_string());
            match tracker_msg.as_ref() {
                "TrackerMsg" => {
                    align_id.send_new(TrackerMsg::new(
                        kp1.darvis_vector_of_keypoint(),
                        des1.grayscale_mat(),
                        msg.get_actor_ids().clone()
                    )).unwrap();
                },
                "TrackerMsgKLT" => {
                    align_id.send_new(TrackerMsgKLT::new(
                        msg.get_frame().clone(),
                        kp1.darvis_vector_of_keypoint(),
                        des1.grayscale_mat(),
                        msg.get_actor_ids().clone()
                    )).unwrap();
                },
                _ => {
                    println!("Invalid Message type: selecting TrackerMsg");
                    align_id.send_new(TrackerMsg::new(
                        kp1.darvis_vector_of_keypoint(),
                        des1.grayscale_mat(),
                        msg.get_actor_ids().clone()
                    )).unwrap();
                },
            }
        }
        Ok(Status::done(()))
    }
}

fn set_extractor_settings(orb: &mut PtrOfORB) {
    let max_features: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS.to_string(), "max_features".to_string());
    let scale_factor: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS.to_string(), "scale_factor".to_string());
    let n_levels: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS.to_string(), "n_levels".to_string());
    let fast_threshold: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS.to_string(), "fast_threshold".to_string());

    let res1 = orb.set_max_features(max_features);
    let res2 = orb.set_max_features(max_features);
    let res3 = orb.set_scale_factor(scale_factor);
    let res4 = orb.set_n_levels(n_levels);
    let res5 = orb.set_fast_threshold(fast_threshold);

    if res1.is_err() || res2.is_err() || res3.is_err() || res4.is_err() || res5.is_err() {
        println!("Error setting ORB extractor options from config");
    }
}

impl Function for DarvisOrb {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.orb_extract(_context, message).unwrap();
        Ok(Status::done(()))
    }
}
