use axiom::prelude::*;
use log::warn;
use opencv::{
    prelude::*,
    features2d::{Feature2DTrait},
    types::{PtrOfBRISK, VectorOfKeyPoint},
};
use dvcore::{config::*, matrix::*, plugin_functions::Function};
use crate::{
    actors::messages::{FeatureMsg, ImageMsg},
    registered_modules::TRACKER
};

#[derive(Debug, Clone)]
pub struct DarvisFast;

impl DarvisFast {
    pub fn new() -> DarvisFast {
        DarvisFast {}
    }

    pub fn fast_extract(&mut self, context: Context, message: Message) -> ActorResult<()> {
        if let Some(msg) = message.content_as::<ImageMsg>() {
            let mut kp1 = VectorOfKeyPoint::new();
            let mut des1 = Mat::default();
            let img1: Mat = (&msg.frame).into();

            let fast_threshold: i32 = 20;
            let non_max_suppression: bool = true;

            opencv::features2d::FAST(&img1, &mut kp1, fast_threshold, non_max_suppression).unwrap();

            let mut brisk : PtrOfBRISK = opencv::features2d::BRISK::create(30, 3, 1.0)?;
            brisk.compute(&img1, &mut kp1, &mut des1).unwrap();

            let align_id = context.system.find_aid_by_name(TRACKER).unwrap();
            let traker_msg = GLOBAL_PARAMS.get::<String>(TRACKER, "actor_message");

            match traker_msg.as_ref()
            {
                "FeatureMsg" => {
                    align_id.send_new(FeatureMsg {
                        keypoints: DVVectorOfKeyPoint::new(kp1),
                        descriptors: DVMatrix::new(des1),
                        image_width: img1.cols(),
                        image_height: img1.rows(),
                    }).unwrap();
                },
                _ => {
                    warn!("Invalid Message type: selecting FeatureMsg");
                    align_id.send_new(FeatureMsg {
                        keypoints: DVVectorOfKeyPoint::new(kp1),
                        descriptors: DVMatrix::new(des1),
                        image_width: img1.cols(),
                        image_height: img1.rows(),
                    }).unwrap();
                },
            }
        }
        Ok(Status::done(()))
    }
}

impl Function for DarvisFast {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
        self.fast_extract(_context, message).unwrap();
        Ok(Status::done(()))
    }
}
