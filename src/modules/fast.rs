use axiom::prelude::*;
use opencv::{
    prelude::*,
    features2d::{Feature2DTrait},
    types::{PtrOfBRISK, VectorOfKeyPoint},
};
use darvis::{
    global_params::*,
    dvutils::*,
    plugin_functions::Function
};
use crate::{
    modules::{
        messages::{
            feature_msg::FeatureMsg, image_msg::ImageMsg
        },
    },
    registered_modules::TRACKER
};

#[derive(Debug, Clone)]
pub struct DarvisFast;

impl DarvisFast {
    pub fn new() -> DarvisFast {
        DarvisFast {}
    }

    pub fn fast_extract(&mut self, _context: Context, message: Message) -> ActorResult<()> {
        if let Some(msg) = message.content_as::<ImageMsg>() {
            let mut kp1 = VectorOfKeyPoint::new();
            let mut des1 = Mat::default();
            let img1 = msg.get_frame().grayscale_to_cv_mat();

            let fast_threshold: i32 = 20;
            let non_max_suppression: bool = true;

            opencv::features2d::FAST(&img1, &mut kp1, fast_threshold, non_max_suppression).unwrap();

            let mut brisk : PtrOfBRISK = opencv::features2d::BRISK::create(30, 3, 1.0)?;
            brisk.compute(&img1, &mut kp1, &mut des1).unwrap();

            let align_id = msg.get_actor_ids().get(TRACKER).unwrap();
            let traker_msg: String = GLOBAL_PARAMS.get(TRACKER, "actor_message");

            match traker_msg.as_ref()
            {
                "FeatureMsg" => {
                    align_id.send_new(FeatureMsg::new(
                        DVVectorOfKeyPoint::new(kp1),
                        DVMatrix::new(des1),
                        img1.cols(),
                        img1.rows(),
                        msg.get_actor_ids().clone()
                    )).unwrap();
                },
                _ => {
                    println!("Invalid Message type: selecting FeatureMsg");
                    align_id.send_new(FeatureMsg::new(
                        DVVectorOfKeyPoint::new(kp1),
                        DVMatrix::new(des1),
                        img1.cols(),
                        img1.rows(),
                        msg.get_actor_ids().clone()
                    )).unwrap();
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
