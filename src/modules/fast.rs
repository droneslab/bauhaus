use axiom::prelude::*;

use opencv::{
    prelude::*,
    features2d::{Feature2DTrait},
    types::{PtrOfBRISK, VectorOfKeyPoint},
};

use darvis::base::FrameMsg;
use darvis::config::*;
use darvis::dvutils::*;
use darvis::plugin_functions::Function;

use crate::modules::tracker::*;
use crate::modules::tracker_klt::*;
use crate::registered_modules::TRACKER;


#[derive(Debug, Clone)]
pub struct DarvisFast;

impl DarvisFast
{
    pub fn new() -> DarvisFast {
        DarvisFast {}
    }

// This is the handler that will be used by the actor.
pub fn fast_extract(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<FrameMsg>() {
        let mut kp1 = VectorOfKeyPoint::new();
        let mut des1 = Mat::default();
        let img1 = msg.get_frame().grayscale_to_cv_mat();

        let fast_threshold: i32 = 20;
        let non_max_suppression: bool = true;

        opencv::features2d::FAST(&img1, &mut kp1, fast_threshold, non_max_suppression).unwrap();
            
        let mut brisk : PtrOfBRISK = opencv::features2d::BRISK::create(30, 3, 1.0)?;

        brisk.compute(&img1, &mut kp1, &mut des1).unwrap();

        let align_id = msg.get_actor_ids().get(TRACKER).unwrap();

        let traker_msg: String = GLOBAL_PARAMS.get(TRACKER.to_string(), "actor_message".to_string());

        //align_id.send_new(TrackerMsgKLT::new(msg.get_frame().clone(), kp1.darvis_vector_of_keypoint(), des1.grayscale_mat(), msg.get_actor_ids().clone())).unwrap();
            
        match traker_msg.as_ref()
        {
            "TrackerMsg" => {align_id.send_new(TrackerMsg::new(kp1.darvis_vector_of_keypoint(), des1.grayscale_mat(), msg.get_actor_ids().clone())).unwrap();
            }
            ,
            "TrackerMsgKLT" => {align_id.send_new(TrackerMsgKLT::new(msg.get_frame().clone(), kp1.darvis_vector_of_keypoint(), des1.grayscale_mat(), msg.get_actor_ids().clone())).unwrap();
            }
            ,
            _ => {
                println!("Invalid Message type: selecting TrackerMsg");
                align_id.send_new(TrackerMsg::new(kp1.darvis_vector_of_keypoint(), des1.grayscale_mat(), msg.get_actor_ids().clone())).unwrap();

                }
            ,
        }
    }
    Ok(Status::done(()))
  }

}


impl Function for DarvisFast {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.fast_extract(_context, message).unwrap();
        Ok(Status::done(()))
    }

}
