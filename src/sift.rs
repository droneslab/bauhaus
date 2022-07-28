use opencv::{
    prelude::*,
    features2d::{Feature2DTrait,ORB},
    types::{PtrOfORB, VectorOfKeyPoint},
};
use axiom::prelude::*;
use crate::dvutils::*;
use crate::tracker::*;
use crate::tracker_klt::*;

use crate::base::*;


use crate::actornames::*;


use crate::pluginfunction::*;

#[derive(Debug, Clone)]
pub struct DarvisSift;

impl DarvisSift
{
    pub fn new() -> DarvisSift {
        DarvisSift {}
    }
// This is the handler that will be used by the actor.
pub fn sift_extract(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<FrameMsg>() {
        let mut kp1 = VectorOfKeyPoint::new();
        let mut des1 = Mat::default();
        let img1 = msg.get_frame().grayscale_to_cv_mat();

        let mut sift: PtrOfORB = ORB::default().unwrap();//SIFT::create(0,3,0.04,10.0,1.6).unwrap();

        sift.detect_and_compute(&img1,&Mat::default(), &mut kp1, &mut des1, false).unwrap();

        let align_id = msg.get_actor_ids().get(TRACKER).unwrap();

        let traker_msg: String = get_global_param(&TRACKER.to_string(), &"actor_message".to_string());

        match traker_msg.as_ref()
        {
            "TrackerMsg" => {align_id.send_new(TrackerMsg::new(kp1.darvis_vector_of_keypoint(), des1.grayscale_mat_f32_u8(), msg.get_actor_ids().clone())).unwrap();
            }
            ,
            "TrackerMsgKLT" => {align_id.send_new(TrackerMsgKLT::new(msg.get_frame().clone(), kp1.darvis_vector_of_keypoint(), des1.grayscale_mat_f32_u8(), msg.get_actor_ids().clone())).unwrap();
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


impl Function for DarvisSift {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.sift_extract(_context, message).unwrap();
        Ok(Status::done(()))
    }

}
