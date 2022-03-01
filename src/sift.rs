use opencv::{
    prelude::*,
    core,
    features2d,
    features2d::{Feature2DTrait,SIFT},

    highgui,
    imgproc,
    videoio,
    imgcodecs,
    types::{PtrOfSIFT, VectorOfKeyPoint},
};
use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use crate::dvutils::*;
use crate::align::*;
use crate::vis::*;





use crate::pluginfunction::*;

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct SiftMsg {
    // Vector of image paths to read in/extract
    img_paths: Vec<String>,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl SiftMsg {
    pub fn new(vec: Vec<String>, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            img_paths: vec,
            actor_ids: ids,
        }
    }
}


#[derive(Debug, Clone)]
pub struct DarvisSift;

impl DarvisSift
{
    pub fn new() -> DarvisSift {
        DarvisSift {}
    }
// This is the handler that will be used by the actor.
pub fn sift_extract(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<SiftMsg>() {
        let mut kp1 = VectorOfKeyPoint::new();
        let mut des1 = Mat::default();
        let mut kp2 = VectorOfKeyPoint::new();
        let mut des2 = Mat::default();
        for path in &msg.img_paths {
            let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;
           
            let mut sift: PtrOfSIFT = SIFT::create(0,3,0.04,10.0,1.6).unwrap();


            sift.detect_and_compute(&img,&Mat::default(), &mut kp1, &mut des1, false).unwrap();
            // println!("Processed {}, found {} keypoints", path, kp1.len());

            if kp1.len() > 0 && kp2.len() > 0 {
                let kpvec1 = kp1.darvis_vector_of_keypoint();
                let kpvec2 = kp2.darvis_vector_of_keypoint();
                
                // TODO: Changed this util function to accept a reference to avoid "move" limitation. See if this is ok
                let nades1 = des1.grayscale_mat_f32_u8();
                let nades2 = des2.grayscale_mat_f32_u8();

                // Sent to alignment
                // println!("{:?}", &msg.actor_ids);
                let align_id = &msg.actor_ids.get("align").unwrap();
                let vis_id = &msg.actor_ids.get("vis").unwrap();
                // println!("{}", align_id);
                // TODO: This is just a test send for now. Need to change message to accept the custom DarvisKeyPoint type
                println!("Processed image: {}", path);
                vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();
                align_id.send_new(AlignMsg::new(kpvec1, nades1, kpvec2, nades2, msg.actor_ids.clone())).unwrap();
            }

            kp2 = kp1;
            des2 = des1;
            kp1 = VectorOfKeyPoint::new();
            des1 = Mat::default();
        }
        // context.system.trigger_shutdown();
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
