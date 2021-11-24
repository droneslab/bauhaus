use opencv::{
    prelude::*,
    core,
    features2d,
    features2d::{Feature2DTrait, ORB},
    highgui,
    imgproc,
    videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};
use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use crate::utils::*;
use crate::align::*;
use crate::vis::*;
extern crate nalgebra as na;

use crate::utils;
use crate::align;

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct OrbMsg {
    // Vector of image paths to read in/extract
    img_paths: Vec<String>,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl OrbMsg {
    pub fn new(vec: Vec<String>, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            img_paths: vec,
            actor_ids: ids,
        }
    }
}

#[no_mangle]
// This is the handler that will be used by the actor.
pub fn orb_extract(_: (), _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<OrbMsg>() {
        let mut kp1 = VectorOfKeyPoint::new();
        let mut des1 = Mat::default();
        let mut kp2 = VectorOfKeyPoint::new();
        let mut des2 = Mat::default();

        for path in &msg.img_paths {
            let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;
            let mut orb: PtrOfORB = ORB::default().unwrap();

            orb.detect_and_compute(&img,&Mat::default(), &mut kp1, &mut des1, false).unwrap();
            // println!("Processed {}, found {} keypoints", path, kp1.len());

            if kp1.len() > 0 && kp2.len() > 0 {
                let kpvec1 = cv_vector_of_keypoint_to_na(&kp1);
                let kpvec2 = cv_vector_of_keypoint_to_na(&kp2);

                // TODO: Changed this util function to accept a reference to avoid "move" limitation. See if this is ok
                let nades1 = cv_mat_to_na_grayscale(&des1);
                let nades2 = cv_mat_to_na_grayscale(&des2);

                // Sent to alignment
                // println!("{:?}", &msg.actor_ids);
                let align_id = &msg.actor_ids.get("align").unwrap();
                let vis_id = &msg.actor_ids.get("vis").unwrap();
                // println!("{}", align_id);
                // TODO: This is just a test send for now. Need to change message to accept the custom DmatKeypoint type
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
