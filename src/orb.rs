#[allow(unused_imports)]
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

// Axiom stuff
use axiom::prelude::*;
use serde::{Deserialize, Serialize};

use crate::utils;
use crate::align;

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct OrbMsg {
    // Vector of image paths to read in/extract
    img_paths: Vec<String>,
    alignment_id: axiom::actors::Aid,
}

impl OrbMsg {
    pub fn new(vec: Vec<String>, align_id: axiom::actors::Aid) -> Self {
        Self {
            img_paths: vec,
            alignment_id: align_id,
        }
    }
}

// This is the handler that will be used by the actor.
pub async fn orb_extract(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<OrbMsg>() {
        println!("{:?}", context);
        for path in &msg.img_paths {
            let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;
            let mut orb: PtrOfORB = ORB::default().unwrap();
            let mut kp = VectorOfKeyPoint::new();
            let mut des = Mat::default();

            orb.detect_and_compute(&img,&Mat::default(), &mut kp, &mut des, false).unwrap();
            println!("Processed {}, found {} keypoints", path, kp.len());

            let kpvec = utils::cv_vector_of_keypoint_to_na(kp);

            // TODO: Changed this util function to accept a reference to avoid "move" limitation. See if this is ok
            let namat = utils::cv_mat_to_na_grayscale(&des);
            let namat2 = utils::cv_mat_to_na_grayscale(&des);
            let namat3 = utils::cv_mat_to_na_grayscale(&des);
            let namat4 = utils::cv_mat_to_na_grayscale(&des);

            // Sent to alignment
            // TODO: This is just a test send for now. Need to change message to accept the custom DmatKeypoint type
            &msg.alignment_id.send_new(align::AlignMsg::new(namat, namat2, namat3, namat4)).unwrap();
        }
        // context.system.trigger_shutdown();
    }
    Ok(Status::done(()))
}
