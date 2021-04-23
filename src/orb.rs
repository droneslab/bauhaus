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

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct OrbMsg {
    // Vector of image paths to read in/extract
    img_paths: Vec<String>,
}

impl OrbMsg {
    pub fn new(vec: Vec<String>) -> Self {
        Self {
            img_paths: vec,
        }
    }
}

// This is the handler that will be used by the actor.
pub async fn orb_extract(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<OrbMsg>() {
        println!("{:?}", context);
        for path in &msg.img_paths {
            let img = imgcodecs::imread(path, imgcodecs::IMREAD_COLOR).unwrap();
            let mut orb: PtrOfORB = ORB::default().unwrap();
            let mut kp = VectorOfKeyPoint::new();
            let mut des = Mat::default();

            orb.detect_and_compute(&img,&Mat::default(), &mut kp, &mut des, false).unwrap();
            println!("Processed {}, found {} keypoints", path, kp.len());
        }
        // context.system.trigger_shutdown();
    }
    Ok(Status::done(()))
}
