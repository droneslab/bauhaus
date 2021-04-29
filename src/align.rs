// Accepts a message of two image extracted data (kps, des), computes homography
extern crate nalgebra as na;
use na::*;
use axiom::prelude::*;
use serde::{Deserialize, Serialize};

// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct AlignMsg {
    // Vector of image paths to read in/extract
    img1_kps: DMatrix<f64>,
    img1_des: DMatrix<f64>,
    img2_kps: DMatrix<f64>,
    img2_des: DMatrix<f64>,
}

impl AlignMsg {
    pub fn new(kps1: DMatrix<f64>, des1: DMatrix<f64>, kps2: DMatrix<f64>, des2: DMatrix<f64>) -> Self {
        Self {
            img1_kps: kps1,
            img1_des: des1,
            img2_kps: kps2,
            img2_des: des2,
        }
    }
}

// This is the handler that will be used by the actor.
pub async fn align(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<AlignMsg>() {
        println!("{:?}", context);
        println!("{:?}", &msg.img1_kps.nrows());
        println!("{:?}", &msg.img1_des.nrows());
        println!("{:?}", &msg.img2_kps.nrows());
        println!("{:?}", &msg.img2_des.nrows());
    }
    Ok(Status::done(()))
}
