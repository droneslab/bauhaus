// Accepts a message of two image extracted data (kps, des), computes homography
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
    types::{VectorOfKeyPoint},
};

extern crate nalgebra as na;
use na::*;
use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use crate::utils::*;

// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct AlignMsg {
    // Vector of image paths to read in/extract
    img1_kps: Vec<DmatKeyPoint>,
    img1_des: na::DMatrix<u8>,
    img2_kps: Vec<DmatKeyPoint>,
    img2_des: na::DMatrix<u8>,
}

impl AlignMsg {
    pub fn new(kps1: Vec<DmatKeyPoint>, des1: DMatrix<u8>, kps2: Vec<DmatKeyPoint>, des2: na::DMatrix<u8>) -> Self {
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
        println!("Align actor img1 kps {:?}", &msg.img1_kps.len());
        println!("Align actor img1 des {:?}", &msg.img1_des.nrows());
        println!("Align actor img2 kps {:?}", &msg.img2_kps.len());
        println!("Align actor img2 des {:?}", &msg.img2_des.nrows());

        // Convert back to cv structures
        let mut kp1 = na_keypoint_to_cv_vector_of_keypoint(&msg.img1_kps);
        let mut des1 = na_grayscale_to_cv_mat(&msg.img1_des);
        let mut kp2 = na_keypoint_to_cv_vector_of_keypoint(&msg.img2_kps);
        let mut des2 = na_grayscale_to_cv_mat(&msg.img2_des);
       
        println!("After conversion!"); 
        println!("Align actor img1 kps {:?}", kp1.len());
        println!("Align actor img1 des {:?}", des1.rows());
        println!("Align actor img2 kps {:?}", kp2.len());
        println!("Align actor img2 des {:?}", des2.rows());
    }
    Ok(Status::done(()))
}
