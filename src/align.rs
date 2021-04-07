// Accepts a message of two image extracted data (kps, des), computes homography

use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use opencv::{
    core,
    calib3d,
    prelude::*,
    imgcodecs,
    types::{VectorOfKeyPoint},
};

// serde derives for serialization of opencv-rust types
#[derive(Serialize, Deserialize)]
#[serde(remote = "Mat")]
struct MatSerde {
    ptr: *mut core::c_void
}



// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct AlignMsg {
    // Vector of image paths to read in/extract
    // img1_kps: VectorOfKeyPoint,
    #[serde(with = "MatSerde")]
    img1_des: Mat,
    // img2_kps: VectorOfKeyPoint,
    #[serde(with = "MatSerde")]
    img2_des: Mat,
}

impl AlignMsg {
    pub fn new(kps1: VectorOfKeyPoint, des1: Mat, kps2: VectorOfKeyPoint, des2: Mat) -> Self {
        Self {
            // img1_kps: kps1,
            img1_des: des1,
            // img2_kps: kps2,
            img2_des: des2,
        }
    }
}

// This is the handler that will be used by the actor.
pub async fn align(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<AlignMsg>() {
        println!("{:?}", context);
        // for path in &msg.img_paths {
        //     let img = imgcodecs::imread(path, imgcodecs::IMREAD_COLOR).unwrap();
        //     let mut orb: PtrOfORB = ORB::default().unwrap();
        //     let mut kp = VectorOfKeyPoint::new();
        //     let mut des = Mat::default().unwrap();

        //     orb.detect_and_compute(&img,&Mat::default().unwrap(), &mut kp, &mut des, false).unwrap();
        //     println!("Processed {}, found {} keypoints", path, kp.len());
        
        

    }
    Ok(Status::done(()))
}
