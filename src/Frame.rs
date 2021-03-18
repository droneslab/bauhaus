use opencv::{
    core,
    features2d,
    features2d::{Feature2DTrait, ORB},
    highgui,
    imgproc,
    prelude::*,
    videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};

use crate::Pose::*;

mod Frame {
    pub struct Frame {
        id: u64,
        timestamp: u64,
        keyPoints: VectorOfKeyPoint,
        descriptors: core::Mat,
        pose: Pose,
        depthThreshold: u64,
        camParams: Mat
    }
}

// impl Frame {
//     fn setPose(&self, pose: Pose) {
//         self.pose = pose;
//     }
// }
