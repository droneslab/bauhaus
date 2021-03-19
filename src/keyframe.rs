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

use abow;

use crate::Pose::*;

struct KeyFrame {
    id: u64,
    timestamp: u64,
    mapPoints: Vec,
    bow: abow::BoW,
    pose: Pose,
    map: &mut,
    bow_db: &mut,
    scale, f64,
    depthThreshold: f64
}