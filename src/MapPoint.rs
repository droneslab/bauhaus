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


use crate::KeyFrame::*;

struct MapPoint {
    id: u64,
    firstKeyframeId: u64,
    position: Vec,
    keyFrameRef: &mut,
    keyFrameList: list<&mut>
    map: &mut,
    depthThreshold: f64,
    //keypoint: VectorOfKeyPoint,
    //descriptor: PtrOfORB
}

