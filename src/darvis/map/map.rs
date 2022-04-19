use std::collections::HashMap;
use crate::map::{
    keyframe::KeyFrame,
    mappoint::MapPoint
};

#[derive(Debug)]
pub struct Map {
    keyframes: HashMap<u32, KeyFrame>,
    mappoints: HashMap<u32, MapPoint>,

    pub num_kfs: u32,
    pub num_mps: u32
    // Sofiya: following are in orbslam, not sure if we need:
    // referenceMapPoints: Vec<MapPoint>
    // mvpKeyFrameOrigins: Vec<KeyFrame>
    // mvpReferenceMapPoints: Vec<MapPoint>
    // mnMaxKFid: u32
}

impl Map {
    pub fn new() -> Map {
        Map {
            keyframes: HashMap::new(),
            mappoints: HashMap::new(),
            num_kfs: 0,
            num_mps: 0
        }
    }
}
