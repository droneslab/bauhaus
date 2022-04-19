use std::collections::HashMap;
use crate::map::{
    keyframe::KeyFrame,
    mappoint::MapPoint
};

#[derive(Debug, Clone)]
pub struct Map {
    keyframes: HashMap<u64, KeyFrame>,
    last_kf_id: u64,
    mappoints: HashMap<u64, MapPoint>,
    last_mp_id: u64,

    pub num_kfs: u64,
    pub num_mps: u64
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
            last_kf_id: 0,
            mappoints: HashMap::new(),
            last_mp_id: 0,
            num_kfs: 0,
            num_mps: 0
        }
    }

    pub fn insert_kf(&mut self, kf: KeyFrame, id: u64) {
        self.last_kf_id += id;
        self.num_kfs += 1;
        self.keyframes.insert(kf.id, kf);
        println!("Inserted kf!");
    }
}
