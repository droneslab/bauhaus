use std::collections::HashMap;

use crate::map::{
    keyframe::KeyFrame,
    mappoint::MapPoint
};

pub type Id = i32;

#[derive(Debug, Clone)]
pub struct Map {
    pub imu_initialized: bool,

    keyframes: HashMap<Id, KeyFrame>, // = mspKeyFrames
    last_kf_id: Id, // = mnMaxKFid
    _mappoints: HashMap<Id, MapPoint>, // = mspMapPoints
    _last_mp_id: Id,

    initial_kf_id: Id,

    pub num_kfs: u64,
    pub num_mps: u64
    // Sofiya: following are in orbslam3, not sure if we need:
    // mvpKeyFrameOrigins: Vec<KeyFrame>
    // mvBackupKeyFrameOriginsId: Vec<: u32>
    // mpFirstRegionKF: KeyFrame* 
    // static const int THUMB_WIDTH
    // static const int THUMB_HEIGHT
    // mpKFinitial: KeyFrame*
    // mpKFlowerID: KeyFrame*
    // referenceMapPoints: Vec<MapPoint>
    // mvpReferenceMapPoints: Vec<MapPoint>
    // mnInitKFid: u32

    // Sofiya: following are in orbslam3, probably don't need
    // mbFail: bool
    // nNextId: : u32
    // mbImuInitialized: bool
    // mnMapChange: bool
    // mnMapChangeNotified: bool
    // mnBigChangeIdx: i32
}

impl Map {
    pub fn new() -> Map {
        Map {
            keyframes: HashMap::new(),
            last_kf_id: 0,
            _mappoints: HashMap::new(),
            _last_mp_id: 0,
            initial_kf_id: 0,
            num_kfs: 0,
            num_mps: 0,
            imu_initialized: false
        }
    }

    pub fn new_keyframe(&mut self, kf: KeyFrame) {
        // Sofiya: Not sure this works
        if self.keyframes.is_empty() {
            println!("First KF: {}; Map init KF: {}", kf.id, self.initial_kf_id);
            self.initial_kf_id = kf.id;
            // self.lowest_kf = kf; // TODO: not sure what this is? from ORBSLAM3:Map.cc:67
        }

        self.last_kf_id += 1;
        self.keyframes.insert(self.last_kf_id, kf);

        self.num_kfs += 1;
        println!("Inserted kf!");
    }

    pub fn new_mappoint(&mut self, _mp: MapPoint, _id: Id) {
        // TODO
    }

}
