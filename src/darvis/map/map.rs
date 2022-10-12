use std::collections::HashMap;

use crate::map::{
    keyframe::KeyFrame,
    mappoint::MapPoint,
};
use crate::{
    utils::sensor::*,
    global_params::{GLOBAL_PARAMS, SYSTEM_SETTINGS},
};

pub type Id = i32;

#[derive(Debug, Clone)]
pub struct Map<S: SensorType> {
    pub id: Id,

    pub imu_initialized: bool, // isImuInitialized(), set true by local mapper

    keyframes: HashMap<Id, KeyFrame<S>>, // = mspKeyFrames
    last_kf_id: Id, // = mnMaxKFid
    pub mappoints: HashMap<Id, MapPoint>, // = mspMapPoints
    _last_mp_id: Id,

    initial_kf_id: Id,

    pub num_kfs: u32,
    pub num_mps: u32,

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

impl<S: SensorType> Map<S> {
    pub fn new<S2: SensorType>() -> Map<S> {
        Map {
            id: 0, // TODO (Multimaps): this should increase when new maps are made
            keyframes: HashMap::new(),
            last_kf_id: 0,
            mappoints: HashMap::new(),
            _last_mp_id: 0,
            initial_kf_id: 0,
            num_kfs: 0,
            num_mps: 0,
            imu_initialized: false,
        }
    }

    pub fn get_keyframe(&self, id: &Id) -> Option<&KeyFrame<S>> {
        self.keyframes.get(id)
    }

    pub fn get_mappoint(&self, id: &Id) -> Option<&MapPoint> {
        self.mappoints.get(id)
    }

    pub fn tracked_mappoints_for_keyframe(&self, kf_id: Id, min_observations: u32) -> i32{
        // KeyFrame::TrackedMapPoints(const int &minObs)
        // Sofiya TODO: it would be nice to add another read/write lock on each keyframe
        // and move this kind of code into the keyframe class instead of the map class
        let mut num_points = 0;
        let check_observations = min_observations > 0;
        for (index, mp_id) in &self.keyframes.get(&kf_id).unwrap().mappoint_matches {
            let mappoint = self.mappoints.get(&mp_id).unwrap();
            if check_observations {
                if mappoint.observations() >= min_observations {
                    num_points += 1;
                }
            } else {
                    num_points += 1;
            }
        }

        return num_points;
    }


    //* BEHIND MAP ACTOR *//
    pub(in crate::map) fn discard_mappoint(&mut self, id: &Id) {
        self.mappoints.remove(id);
        println!("MapPoint removed {}", id);
        // Following is done in ORB SLAM , check if this is need to be done.
        // pMP.mbTrackInView= false;      
        // pMP.last_frame_seen = self.current_frame.unwrap().id;     
    }

    pub(in crate::map) fn increase_found(&mut self, id: &Id, n : i32)
    {
        self.mappoints.get_mut(id).unwrap().mnFound += n;
        println!("MapPoint founc increased {}", id);
    }

    pub(in crate::map) fn new_keyframe(&mut self, kf: KeyFrame<S>) {
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

    pub(in crate::map) fn new_mappoint(&mut self, _mp: MapPoint, _id: Id) {
        // TODO
    }

}
