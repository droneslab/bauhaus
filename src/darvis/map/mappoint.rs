use na::Vector3;
use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use crate::{
    map::map::Id,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapPoint {
    id: Id,
    first_keyframe_id: Id,
    pub mbTrackInView: bool,

    pub position: Vector3<f64>,
    ref_keyframe: u64,
    seen_by_keyframes: Vec<Id>,
    depth_threshold: f64,
    pub last_frame_seen: Id,
    pub mbTrackInView: bool,
    pub mnFound: i32,
    pub num_observations: i32
}


impl MapPoint {
    pub fn new(id: i32, first_keyframe_id: i32, pos: &Vector3<f64>, ref_keyframe: u64) -> Self {
        Self {
            id: id,
            first_keyframe_id: first_keyframe_id,
            position: pos.clone(),
            ref_keyframe: ref_keyframe,
            seen_by_keyframes: Vec::new(),
            depth_threshold: 0.0,
            mbTrackInView: false,
            last_frame_seen: first_keyframe_id,
            num_observations: 0
            mnFound: 1,
        }
        // SetWorldPos(Pos);

        // mNormalVector.setZero();

        // mbTrackInViewR = false;
        // mbTrackInView = false;

        // // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        // unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        // mnId=nNextId++;        
    }

    // void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
    //     unique_lock<mutex> lock2(mGlobalMutex);
    //     unique_lock<mutex> lock(mMutexPos);
    //     mWorldPos = Pos;
    // }
    pub fn set_world_pos(&mut self, pos : &Vector3<f64>)  {
        //unique_lock<mutex> lock2(mGlobalMutex);
        //unique_lock<mutex> lock(mMutexPos);
        self.position = pos.clone();
    }
    
    // Eigen::Vector3f MapPoint::GetWorldPos() {
    //     unique_lock<mutex> lock(mMutexPos);
    //     return mWorldPos;
    // }
    pub fn get_world_pos(&self) -> &Vector3<f64> {
        &self.position
    }

    pub fn observations(&self) -> u32 {
        todo!("Add Observation field");
    }

}