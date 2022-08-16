
use na::Vector3;
use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use crate::map::map::Id;

use super::{keyframe::KeyFrame, map::Map};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapPoint {
    id: Id,
    first_keyframe_id: Id,
    // 3-Vector
    position: Vector3<f32>, // na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
    ref_keyframe: u64,
    seen_by_keyframes: Vec<Id>,
    depth_threshold: f64,
    pub mbTrackInView: bool,
    pub mnLastFrameSeen: Id,
}


impl MapPoint
{

//     MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map* pMap):
//     mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
//     mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
//     mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
//     mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
//     mnOriginMapId(pMap->GetId())
// {
//     SetWorldPos(Pos);

//     mNormalVector.setZero();

//     mbTrackInViewR = false;
//     mbTrackInView = false;

//     // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
//     unique_lock<mutex> lock(mpMap->mMutexPointCreation);
//     mnId=nNextId++;
// }

    pub fn new(id: i32, first_keyframe_id: i32, pos: &Vector3<f32>, ref_keyframe: u64) -> Self
    {
        Self {
            id: id,
            first_keyframe_id: first_keyframe_id,
            position: pos.clone(),
            ref_keyframe: ref_keyframe,
            seen_by_keyframes: Vec::new(),
            depth_threshold: 0.0,
            mbTrackInView: false,
            mnLastFrameSeen: first_keyframe_id,
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
    pub fn set_world_pos(&mut self, pos : &Vector3<f32>) 
    {
        //unique_lock<mutex> lock2(mGlobalMutex);
        //unique_lock<mutex> lock(mMutexPos);
        self.position = pos.clone();
    }
    
    // Eigen::Vector3f MapPoint::GetWorldPos() {
    //     unique_lock<mutex> lock(mMutexPos);
    //     return mWorldPos;
    // }
    pub fn get_world_pos(&self) -> &Vector3<f32> 
    {
        &self.position
    }

    pub fn observations(&self) -> u32
    {
        todo!("Add Observation field");
        0
    }

}