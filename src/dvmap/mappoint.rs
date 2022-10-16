use std::collections::HashMap;
use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use crate::matrix::DVVector3;
use super::{pose::Pose, map::Id};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapPoint {
    // List of variables set in constructor in orbslam3, don't think we need these all but copying for reference
    //     mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    //     mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    //     mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    //     mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), 

    id: Id,
    position: DVVector3<f64>, // Sofiya: why a vector3 instead of a Pose? Same in ORBSLAM3 too but I thought mps had poses...

    // Map connections
    // Sofiya: it would be nice if we could guarantee that the connections are updated/correct
    // rather than duplicating all these connections across all the objects and hoping we remember
    // to update them correctly after a map modification
    origin_map_id: Id,
    ref_kf: Id, // = mnFirstKFid and mpRefKF ... first one is Id, second is a pointer to the KF, but we only keep Ids 

    // Observations
    // This is a part of "map connections" but I don't think we can avoid keeping this here.
    pub observations: HashMap<Id, i32>, // mObservations ; Keyframes observing the point and associated index in keyframe

    // Sofiya: Not sure if we need ... pretty sure these are state variables that are changed by tracking, so move this to tracking?
    mbTrackInView: bool,
    // following two are only set by tracking and only checked by local mapping for mappoint culling, but not sure what the diff is b/w visible and found
    pub nvisible: i32, //mnvisible
    pub nfound: i32, //mnfound

    // Variables used by merging
    normal_vector: DVVector3<f64>,  // mNormalVector ; Mean viewing direction


    // Variables in ORBSLAM, DON'T Set these!!
    // mnFirstFrame ... literally never used meaningfully
    // mnLastFrameSeen ... similar to "mnTrackReferenceForFrame" in KeyFrame. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
}

impl MapPoint {
    pub fn hidden_new(position: DVVector3<f64>, ref_kf: Id, origin_map_id: Id, mp_id: Id) -> Self {
        Self {
            id: mp_id,
            position,
            origin_map_id,
            ref_kf,
            observations: HashMap::new(),
            mbTrackInView: false,
            nvisible: 1,
            nfound: 1,
            normal_vector: DVVector3::zeros::<f64>()
        }
    }
    pub(super) fn new_with_id(position: DVVector3<f64>, ref_kf: Id, origin_map_id: Id, mp_id: Id) -> Self {
        // Note: Only callable from the map actor, who assigns a non-conflicting kf id
        Self::hidden_new(position, ref_kf, origin_map_id, mp_id)
    }

    pub fn new(position: DVVector3<f64>, ref_kf: Id, origin_map_id: Id) -> Self {
        Self::hidden_new(position, ref_kf, origin_map_id, -1)
    }

    // Sofiya: Two constructors, this one takes an idxF that I think is the index in observations
    // Need to figure out why sometimes this one is called instead of the other one 
    // MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    // mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    // mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    // mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    // mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
    // {
    //     SetWorldPos(Pos);

    //     Eigen::Vector3f Ow;
    //     if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
    //         Ow = pFrame->GetCameraCenter();
    //     }
    //     else{
    //         Eigen::Matrix3f Rwl = pFrame->GetRwc();
    //         Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
    //         Eigen::Vector3f twl = pFrame->GetOw();

    //         Ow = Rwl * tlr + twl;
    //     }
    //     mNormalVector = mWorldPos - Ow;
    //     mNormalVector = mNormalVector / mNormalVector.norm();

    //     Eigen::Vector3f PC = mWorldPos - Ow;
    //     const float dist = PC.norm();
    //     const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
    //                                             : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
    //                                                                         : pFrame -> mvKeysRight[idxF].octave;
    //     const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    //     const int nLevels = pFrame->mnScaleLevels;

    //     mfMaxDistance = dist*levelScaleFactor;
    //     mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    //     pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    //     // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    //     unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    //     mnId=nNextId++;
    // }


    pub fn set_position(&mut self, pos : &DVVector3<f64>)  {
        self.position = pos.clone();
    }

    pub fn get_position(&self) -> &DVVector3<f64> {
        &self.position
    }
}