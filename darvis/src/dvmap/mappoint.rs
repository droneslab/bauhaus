use std::{fmt::Debug};
use array2d::Array2D;
use dvcore::{matrix::DVMatrix, config::{Sensor, GLOBAL_PARAMS, SYSTEM_SETTINGS}};
use log::{info, error, debug};
use na::Vector3;
use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use crate::{matrix::DVVector3, modules::orbmatcher::{descriptor_distance, SCALE_FACTORS}, registered_modules::FEATURE_DETECTION};
use super::{map::{Id, Map}, observations::Observations};

// Note: Implementing typestate for like here: http://cliffle.com/blog/rust-typestate/#a-simple-example-the-living-and-the-dead
// This way we can encode mappoints that have been created but not inserted into the map as a separate type than mappoints that are legit.
// This prevents making the following mistake:
// 1 - create a mappoint with MapPoint::new() with an id = -1.
// 2 - connect the mappoint to a keyframe, which would incorrectly set the ID to be -1.
// 3 - insert the mappoint into the map, which gives it a proper id.
// The correct order is to flip steps 2 and 3, so by the time an observation is set, we KNOW the mappoint Id has already been set too.
// We can make this a compile-time error by adding state types in the following functions:
// Creating a mappoint returns a MapPoint<PrelimMapItem>
// Inserting into the map turns a MapPoint<PrelimMapItem> into a MapPoint<FullMapItem>
// The function in step 2 (to add a mappoint to a keyframe) takes a MapPoint<FullMapItem>

// Typestate...Mappoint information that is ALWAYS available, regardless of mappoint state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapPoint<M: MapPointState> {
    // List of variables set in constructor in orbslam3, don't think we need these all but copying for reference
    //     mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    //     mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    //     mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    //     mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), 
    pub position: DVVector3<f64>, // Sofiya: why a vector3 instead of a Pose? Same in ORBSLAM3 too but I thought mps had poses...

    // Map connections
    // Sofiya: it would be nice if we could guarantee that the connections are updated/correct
    // rather than duplicating all these connections across all the objects and hoping we remember
    // to update them correctly after a map modification
    origin_map_id: Id,
    ref_kf: Id, // = mnFirstKFid and mpRefKF ... first one is Id, second is a pointer to the KF, but we only keep Ids 

    // Variables in ORBSLAM, DON'T Set these!!
    // mnFirstFrame ... literally never used meaningfully
    // mnLastFrameSeen ... similar to "mnTrackReferenceForFrame" in KeyFrame. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
    // mbTrackInView ... only used by tracking to keep track of which mappoints to show. Just keep this data saved in tracking locally

    // Typestate...This reassures the compiler that the parameter gets used.
    full_mp_info: M,
}

// Typestate...State options.
#[derive(Clone, Debug)]
pub struct PrelimMapPoint {} // prelimary map item created locally but not inserted into the map yet
#[derive(Clone, Debug)]
pub struct FullMapPoint { // Full map item inserted into the map with the following additional fields
    id: Id,

    // Observations, this is a part of "map connections" but I don't think we can avoid keeping this here.
    observations: Observations, // mObservations ; Keyframes observing the point and associated index in keyframe

    // Best descriptor used for fast matching
    best_descriptor: DVMatrix,

    // Variables used by merging
    normal_vector: DVVector3<f64>,  // mNormalVector ; Mean viewing direction

    // Scale invariance distances
    max_distance: f64,
    min_distance: f64,

    // following two are only set by tracking and only checked by local mapping for mappoint culling, but not sure what the diff is b/w visible and found
    nvisible: i32, //mnvisible
    nfound: i32, //mnfound

    sensor: Sensor
}

pub trait MapPointState {}
impl MapPointState for PrelimMapPoint {}
impl MapPointState for FullMapPoint {}

impl MapPoint<PrelimMapPoint> {
    pub fn new(position: DVVector3<f64>, ref_kf: Id, origin_map_id: Id) -> Self {
        Self {
            position,
            origin_map_id,
            ref_kf,
            full_mp_info: PrelimMapPoint{} 
        }
    }
    // TODO: Two constructors, this one takes an idxF that I think is the index in observations
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
}

impl MapPoint<FullMapPoint> {
    pub(super) fn new(prelim_mappoint: MapPoint<PrelimMapPoint>, id: Id) -> Self {
    let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

        Self {
            position: prelim_mappoint.position,
            origin_map_id: prelim_mappoint.origin_map_id,
            ref_kf: prelim_mappoint.ref_kf,
            full_mp_info: FullMapPoint{
                id,
                observations: Observations::new(sensor),
                normal_vector: DVVector3::zeros::<f64>(),
                max_distance: 0.0,
                min_distance: 0.0,
                nvisible: 1,
                nfound: 1,
                best_descriptor: DVMatrix::empty(),
                sensor: sensor
            },
        }
    }

    pub fn get_id(&self) -> Id { self.full_mp_info.id }
    pub fn get_observations(&self) -> &Observations { &self.full_mp_info.observations }
    pub fn get_best_descriptor(&self) -> &DVMatrix { &self.full_mp_info.best_descriptor }
    pub fn get_normal(&self) -> DVVector3<f64> {
        // Eigen::Vector3f MapPoint::GetNormal() 
        self.full_mp_info.normal_vector
    }

    pub fn get_max_distance_invariance(&self) -> f64 {
        // float MapPoint::GetMaxDistanceInvariance()
        1.2 * self.full_mp_info.max_distance
    }

    pub fn get_min_distance_invariance(&self) -> f64 {
        // float MapPoint::GetMinDistanceInvariance()
        0.8 * self.full_mp_info.min_distance
    }

    pub fn increase_found(&mut self, n: &i32) {
        // void MapPoint::IncreaseFound(int n)
        self.full_mp_info.nfound += n;
    }

    pub fn add_observation(&mut self, kf_id: &Id, num_keypoints_left_for_kf: u32, index: u32) {
        self.full_mp_info.observations.add_observation(kf_id, num_keypoints_left_for_kf, index);
    }

    pub fn get_norm_and_depth(&self, map: &Map) -> Option<(f64, f64, DVVector3<f64>)> {
        // Part 1 of void MapPoint::UpdateNormalAndDepth()
        if self.full_mp_info.observations.is_empty() {
            return None;
        }

        let (n, normal) = self.full_mp_info.observations.get_normal(map, &self.position);

        let ref_kf = map.get_keyframe(&self.ref_kf).unwrap();
        let pc = (*self.position) - (*ref_kf.get_camera_center());
        let dist = pc.norm();

        let level = self.full_mp_info.observations.get_level(ref_kf);
        let level_scale_factor = SCALE_FACTORS[level as usize] as f64;
        let n_levels = GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "n_levels");

        let max_distance = dist * level_scale_factor;
        let min_distance = self.full_mp_info.max_distance / (SCALE_FACTORS[(n_levels - 1) as usize] as f64);
        let normal_vector = DVVector3::new(normal / (n as f64));

        Some((max_distance, min_distance, normal_vector))
    }

    pub fn update_norm_and_depth(&mut self, vals: (f64, f64, DVVector3<f64>)) {
        // Part 2 of void MapPoint::UpdateNormalAndDepth()
        self.full_mp_info.max_distance = vals.0;
        self.full_mp_info.min_distance = vals.1;
        self.full_mp_info.normal_vector = vals.2;
    }

    pub fn compute_distinctive_descriptors(&self, map: &Map) -> Option<DVMatrix> {
        // Part 1 of void MapPoint::ComputeDistinctiveDescriptors()
        if self.full_mp_info.observations.len() == 0 {
            error!("mappoint::compute_distinctive_descriptors;No observations");
            return None;
        }

        let descriptors = self.full_mp_info.observations.compute_descriptors(map);
        if descriptors.len() == 0 { 
            error!("mappoint::compute_distinctive_descriptors;No descriptors");
            return None;
         }

        // Compute distances between them
        let mut distances = Array2D::filled_with(0, descriptors.len(), descriptors.len());
        for i in 0..descriptors.len() {
            distances[(i,i)] = 0;
            for j in i+1..descriptors.len() {
                let dist_ij = descriptor_distance(&descriptors[i], &descriptors[j]);
                distances[(i,j)] = dist_ij;
                distances[(j,i)] = dist_ij;
            }
        }

        // Take the descriptor with least median distance to the rest
        let mut best_median = std::i32::MAX;
        let mut best_idx = 0;

        let all_dists: Vec<i32> = distances.elements_row_major_iter().cloned().collect();
        let N = descriptors.len();
        for i in 0..N {
            let mut slice_dists = Vec::new();
            slice_dists.resize(N - i, 0i32);
            slice_dists[..N-i].clone_from_slice(&all_dists[i..N]);
            slice_dists.sort();
            let median = slice_dists[(0.5 * (N-1) as f64) as usize];
            if median < best_median {
                best_median = median;
                best_idx = i;
            }
        }

        Some(DVMatrix::new(descriptors[best_idx].clone()))
    }

    pub fn update_distinctive_descriptors(&mut self, desc: DVMatrix) {
        // Part 2 of void MapPoint::ComputeDistinctiveDescriptors()
        self.full_mp_info.best_descriptor = desc;
    }

    pub fn predict_scale(&self, current_distance: &f64) -> i32 {
        // int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
        let ratio = self.full_mp_info.max_distance / current_distance;
        let scale_factor= GLOBAL_PARAMS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let log_scale_factor = scale_factor.log10();
        let scale = (ratio.log10() / log_scale_factor).ceil() as i32;
        let scale_levels = GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "n_levels");

        let scale = if scale < 0 { 0 } else { scale };
        if scale < 0 {
            return 0;
        } else if scale >= scale_levels.into() {
            return scale_levels - 1;
        } else {
            return scale;
        }
    }
    
}