use std::{collections::BTreeMap, fmt::Debug, sync::atomic::{AtomicI32, Ordering}};
use core::{matrix::DVMatrix, config::{SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor}};
use log::{error, warn};
extern crate nalgebra as na;
use crate::{matrix::DVVector3, modules::orbmatcher::{SCALE_FACTORS, descriptor_distance}, registered_actors::FEATURE_DETECTION};
use super::{map::{Id, Map}, keyframe::KeyFrame, pose::DVTranslation};

#[derive(Debug)]
pub struct MapPoint { // Full map item inserted into the map with the following additional fields
    pub id: Id,
    pub position: DVTranslation, // mWorldPos

    // Observations, this is a part of "map connections" but I don't think we can avoid keeping this here.
    observations: BTreeMap<Id, (i32, i32)>, // mObservations ; Keyframes observing the point and associated index in keyframe. BTreeMap so it is sorted by key
    pub num_obs: i32,

    // Best descriptor used for fast matching
    pub best_descriptor: DVMatrix, // mDescriptor

    // Variables used by merging
    pub normal_vector: DVVector3<f64>,  // mNormalVector ; Mean viewing direction

    // Scale invariance distances
    max_distance: f64,
    min_distance: f64,

    // Map connections
    // TODO (design, map connections): it would be nice if we could guarantee that the connections are updated/correct
    // rather than duplicating all these connections across all the objects and hoping we remember
    // to update them correctly after a map modification
    pub origin_map_id: Id,
    pub ref_kf_id: Id, // mpRefKF
    pub first_kf_id: Id, // mnFirstKFid

    // Set in tracking, used by local mapping to make decision about deleting mappoint
    found: AtomicI32, //nFound
    visible: AtomicI32, // nVisible

    // Used by loop closing
    // todo (design, variable locations) can we avoid having these in here and keep it thread local instead?
    pub ba_global_for_kf: Id, // mnBAGlobalForKF
    pub gba_pose: Option<DVTranslation>, // mTcwGBA
    pub corrected_reference: Option<(Id, i32)>, // (mnCorrectedByKF, mCorrectedReference)

    // Variables in ORBSLAM, DON'T Set these!!
    // mnFirstFrame ... literally never used meaningfully
    // mnLastFrameSeen ... similar to "mnTrackReferenceForFrame" in KeyFrame. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
    // mbTrackInView ... only used by tracking to keep track of which mappoints to show. Just keep this data saved in tracking locally

    sensor: Sensor
}

impl MapPoint {
    pub(super) fn new(position: DVVector3<f64>, ref_kf_id: Id, origin_map_id: Id, id: Id) -> Self {
        Self {
            position,
            origin_map_id,
            ref_kf_id,
            first_kf_id: ref_kf_id,
            found: AtomicI32::new(1),
            visible: AtomicI32::new(1),
            sensor: SETTINGS.get(SYSTEM, "sensor"),
            id,
            observations: BTreeMap::new(),
            normal_vector: DVVector3::zeros::<f64>(),
            max_distance: 0.0,
            min_distance: 0.0,
            best_descriptor: DVMatrix::empty(),
            gba_pose: None,
            num_obs: 0,
            ba_global_for_kf: -1,
            corrected_reference: None
        }
    }

    pub fn get_max_distance_invariance(&self) -> f64 {
        // float MapPoint::GetMaxDistanceInvariance()
        1.2 * self.max_distance
    }
    pub fn get_min_distance_invariance(&self) -> f64 {
        // float MapPoint::GetMinDistanceInvariance()
        0.8 * self.min_distance
    }

    pub fn predict_scale(&self, current_distance: &f64) -> i32 {
        // int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
        let ratio = self.max_distance / current_distance;
        let scale_factor= SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let log_scale_factor = scale_factor.log10();
        let scale = (ratio.log10() / log_scale_factor).ceil() as i32;
        let scale_levels = SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels");

        let scale = if scale < 0 { 0 } else { scale };
        if scale < 0 {
            return 0;
        } else if scale >= scale_levels {
            return scale_levels - 1;
        } else {
            return scale;
        }
    }


    pub fn get_norm_and_depth(&self, map: &Map) -> Option<(f64, f64, DVVector3<f64>)> {
        // Part 1 of void MapPoint::UpdateNormalAndDepth()
        if self.observations.is_empty() {
            return None;
        }

        let (n, normal) = self.get_obs_normal(map, &self.position);

        let ref_kf = map.keyframes.get(&self.ref_kf_id).unwrap();
        let pc = (*self.position) - (*ref_kf.get_camera_center());
        let dist = pc.norm();

        let level = self.get_level(ref_kf);
        let level_scale_factor = SCALE_FACTORS[level as usize] as f64;
        let n_levels = SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels");

        let max_distance = dist * level_scale_factor;
        let min_distance = max_distance / (SCALE_FACTORS[(n_levels - 1) as usize] as f64);
        let normal_vector = DVVector3::new(normal / (n as f64));

        Some((max_distance, min_distance, normal_vector))
    }

    pub fn update_norm_and_depth(&mut self, vals: (f64, f64, DVVector3<f64>)) {
        // Part 2 of void MapPoint::UpdateNormalAndDepth()
        self.max_distance = vals.0;
        self.min_distance = vals.1;
        self.normal_vector = vals.2;
    }

    pub fn compute_distinctive_descriptors(&self, map: &Map) -> Option<DVMatrix> {
        // Part 1 of void MapPoint::ComputeDistinctiveDescriptors()
        if self.num_obs == 0 {
            error!("mappoint::compute_distinctive_descriptors;No observations");
            return None;
        }

        let descriptors = self.compute_descriptors(map);
        if descriptors.len() == 0 { 
            error!("mappoint::compute_distinctive_descriptors;No descriptors");
            return None;
         }

        // Compute distances between them
        let mut distances = vec![vec![i32::MAX; descriptors.len()]; descriptors.len()];
        for i in 0..descriptors.len() {
            distances[i][i] = 0;

            for j in i+1..descriptors.len() {
                let dist_ij = descriptor_distance(&descriptors[i], &descriptors[j]);

                distances[i][j] = dist_ij;
                distances[j][i] = dist_ij;
            }
        }

        // Take the descriptor with least median distance to the rest
        let mut best_median = std::i32::MAX;
        let mut best_idx = 0;

        let num_descriptors = descriptors.len();
        for i in 0..num_descriptors {
            let mut v_dists = distances[i].clone();

            v_dists.sort();

            let median = v_dists[((v_dists.len() - 1)/2) as usize];

            if median < best_median {
                best_median = median;
                best_idx = i;
            }
        }

        // TODO (timing) ... this clone might take a while
        Some(DVMatrix::new(descriptors[best_idx].clone()))
    }

    pub fn update_distinctive_descriptors(&mut self, desc: DVMatrix) {
        // Part 2 of void MapPoint::ComputeDistinctiveDescriptors()
        self.best_descriptor = desc;
    }

    pub fn increase_found(& self, num: i32) {
        self.found.fetch_add(num, Ordering::SeqCst);
    }
    pub fn increase_visible(& self, num: i32) {
        self.visible.fetch_add(num, Ordering::SeqCst);
    }
    pub fn get_found_ratio(& self) -> f32 {
        self.found.load(Ordering::SeqCst) as f32 / self.visible.load(Ordering::SeqCst) as f32
    }
    pub fn get_found(& self) -> i32 {
        self.found.load(Ordering::SeqCst)
    }
    pub fn get_visible(& self) -> i32 {
        self.visible.load(Ordering::SeqCst)
    }

    //** Observations */////////////////////////////////////////////////////////////////////////////////
    pub fn get_observations(&self) -> &BTreeMap<Id, (i32, i32)> { &self.observations }
    pub fn get_index_in_keyframe(&self, kf_id: Id) -> (i32, i32) { 
        match self.observations.get(&kf_id) {
            Some((left, right)) => (*left, *right),
            None => (-1, -1)
        }
    }
    pub(super) fn delete_observation(&mut self, kf_id: &Id) -> bool {
        let _span = tracy_client::span!("delete_observation");

        // void MapPoint::EraseObservation(KeyFrame* pKF)
        if let Some((left_index, right_index)) = self.observations.get(kf_id) {
            if *left_index != -1 {
                // TODO (Stereo)
                // if(!pKF->mpCamera2 && pKF->mvuRight[leftIndex]>=0)
                //     nObs-=2;
                // else
                //     nObs--;
                self.num_obs -= 1;
            }
            if *right_index != -1 {
                self.num_obs -= 1;
            }
            self.observations.remove(kf_id);

            // If only 2 observations or less, discard point
            if self.num_obs <= 2 {
                return true;
            }

            if self.ref_kf_id == *kf_id {
                self.ref_kf_id = *self.observations.iter().next().unwrap().0; // Set to first key in hashmap
            }
        } else {
            warn!("Deleting kf, has reference to this mappoint but this mappoint does not have reference to kf. KF id: {}, MP id: {}", kf_id, self.id);
        }
        return false;
    }

    pub(super) fn add_observation(&mut self, kf_id: &Id, num_keypoints_left_for_kf: u32, index: u32) {
        let (mut left_index, mut right_index) = match self.observations.get(kf_id) {
            Some((left, right)) => (*left, *right),
            None => (-1, -1)
        };

        match self.sensor.frame() {
            FrameSensor::Stereo => {
                if index >= num_keypoints_left_for_kf {
                    right_index = index as i32;
                } else {
                    left_index = index as i32;
                }
            },
            _ => left_index = index as i32
        }
        // TODO (Stereo)
        // if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
        //     nObs+=2;
        // else
        //     nObs++;
        self.num_obs += 1;

        self.observations.insert(*kf_id, (left_index, right_index));
    }

    fn get_obs_normal(&self, map: &Map, position: &DVVector3<f64>) -> (i32, nalgebra::Vector3<f64>) { 
        let mut normal = na::Vector3::<f64>::zeros();
        let mut n = 0;
        let position_opencv = **position;
        for (id, _) in &self.observations {
            if let Some(kf) = map.keyframes.get(&id) {
                let mut camera_center = kf.get_camera_center();
                let owi = *camera_center;
                let normali = position_opencv - owi;
                normal = normal + normali / normali.norm();
                n += 1;

                match self.sensor.frame() {
                    FrameSensor::Stereo => {
                        camera_center = kf.get_right_camera_center();
                        let owi = *camera_center;
                        let normali = position_opencv - owi;
                        normal = normal + normali / normali.norm();
                        n += 1;
                    },
                    _ => {}
                }
            } else {
                error!("Mappoint {} has observation of keyframe {} but it is not in the map", self.id, id);
            }
        }
        (n, normal)
    }

    fn compute_descriptors(&self, map: &Map) -> Vec::<opencv::core::Mat> {
        let mut descriptors = Vec::<opencv::core::Mat>::new();
        for (id, (index1, index2)) in &self.observations {
            match map.keyframes.get(&id) {
                Some(kf) => {
                    descriptors.push((*kf.features.descriptors.row(*index1 as u32)).clone());
                    match self.sensor.frame() {
                        FrameSensor::Stereo => descriptors.push((*kf.features.descriptors.row(*index2 as u32)).clone()),
                        _ => {}
                    }

                },
                None => {
                    error!("Mappoint {} has observation of keyframe {} but it is not in the map", self.id, id);
                }

            };
        }
        descriptors
    }

    fn get_level(&self, kf: &KeyFrame) -> i32 {
        let (left_index, right_index) = self.observations.get(&kf.id).unwrap();
        if *left_index != -1 {
            kf.features.get_octave(*left_index as usize)
        } else {
            kf.features.get_octave((right_index - kf.features.num_keypoints as i32) as usize)
        }
    }
}


impl Clone for MapPoint {
    fn clone(&self) -> Self {
        Self {
            position: self.position.clone(),
            origin_map_id: self.origin_map_id,
            ref_kf_id: self.ref_kf_id,
            first_kf_id: self.first_kf_id,
            id: self.id,
            observations: self.observations.clone(),
            normal_vector: self.normal_vector.clone(),
            max_distance: self.max_distance,
            min_distance: self.min_distance,
            best_descriptor: self.best_descriptor.clone(),
            num_obs: self.num_obs,
            found: AtomicI32::new(self.found.load(Ordering::Relaxed)),
            visible: AtomicI32::new(self.visible.load(Ordering::Relaxed)),
            sensor: self.sensor.clone(),
            gba_pose: self.gba_pose.clone(),
            ba_global_for_kf: self.ba_global_for_kf,
            corrected_reference: self.corrected_reference
        }
    }
}
