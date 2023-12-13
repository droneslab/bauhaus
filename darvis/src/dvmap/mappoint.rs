use std::{fmt::Debug, collections::HashMap, sync::atomic::{AtomicI32, Ordering}};
use dvcore::{matrix::DVMatrix, config::{SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor}};
use log::error;
extern crate nalgebra as na;
use crate::{matrix::DVVector3, modules::orbmatcher::{descriptor_distance, SCALE_FACTORS}, registered_actors::FEATURE_DETECTION};
use super::{map::{Id, Map}, keyframe::{Frame, KeyFrame}, pose::DVTranslation};

// Paper note: Implementing typestate like here: http://cliffle.com/blog/rust-typestate/#a-simple-example-the-living-and-the-dead
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
#[derive(Debug)]
pub struct MapPoint<M: MapPointState> {
    pub position: DVTranslation, 

    // Map connections
    // TODO (design): it would be nice if we could guarantee that the connections are updated/correct
    // rather than duplicating all these connections across all the objects and hoping we remember
    // to update them correctly after a map modification
    pub origin_map_id: Id,
    ref_kf_id: Id, // mpRefKF
    pub first_kf_id: Id, // mnFirstKFid

    // Variables in ORBSLAM, DON'T Set these!!
    // mnFirstFrame ... literally never used meaningfully
    // mnLastFrameSeen ... similar to "mnTrackReferenceForFrame" in KeyFrame. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
    // mbTrackInView ... only used by tracking to keep track of which mappoints to show. Just keep this data saved in tracking locally

    // Typestate...This reassures the compiler that the parameter gets used.
    full_mp_info: M,


    // Set in tracking, used by local mapping to make decision about deleting keyframe
    found: AtomicI32, //nFound
    visible: AtomicI32, // nVisible

    sensor: Sensor
}

// Typestate...State options.
#[derive(Clone, Debug)]
pub struct PrelimMapPoint {} // prelimary map item created locally but not inserted into the map yet
#[derive(Clone, Debug)]
pub struct FullMapPoint { // Full map item inserted into the map with the following additional fields
    id: Id,

    // Observations, this is a part of "map connections" but I don't think we can avoid keeping this here.
    observations: HashMap<Id, (i32, i32)>, // mObservations ; Keyframes observing the point and associated index in keyframe
    num_obs: i32,

    // Best descriptor used for fast matching
    best_descriptor: DVMatrix,

    // Variables used by merging
    normal_vector: DVVector3<f64>,  // mNormalVector ; Mean viewing direction

    // Scale invariance distances
    max_distance: f64,
    min_distance: f64,
}

pub trait MapPointState {}
impl MapPointState for PrelimMapPoint {}
impl MapPointState for FullMapPoint {}

impl MapPoint<PrelimMapPoint> {
    pub fn new(position: DVVector3<f64>, ref_kf_id: Id, origin_map_id: Id) -> Self {
        Self {
            position,
            origin_map_id,
            ref_kf_id,
            first_kf_id: ref_kf_id,
            full_mp_info: PrelimMapPoint{},
            found: AtomicI32::new(1),
            visible: AtomicI32::new(1),
            sensor: SETTINGS.get(SYSTEM, "sensor")
        }
    }
}

impl MapPoint<FullMapPoint> {
    pub(super) fn new(prelim_mappoint: MapPoint<PrelimMapPoint>, id: Id) -> Self {
        Self {
            position: prelim_mappoint.position,
            origin_map_id: prelim_mappoint.origin_map_id,
            ref_kf_id: prelim_mappoint.ref_kf_id,
            first_kf_id: prelim_mappoint.first_kf_id,
            full_mp_info: FullMapPoint{
                id,
                observations: HashMap::new(),
                normal_vector: DVVector3::zeros::<f64>(),
                max_distance: 0.0,
                min_distance: 0.0,
                best_descriptor: DVMatrix::empty(),
                num_obs: 0
            },
            found: AtomicI32::new(1),
            visible: AtomicI32::new(1),
            sensor: prelim_mappoint.sensor
        }
    }

    pub fn get_id(&self) -> Id { self.full_mp_info.id }
    pub fn get_observations(&self) -> &HashMap<Id, (i32, i32)> { &self.full_mp_info.observations }
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

    pub fn is_in_keyframe(&self, kf_id: Id) -> bool {
        self.full_mp_info.observations.contains_key(&kf_id)
    }

    pub fn predict_scale(&self, current_distance: &f64) -> i32 {
        // int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
        let ratio = self.full_mp_info.max_distance / current_distance;
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
        if self.full_mp_info.observations.is_empty() {
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

    pub(super) fn compute_distinctive_descriptors(&self, map: &Map) -> Option<DVMatrix> {
        // Part 1 of void MapPoint::ComputeDistinctiveDescriptors()
        if self.full_mp_info.num_obs == 0 {
            error!("mappoint::compute_distinctive_descriptors;No observations");
            return None;
        }

        let descriptors = self.compute_descriptors(map);
        if descriptors.len() == 0 { 
            error!("mappoint::compute_distinctive_descriptors;No descriptors");
            return None;
         }

        // Compute distances between them
        let mut distances = HashMap::new();
        for i in 0..descriptors.len() {
            distances.insert((i,i), 0);
            for j in i+1..descriptors.len() {
                let dist_ij = descriptor_distance(&descriptors[i], &descriptors[j]);
                distances.insert((i,j), dist_ij);
                distances.insert((j,i), dist_ij);
            }
        }

        // Take the descriptor with least median distance to the rest
        let mut best_median = std::i32::MAX;
        let mut best_idx = 0;

        let num_descriptors = descriptors.len();
        for i in 0..num_descriptors {
            let mut v_descriptors = Vec::new();
            for (key, dist) in &distances {
                if key.0 == i || key.1 == i {
                    v_descriptors.push(dist);
                }
            }
            v_descriptors.sort();
            let median = v_descriptors[(v_descriptors.len()/2) as usize];
            if median < &best_median {
                best_median = *median;
                best_idx = i;
            }
        }

        // TODO (timing) ... this clone might take a while
        Some(DVMatrix::new(descriptors[best_idx].clone()))
    }

    pub fn update_distinctive_descriptors(&mut self, desc: DVMatrix) {
        // Part 2 of void MapPoint::ComputeDistinctiveDescriptors()
        self.full_mp_info.best_descriptor = desc;
    }

    pub fn increase_found(& self) {
        self.found.fetch_add(1, Ordering::SeqCst);
    }
    pub fn increase_visible(& self) {
        self.visible.fetch_add(1, Ordering::SeqCst);
    }
    pub fn get_found_ratio(&self) -> f32 {
        // println!("mp_id {}, found {:?}, visible {:?}, ratio {:?}", mp_id, self.found.get(mp_id), self.visible.get(mp_id), *self.found.get(mp_id)? as f32 / *self.visible.get(mp_id)? as f32);
        self.found.load(Ordering::SeqCst) as f32 / self.visible.load(Ordering::SeqCst) as f32
    }

    //** Observations */////////////////////////////////////////////////////////////////////////////////
    pub fn erase_observation(&mut self, kf_id: &Id) -> bool {
        if let Some((left_index, right_index)) = self.full_mp_info.observations.get(kf_id) {
            if *left_index != -1 {
                // TODO (Stereo)
                // if(!pKF->mpCamera2 && pKF->mvuRight[leftIndex]>=0)
                //     nObs-=2;
                // else
                //     nObs--;
                self.full_mp_info.num_obs -= 1;
            }
            if *right_index != -1 {
                self.full_mp_info.num_obs -= 1;
            }
            self.full_mp_info.observations.remove(kf_id);

            if self.ref_kf_id == *kf_id {
                self.ref_kf_id = *self.full_mp_info.observations.iter().next().unwrap().0; // Set to first key in hashmap
            }

            // If only 2 observations or less, discard point
            if self.full_mp_info.num_obs <= 2 {
                return true;
            }
        }
        return false;
    }

    pub fn add_observation(&mut self, kf_id: &Id, num_keypoints_left_for_kf: u32, index: u32) {
        let (mut left_index, mut right_index) = match self.full_mp_info.observations.get(kf_id) {
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
        self.full_mp_info.num_obs += 1;

        self.full_mp_info.observations.insert(*kf_id, (left_index, right_index));
    }

    fn get_obs_normal(&self, map: &Map, position: &DVVector3<f64>) -> (i32, nalgebra::Vector3<f64>) { 
        let mut normal = na::Vector3::<f64>::zeros();
        let mut n = 0;
        let position_opencv = **position;
        for (id, _) in &self.full_mp_info.observations {
            let kf = map.keyframes.get(&id).unwrap();
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
        }
        (n, normal)
    }

    fn compute_descriptors(&self, map: &Map) -> Vec::<opencv::core::Mat> {
        let mut descriptors = Vec::<opencv::core::Mat>::new();
        for (id, (index1, index2)) in &self.full_mp_info.observations {
            let kf = map.keyframes.get(&id).unwrap();
            descriptors.push(kf.features.descriptors.row(*index1 as u32).unwrap());
            match self.sensor.frame() {
                FrameSensor::Stereo => descriptors.push(kf.features.descriptors.row(*index2 as u32).unwrap()),
                _ => {}
            }
        }
        descriptors
    }

    fn get_level(&self, kf: &KeyFrame) -> i32 {
        let (left_index, right_index) = self.full_mp_info.observations.get(&kf.id).unwrap();
        if *left_index != -1 {
            kf.features.get_octave(*left_index as usize)
        } else {
            kf.features.get_octave((right_index - kf.features.num_keypoints as i32) as usize)
        }
    }
}


impl<M> Clone for MapPoint<M> where M: MapPointState + Clone {
    fn clone(&self) -> Self {
        Self {
            position: self.position.clone(),
            origin_map_id: self.origin_map_id,
            ref_kf_id: self.ref_kf_id,
            first_kf_id: self.first_kf_id,
            full_mp_info: self.full_mp_info.clone(),
            found: AtomicI32::new(self.found.load(Ordering::Relaxed)),
            visible: AtomicI32::new(self.visible.load(Ordering::Relaxed)),
            sensor: self.sensor.clone(),
        }
    }
}
