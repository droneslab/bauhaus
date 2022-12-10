use std::{collections::{HashMap}, cmp::min};
use chrono::{DateTime, Utc};
use derivative::Derivative;
use dvcore::{matrix::{DVVector3}};
use log::{error, info, debug};
use serde::{Deserialize, Serialize};
use crate::{dvmap::{map::Id, pose::Pose, frame::*},modules::{imu::*},};
use super::{mappoint::{MapPoint, FullMapPoint}, map::{Map}, features::Features, bow::{BoW, self}};

// Typestate...Keyframe information that is ALWAYS available, regardless of keyframe state.
// Uncomment this if doing serialization/deserialization: unsafe impl<S: SensorType> Sync for KeyFrame<S> {}
#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct KeyFrame<K: KeyFrameState> {
    pub timestamp: DateTime<Utc>,
    pub frame_id: Id, // Id of frame it is based on
    pub pose: Pose,

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: Option<BoW>,

    // Mappoints
    // Note: u32 is index in array, Id is mappoint Id ... equal to vector in ORBSLAM3
    // because it just allocates an N-size vector and has a bunch of empty entries
    pub mappoint_matches: HashMap::<u32, (Id, bool)>, // mvpmappoints 

    // scale: f64,
    // depth_threshold: f64,

    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub imu_bias: Option<IMUBias>,
    pub imu_preintegrated: Option<IMUPreIntegrated>,
    // pub imu_calib: IMUCalib,

    // Stereo //
    pub stereo_baseline: f64,

    pub full_kf_info: K,

    // Don't add these in!! read explanations below
    // mnTrackReferenceForFrame ... used in tracking to decide whether to add a kf/mp into tracking's local map. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
}

// Typestate...State options.
#[derive(Clone, Debug, Default, Copy, Serialize, Deserialize)]
pub struct PrelimKeyFrame {} // prelimary map item created locally but not inserted into the map yet
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct FullKeyFrame { // Full map item inserted into the map with the following additional fields
    pub id: Id,
    pub origin_map_id: Id, // mnOriginMapId

    // Map connections ... Parent, children, neighbors
    pub parent: Option<Id>,
    pub children: Vec<Id>,
    connected_keyframes: ConnectedKeyFrames,// also sometimes called covisibility keyframes in ORBSLAM3

    // Sofiya: I think we can clean this up and get rid of these
    // Variables used by KF database
    // pub loop_query: u64, //mnLoopQuery
    // pub loop_words: i32, //mnLoopWords
    // pub reloc_query: u64, //mnRelocQuery
    // pub reloc_words: i32, //mnRelocWords
    // pub merge_query: u64, //mnMergeQuery
    // pub merge_words: i32, //mnMergeWords
    // pub place_recognition_query: u64, //mnPlaceRecognitionQuery
    // pub place_recognition_words: i32, //mnPlaceRecognitionWords
    // pub place_recognition_score: f32, //mPlaceRecognitionScore
    // // Variables used by loop closing
    // pub mnBAGlobalForKF: u64,
    // // Variables used by merging
    // pub mnMergeCorrectedForKF: u64,
    // pub mnBALocalForMerge: u64,
}

pub trait KeyFrameState: Send + Sync {}
impl KeyFrameState for PrelimKeyFrame {}
impl KeyFrameState for FullKeyFrame {}

impl KeyFrame<PrelimKeyFrame> {
    pub fn new(frame: &Frame) -> KeyFrame<PrelimKeyFrame> {
        KeyFrame {
            timestamp: frame.timestamp,
            frame_id: frame.id,
            mappoint_matches: frame.mappoint_matches.clone(),
            pose: frame.pose.unwrap(),
            features: frame.features.clone(),
            imu_bias: frame.imu_bias,
            imu_preintegrated: frame.imu_preintegrated,
            stereo_baseline: 0.0, // TODO (Stereo)
            full_kf_info: PrelimKeyFrame{},
            bow: frame.bow.clone()
        }
    }
}

impl KeyFrame<FullKeyFrame> {
    pub(super) fn new(prelim_keyframe: &KeyFrame<PrelimKeyFrame>, origin_map_id: Id, id: Id) -> Self {
        let bow = match &prelim_keyframe.bow {
            Some(bow) => Some(bow.clone()),
            None => {
                let mut bow = BoW::new();
                //debug!("mbowvector for keyframe {} with frame id {}", id, prelim_keyframe.frame_id);
                bow::VOCABULARY.transform(&prelim_keyframe.features.descriptors, &mut bow);
                Some(bow)
            }
        };


        Self {
            timestamp: prelim_keyframe.timestamp,
            frame_id: prelim_keyframe.frame_id,
            mappoint_matches: prelim_keyframe.mappoint_matches.clone(),
            pose: prelim_keyframe.pose,
            features: prelim_keyframe.features.clone(),
            bow,
            imu_bias: prelim_keyframe.imu_bias,
            imu_preintegrated: prelim_keyframe.imu_preintegrated,
            stereo_baseline: prelim_keyframe.stereo_baseline,
            full_kf_info: FullKeyFrame{
                id,
                origin_map_id,
                ..Default::default()
            },
        }
    }

    pub fn id(&self) -> Id { self.full_kf_info.id }

    pub fn get_mappoint(&self, index: &u32) -> Id {
        self.mappoint_matches.get(index).unwrap().0
    }
    pub fn has_mappoint(&self, index: &u32) -> bool {
        self.mappoint_matches.get(index).is_some()
    }

    pub fn add_mappoint(&mut self, mp: &MapPoint<FullMapPoint>, index: u32, is_outlier: bool) {
        // KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
        self.mappoint_matches.insert(index, (mp.get_id(), is_outlier));
    }

    pub fn erase_mappoint_match(&mut self, (left_index, right_index): (i32, i32)) {
        // self.mappoint_matches.remove(id);
        if left_index != -1 {
            self.mappoint_matches.remove(&(left_index as u32));
        }
        if right_index != -1 {
            self.mappoint_matches.remove(&(right_index as u32));
        }
    }

    pub fn add_connection(&mut self, kf_id: &Id, weight: i32) {
        self.full_kf_info.connected_keyframes.add_connection(kf_id, weight);
    }

    pub fn get_connections(&self, num: i32) -> Vec<Id> {
        self.full_kf_info.connected_keyframes.get_connections(num)
    }

    pub fn insert_all_connections(&mut self, new_connections: HashMap::<Id, i32>, is_init_kf: bool) -> Option<Id> {
        self.full_kf_info.connected_keyframes.insert_all_connections(new_connections);
        if self.full_kf_info.parent.is_none() && !is_init_kf { 
            let parent_id = self.full_kf_info.connected_keyframes.first();
            self.change_parent(parent_id);
            debug!("keyframe;inserted parent;{};for child;{}", parent_id, self.full_kf_info.id);
            Some(self.full_kf_info.connected_keyframes.first())
        } else {
            debug!("keyframe;not inserting parent for kf;{}", self.full_kf_info.id);
            None
        }
    }

    pub fn change_parent(&mut self, id: Id) {
        if id == self.full_kf_info.id {
            error!("keyframe::change_parent;parent and child are the same KF");
        }
        self.full_kf_info.parent = Some(id);
    }

    pub fn add_child(&mut self, id: Id) {
        self.full_kf_info.children.push(id);
    }

    pub fn get_camera_center(&self) -> DVVector3<f64> {
        self.pose.inverse().get_translation()
        // Note: In Orbslam, this is: mTwc.translation()
        // and mTwc is inverse of the pose
    }

    pub fn get_right_camera_center(&self) -> DVVector3<f64> {
        todo!("TODO (IMU)");
        // NOt sure what mTlr is, it comes from the settings but might get updated somewhere.
        //    return (mTwc * mTlr).translation();
        // this needs to be generic on sensor, so it can't be called if the sensor doesn't have a right camera
    }

    pub fn compute_scene_median_depth(&self, mappoints: &HashMap<Id, MapPoint<FullMapPoint>>, q: i32) -> f64 {
        if self.features.num_keypoints == 0 {
            return -1.0;
        }

        let mut depths = Vec::new();
        // depths.reserve(self.keypoints_data.num_keypoints as usize); // probably unnecessary?
        let rot = self.pose.get_rotation();
        let rcw2 = rot.row(2);
        let zcw = self.pose.get_translation()[2];

        for (_, (mp_id, _)) in &self.mappoint_matches {
            let world_pos = *(mappoints.get(mp_id).unwrap().position);
            let z = (rcw2 * world_pos)[0] + zcw; // first part of this term is scalar but still need to get it from Matrix<1,1> to f64
            depths.push(z);
        }

        depths.sort_by(|a, b| a.total_cmp(&b));
        depths[(depths.len()-1) / q as usize]
    }

    pub fn tracked_mappoints(&self, map: &Map, min_observations: u32) -> i32{
        // KeyFrame::TrackedMapPoints(const int &minObs)
        if min_observations > 0 {
            return self.mappoint_matches.len() as i32;
        }

        let mut num_points = 0;
        for (_, (mp_id, _)) in &self.mappoint_matches {
            let mappoint = map.get_mappoint(&mp_id).unwrap();
            if mappoint.get_observations().len() >= (min_observations as usize) {
                num_points += 1;
            }
        }

        num_points
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, Default)]
struct ConnectedKeyFrames {
    // Note: Two ways of storing this data ...
    // `ordered` is an ordered list of kf IDs based on weight, for fast lookup of the top N connected KFs
    // `map` is a hashmap of kf id to weight, for fast lookup of a kf's weight
    // This is essentially the setup in ORBSLAM3, with the additional optimization that
    // `ordered` is only sorted when a KF with a weight > cutoff_weight is inserted
    // (because `ordered` is only ever used for the top 30 keyframes in the vector)
    ordered: Vec<(Id, i32)>, // mvpOrderedConnectedKeyFrames and mvOrderedWeights
    map: HashMap<Id, i32>, // mConnectedKeyFrameWeights
    cutoff_weight: i32,
}
impl ConnectedKeyFrames {
    pub(super) fn add_connection(&mut self, kf_id: &Id, weight: i32) {
        *self.map.entry(*kf_id).or_insert(weight) = weight;
        // TODO (verify): Sorting might be backwards? Not quite clear whether low weight = earlier index or vice versa
        if weight > self.cutoff_weight { self.sort_ordered(); }
    }

    pub fn insert_all_connections(&mut self, new_connections: HashMap::<Id, i32>) {
        // Turn hashmap into vector and sort by weights
        self.ordered = new_connections.iter()
            .map(|(key, value)| { (key.clone(), value.clone()) })
            .collect::<Vec<(Id, i32)>>(); 
        self.sort_ordered();

        self.map = new_connections;
    }

    pub fn first(&self) -> Id {
        self.ordered[0].0 //.1
    }

    pub fn get_connections(&self, num: i32) -> Vec<Id> {
    
       let max_len = min(self.ordered.len(), num as usize);
       let (conections, _) : (Vec<i32>, Vec<i32>) = self.ordered[0..max_len].iter().cloned().unzip();
       info!("connections : {:?}", conections);
       conections
    }

    fn sort_ordered(&mut self) {
        // TODO (verify): Sorting might be backwards? Not quite clear whether low weight = earlier index or vice versa
        //KeyFrame::UpdateBestCovisibles
        self.ordered.sort_by(|(_,w1), (_,w2)| w2.cmp(&w1));
        let max_len = self.ordered.len(); 
        if max_len >= 30 {
            self.cutoff_weight = self.ordered[30].1;
        } else {
            self.cutoff_weight =0;
        }
    }
}