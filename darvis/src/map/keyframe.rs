use std::{cmp::min, collections::{HashMap, HashSet}};
use core::{config::{ SETTINGS, SYSTEM}, matrix::{DVMatrix3, DVVector3}, sensor::Sensor, system::Timestamp};
use log::{error, warn};
use crate::{map::{map::Id, pose::Pose},modules::{bow::DVBoW, imu::*}, registered_actors::VOCABULARY_MODULE};
use super::{features::Features, frame::Frame, map::{Map, MapItems}, mappoint::MapPoint,};
use crate::modules::module_definitions::VocabularyModule;

#[derive(Debug, Clone)]
pub struct KeyFrame {
    pub id: Id,
    pub timestamp: Timestamp,
    pose: Pose,
    pub frame_id: i32,

    // Map connections
    pub origin_map_id: Id, // mnOriginMapId
    connections: ConnectedKeyFrames, // Parent, children, neighbors
    pub(super) mappoint_matches: MapPointMatches, // mvpmappoints , mvbOutlier
    pub ref_kf_id: Option<Id>, //mpReferenceKF
    pub parent: Option<Id>,
    pub children: HashSet<Id>,
    pub(super) loop_edges: HashSet<Id>, // mvpLoopEdges
    pub prev_kf_id: Option<Id>, // mpPrevKF
    pub next_kf_id: Option<Id>, // mpNextKF
    // pub pose_relative_to_parent: Option<Pose>, // mTcp. Pose relative to parent (this is computed when KF is deleted)

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: Option<DVBoW>,

    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub imu_data: ImuDataFrame,
    pub imu_position: Option<DVVector3<f64>>, // mOwb
    pub sensor: Sensor,

    // todo (design, fine-grained locking) I would like to get rid of this but until we have fine-grained locking this is the only way to prevent deletion without taking the entire map
    pub dont_delete: bool, // mbNotErase

    // todo (design, variable locations) loop closing can we avoid having this in the kf?
    //... possibly not, used by imu initialization (local mapping) as well
    pub ba_global_for_kf: Id, // mnBAGlobalForKF
    pub tcw_bef_gba: Option<Pose>, // mTcwBefGBA
    pub gba_pose: Option<Pose>, // mTcwGBA 
    pub bias_gba: Option<ImuBias>, // mBiasGBA
    pub vwb_gba: Option<DVVector3<f64>>, // mVwbGBA
    pub vwb_bef_gba: Option<DVVector3<f64>>, // mVwbBefGBA

    // DON'T SET THESE! Either never used, or moved into actor implementation
    // mbCurrentPlaceRecognition
    // // Variables used by loop closing
    // pub mnBAGlobalForKF: u64,
}
impl KeyFrame {
    pub fn new(frame: Frame, origin_map_id: Id, id: Id, map_imu_initialized: bool) -> Self {
        let bow = match frame.bow {
            Some(bow) => Some(bow),
            None => {
                let mut bow = DVBoW::new();
                VOCABULARY_MODULE.transform(&frame.features.descriptors, &mut bow);
                Some(bow)
            }
        };

        let mut imu_data = frame.imu_data;
        imu_data.is_imu_initialized = map_imu_initialized;

        let mut kf = Self {
            timestamp: frame.timestamp,
            frame_id: frame.frame_id,
            mappoint_matches: frame.mappoint_matches,
            pose: frame.pose.expect("Frame should have pose by now"),
            gba_pose: None,
            features: frame.features,
            bow,
            id,
            origin_map_id,
            connections: ConnectedKeyFrames::new(),
            ref_kf_id: frame.ref_kf_id,
            sensor: SETTINGS.get::<Sensor>(SYSTEM, "sensor"),
            parent: None,
            children: HashSet::new(),
            loop_edges: HashSet::new(),
            dont_delete: false,
            ba_global_for_kf: -1,
            prev_kf_id: None,
            next_kf_id: None,
            imu_data,
            bias_gba: None,
            vwb_gba: None,
            imu_position: None,
            vwb_bef_gba: None,
            tcw_bef_gba: None,
        };

        kf.set_pose(frame.pose.expect("Frame should have pose by now"));
        kf
    }

    pub fn _print_mappoints(&self) {
        println!("Mappoints for keyframe {}", self.id);
        for item in &self.mappoint_matches.matches {
            if let Some((mp_id, _outlier)) = item {
                println!("{}", mp_id);
            } else {
                println!("None");
            }
        }
        println!();
    }

    // I'm trying to keep as much data public as possible unless updating it requires a complicated function that needs to be called each time.
    // In that case we'd rather call these specific functions each time so the data doesn't get updated incorrectly.
    // This is why connections and mappoint matches are private with getters and setters. But I don't like how this looks, either.
    pub fn get_pose(&self) -> Pose { self.pose }
    pub fn set_pose(&mut self, new_pose: Pose) {
        self.pose = new_pose;

        if self.sensor.is_imu() {
            let pose_inverse = self.pose.inverse();
            self.imu_position = Some(
                DVVector3::new(
                    *pose_inverse.get_rotation() * *ImuCalib::new().tcb.get_translation() + *pose_inverse.get_translation()
                )
            );
        }
    }

    pub fn get_connected_kf_weight(&self, kf_id: Id) -> i32{ self.connections.get_weight(&kf_id) }
    pub fn add_connection(&mut self, kf_id: Id, weight: i32) { self.connections.add(&kf_id, weight); }
    pub fn add_all_connections(&mut self, new_connections: HashMap::<Id, i32>, is_init_kf: bool) -> Option<Id> { 
        self.connections.add_all(new_connections);
        if self.parent.is_none() && !is_init_kf { 
            let new_parent = Some(self.connections.first_connected_kf());
            self.parent = new_parent;
            new_parent
        } else {
            None
        }
    }
    pub fn delete_connection(&mut self, kf_id: Id) { self.connections.delete(&kf_id); }
    pub fn get_loop_edges(&self) -> &HashSet<Id> { &self.loop_edges }

    pub fn clone_matches(&self) -> MapPointMatches { self.mappoint_matches.clone() }
    pub fn get_mp_matches(&self) -> &Vec<Option<(i32, bool)>> { &self.mappoint_matches.matches }
    pub fn get_mp_match(&self, index: &u32) -> Option<(Id, bool)> { self.mappoint_matches.get(*index as usize) }
    pub fn get_mp_match_index(&self, id: &Id) -> Option<usize> { 
        self.mappoint_matches.matches.iter().position(|item| item.is_some() && item.unwrap().0 == *id)
    }
    pub fn _mp_match_len(&self) -> usize { self.mappoint_matches.matches.iter().filter(|m| m.is_some()).count() }

    pub fn get_tracked_mappoints(&self, map: &Map, min_observations: u32) -> i32 { self.mappoint_matches.tracked_mappoints(map, min_observations) }
    pub fn _debug_get_mps_count(&self) -> i32 { self.mappoint_matches.debug_count }

    pub fn get_camera_center(&self) -> DVVector3<f64> {
        self.pose.inverse().get_translation()
        // Note: In Orbslam, this is: mTwc.translation()
        // and mTwc is inverse of the pose
    }

    pub fn get_right_camera_center(&self) -> DVVector3<f64> {
        todo!("Stereo");
        // Not sure what mTlr is, it comes from the settings but might get updated somewhere.
        //    return (mTwc * mTlr).translation();
        // this needs to be generic on sensor, so it can't be called if the sensor doesn't have a right camera
    }

    pub fn compute_scene_median_depth(&self, mappoints: &MapItems<MapPoint>, q: i32) -> f64 {
        if self.features.num_keypoints == 0 {
            return -1.0;
        }

        let mut depths = Vec::new();
        let rot = self.pose.get_rotation();
        let rcw2 = rot.row(2);
        let zcw = self.pose.get_translation()[2];

        for index in 0..self.mappoint_matches.matches.len() {
            if let Some((mp_id, _)) = self.mappoint_matches.matches[index as usize] {
                if mappoints.get(&mp_id).is_none() {
                    error!("Map point not found in map. This shouldn't happen!! KF ID = {}, MP ID = {}", self.id, mp_id);
                    continue;
                }
                let world_pos = *(mappoints.get(&mp_id).unwrap().position);
                let z = (rcw2 * world_pos)[0] + zcw; // first part of this term is scalar but still need to get it from Matrix<1,1> to f64
                depths.push(z);
            }
        }

        depths.sort_by(|a, b| a.total_cmp(&b));
        depths[(depths.len()-1) / q as usize]
    }

    pub fn get_right_pose(&self) -> Pose {
        todo!("Stereo");
        // Sophus::SE3<float> KeyFrame::GetRightPose() {
        //     unique_lock<mutex> lock(mMutexPose);

        //     return mTrl * mTcw;
        // }
    }

    // * CONNECTED KEYFRAMES *//
    pub fn get_covisibility_keyframes(&self, num: i32) -> Vec<Id> {
        // vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames(), KeyFrame::GetBestCovisibilityKeyFrames
        // To get all connections, pass in i32::MAX as `num`
        // num is the target number of keyframes to return
        let max_len = min(self.connections.ordered_connected_keyframes.len(), num as usize);

        let (connections, _) : (Vec<i32>, Vec<i32>) = self.connections.ordered_connected_keyframes[0..max_len].iter().cloned().unzip();
        connections
    }

    pub fn get_covisibles_by_weight(&self, weight: i32) -> Vec<Id> {
        // vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
        // Like get_covisibility_keyframes, but instead of returning the best x keyframes,
        // it returns all keyframes with a weight > x
        self.connections.ordered_connected_keyframes.iter()
            .filter(|(_, w)| w > &weight)
            .map(|(kf_id, _)| *kf_id)
            .collect()
    }

    pub fn get_connected_keyframes(&self) -> &HashMap<Id, i32> {
        // set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
        // Connected/covisible keyframes sorted by weight
        &self.connections.map_connected_keyframes
    }

    // *IMU *//
    pub fn get_imu_rotation(&self) -> DVMatrix3<f64> {
        // Eigen::Matrix3f KeyFrame::GetImuRotation()
        // Note: in Orbslam this is: (mTwc * mImuCalib.mTcb).rotationMatrix();
        // and mTwc is inverse of the pose
        DVMatrix3::new(*self.pose.inverse().get_rotation() * *ImuCalib::new().tcb.get_rotation())
    }
    pub fn get_imu_position(&self) -> DVVector3<f64> {
        // Eigen::Vector3f KeyFrame::GetImuPosition()
        self.imu_position.expect("IMU position not set")
    }
    pub fn get_imu_pose(&self) -> Pose {
        // Sophus::SE3f KeyFrame::GetImuPose()
        self.pose.inverse() * ImuCalib::new().tcb
    }

}


#[derive(Debug, Clone)]
pub struct MapPointMatches {
    // Mappoints //
    // Note: Id is mappoint Id, bool is if it's an oultier
    pub matches: Vec<Option<(Id, bool)>>,
    pub debug_count: i32, // keep track of count for debugging
}
impl MapPointMatches {
    pub fn new(num_keypoints: usize) -> Self {
        Self {
            matches: vec![None; num_keypoints],
            debug_count: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.matches.len()
    }

    pub fn get(&self, index: usize) -> Option<(Id, bool)> {
        self.matches[index]
    }

    pub fn add(&mut self, index: u32, mp_id: Id, is_outlier: bool) -> Option<Id> {
        let old_mp = self.matches[index as usize];
        self.matches[index as usize] = Some((mp_id, is_outlier));
        self.debug_count += 1;
        match old_mp {
            Some((old_mp_id, _)) => Some(old_mp_id),
            None => None
        }
    }

    pub fn delete_at_indices(&mut self, indices: (i32, i32)) -> (Option<(Id, bool)>, Option<(Id, bool)>) {
        // Indices are (left, right). Right should be -1 for mono. Maybe we can rewrite this to make it more clear?
        // TODO (mvp): removed the code that set's mappoint's last_frame_seen to the frame ID. Is that ok or does it need to be in here?
        let (mut first_mp_id, mut second_mp_id) = (None, None);
        if indices.0 != -1 {
            first_mp_id = self.matches[indices.0 as usize];
            self.matches[indices.0 as usize] = None;
            self.debug_count -= 1;
        }
        if indices.1 != -1 {
            second_mp_id = self.matches[indices.1 as usize];
            self.matches[indices.1 as usize]= None;
            self.debug_count -= 1;
        }
        (first_mp_id, second_mp_id)
    }

    pub fn clear(&mut self) {
        self.matches = vec![None; self.matches.len()];
        self.debug_count = 0;
    }

    pub fn is_outlier(&self, index: &u32) -> bool {
        self.matches[*index as usize].unwrap().1
    }

    pub fn set_outlier(&mut self, index: usize, is_outlier: bool) {
        // need as_mut().unwrap() or it won't actually change the internal value!!!
        self.matches[index].as_mut().unwrap().1 = is_outlier;
    }

    pub fn _mappoints_with_observations(&self, map: &Map) -> i32 {
        (&self.matches)
        .into_iter()
        .filter(|item| {
            match item {
                Some((mp_id, _)) => {
                    match map.mappoints.get(mp_id) {
                        Some(mp) => mp.get_observations().len() > 0,
                        None => false
                    }
                },
                None => false
            }
        })
        .count() as i32
    }

    pub fn tracked_mappoints(&self, map: &Map, min_observations: u32) -> i32 {
        // KeyFrame::TrackedMapPoints(const int &minObs)
        // To get all mappoints with any observations at all, pass in 1 as `min_observations`
        let check_obs = min_observations > 0;
        (&self.matches)
        .into_iter()
        .filter(|item| {
            match item {
                Some((mp_id, _)) => {
                    match map.mappoints.get(mp_id) {
                        Some(mappoint) => {
                            if check_obs {
                                mappoint.num_obs >= (min_observations as i32)
                            } else {
                                true
                            }
                        },
                        None => {
                            warn!("Mappoint {} not in map", mp_id);
                            false
                        }
                    }
                },
                None => false
            }
        })
        .count() as i32
    }


}

#[derive(Debug, Clone)]
pub struct ConnectedKeyFrames {
    // Connected Keyframes... also sometimes called covisibility keyframes in ORBSLAM3
    // Note: Two ways of storing this data ...
    // `ordered` is an ordered list of kf IDs based on weight, for fast lookup of the top N connected KFs
    // `map` is a hashmap of kf id to weight, for fast lookup of a kf's weight
    // This is essentially the setup in ORBSLAM3, with the additional optimization that
    // `ordered` is only sorted when a KF with a weight > cutoff_weight is inserted
    // (because `ordered` is only ever used for the top 30 keyframes in the vector)
    ordered_connected_keyframes: Vec<(Id, i32)>, // mvpOrderedConnectedKeyFrames and mvOrderedWeights
    map_connected_keyframes: HashMap<Id, i32>, // mConnectedKeyFrameWeights
    connected_keyframes_cutoff_weight: i32,
}

impl ConnectedKeyFrames {
    fn new() -> Self {
        Self {
            ordered_connected_keyframes: Vec::new(),
            map_connected_keyframes: HashMap::new(),
            connected_keyframes_cutoff_weight: 0,
        }
    }

    fn add(&mut self, kf_id: &Id, weight: i32) {
        match self.map_connected_keyframes.contains_key(kf_id) {
            true => {
                self.map_connected_keyframes.insert(*kf_id, weight); // updates value in hashmap
            },
            false => {
                self.map_connected_keyframes.insert(*kf_id, weight);
                self.ordered_connected_keyframes.push((*kf_id, weight));
            }
        }

        //TODO (mvp)...verify sorting in ConnectedKeyFrames, might be backwards. Not quite clear whether low weight = earlier index or vice versa
        if weight > self.connected_keyframes_cutoff_weight { self.sort_ordered(); }
    }

    fn delete(&mut self, kf_id: &Id) {
        if self.map_connected_keyframes.contains_key(kf_id) {
            self.map_connected_keyframes.remove(kf_id);
            self.ordered_connected_keyframes.retain(|(key, _)| key != kf_id);
            self.sort_ordered();
        }
    }

    fn add_all(&mut self, new_connections: HashMap::<Id, i32>) {
        // Turn hashmap into vector and sort by weights
        self.ordered_connected_keyframes = new_connections.iter()
            .map(|(key, value)| { (*key, *value) })
            .collect::<Vec<(Id, i32)>>(); 
        self.sort_ordered();

        self.map_connected_keyframes = new_connections;
    }

    fn first_connected_kf(&self) -> Id {
        self.ordered_connected_keyframes[0].0
    }

    fn sort_ordered(&mut self) {
        //KeyFrame::UpdateBestCovisibles
        self.ordered_connected_keyframes.sort_by(|(_,w1), (_,w2)| w2.cmp(&w1));
        let max_len = self.ordered_connected_keyframes.len(); 
        if max_len > 30 {
            self.connected_keyframes_cutoff_weight = self.ordered_connected_keyframes[30].1;
        } else {
            self.connected_keyframes_cutoff_weight = 0;
        }
    }

    fn get_weight(&self, other_kf_id: &Id) -> i32 {
        // int KeyFrame::GetWeight(KeyFrame *pKF)
        if self.map_connected_keyframes.contains_key(other_kf_id) {
            return *self.map_connected_keyframes.get(other_kf_id).unwrap();
        } else {
            return 0;
        }
    }
}