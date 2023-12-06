use std::{collections::{HashMap, HashSet}, cmp::min};
use dvcore::{matrix::{DVVector3, DVVectorOfKeyPoint, DVMatrix}, config::{ SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor}};
use log::{error, info, debug};
use crate::{dvmap::{map::Id, pose::DVPose},modules::{imu::*, camera::CAMERA_MODULE}, actors::tracking_backend::TrackedMapPointData,};
use super::{mappoint::{MapPoint, FullMapPoint}, map::{Map, MapItemHashMap}, features::Features, bow::{BoW, self}, misc::Timestamp};

#[derive(Debug, Clone)]
pub struct Frame {
    pub frame_id: Id,
    pub timestamp: Timestamp,

    pub mappoint_matches: MapPointMatches, // mvpmappoints , mvbOutlier

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: Option<BoW>,

    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub imu_bias: Option<IMUBias>,
    pub imu_preintegrated: Option<IMUPreIntegrated>,
    pub sensor: Sensor,

    pub pose: Option<DVPose>,
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // Don't add these in!! read explanations below
    // mnTrackReferenceForFrame ... used in tracking to decide whether to add a kf/mp into tracking's local map. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
}
impl Frame {
pub fn new(
        frame_id: Id, keypoints_vec: DVVectorOfKeyPoint, descriptors_vec: DVMatrix,
        im_width: u32, im_height: u32,
        timestamp: Timestamp
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let imu_bias = match sensor.is_imu() {
            true => todo!("IMU"),
            false => None
        };
        let features = Features::new(keypoints_vec, descriptors_vec, im_width, im_height, sensor)?;
        let num_keypoints = features.num_keypoints as usize;
        let frame = Self {
            frame_id,
            timestamp,
            features,
            imu_bias,
            sensor,
            bow: None,
            mappoint_matches: MapPointMatches::new(num_keypoints),
            imu_preintegrated: None,
            pose: None,
            ref_kf_id: None
        };
        Ok(frame)
    }

    pub fn new_clone(frame: &Frame) -> Frame {
        let bow = match &frame.bow {
            Some(bow) => Some(bow.clone()),
            None => {
                let mut bow = BoW::new();
                bow::VOCABULARY.transform(&frame.features.descriptors, &mut bow);
                Some(bow)
            }
        };

        Frame {
            timestamp: frame.timestamp,
            frame_id: frame.frame_id,
            mappoint_matches: frame.mappoint_matches.clone(),
            pose: frame.pose,
            features: frame.features.clone(),
            imu_bias: frame.imu_bias,
            imu_preintegrated: frame.imu_preintegrated,
            bow,
            ref_kf_id: frame.ref_kf_id,
            sensor: frame.sensor,
        }
    }

    pub fn compute_bow(&mut self) {
        if self.bow.is_none() {
            self.bow = Some(BoW::new());
            bow::VOCABULARY.transform(&self.features.descriptors, &mut self.bow.as_mut().unwrap());
        }
    }

    pub fn check_close_tracked_mappoints(&self) -> (i32, i32) {
        self.features.check_close_tracked_mappoints(CAMERA_MODULE.th_depth as f32, &self.mappoint_matches.matches)
    }

    pub fn is_in_frustum(&self, mappoint: &MapPoint<FullMapPoint>, viewing_cos_limit: f64) -> (Option<TrackedMapPointData>, Option<TrackedMapPointData>) {
        // Combination of:
        // bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
        // bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight)

        let left = self.check_frustum(viewing_cos_limit, &mappoint, false);
        let right = match self.sensor.frame() {
            FrameSensor::Stereo => { self.check_frustum(viewing_cos_limit, &mappoint, true) },
            _ => { None }
        };
        (left, right)
    }

        fn check_frustum(&self, viewing_cos_limit: f64, mappoint: &MapPoint<FullMapPoint>, is_right: bool) -> Option<TrackedMapPointData> {
        // 3D in absolute coordinates
        let pos = mappoint.position;

        // 3D in camera coordinates
        let (mr, mt, twc);
        if is_right {
            todo!("Stereo");
            // let rrl = mTrl.rotationMatrix();
            // let trl = mTrl.translation();
            // mr = rrl * mRcw;
            // mt = rrl * mtcw + trl;
            // twc = mRwc * mTlr.translation() + mOw;
        } else {
            mr = self.pose.unwrap().get_rotation(); // mr = mRcw
            mt = self.pose.unwrap().get_translation(); // mt = mtcw;
            twc = self.pose.unwrap().inverse().get_translation(); // twc = mOw;
        }
        let pos_camera = *mr * *pos + *mt;
        let pc_dist = pos_camera.norm();

        // Check positive depth
        let pc_z = pos_camera[2];
        let inv_z = 1.0 / pc_z;
        if pc_z < 0.0 { return None; }

        let (uvx, uvy) = match is_right {
            true => todo!("Stereo"), //self.camera2.project(pos_camera),
            false => CAMERA_MODULE.project(DVVector3::new(pos_camera))
        };
        if !self.features.image_bounds.check_bounds(uvx, uvy) {
            return None;
        }

        // Check distance is in the scale invariance region of the MapPoint
        let max_distance = mappoint.get_max_distance_invariance();
        let min_distance = mappoint.get_min_distance_invariance();
        let po = *pos - *twc;
        let dist = po.norm();
        if dist < min_distance || dist > max_distance {
            return None;
        }

        // Check viewing angle
        let pn = mappoint.get_normal();
        let view_cos = po.dot(&*pn) / dist;
        if view_cos < viewing_cos_limit {
            return None;
        }

        // Data used by the tracking
        Some(TrackedMapPointData {
            predicted_level: mappoint.predict_scale(&dist),
            view_cos: view_cos,
            proj_x: uvx,
            proj_y: uvy,
            track_depth: pc_dist,
        })
    }


    // don't pass through just call
    // pub fn get_features_in_area(&self, x: &f64, y: &f64, r: f64, min_level: i32, max_level: i32) -> Vec<u32> {
    //     self.features.get_features_in_area(x, y, r, &self.features.image_bounds, Some((min_level, max_level)))
    // }
    pub fn get_pose_in_world_frame(&self, map: &Map) -> DVPose {
        let pose = self.pose.unwrap();
        let ref_kf_id = self.ref_kf_id.unwrap();
        let ref_kf = map.keyframes.get(&ref_kf_id).expect("Can't get ref kf from map");
        let ref_kf_pose = ref_kf.pose;
        pose * ref_kf_pose.inverse()
    }

}

#[derive(Debug, Clone)]
pub struct KeyFrame {
    pub frame_id: Id,
    pub timestamp: Timestamp,

    pub mappoint_matches: MapPointMatches, // mvpmappoints , mvbOutlier

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: Option<BoW>,

    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub imu_bias: Option<IMUBias>,
    pub imu_preintegrated: Option<IMUPreIntegrated>,
    pub sensor: Sensor,

    pub pose: DVPose,
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // Map connections and info
    pub id: Id,
    pub origin_map_id: Id, // mnOriginMapId
    pub connections: ConnectedKeyFrames, // Parent, children, neighbors

    // TODO (mvp): I think we can clean this up and get rid of these
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
impl KeyFrame {
    pub(super) fn new(frame: Frame, origin_map_id: Id, id: Id) -> Self {
        let bow = match frame.bow {
            Some(bow) => Some(bow),
            None => {
                let mut bow = BoW::new();
                bow::VOCABULARY.transform(&frame.features.descriptors, &mut bow);
                Some(bow)
            }
        };

        Self {
            timestamp: frame.timestamp,
            frame_id: frame.frame_id,
            mappoint_matches: frame.mappoint_matches,
            pose: frame.pose.expect("Frame should have pose by now"),
            features: frame.features,
            bow,
            imu_bias: frame.imu_bias,
            imu_preintegrated: frame.imu_preintegrated,
            id,
            origin_map_id,
            connections: ConnectedKeyFrames::new(),
            ref_kf_id: frame.ref_kf_id,
            sensor: frame.sensor
        }
    }

    pub fn get_camera_center(&self) -> DVVector3<f64> {
        self.pose.inverse().get_translation()
        // Note: In Orbslam, this is: mTwc.translation()
        // and mTwc is inverse of the pose
    }

    pub fn get_right_camera_center(&self) -> DVVector3<f64> {
        todo!("IMU");
        // NOt sure what mTlr is, it comes from the settings but might get updated somewhere.
        //    return (mTwc * mTlr).translation();
        // this needs to be generic on sensor, so it can't be called if the sensor doesn't have a right camera
    }

    pub fn compute_scene_median_depth(&self, mappoints: &MapItemHashMap<MapPoint<FullMapPoint>>, q: i32) -> f64 {
        if self.features.num_keypoints == 0 {
            return -1.0;
        }

        let mut depths = Vec::new();
        let rot = self.pose.get_rotation();
        let rcw2 = rot.row(2);
        let zcw = self.pose.get_translation()[2];

        for mp_match in &self.mappoint_matches.matches {
            if let Some((mp_id, _)) = mp_match {
                let world_pos = *(mappoints.get(mp_id).unwrap().position);
                let z = (rcw2 * world_pos)[0] + zcw; // first part of this term is scalar but still need to get it from Matrix<1,1> to f64
                depths.push(z);
            }
        }

        depths.sort_by(|a, b| a.total_cmp(&b));
        depths[(depths.len()-1) / q as usize]
    }
    pub fn get_right_pose(&self) -> DVPose {
        todo!("Stereo");
        // Sophus::SE3<float> KeyFrame::GetRightPose() {
        //     unique_lock<mutex> lock(mMutexPose);

        //     return mTrl * mTcw;
        // }
    }

}


#[derive(Debug, Clone)]
pub struct MapPointMatches {
    // Mappoints //
    // Note: Id is mappoint Id, bool is if it's an oultier
    pub matches: Vec<Option<(Id, bool)>>
}
impl MapPointMatches {
    pub fn new(num_keypoints: usize) -> Self {
        Self {
            matches: vec![None; num_keypoints]
        }
    }

    pub fn get_mappoint(&self, index: &u32) -> Id {
        self.matches[*index as usize].unwrap().0
    }
    pub fn has_mappoint(&self, index: &u32) -> bool {
        self.matches[*index as usize].is_some()
    }

    pub fn add_mappoint(&mut self, index: u32, mp_id: Id, is_outlier: bool) {
        self.matches[index as usize] = Some((mp_id, is_outlier));
    }

    pub fn delete(&mut self, indices: (i32, i32)) {
        // TODO (mvp): removed the code that set's mappoint's last_frame_seen to the frame ID. Is that ok or does it need to be in here?
        if indices.0 != -1 {
            self.matches[indices.0 as usize] = None;
        }
        if indices.1 != -1 {
            self.matches[indices.1 as usize]= None;
        }
    }

    pub fn clear_mappoints(&mut self) {
        self.matches = vec![None; self.matches.len()];
    }

    pub fn is_outlier(&self, index: &u32) -> bool {
        self.matches[*index as usize].unwrap().1
    }

    pub fn set_mp_outlier(&mut self, index: &u32, is_outlier: bool) {
        self.matches[*index as usize].unwrap().1 = is_outlier;
    }

    pub fn discard_outliers(&mut self) -> Vec<Id> {
        let mut discards = vec![];
        for mp_match in &mut self.matches {
            if let Some((mp_id, is_outlier)) = mp_match {
                if *is_outlier {
                    discards.push(mp_id.clone());
                    *mp_match = None;
                }
            }
        }
        discards
    }

    pub fn get_num_mappoints_with_observations(&self, map: &Map) -> i32 {
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

    pub fn delete_mappoints_without_observations(&mut self, map: &Map) {
        for mp_match in &mut self.matches {
            if let Some((mp_id, _)) = mp_match {
                match map.mappoints.get(mp_id) {
                    Some(mp) => {
                        if mp.get_observations().len() == 0 {
                            *mp_match = None;
                        }
                    },
                    None => {}
                }
            }
        }
    }

    pub fn tracked_mappoints(&self, map: &Map, min_observations: u32) -> i32{
        // KeyFrame::TrackedMapPoints(const int &minObs)
        if min_observations > 0 {
            return self.matches.len() as i32;
        }

        let mut num_points = 0;
        for mp_match in &self.matches {
            if let Some((mp_id, _)) = mp_match {
                let mappoint = map.mappoints.get(&mp_id).unwrap();
                if mappoint.get_observations().len() >= (min_observations as usize) {
                    num_points += 1;
                }
            }
        }

        num_points
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

    pub parent: Option<Id>,
    pub children: HashSet<Id>,
}

impl ConnectedKeyFrames {
    pub fn new() -> Self {
        Self {
            ordered_connected_keyframes: Vec::new(),
            map_connected_keyframes: HashMap::new(),
            connected_keyframes_cutoff_weight: 0,
            parent: None,
            children: HashSet::new(),
        }
    }

    pub fn add(&mut self, kf_id: &Id, weight: i32) {
        *self.map_connected_keyframes.entry(*kf_id).or_insert(weight) = weight;
        self.ordered_connected_keyframes.push((*kf_id, weight));
        //TODO (mvp)...verify sorting in ConnectedKeyFrames, might be backwards. Not quite clear whether low weight = earlier index or vice versa
        if weight > self.connected_keyframes_cutoff_weight { self.sort_ordered(); }
    }

    pub fn delete(&mut self, kf_id: &Id) {
        if self.map_connected_keyframes.contains_key(kf_id) {
            self.map_connected_keyframes.remove(kf_id);
            self.ordered_connected_keyframes.retain(|(key, _)| key != kf_id);
            self.sort_ordered();
        }
    }

    pub fn add_all(&mut self, new_connections: HashMap::<Id, i32>, is_init_kf: bool) -> Option<Id> {
        // Turn hashmap into vector and sort by weights
        self.ordered_connected_keyframes = new_connections.iter()
            .map(|(key, value)| { (*key, *value) })
            .collect::<Vec<(Id, i32)>>(); 
        self.sort_ordered();

        self.map_connected_keyframes = new_connections;

        if self.parent.is_none() && !is_init_kf { 
            let parent_id = self.first_connected_kf();
            self.change_parent(Some(parent_id));
            Some(self.first_connected_kf())
        } else {
            None
        }
    }

    fn first_connected_kf(&self) -> Id {
        self.ordered_connected_keyframes[0].0 //.1
    }

    pub fn get_covisibility_keyframes(&self, num: i32) -> Vec<Id> {
        //vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames(), KeyFrame::GetConnectedKeyFrames
        // To get all connections, pass in i32::MAX as `num`
       let max_len = min(self.ordered_connected_keyframes.len(), num as usize);
       let (connections, _) : (Vec<i32>, Vec<i32>) = self.ordered_connected_keyframes[0..max_len].iter().cloned().unzip();
    //    debug!("Get covisibility keyframes for kf {}, {:?}, {:?}", self.id, self.ordered_connected_keyframes, self.map_connected_keyframes);
       connections
    }

    fn sort_ordered(&mut self) {
        //KeyFrame::UpdateBestCovisibles
        // TODO (mvp)...verify sorting in ConnectedKeyFrames, might be backwards. Not quite clear whether low weight = earlier index or vice versa
        self.ordered_connected_keyframes.sort_by(|(_,w1), (_,w2)| w2.cmp(&w1));
        let max_len = self.ordered_connected_keyframes.len(); 
        if max_len > 30 {
            self.connected_keyframes_cutoff_weight = self.ordered_connected_keyframes[30].1;
        } else {
            self.connected_keyframes_cutoff_weight = 0;
        }
    }

    pub fn get_weight(&self, other_kf_id: &Id) -> i32 {
        // int KeyFrame::GetWeight(KeyFrame *pKF)
        if self.map_connected_keyframes.contains_key(other_kf_id) {
            return *self.map_connected_keyframes.get(other_kf_id).unwrap();
        } else {
            return 0;
        }
    }

    pub fn change_parent(&mut self, some_id: Option<Id>) {
        self.parent = some_id;
    }

    pub fn add_child(&mut self, id: Id) {
        self.children.insert(id);
    }

    pub fn erase_child(&mut self, id: Id) {
        self.children.remove(&id);
    }
}