use std::{collections::{HashMap, HashSet}, cmp::min};
use derivative::Derivative;
use dvcore::{matrix::{DVVector3, DVVectorOfKeyPoint, DVMatrix}, config::{ SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor}};
use log::{error, info, debug};
use logging_timer::time;
use serde::{Deserialize, Serialize};
use crate::{dvmap::{map::Id, pose::DVPose},modules::{imu::*, camera::CAMERA_MODULE}, actors::tracking_backend::TrackedMapPointData,};
use super::{mappoint::{MapPoint, FullMapPoint}, map::{Map, MapItemHashMap}, features::Features, bow::{BoW, self}, misc::Timestamp};

// TODO (design)... It would be nice to re-think how we do the states. We shouldn't have options in the regular Frame struct, there should be Frame generics that either have those filled in or don't. But creating more states means we have a lot more duplicated code for each function that can be called on multiple states. 

// Typestate...Frame/KeyFrame information that is ALWAYS available, regardless of frame/keyframe state.
#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct Frame<K: FrameState> {
    pub frame_id: Id,
    pub timestamp: Timestamp,
    pub pose: Option<DVPose>,

    // Image and reference KF //
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: Option<BoW>,

    // Mappoints //
    // Note: u32 is index in array, Id is mappoint Id, bool is if it's an oultier
    // equal to the vec in ORBSLAM3 bc it just allocates an N-size vector and has a bunch of empty entries
    pub mappoint_matches: HashMap::<u32, (Id, bool)>, // mvpmappoints , mvbOutlier

    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub imu_bias: Option<IMUBias>,
    pub imu_preintegrated: Option<IMUPreIntegrated>,

    // Idk where to put this
    // depth_threshold: u64,
    pub sensor: Sensor,

    // scale: f64,
    // depth_threshold: f64,

    // pub imu_calib: IMUCalib,
    pub full_kf_info: K,

    // Don't add these in!! read explanations below
    // mnTrackReferenceForFrame ... used in tracking to decide whether to add a kf/mp into tracking's local map. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
}

pub trait FrameState: Send + Sync {}
impl FrameState for InitialFrame {}
impl FrameState for PrelimKeyFrame {}
impl FrameState for FullKeyFrame {}

// Functions valid on ALL types of Frames/KeyFrames
impl<T: FrameState> Frame<T> {
    pub fn add_mappoint(&mut self, index: u32, mp_id: Id, is_outlier: bool) {
        self.mappoint_matches.insert(index, (mp_id, is_outlier));
    }
    pub fn delete_mappoint_match(&mut self, index: u32) {
        self.mappoint_matches.remove(&index);
        // TODO (mvp): removed the code that set's mappoint's last_frame_seen to the frame ID
        // I'm not sure we want to be using this
    }

    pub fn clear_mappoints(&mut self) {
        self.mappoint_matches = HashMap::new();
    }

    pub fn is_mp_outlier(&self, index: &u32) -> bool {
        self.mappoint_matches.get(index).unwrap().1
    }

    pub fn set_mp_outlier(&mut self, index: &u32, is_outlier: bool) {
        self.mappoint_matches
            .entry(*index)
            .and_modify(|(_, iso)| *iso = is_outlier);
    }

    pub fn discard_outliers(&mut self) -> Vec<(u32, (Id, bool))> {
        let discarded = self.mappoint_matches.extract_if(|_, (_, is_outlier)| *is_outlier).collect();
        discarded
    }

    pub fn get_num_mappoints_with_observations(&self, map: &Map) -> i32 {
        (&self.mappoint_matches)
            .into_iter()
            .filter(|(_, (mp_id, _))| map.get_mappoint(mp_id).unwrap().get_observations().len() > 0)
            .count() as i32
    }

    pub fn delete_mappoints_without_observations(&mut self, map: &Map) {
        self.mappoint_matches
            .retain(|_, (mp_id, _)| map.get_mappoint(mp_id).unwrap().get_observations().len() > 0);
    }

    pub fn check_close_tracked_mappoints(&self) -> (i32, i32) {
        self.features.check_close_tracked_mappoints(CAMERA_MODULE.th_depth as f32, &self.mappoint_matches)
    }

    pub fn get_pose_in_world_frame(&self, map: &Map) -> DVPose {
        let pose = self.pose.expect("Frame doesn't have a pose");
        let ref_kf_id = self.ref_kf_id.expect("Frame doesn't have a ref kf id");
        let ref_kf = map.get_keyframe(&ref_kf_id).expect("Can't get ref kf from map");
        let ref_kf_pose = ref_kf.pose.expect("Ref kf doesn't have a pose");
        pose * ref_kf_pose.inverse()
    }
}

// Basic frame read from the image file, not upgraded to a keyframe yet
#[derive(Clone, Debug, Default, Copy, Serialize, Deserialize)]
pub struct InitialFrame {}

impl Frame<InitialFrame> {
    #[time("Frame<InitialFrame>::{}")]
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
        let frame = Self {
            frame_id,
            timestamp,
            features: Features::new(keypoints_vec, descriptors_vec, im_width, im_height, sensor)?,
            imu_bias,
            sensor,
            ..Default::default()
        };
        Ok(frame)
    }

    pub fn compute_bow(&mut self) {
        if self.bow.is_none() {
            self.bow = Some(BoW::new());
            bow::VOCABULARY.transform(&self.features.descriptors, &mut self.bow.as_mut().unwrap());
        }
    }

    pub fn is_in_frustum(&self, mp_id: Id, viewing_cos_limit: f64, map: &Map) -> (Option<TrackedMapPointData>, Option<TrackedMapPointData>) {
        // Combination of:
        // bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
        // bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight)
        let mappoint = map.get_mappoint(&mp_id).unwrap();

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


    pub fn get_features_in_area(&self, x: &f64, y: &f64, r: f64, min_level: i32, max_level: i32) -> Vec<u32> {
        self.features.get_features_in_area(x, y, r, &self.features.image_bounds, Some((min_level, max_level)))
    }
}

// A frame upgraded to a prelimary keyframe, created locally but not inserted into the map yet
#[derive(Clone, Debug, Default, Copy, Serialize, Deserialize)]
pub struct PrelimKeyFrame {}

impl Frame<PrelimKeyFrame> {
    #[time("Frame<PrelimKeyFrame>::{}")]
    pub fn new(frame: Frame<InitialFrame>) -> Frame<PrelimKeyFrame> {
        if frame.pose.is_none() {
            panic!("Frame needs a pose before converting to KeyFrame!");
        }
        Frame {
            timestamp: frame.timestamp,
            frame_id: frame.frame_id,
            mappoint_matches: frame.mappoint_matches,
            pose: frame.pose,
            features: frame.features,
            imu_bias: frame.imu_bias,
            imu_preintegrated: frame.imu_preintegrated,
            full_kf_info: PrelimKeyFrame{},
            bow: frame.bow,
            ref_kf_id: frame.ref_kf_id,
            sensor: frame.sensor,
        }
    }
    #[time("Frame<PrelimKeyFrame>::{}")]
    pub fn new_clone(frame: &Frame<InitialFrame>) -> Frame<PrelimKeyFrame> {
        if frame.pose.is_none() {
            panic!("Frame needs a pose before converting to KeyFrame!");
        }
        Frame {
            timestamp: frame.timestamp,
            frame_id: frame.frame_id,
            mappoint_matches: frame.mappoint_matches.clone(),
            pose: frame.pose,
            features: frame.features.clone(),
            imu_bias: frame.imu_bias,
            imu_preintegrated: frame.imu_preintegrated,
            full_kf_info: PrelimKeyFrame{},
            bow: frame.bow.clone(),
            ref_kf_id: frame.ref_kf_id,
            sensor: frame.sensor,
        }
    }

}

// Full keyframe inserted into the map with the following additional fields
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct FullKeyFrame {
    pub id: Id,
    pub origin_map_id: Id, // mnOriginMapId

    // Map connections ... Parent, children, neighbors
    pub parent: Option<Id>,
    pub children: HashSet<Id>,

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

impl Frame<FullKeyFrame> {
    #[time("Frame<FullKeyFrame>::{}")]
    pub(super) fn new(prelim_keyframe: Frame<PrelimKeyFrame>, origin_map_id: Id, id: Id) -> Self {
        let bow = match prelim_keyframe.bow {
            Some(bow) => Some(bow),
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
            mappoint_matches: prelim_keyframe.mappoint_matches,
            pose: prelim_keyframe.pose,
            features: prelim_keyframe.features,
            bow,
            imu_bias: prelim_keyframe.imu_bias,
            imu_preintegrated: prelim_keyframe.imu_preintegrated,
            full_kf_info: FullKeyFrame{
                id,
                origin_map_id,
                ..Default::default()
            },
            ref_kf_id: prelim_keyframe.ref_kf_id,
            sensor: prelim_keyframe.sensor
        }
    }

    pub fn id(&self) -> Id { self.full_kf_info.id }

    pub fn get_mappoint(&self, index: &u32) -> Id {
        self.mappoint_matches.get(index).unwrap().0
    }
    pub fn has_mappoint(&self, index: &u32) -> bool {
        self.mappoint_matches.get(index).is_some()
    }

    pub fn erase_mappoint_match(&mut self, (left_index, right_index): (i32, i32)) {
        if left_index != -1 {
            self.mappoint_matches.remove(&(left_index as u32));
        }
        if right_index != -1 {
            self.mappoint_matches.remove(&(right_index as u32));
        }
    }

    pub fn add_connection(&mut self, kf_id: &Id, weight: i32) {
        *self.full_kf_info.map_connected_keyframes.entry(*kf_id).or_insert(weight) = weight;
        //TODO...verify sorting in ConnectedKeyFrames, might be backwards. Not quite clear whether low weight = earlier index or vice versa
        if weight > self.full_kf_info.connected_keyframes_cutoff_weight { self.sort_ordered(); }
    }

    pub fn erase_connection(&mut self, kf_id: &Id) {
        if self.full_kf_info.map_connected_keyframes.contains_key(kf_id) {
            self.full_kf_info.map_connected_keyframes.remove(kf_id);
            self.sort_ordered();
        }
    }

    pub fn insert_all_connections(&mut self, new_connections: HashMap::<Id, i32>, is_init_kf: bool) -> Option<Id> {
        // Turn hashmap into vector and sort by weights
        self.full_kf_info.ordered_connected_keyframes = new_connections.iter()
            .map(|(key, value)| { (*key, *value) })
            .collect::<Vec<(Id, i32)>>(); 
        self.sort_ordered();

        self.full_kf_info.map_connected_keyframes = new_connections;

        if self.full_kf_info.parent.is_none() && !is_init_kf { 
            let parent_id = self.first_connected_kf();
            self.change_parent(Some(parent_id));
            Some(self.first_connected_kf())
        } else {
            None
        }
    }

    fn first_connected_kf(&self) -> Id {
        self.full_kf_info.ordered_connected_keyframes[0].0 //.1
    }

    pub fn get_covisibility_keyframes(&self, num: i32) -> Vec<Id> {
        //vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames(), KeyFrame::GetConnectedKeyFrames
        // To get all connections, pass in i32::MAX as `num`
       let max_len = min(self.full_kf_info.ordered_connected_keyframes.len(), num as usize);
       let (conections, _) : (Vec<i32>, Vec<i32>) = self.full_kf_info.ordered_connected_keyframes[0..max_len].iter().cloned().unzip();
       conections
    }

    fn sort_ordered(&mut self) {
        //KeyFrame::UpdateBestCovisibles
        //TODO...verify sorting in ConnectedKeyFrames, might be backwards. Not quite clear whether low weight = earlier index or vice versa
        self.full_kf_info.ordered_connected_keyframes.sort_by(|(_,w1), (_,w2)| w2.cmp(&w1));
        let max_len = self.full_kf_info.ordered_connected_keyframes.len(); 
        if max_len >= 30 {
            self.full_kf_info.connected_keyframes_cutoff_weight = self.full_kf_info.ordered_connected_keyframes[30].1;
        } else {
            self.full_kf_info.connected_keyframes_cutoff_weight =0;
        }
    }

    pub fn get_weight(&self, other_kf_id: &Id) -> i32 {
        // int KeyFrame::GetWeight(KeyFrame *pKF)
        if self.full_kf_info.map_connected_keyframes.contains_key(other_kf_id) {
            return *self.full_kf_info.map_connected_keyframes.get(other_kf_id).unwrap();
        } else {
            return 0;
        }
    }

    pub fn change_parent(&mut self, some_id: Option<Id>) {
        if some_id.is_some() && some_id.unwrap() == self.full_kf_info.id {
            error!("keyframe::change_parent;parent and child are the same KF");
        }
        self.full_kf_info.parent = some_id;
    }

    pub fn add_child(&mut self, id: Id) {
        self.full_kf_info.children.insert(id);
    }

    pub fn erase_child(&mut self, id: Id) {
        self.full_kf_info.children.remove(&id);
    }

    pub fn get_camera_center(&self) -> DVVector3<f64> {
        self.pose.unwrap().inverse().get_translation()
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
        let rot = self.pose.unwrap().get_rotation();
        let rcw2 = rot.row(2);
        let zcw = self.pose.unwrap().get_translation()[2];

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

    pub fn get_right_pose(&self) -> DVPose {
        todo!("Stereo");
        // Sophus::SE3<float> KeyFrame::GetRightPose() {
        //     unique_lock<mutex> lock(mMutexPose);

        //     return mTrl * mTcw;
        // }
    }

    pub fn get_features_in_area(&self, x: &f64, y: &f64, r: f64, b_right: bool) -> Vec<u32> {
        self.features.get_features_in_area(x, y, r, &self.features.image_bounds, None)
    }

}
