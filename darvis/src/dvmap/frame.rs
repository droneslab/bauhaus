use std::collections::{HashMap, HashSet};
use chrono::{DateTime, Utc};
use dvcore::config::{GLOBAL_PARAMS, Sensor, FrameSensor};
use serde::{Deserialize, Serialize};

use crate::{config::{SYSTEM_SETTINGS}, matrix::*, modules::{imu::IMUBias, imu::IMUPreIntegrated, camera::*,}, actors::tracking_backend::TrackedMapPointData, dvmap::mappoint::{FullMapPoint, MapPoint}};
use super::{pose::Pose, map::{Id, Map}, features::{FRAME_GRID_ROWS, FRAME_GRID_COLS, Features}, bow::{DVVocabulary, BoW}};

#[derive(Debug, Clone, Default)]
pub struct Frame {
    pub id: Id,
    pub timestamp: DateTime<Utc>,
    pub pose: Option<Pose>,

    // Image and reference KF //
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: BoW,

    // Mappoints //
    // Note: u32 is index in array, Id is mappoint Id, bool is if it's an oultier
    // equal to the vec in ORBSLAM3 bc it just allocates an N-size vector and has a bunch of empty entries
    pub mappoint_matches: HashMap::<u32, (Id, bool)>, // mvpmappoints , mvbOutlier

    // IMU //
    pub imu_bias: Option<IMUBias>,
    pub imu_preintegrated: Option<IMUPreIntegrated>,

    // Stereo //
    pub stereo_baseline: f64,

    // Idk where to put this
    // depth_threshold: u64,
    pub camera: Camera,
    pub sensor: Sensor,
}

impl Frame {
    pub fn new(
        id: Id, keypoints_vec: DVVectorOfKeyPoint, descriptors_vec: DVMatrix,
        im_width: i32, im_height: i32, camera: &Camera
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let sensor = GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor");

        Ok(Frame{
            id,
            timestamp: Utc::now(),
            features: Features::new(keypoints_vec, descriptors_vec, im_width, im_height, camera, sensor)?,
            imu_bias: None, // TODO (IMU)
            camera: camera.clone(),
            sensor,
            ..Default::default()
        })
    }

    pub fn compute_bow(&mut self, voc: &DVVocabulary) {
        if self.bow.is_empty {
            voc.transform(&self.features.descriptors, &mut self.bow);
        }
    }

    //* MapPoints */
    pub fn add_mappoint(&mut self, index: u32, mp_id: Id, is_outlier: bool) {
        self.mappoint_matches.insert(index, (mp_id, is_outlier));
    }

    pub fn delete_mappoint_match(&mut self, index: u32) {
        self.mappoint_matches.remove(&index);
        // Sofiya: removed the code that set's mappoint's last_frame_seen to the frame ID
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

    pub fn discard_outliers(&mut self) -> i32 {
        let mps_at_start = self.mappoint_matches.len() as i32;
        self.mappoint_matches.retain(|_, (_, is_outlier)| *is_outlier);
        mps_at_start - self.mappoint_matches.len() as i32
    }

    pub fn get_num_mappoints_with_observations(&self, map: &Map) -> i32 {
        (&self.mappoint_matches)
            .into_iter()
            .filter(|(_, (mp_id, _))| map.get_mappoint(mp_id).unwrap().get_observations().len() > 0)
            .count() as i32
    }

    pub fn delete_mappoints_without_observations(&mut self, map: &Map) {
        self.mappoint_matches
            .retain(|_, (mp_id, _)| map.get_mappoint(mp_id).unwrap().get_observations().len() == 0);
    }

    pub fn check_close_tracked_mappoints(&self) -> (i32, i32) {
        self.features.check_close_tracked_mappoints(self.camera.th_depth as f32, &self.mappoint_matches)
    }

    pub fn is_in_frustum(&self, mp_id: Id, viewing_cos_limit: f64, map: &Map) -> (Option<TrackedMapPointData>, Option<TrackedMapPointData>) {
        // Combination of:
        // bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
        // bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight)
        let mappoint = map.get_mappoint(&mp_id).unwrap();

        let left = self.check_frustum(mp_id, viewing_cos_limit, &mappoint, false);
        let right = match self.sensor.frame() {
            FrameSensor::Stereo => { self.check_frustum(mp_id, viewing_cos_limit, &mappoint, true) },
            _ => { None }
        };
        (left, right)
    }

    fn check_frustum(&self, mp_id: Id, viewing_cos_limit: f64, mappoint: &MapPoint<FullMapPoint>, is_right: bool) -> Option<TrackedMapPointData> {
        // 3D in absolute coordinates
        let pos = mappoint.position;

        // 3D in camera coordinates
        let (mr, mt, twc);
        if is_right {
            todo!("TODO (stereo)");
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
            true => todo!("TODO (stereo)"), //self.camera2.project(pos_camera),
            false => self.camera.project(DVVector3::new(pos_camera))
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
        self.features.get_features_in_area(x, y, r, min_level, max_level, &self.features.image_bounds)
    }
}