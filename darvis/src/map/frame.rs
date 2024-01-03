use core::{sensor::{Sensor, FrameSensor}, matrix::{DVVectorOfKeyPoint, DVMatrix, DVVector3}, config::{SETTINGS, SYSTEM}};

use crate::{modules::{imu::{IMUBias, IMUPreIntegrated}, camera::CAMERA_MODULE}, actors::tracking_backend::TrackedMapPointData};

use super::{map::{Id, Map}, misc::Timestamp, pose::Pose, keyframe::MapPointMatches, features::Features, bow::{BoW, self}, mappoint::MapPoint};


#[derive(Debug, Clone)]
pub struct Frame {
    pub frame_id: Id,
    pub timestamp: Timestamp,

    pub pose: Option<Pose>,
    pub mappoint_matches: MapPointMatches, // mvpmappoints , mvbOutlier
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: Option<BoW>,

    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub(super) imu_bias: Option<IMUBias>,
    pub(super) imu_preintegrated: Option<IMUPreIntegrated>,
 
    sensor: Sensor,

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

    pub fn is_in_frustum(&self, mappoint: &MapPoint, viewing_cos_limit: f64) -> (Option<TrackedMapPointData>, Option<TrackedMapPointData>) {
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

    fn check_frustum(&self, viewing_cos_limit: f64, mappoint: &MapPoint, is_right: bool) -> Option<TrackedMapPointData> {
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
        if !self.features.check_bounds(uvx, uvy) {
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
        let pn = mappoint.normal_vector;
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


    pub fn get_pose_relative(&self, map: &Map) -> Pose {
        let pose = self.pose.unwrap();
        let ref_kf_id = self.ref_kf_id.unwrap();
        let ref_kf = map.keyframes.get(&ref_kf_id).expect("Can't get ref kf from map");
        let ref_kf_pose = ref_kf.pose;
        pose * ref_kf_pose.inverse()
    }

    pub fn delete_mappoint_outliers(&mut self) -> Vec<Id> {
        // Should only be called on a Frame.
        // In the chance you might want to call this on a keyframe, you also need to delete the mappoints' observations to the kf!
        let mut discards = vec![];
        for mp_match in &mut self.mappoint_matches.matches {
            if let Some((mp_id, is_outlier)) = mp_match {
                if *is_outlier {
                    discards.push(mp_id.clone());
                    *mp_match = None;
                }
            }
        }
        discards
    }

    pub fn delete_mappoints_without_observations(&mut self, map: &Map) {
        // Should only be called on a Frame.
        // In the chance you might want to call this on a keyframe, you also need to delete the mappoints' observations to the kf!
        for mp_match in &mut self.mappoint_matches.matches {
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
}
