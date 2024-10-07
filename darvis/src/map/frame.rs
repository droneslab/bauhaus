use core::{config::{SETTINGS, SYSTEM}, matrix::{DVMatrix, DVMatrix3, DVVector3, DVVectorOfKeyPoint}, sensor::{FrameSensor, ImuSensor, Sensor}, system::Timestamp};
use log::debug;
use opencv::core::{KeyPoint, Mat};

use crate::modules::{imu::{ImuCalib, ImuDataFrame}, module_definitions::VocabularyModule};

use crate::{actors::tracking_backend::TrackedMapPointData, modules::{bow::DVBoW, imu::{ImuBias, ImuPreIntegrated}}, registered_actors::{CAMERA_MODULE, VOCABULARY_MODULE}};

use super::{features::Features, keyframe::MapPointMatches, map::{Id, Map}, mappoint::MapPoint, pose::{DVTranslation, Pose}};
use crate::modules::module_definitions::CameraModule;


#[derive(Debug, Clone)]
pub struct Frame {
    pub frame_id: Id,
    pub timestamp: Timestamp,

    pub image: Option<opencv::core::Mat>,

    pub pose: Option<Pose>,
    pub mappoint_matches: MapPointMatches, // mvpmappoints , mvbOutlier
    pub ref_kf_id: Option<Id>, //mpReferenceKF

    // Vision //
    pub features: Features, // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    pub bow: Option<DVBoW>,

    // IMU //
    // Imu preintegration from last keyframe
    pub imu_data: ImuDataFrame,
 
    sensor: Sensor,

    // Don't add these in!! read explanations below
    // mnTrackReferenceForFrame ... used in tracking to decide whether to add a kf/mp into tracking's local map. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
}
impl Frame {
    pub fn new(
        frame_id: Id, keypoints_vec: DVVectorOfKeyPoint, descriptors_vec: DVMatrix,
        im_width: u32, im_height: u32, image: Option<opencv::core::Mat>,
        prev_frame: Option<& Frame>,
        timestamp: Timestamp
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let features = Features::new(keypoints_vec, descriptors_vec, im_width, im_height, sensor)?;
        let num_keypoints = features.num_keypoints as usize;
        let frame = Self {
            frame_id,
            timestamp,
            features,
            sensor,
            image,
            bow: None,
            mappoint_matches: MapPointMatches::new(num_keypoints),
            imu_data: ImuDataFrame::new(prev_frame),
            pose: None,
            ref_kf_id: None,
        };
        Ok(frame)
    }

    pub fn new_no_features(
        frame_id: Id, 
        image: Option<opencv::core::Mat>,
        timestamp: Timestamp,
        prev_frame: Option<& Frame>
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let features = Features::empty();
        let num_keypoints = features.num_keypoints as usize;
        let imu_bias = match sensor.imu() {
            ImuSensor::Some => Some(ImuBias::new()),
            _ => None
        };
        let frame = Self {
            frame_id,
            timestamp,
            features,
            sensor,
            image,
            bow: None,
            mappoint_matches: MapPointMatches::new(num_keypoints),
            imu_data: ImuDataFrame::new(prev_frame),
            pose: None,
            ref_kf_id: None,
        };
        Ok(frame)
    }

    pub fn new_clone(frame: &Frame) -> Frame {
        Frame {
            timestamp: frame.timestamp,
            frame_id: frame.frame_id,
            mappoint_matches: frame.mappoint_matches.clone(),
            pose: frame.pose,
            features: frame.features.clone(),
            bow: frame.bow.clone(),
            ref_kf_id: frame.ref_kf_id,
            sensor: frame.sensor,
            image: frame.image.clone(),
            imu_data: frame.imu_data.clone(),
        }
    }

    pub fn compute_bow(&mut self) {
        if self.bow.is_none() {
            self.bow = Some(DVBoW::new());
            VOCABULARY_MODULE.transform(&self.features.descriptors, &mut self.bow.as_mut().unwrap());
        }
    }

    pub fn replace_features(&mut self, keypoints: DVVectorOfKeyPoint, descriptors: DVMatrix) -> Result<(), Box<dyn std::error::Error>> {
        self.features = Features::new(keypoints, descriptors, self.features.image_width, self.features.image_height, self.sensor)?;
        self.mappoint_matches = MapPointMatches::new(self.features.num_keypoints as usize); // Mappoint match length needs to match keypoints length
        Ok(())
    }

    pub fn get_camera_center(&self) -> Option<DVTranslation> {
        Some(DVTranslation::new(-self.pose?.get_rotation().transpose() * *self.pose?.get_translation()))
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
        let _inv_z = 1.0 / pc_z; // I think used for stereo or rgbd? pMP->mTrackProjXR = uv(0) - mbf*invz; 
        if pc_z < 0.0 { return None; }

        let (uvx, uvy) = match is_right {
            true => todo!("Stereo"), //self.camera2.project(pos_camera),
            false => CAMERA_MODULE.project(DVVector3::new(pos_camera))
        };
        if !self.features.is_in_image(uvx, uvy) {
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
            view_cos,
            proj_x: uvx,
            proj_y: uvy,
            track_depth: pc_dist,
        })
    }


    pub fn get_pose_relative(&self, map: &Map) -> Pose {
        let pose = self.pose.unwrap();
        let ref_kf_id = self.ref_kf_id.unwrap();
        let ref_kf = map.keyframes.get(&ref_kf_id).expect("Can't get ref kf from map");
        let ref_kf_pose = ref_kf.get_pose();
        pose * ref_kf_pose.inverse()
    }

    pub fn delete_mappoint_outliers(&mut self) -> Vec<(Id, usize)> {
        // Should only be called on a Frame.
        // In the chance you might want to call this on a keyframe, you also need to delete the mappoints' observations to the kf!
        let mut discards = vec![];
        for i in 0..self.mappoint_matches.len() {
            if let Some((mp_id, is_outlier)) = self.mappoint_matches.get(i as usize) {
                if is_outlier {
                    discards.push((mp_id, i));
                    self.mappoint_matches.delete_at_indices((i as i32, -1));
                }
            }
        }
        discards
    }

    pub fn delete_mappoints_without_observations(&mut self, map: &Map) {
        // Should only be called on a Frame.
        // In the chance you might want to call this on a keyframe, you also need to delete the mappoints' observations to the kf!
        for i in 0..self.mappoint_matches.len() {
            if let Some((mp_id, is_outlier)) = self.mappoint_matches.get(i as usize) {
                match map.mappoints.get(&mp_id) {
                    Some(mp) => {
                        if mp.get_observations().len() == 0 {
                            self.mappoint_matches.delete_at_indices((i as i32, -1));
                        }
                    },
                    None => {}
                }
            }
        }
    }

    // *IMU *//
    pub fn get_imu_rotation(&self) -> DVMatrix3<f64> {
        // Eigen::Matrix3f KeyFrame::GetImuRotation()
        // Note: in Orbslam this is: (mTwc * mImuCalib.mTcb).rotationMatrix();
        // and mTwc is inverse of the pose
        (self.pose.expect("Should have pose by now").inverse() * ImuCalib::new().tcb).get_rotation()
    }

    pub fn get_imu_position(&self) -> DVVector3<f64> {
        // Eigen::Matrix<float,3,1> Frame::GetImuPosition() const {
        // Note: in Orbslam this is: 
        // mRwc * mImuCalib.mTcb.translation() + mOw;
        // where mOw = twc (inverse of pose translation)

        DVVector3::new(
            *self.pose.unwrap().get_rotation() * *ImuCalib::new().tcb.get_translation() + *self.pose.unwrap().inverse().get_translation()
        )
    }

    pub fn set_imu_pose_velocity(&mut self, new_pose: Pose, vwb: nalgebra::Vector3<f64>) {
        // void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb)
        debug!("SET VELOCITY: {:?} ", vwb);
        self.imu_data.velocity = Some(DVVector3::new(vwb));
        let new_pose = new_pose.inverse(); // Tbw
        self.pose = Some(ImuCalib::new().tcb * new_pose);
    }

}
