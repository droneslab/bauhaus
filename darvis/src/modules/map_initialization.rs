use core::config::{SETTINGS, SYSTEM};
use core::matrix::{DVVector3, DVVectorOfPoint2f};
use core::sensor::{Sensor, FrameSensor, ImuSensor};
use core::system::Timestamp;
use std::collections::BTreeSet;
use log::{debug, info, warn};
use core::matrix::DVVectorOfPoint3f;
use opencv::prelude::KeyPointTraitConst;
use crate::map::map::Id;
use crate::map::{frame::Frame, pose::Pose};
use crate::modules::optimizer;
use crate::registered_actors::{self, CAMERA_MODULE, FEATURE_MATCHING_MODULE, FULL_MAP_OPTIMIZATION_MODULE};
use crate::MapLock;
use crate::modules::module_definitions::CameraModule;
use super::module_definitions::{FeatureMatchingModule, MapInitializationModule};


pub struct MapInitialization {
    // Monocular
    pub mp_matches: Vec<i32>,// ini_matches .. mvIniMatches;
    pub prev_matched: DVVectorOfPoint2f,// std::vector<cv::Point2f> mvbPrevMatched;
    pub p3d: DVVectorOfPoint3f,// std::vector<cv::Point3f> mvIniP3D;
    pub ready_to_initialize: bool,
    pub initial_frame: Option<Frame>,
    pub last_frame: Option<Frame>,
    pub current_frame: Option<Frame>,
    sensor: Sensor,
}
impl MapInitializationModule for MapInitialization {
    type Frame = Frame;
    type Map = MapLock;
    type InitializationResult = Option<(Pose, i32, i32, BTreeSet<Id>, Timestamp, f64)>;

    fn try_initialize(&mut self, current_frame: &Frame) -> Result<bool, Box<dyn std::error::Error>> {
        // Only set once at beginning
        if self.initial_frame.is_none() {
            self.initial_frame = Some(current_frame.clone());
        }
        // If we never did initialization (ie, only 1 frame passed): initial, current, and last frame will all be set to the 1st frame.
        // If we've done the first step of initialization, update last frame and current frame.
        self.last_frame = match &self.current_frame {
            Some(frame) => Some(frame.clone()),
            None => Some(current_frame.clone())
        };
        self.current_frame = Some(current_frame.clone());

        match self.sensor.frame() {
            FrameSensor::Mono => self.monocular_initialization(),
            _ => self.stereo_initialization()
        }
    }

    fn create_initial_map(&mut self, map: &mut Self::Map) -> Self::InitializationResult {
        match self.sensor.frame() {
            FrameSensor::Mono => self.create_initial_map_monocular(map),
            _ => self.create_initial_map_stereo()
        }
    }
}

impl MapInitialization {
    pub fn new() -> Self {
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");

        Self {
            mp_matches: Vec::new(),
            prev_matched: DVVectorOfPoint2f::empty(),
            p3d: DVVectorOfPoint3f::empty(),
            ready_to_initialize: false,
            initial_frame: None,
            last_frame: None,
            current_frame: None,
            sensor
        }
    }

    fn monocular_initialization(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        let current_frame = self.current_frame.as_ref().unwrap();
        let initial_frame = self.initial_frame.as_ref().unwrap();
        let last_frame = self.last_frame.as_ref().unwrap();

        if !self.ready_to_initialize && current_frame.features.num_keypoints > 100 {
            // Set Reference Frame
            for i in 0..current_frame.features.num_keypoints as usize {
                self.prev_matched.push(current_frame.features.get_keypoint(i).0.pt().clone());
            }

            match self.sensor.imu() {
                ImuSensor::Some => {
                    todo!("IMU");
                    //Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L2467

                    // if(mpImuPreintegratedFromLastKF)
                    // {
                    //     delete mpImuPreintegratedFromLastKF;
                    // }
                    // mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                    // self.current_frame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
                },
                _ => {}
            }
            self.ready_to_initialize = true;
            return Ok(false);
        } else {
            if current_frame.features.num_keypoints <=100 || matches!(self.sensor.imu(), ImuSensor::Some) && last_frame.timestamp - initial_frame.timestamp > 1.0 {
                self.ready_to_initialize = false;
                return Ok(false);
            }

            // Find correspondences
            let (mut num_matches, mp_matches) = FEATURE_MATCHING_MODULE.search_for_initialization(
                &initial_frame, 
                &current_frame,
                &mut self.prev_matched,
                100
            );
            self.mp_matches = mp_matches;

            // Check if there are enough correspondences
            if num_matches < 100 {
                self.ready_to_initialize = false;
                return Ok(false);
            };

            if let Some((tcw, v_p3d, vb_triangulated)) = CAMERA_MODULE.two_view_reconstruction(
                initial_frame.features.get_all_keypoints(),
                current_frame.features.get_all_keypoints(),
                & self.mp_matches,
            ) {
                self.p3d = v_p3d;

                for index in 0..self.mp_matches.len() {
                    if self.mp_matches[index] >= 0 && !vb_triangulated[index as usize] {
                        self.mp_matches[index] = -1;
                        num_matches -= 1;
                    }
                }

                self.initial_frame.as_mut().unwrap().pose = Some(Pose::default());
                self.current_frame.as_mut().unwrap().pose = Some(tcw);

                return Ok(true);
            } else {
                debug!("MonocularInitialization, ReconstructWithTwoViews... failure");
                return Ok(false);
            }
        }
    }

    fn stereo_initialization(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        todo!("Stereo: StereoInitialization");
    }

    pub fn create_initial_map_monocular(
        &mut self, map: &mut MapLock
    ) -> Option<(Pose, i32, i32, BTreeSet<Id>, Timestamp, f64)> {
        // TODO (design, rust issues) - we have to do some pretty gross things with calling functions in this section
        // so that we can have multiple references to parts of the map. This should get cleaned up, but I'm not sure how.

        let (curr_kf_id, initial_kf_id) = {
            let mut lock = map.write();

            match self.sensor {
                Sensor(FrameSensor::Mono, ImuSensor::Some) => todo!("IMU"), // pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);
                _ => {}
            }

            if lock.last_kf_id == 0 {
                lock.initial_kf_id = lock.last_kf_id + 1;
            }

            let num_keypoints = self.initial_frame.as_ref().unwrap().features.num_keypoints;

            // Give up ownership of self.initial_frame and self.current_frame to avoid clone, we don't need it from now on
            let mut initial_frame = None;
            std::mem::swap(&mut initial_frame, &mut self.initial_frame);
            let mut current_frame = None;
            std::mem::swap(&mut current_frame, &mut self.current_frame);

            // Create KeyFrames
            let initial_kf_id = lock.insert_keyframe_to_map(
                initial_frame.unwrap(),
                true
            );
            let curr_kf_id = lock.insert_keyframe_to_map(
                current_frame.unwrap(),
                true
            );

            let mut count = 0;
            for kf1_index in 0..self.mp_matches.len() {
                // Get index for kf1 and kf2
                let kf2_index = self.mp_matches[kf1_index];
                if kf2_index == -1 {
                    continue;
                }
                count +=1;

                // Create mappoint
                // This also adds the observations from mappoint -> keyframe and keyframe -> mappoint
                let point = self.p3d.get(kf1_index).unwrap();
                let world_pos = DVVector3::new_with(point.x as f64, point.y as f64, point.z as f64);
                let origin_map_id = lock.id;
                let _id = lock.insert_mappoint_to_map(
                    world_pos, 
                    curr_kf_id, 
                    origin_map_id,
                    vec![(initial_kf_id, num_keypoints, kf1_index), (curr_kf_id, num_keypoints, kf2_index as usize)]
                );
            }
            info!("Monocular initialization created new map with {} mappoints", count);

            // Update Connections
            lock.update_connections(initial_kf_id);
            lock.update_connections(curr_kf_id);
            (curr_kf_id, initial_kf_id)
        };

        // Bundle Adjustment
        FULL_MAP_OPTIMIZATION_MODULE.optimize(map, 20, true, 0);

        let median_depth = {
            let lock = map.read();
            lock.keyframes.get(&initial_kf_id)?.compute_scene_median_depth(& lock.mappoints, 2)
        };
        let inverse_median_depth = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => 4.0 / median_depth,
            _ => 1.0 / median_depth
        };

        if median_depth < 0.0 || map.read().keyframes.get(&curr_kf_id)?.get_tracked_mappoints(&map.read(), 1) < 50 {
            // reset active map
            warn!("map::create_initial_map_monocular;wrong initialization");
            return None;
        }

        // Scale initial baseline
        {
            let mut lock = map.write();
            let curr_kf = lock.keyframes.get_mut(&curr_kf_id)?;
            let new_trans = *(curr_kf.pose.get_translation()) * inverse_median_depth;
            curr_kf.pose.set_translation(new_trans);
        }


        // Scale points
        let mp_matches = map.write().keyframes.get_mut(&initial_kf_id)?.get_mp_matches().clone();
        for item in mp_matches {
            if let Some((mp_id, _)) = item {
                {
                    let mut lock = map.write();
                    let mp = lock.mappoints.get_mut(&mp_id)?;
                    mp.position = DVVector3::new((*mp.position) * inverse_median_depth);
                }

                let norm_and_depth = map.read().mappoints.get(&mp_id)
                    .and_then(|mp| {mp.get_norm_and_depth(& map.read())})?;
                map.write().mappoints.get_mut(&mp_id)
                    .map(|mp| mp.update_norm_and_depth(norm_and_depth));
            }
        }

        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                todo!("IMU");
            //     pKFcur->mPrevKF = pKFini;
            //     pKFini->mNextKF = pKFcur;
            //     pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
            },
            _ => {}
        };

        // TODO (mvp): commented this out because I don't think they ever use it??
        // Compute here initial velocity
        // let delta_t = self.keyframes.get(&self.last_kf_id).unwrap().pose * self.keyframes.get(&1).unwrap().pose.inverse();
        // let velocity = false;
        // Eigen::Vector3f phi = deltaT.so3().log(); need to convert to rust
        // let initial_frame_ts = inidata.initial_frame.as_ref().unwrap().timestamp;
        // let curr_frame_ts = inidata.current_frame.as_ref().unwrap().timestamp;
        // let last_frame_ts = inidata.last_frame.as_ref().unwrap().timestamp;
        // let aux = (curr_frame_ts - last_frame_ts).to_std().unwrap().as_secs() / (curr_frame_ts - initial_frame_ts).to_std().unwrap().as_secs();
        // phi *= aux;

        // TODO (multimaps)
        // mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini)
        let (curr_kf_pose, relevant_mappoints, curr_kf_timestamp) = {
            let lock = map.read();

            let curr_kf_pose = lock.keyframes.get(&curr_kf_id)?.pose;
            let curr_kf_timestamp = lock.keyframes.get(&curr_kf_id)?.timestamp;

            // Update tracking with new info
            let relevant_mappoints = lock.mappoints.keys().cloned().collect();
            (curr_kf_pose, relevant_mappoints, curr_kf_timestamp)
        };


        Some((curr_kf_pose, curr_kf_id, initial_kf_id, relevant_mappoints, curr_kf_timestamp, inverse_median_depth))
    }

    pub fn create_initial_map_stereo(&mut self) -> Option<(Pose, i32, i32, BTreeSet<Id>, Timestamp, f64)> {
        todo!("Stereo: create_initial_map_stereo");
    }
    
}
