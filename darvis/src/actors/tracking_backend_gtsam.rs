extern crate g2o;
use log::{warn, info, debug, error};
use nalgebra::{Isometry3, Matrix, Matrix3, Point3, Quaternion, Vector3, Vector6};
use std::{collections::{BTreeMap, BTreeSet, HashMap, VecDeque}, fmt::{self, Formatter}, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{prelude::*, types::VectorOfKeyPoint,types::VectorOfPoint2f, types::VectorOfu8, types::VectorOfMat};
use gtsam::{
    inference::symbol::Symbol, linear::noise_model::{DiagonalNoiseModel, IsotropicNoiseModel}, navigation::combined_imu_factor::{CombinedImuFactor, PreintegratedCombinedMeasurements, PreintegrationCombinedParams}, nonlinear::{
        isam2::ISAM2, levenberg_marquardt_optimizer::LevenbergMarquardtOptimizer, levenberg_marquardt_params::LevenbergMarquardtParams, nonlinear_factor_graph::NonlinearFactorGraph, values::Values
    },
};
use std::fmt::Debug;
use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{
        messages::{ImageMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    }, map::{features::Features, frame::Frame, keyframe::MapPointMatches, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, ImuModule}, optimizer, orbslam_extractor::ORBExtractor, relocalization::Relocalization}, registered_actors::{CAMERA_MODULE, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}, MAP_INITIALIZED
};

use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{ImagePathMsg, InitKeyFrameMsg, NewKeyFrameMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackedMapPointData};
use crate::modules::module_definitions::MapInitializationModule;
use crate::registered_actors::IMU;

pub struct TrackingBackendGTSAM {
    system: System,
    sensor: Sensor,

    /// Frontend
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    init_id: Id,

    /// Backend
    state: TrackingState,

    // Frames
    last_frame: Option<Frame>,
    curr_frame_id: i32,
    current_frame: Frame,

    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_timestamp: Option<Timestamp>,

    // Modules 
    imu: IMU,
    graph_solver: GraphSolver,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

    /// References to map
    map_initialized: bool,
    map: ReadWriteMap,
    initialization: Option<MapInitialization>, // data sent to map actor to initialize new map
    map_scale: f64,

    // Local data used for different stages of tracking
    matches_inliers : i32, // mnMatchesInliers ... Current matches in frame
    // local_keyframes: BTreeSet<Id>, //mvpLocalKeyFrames 
    // local_mappoints: BTreeSet<Id>, //mvpLocalMapPoints
    track_in_view: HashMap::<Id, TrackedMapPointData>, // mbTrackInView , member variable in Mappoint
    // track_in_view_r: HashMap::<Id, TrackedMapPointData>, // mbTrackInViewR, member variable in Mappoint
    // kf_track_reference_for_frame: HashMap::<Id, Id>, // mnTrackReferenceForFrame, member variable in Keyframe
    // mp_track_reference_for_frame: HashMap::<Id, Id>,  // mnTrackReferenceForFrame, member variable in Mappoint
    // last_frame_seen: HashMap::<Id, Id>, // mnLastFrameSeen, member variable in Mappoint
    // non_tracked_mappoints: HashMap<Id, i32>, // just for testing


    // Specific to optical flow module
    last_keyframe_mappoint_matches: Option<MapPointMatches>, // mnLastFrameMatches

    /// Global defaults
    // localization_only_mode: bool,
    max_frames_to_insert_kf : i32 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
    min_frames_to_insert_kf: i32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
}

impl Actor for TrackingBackendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let _orb_extractor_right: Option<Box<dyn FeatureExtractionModule>> = match sensor.frame() {
            FrameSensor::Stereo => Some(Box::new(ORBExtractor::new(false))),
            FrameSensor::Mono | FrameSensor::Rgbd => None,
        };
        let orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>> = match sensor.is_mono() {
            true => Some(Box::new(ORBExtractor::new(true))),
            false => None
        };
        let imu = IMU::new();

        let mut actor = TrackingBackendGTSAM {
            system,
            graph_solver: GraphSolver::new(),
            orb_extractor_left: Box::new(ORBExtractor::new(false)),
            // _orb_extractor_right,
            orb_extractor_ini,
            map_initialized: false,
            init_id: 0,
            sensor,
            map,
            initialization: Some(MapInitialization::new()),
            map_scale: 1.0,
            // localization_only_mode: SETTINGS.get::<bool>(SYSTEM, "localization_only_mode"),
            max_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "max_frames_to_insert_kf"),
            min_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_frames_to_insert_kf"),
            state: TrackingState::NotInitialized,
            ref_kf_id: None,
            frames_since_last_kf: 0,
            last_kf_timestamp: None,
            imu,
            // relocalization: Relocalization{last_reloc_frame_id: 0, timestamp_lost: None},
            trajectory_poses: Vec::new(),
            // min_num_features: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_num_features")  as u32,
            matches_inliers: 0,
            // local_keyframes: BTreeSet::new(),
            // local_mappoints: BTreeSet::new(),
            track_in_view: HashMap::new(),
            // track_in_view_r: HashMap::new(),
            // kf_track_reference_for_frame: HashMap::new(),
            // mp_track_reference_for_frame: HashMap::new(),
            // last_frame_seen: HashMap::new(),
            // non_tracked_mappoints: HashMap::new(),
            last_keyframe_mappoint_matches: None,
            last_frame: None,
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            curr_frame_id: -1
        };
        tracy_client::set_thread_name!("tracking gtsam");

        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message) {
                break;
            }
            actor.map.match_map_version();
        }
    }

}

impl TrackingBackendGTSAM {
    fn handle_message(&mut self, message: MessageBox) -> bool {
        if message.is::<ImagePathMsg>() || message.is::<ImageMsg>() {
            if self.system.queue_full() {
                // Abort additional work if there are too many frames in the msg queue.
                info!("Tracking gtsam dropped 1 frame");
                return false;
            }

            let (image, timestamp, imu_measurements) = if message.is::<ImagePathMsg>() {
                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (image::read_image_file(&msg.image_path), msg.timestamp, msg.imu_measurements)
            } else {
                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (msg.image, msg.timestamp, msg.imu_measurements)
            };

            let created_kf = if matches!(self.state, TrackingState::NotInitialized) {
                // If map is not initialized yet, just extract features and try to initialize
                self.initialize_map(&image, timestamp).unwrap();
                false
            } else {
                // If this is not the first image, track features from the last image
                self.track(&image, timestamp, imu_measurements).unwrap()
            };

            self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
                keypoints: DVVectorOfKeyPoint::empty(),
                image,
                timestamp,
            }));

            self.current_frame.ref_kf_id = self.ref_kf_id;

            match self.state {
                TrackingState::Ok | TrackingState::RecentlyLost => {
                    self.update_trajectory_in_logs(created_kf).expect("Could not save trajectory")
                },
                _ => {},
            };
            self.last_frame = Some(self.current_frame.clone());
            self.curr_frame_id += 1;
            self.frames_since_last_kf = if created_kf { 0 } else { self.frames_since_last_kf + 1 };
        } else if message.is::<InitKeyFrameMsg>() {
            // Received from local mapping after it inserts a keyframe
            let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

            self.ref_kf_id = Some(msg.kf_id);
        } else if message.is::<LastKeyFrameUpdatedMsg>() {
            // Received from local mapping after it culls and creates new MPs for the last inserted KF
            self.state = TrackingState::Ok;
        } else if message.is::<UpdateFrameIMUMsg>() {
            todo!("IMU ... process message from local mapping!");
        } else if message.is::<ShutdownMsg>() {
            return true;
        } else {
            warn!("Tracking backend received unknown message type!");
        }
        return false;
    }

    fn initialize_map(&mut self, curr_img: &opencv::core::Mat, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        let (keypoints, descriptors) = self.extract_features(curr_img.clone(), self.curr_frame_id);

        self.current_frame = Frame::new(
            self.curr_frame_id, 
            keypoints,
            descriptors,
            curr_img.cols() as u32,
            curr_img.rows() as u32,
            Some(curr_img.clone()),
            self.last_frame.as_ref(),
            self.map.read()?.imu_initialized,
            timestamp,
        ).expect("Could not create frame!");

        if self.initialization.is_none() {
            self.initialization = Some(MapInitialization::new());
        }
        let init_success = self.initialization.as_mut().unwrap().try_initialize(&self.current_frame, &mut self.imu.imu_preintegrated_from_last_kf)?;
        if init_success {
            let (ini_kf_id, curr_kf_id);
            {
                (ini_kf_id, curr_kf_id) = match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map,  &mut self.imu.imu_preintegrated_from_last_kf)? {
                    Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, curr_kf_timestamp, map_scale)) => {
                        // Map needs to be initialized before tracking can begin. Received from map actor
                        self.frames_since_last_kf = 0;
                        // self.local_keyframes.insert(curr_kf_id);
                        // self.local_keyframes.insert(ini_kf_id);
                        // self.local_mappoints = local_mappoints;
                        self.ref_kf_id = Some(curr_kf_id);
                        self.last_kf_timestamp = Some(curr_kf_timestamp);
                        self.map_scale = map_scale;
                        if self.sensor.is_imu() {
                            self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(self.map.read()?.get_keyframe( curr_kf_id).imu_data.imu_preintegrated.as_ref().unwrap().get_updated_bias());
                        }

                        {
                            // Set current frame's updated info from map initialization
                            self.current_frame.pose = Some(curr_kf_pose);
                            self.current_frame.ref_kf_id = Some(curr_kf_id);
                            self.last_keyframe_mappoint_matches = Some(self.map.read()?.get_keyframe(curr_kf_id).clone_matches());
                        }
                        self.state = TrackingState::Ok;

                        // Log initial pose in shutdown actor
                        self.system.send(SHUTDOWN_ACTOR, 
                        Box::new(TrajectoryMsg{
                                pose: self.map.read()?.get_keyframe(ini_kf_id).get_pose(),
                                ref_kf_id: ini_kf_id,
                                timestamp: self.map.read()?.get_keyframe(ini_kf_id).timestamp,
                                map_version: self.map.read()?.version
                            })
                        );

                        (ini_kf_id, curr_kf_id)
                    },
                    None => {
                        panic!("Could not create initial map");
                    }
                };
            }

            MAP_INITIALIZED.store(true, Ordering::SeqCst);
            self.map_initialized = true;

            // Send first two keyframes to local mapping
            self.system.send(LOCAL_MAPPING, Box::new(
                InitKeyFrameMsg { kf_id: ini_kf_id, map_version: self.map.read()?.version }
            ));
            self.system.send(LOCAL_MAPPING,Box::new(
                InitKeyFrameMsg { kf_id: curr_kf_id, map_version: self.map.read()?.version }
            ));

            self.state = TrackingState::Ok;
            sleep(Duration::from_millis(50)); // Sleep just a little to allow local mapping to process first keyframe
        } else {
            self.state = TrackingState::NotInitialized;
        }
        Ok(())
    }

    fn track(
        &mut self,
        curr_img: & opencv::core::Mat, timestamp: Timestamp,
        mut imu_measurements: ImuMeasurements,
    ) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track");

        let mut created_keyframe = false;
        self.current_frame = Frame::new_no_features(
            self.curr_frame_id, 
            Some(curr_img.clone()),
            timestamp,
            Some(& self.last_frame.as_mut().unwrap())
        ).expect("Could not create frame!");

        // Track features
        let (tracked_kp, total_kp) = self.optical_flow()?;
        debug!("Optical flow, tracked {} from original {}", tracked_kp, total_kp);

        let num_tracked_mappoints = self.map_feature_tracks_to_mappoints()?;
        debug!("Tracked {} mappoints", num_tracked_mappoints);

        // Solve VIO graph. Includes preintegration
        let optimized = self.graph_solver.solve(&mut self.current_frame, &self.last_frame.as_ref().unwrap(), &mut imu_measurements)?;

        if optimized {
            // Create new keyframe
            if self.need_new_keyframe()? {
                created_keyframe = true;
                self.extract_features_and_add_to_existing()?;
                self.create_new_keyframe()?;
                println!("Created new keyframe");
            }
            Ok(created_keyframe)
        } else {
            self.state = TrackingState::NotInitialized;
            return Ok(false);
        }
    }

    fn extract_features(&mut self, image: opencv::core::Mat, curr_frame_id: i32) -> (DVVectorOfKeyPoint, DVMatrix) {
        let _span = tracy_client::span!("extract features");

        let (keypoints, descriptors) = if !self.map_initialized || (curr_frame_id - self.init_id < self.max_frames_to_insert_kf) {
            self.orb_extractor_ini.as_mut().unwrap().extract(DVMatrix::new(image)).unwrap()
        } else if self.sensor.is_mono() {
            self.orb_extractor_left.extract(DVMatrix::new(image)).unwrap()
        } else {
            todo!("Stereo, RGBD")
        };
        (keypoints, descriptors)
    }

    fn optical_flow(&mut self) -> Result<(u32, usize), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("optical_flow");

        let mut status = VectorOfu8::new();

        let frame1 = self.last_frame.as_mut().unwrap();

        let mut points1: VectorOfPoint2f = frame1.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();
        let mut points2 = VectorOfPoint2f::new();

        //this function automatically gets rid of points for which tracking fails
        let mut err = opencv::types::VectorOff32::default();
        let win_size = opencv::core::Size::new(24, 24);
        let termcrit = opencv::core::TermCriteria {
          typ: 3,
          max_count: 30,
          epsilon: 0.1,
        };
        let max_level = 3;
        opencv::video::calc_optical_flow_pyr_lk(
          & frame1.image.as_ref().unwrap(), & self.current_frame.image.as_ref().unwrap(), &mut points1, &mut points2, &mut status, &mut err, win_size, max_level, termcrit, 0, 0.001,
        )?;

        //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
        let mut indez_correction = 0;
        let mut new_descriptors = VectorOfMat::new();
        let mut new_keypoints = VectorOfKeyPoint::new();

        for i in 0..status.len() {
          let pt = points2.get(i - indez_correction)?;
          if (status.get(i)? == 0) || pt.x < 0.0 || pt.y < 0.0 {
            if pt.x < 0.0 || pt.y < 0.0 {
              status.set(i, 0)?;
            }
            points1.remove(i - indez_correction)?;
            points2.remove(i - indez_correction)?;
            frame1.features.remove_keypoint_and_descriptor(i - indez_correction)?;

            indez_correction = indez_correction + 1;
            } else {
                let curr_kp = frame1.features.get_keypoint(i - indez_correction).0;
                new_keypoints.push(opencv::core::KeyPoint::new_coords(pt.x, pt.y, curr_kp.size(), curr_kp.angle(), curr_kp.response(), curr_kp.octave(), curr_kp.class_id()).expect("Failed to create keypoint"));

                new_descriptors.push((* frame1.features.descriptors.row(i as u32)).clone());
            }
        }

        let mut new_descs_as_mat = opencv::core::Mat::default();
        opencv::core::vconcat(&new_descriptors, &mut new_descs_as_mat).expect("Failed to concatenate");

        self.current_frame.replace_features(DVVectorOfKeyPoint::new(new_keypoints), DVMatrix::new(new_descs_as_mat))?;

        Ok((self.current_frame.features.num_keypoints, status.len()))
    }

    fn map_feature_tracks_to_mappoints(&mut self) -> Result<i32, Box<dyn std::error::Error> > {
        let _span = tracy_client::span!("map_feature_tracks_to_mappoints");

        let map = self.map.read()?;
        let ref_kf = map.get_keyframe(self.ref_kf_id.unwrap());
        let mut added = 0;

        println!("CURRENT FRAME KEYPOINTS LEN: {}", self.current_frame.features.get_all_keypoints().len());
        println!("REF KF KEYPOINTS LEN: {}", ref_kf.features.get_all_keypoints().len());
        println!("REF KF MAPPOINT MATCHES LEN: {}", ref_kf.get_mp_matches().len());

        for idx1 in 0..self.current_frame.features.get_all_keypoints().len() - 1 {
            if let Some((mp_id, is_outlier)) = ref_kf.get_mp_match(&(idx1 as u32)) {
                if !is_outlier {
                    self.current_frame.mappoint_matches.add(idx1 as u32, mp_id, false);
                    added += 1;

                    // Adds depth to track_in_view, needed for local bundle adjustment
                    // let mp = map.get_mappoint(mp_id);
                    // let (tracked_data_left, _tracked_data_right) = self.current_frame.is_in_frustum(mp, 0.5);
                    // if tracked_data_left.is_some() {
                    //     map.mappoints.get(&mp_id).unwrap().increase_visible(1);
                    //     self.track_in_view.insert(mp_id, tracked_data_left.unwrap());
                    // } else {
                    //     self.track_in_view.remove(&mp_id);
                    // }

                }
            }
        }

        Ok(added)
    }

    fn _calculate_3d_points(&mut self) {
        // Convert keypoints into array of points.
        // std::vector<cv::Point2f> left_points;
        // std::vector<cv::Point2f> right_points;
        // // The reason we pass in right frame first is because we don't want to modify
        // // the left frames is_initial_ book-keeping.
        // // This means right will be the 'initial' and the left_frame
        // // will be the 'current' frame.
        // VisionFactor* matches;
        // // Assure that every point has a match.
        // float best_percent = config_.best_percent_;
        // config_.best_percent_ = 1.0;
        // matches = GetFeatureMatches(right_frame, left_frame);
        // config_.best_percent_ = best_percent;
        // if (matches->feature_matches.size() == 0) {
        //     return;
        // }
        // for (FeatureMatch match : matches->feature_matches) {
        //     const auto& left_pt = left_frame->keypoints_[match.feature_idx_current].pt;
        //     const auto& right_pt =
        //         right_frame->keypoints_[match.feature_idx_initial].pt;
        //     // if (fabs(left_pt.y - right_pt.y) > 5) continue;
        //     right_points.push_back(right_pt);
        //     left_points.push_back(left_pt);
        //     if (false) {
        //     printf("[%6.1f %6.1f] [%6.1f %6.1f]\n",
        //             left_pt.x,
        //             left_pt.y,
        //             right_pt.x,
        //             right_pt.y);
        //     }
        // }
        // cv::Mat triangulated_points;
        // cv::triangulatePoints(config_.projection_left,
        //                         config_.projection_right,
        //                         left_points,
        //                         right_points,
        //                         triangulated_points);
        // // Make sure all keypoints are matched to something.
        // CHECK_EQ(triangulated_points.cols, matches->feature_matches.size());
        // for (int64_t c = 0; c < triangulated_points.cols; ++c) {
        //     cv::Mat col = triangulated_points.col(c);
        //     points->push_back(
        //     Vector3f(col.at<float>(0, 0),
        //             col.at<float>(1, 0),
        //             col.at<float>(2, 0)) /col.at<float>(3, 0));
        // }
        // // Generate Debug Images if needed
        // if (config_.debug_images_) {
        //     debug_stereo_images_.push_back(CreateStereoDebugImage(*left_frame,
        //                                                         *right_frame,
        //                                                         *matches));
        // }
        // free(matches);

    }

    fn update_trajectory_in_logs(
        &mut self, created_new_kf: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let relative_pose = {
            let reference_pose = if created_new_kf {
                // If we created a new kf this round, the relative keyframe pose is the same as the current frame pose.
                self.current_frame.pose.unwrap()
            } else if self.last_frame.as_ref().unwrap().pose.is_none() {
                // This condition should only happen for the first frame after initialization
                self.map.read()?.get_keyframe(self.ref_kf_id.unwrap()).get_pose().inverse()
            } else {
                self.last_frame.as_ref().unwrap().pose.unwrap()
            };

            self.current_frame.pose.unwrap() * reference_pose.inverse()
        };

        self.trajectory_poses.push(relative_pose);

        info!("Frame {} pose: {:?}", self.current_frame.frame_id, self.current_frame.pose);

        self.system.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: self.current_frame.pose.unwrap().inverse(),
                ref_kf_id: self.current_frame.ref_kf_id.unwrap(),
                timestamp: self.current_frame.timestamp,
                map_version: self.map.read()?.version
            })
        );

        let map = self.map.read()?;
        let bla = map.get_keyframe(self.ref_kf_id.unwrap()).get_mp_matches().iter().filter_map(|v| match v { 
           Some((id, _is_outlier)) => Some(*id),
           None => None
        });
        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            pose: self.current_frame.pose.unwrap(),
            mappoint_matches: vec![],
            nontracked_mappoints: HashMap::new(),
            mappoints_in_tracking: bla.collect(),
            timestamp: self.current_frame.timestamp,
            map_version: map.version
        }));

        Ok(())
    }

    fn need_new_keyframe(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        let num_kfs = self.map.read()?.num_keyframes();

        if self.sensor.is_imu() && !self.map.read()?.imu_initialized {
            if (self.current_frame.timestamp - self.last_kf_timestamp.unwrap()) * 1e9 >= 0.25 { // 250 milliseconds
                debug!("Need new keyframe... IMU not initialized");
                return Ok(true);
            } else {
                return Ok(false);
            }
        }

        // Tracked MapPoints in the reference keyframe
        let min_observations = match num_kfs <= 2 {
            true => 2,
            false => 3
        };

        let map_lock = self.map.read()?;
        let tracked_mappoints = map_lock.get_keyframe(self.ref_kf_id.unwrap()).get_tracked_mappoints(&*map_lock, min_observations) as f32;

        // Check how many "close" points are being tracked and how many could be potentially created.
        let (tracked_close, non_tracked_close) = self.current_frame.check_close_tracked_mappoints();
        let need_to_insert_close = (tracked_close<100) && (non_tracked_close>70);

        // Thresholds
        let th_ref_ratio = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::None) => 0.9,
            Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                // Points tracked from the local map
                if self.matches_inliers > 350 { 0.75 } else { 0.90 }
            },
            Sensor(FrameSensor::Stereo, _) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => 0.75,
            Sensor(FrameSensor::Rgbd, ImuSensor::None) => { if num_kfs < 2 { 0.4 } else { 0.75 } }
        };

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        let c1a = self.frames_since_last_kf >= (self.max_frames_to_insert_kf as i32);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        let c1b = self.frames_since_last_kf >= (self.min_frames_to_insert_kf as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
        //Condition 1c: tracking is weak
        let sensor_is_right = match self.sensor {
            Sensor(FrameSensor::Mono, _) | Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => false,
            _ => true
        }; // I do not know why they just select for RGBD or Stereo without IMU
        let c1c = sensor_is_right && ((self.matches_inliers as f32) < tracked_mappoints * 0.25 || need_to_insert_close) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        let c2 = (((self.matches_inliers as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_inliers > 15;

        // println!("c2: {}, inliers: {}, tracked mappoints: {}, th ref ratio: {}", c2, self.matches_inliers, tracked_mappoints, th_ref_ratio);

        // Temporal condition for Inertial cases
        let close_to_last_kf = match self.last_kf_timestamp {
            Some(timestamp) => self.current_frame.timestamp - timestamp >= 0.5 * 1e-9, // 500 milliseconds
            None => false
        };

        let c3 = self.sensor.is_imu() && close_to_last_kf;

        let recently_lost = match self.state {
            TrackingState::RecentlyLost => true,
            _ => false
        };
        let sensor_is_imumono = match self.sensor {
            Sensor(FrameSensor::Rgbd, ImuSensor::Some) => true,
            _ => false
        };
        let c4 = ((self.matches_inliers < 75 && self.matches_inliers > 15) || recently_lost) && sensor_is_imumono;

        // Note: removed code here about checking for idle local mapping and/or interrupting bundle adjustment
        let create_new_kf =  ((c1a||c1b||c1c) && c2)||c3 ||c4;

        tracy_client::Client::running()
            .expect("message! without a running Client")
            .message(format!("need new kf: {} {}", create_new_kf, LOCAL_MAPPING_IDLE.load(Ordering::SeqCst)).as_str(), 2);

        if LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) && create_new_kf {
            self.frames_since_last_kf = 0;
            return Ok(true);
        } else {
            self.frames_since_last_kf += 1;
            return Ok(false);
        }
    }

    fn extract_features_and_add_to_existing(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("extract features");

        let (keypoints, descriptors) = self.extract_features(self.current_frame.image.as_ref().unwrap().clone(), self.curr_frame_id);

        let mut new_keypoints = VectorOfKeyPoint::new();
        let mut new_descriptor = Mat::default();
        let mut new_descriptor_vec = VectorOfMat::new();

        for i in 0..self.current_frame.features.num_keypoints {
            new_keypoints.push(self.current_frame.features.get_keypoint(i as usize).0);
            new_descriptor_vec.push((* self.current_frame.features.descriptors.row(i as u32)).clone());
        }

        for i in 0..keypoints.len() {
            new_keypoints.push(keypoints.get(i as usize)?);
            new_descriptor_vec.push((*descriptors).row(i as i32)?);
        }

        opencv::core::vconcat(&new_descriptor_vec, &mut new_descriptor).expect("Failed to concatenate");

        // println!("Re-extracting features, before: {}, after: {}", frame.features.num_keypoints, new_keypoints.len());

        self.current_frame.replace_features(DVVectorOfKeyPoint::new(new_keypoints), DVMatrix::new(new_descriptor))?;
        Ok(())
    }

    fn create_new_keyframe(&mut self) -> Result<(), Box<dyn std::error::Error>>{
        let _span = tracy_client::span!("create_new_keyframe");

        self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(ImuBias::new());

        self.last_kf_timestamp = Some(self.current_frame.timestamp);

        tracy_client::Client::running()
        .expect("message! without a running Client")
        .message("create new keyframe", 2);

        // KeyFrame created here and inserted into map
        self.system.send(
            LOCAL_MAPPING,
            Box::new( NewKeyFrameMsg{
                keyframe: self.current_frame.clone(),
                tracking_state: self.state,
                matches_in_tracking: self.matches_inliers,
                tracked_mappoint_depths: HashMap::new(), // Todo (Sofiya): can get away without sending this? In regular tracking this is updated during search_local_points, which only searches/updates local MPs that haven't already been matched... so it shouldn't be unusual for local BA to not have a depth value for a particular MP
                map_version: self.map.read()?.version
            } )
        );

        Ok(())
    }


}

pub struct GraphSolver {
    // New graph
    graph_new: NonlinearFactorGraph, // New factors that have not been optimized yet
    values_new: Values, // New values that have not been optimized yet
  
    // Main graph
    graph_main: NonlinearFactorGraph, // Main non-linear GTSAM graph, all created factors
    values_initial: Values, // All created nodes
    
    // Misc GTSAM objects
    isam2: ISAM2, // ISAM2 solvers
    preint_gtsam: PreintegratedCombinedMeasurements, // IMU preintegration

    // IMU preintegration parameters
    // Initialization
    prior_q_g_to_i: Quaternion<f64>, // prior_qGtoI
    prior_p_i_in_g: Vector3<f64>, // prior_pIinG
    prior_v_i_in_g: [f64; 3], // prior_vIinG
    prior_ba: [f64; 3], // prior_ba
    prior_bg: [f64; 3], // prior_bg
    // Noise values from dataset sensor
    accel_noise_density: f64, // accelerometer_noise_density, sigma_a
    gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
    accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
    gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
    // Noise values for initialization
    sigma_prior_rotation: [f64; 3],
    sigma_prior_translation: [f64; 3],
    sigma_velocity: f64,
    sigma_bias: f64,
    // Misc
    sigma_camera: f64,

    // Iterations of the map
    initialized: bool,
    ct_state: u64,
    ct_state_lookup: HashMap<i64, u64>,
    timestamp_lookup: HashMap<u64, i64>,
    measurement_smart_lookup_left: HashMap<i32, gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2>, // Smart lookup for IMU measurements
}

impl GraphSolver {
    pub fn new() -> Self {

        //           <!-- Rotation and Translation from camera frame to IMU frame -->
        //   <rosparam param="R_C0toI">[0.9999717314190615,    -0.007438121416209933,  0.001100323844221122,
        //                              0.00743200596269379,    0.9999574688631824,    0.005461295826837418,
        //                             -0.0011408988276470173, -0.0054529638303831614, 0.9999844816472552]</rosparam>
        //   <rosparam param="p_IinC0">[-0.03921656415229387,   0.00621263233002485,   0.0012210059575531885]</rosparam>

        //   <!-- Initialization -->
        //   <rosparam param="prior_qGtoI">[0.716147, 0.051158, 0.133778, 0.683096]</rosparam>       
        //   <rosparam param="prior_pIinG">[0.925493, -6.214668, 0.422872]</rosparam>       
        //   <rosparam param="prior_vIinG">[1.128364, -2.280640, 0.213326]</rosparam>       
        //   <rosparam param="prior_ba">[0.00489771688759235973,  0.00800897351104824969, 0.03020588299505782420]</rosparam>       
        //   <rosparam param="prior_bg">[-0.00041196751646696610, 0.00992948457018005999, 0.02188282212555122189]</rosparam>       

        //   <param name="sigma_camera"                 type="double"   value="0.306555403" />
        
        //   <!-- Noise Values for ETH Dataset Sensor -->
        //   <param name="accelerometer_noise_density"  type="double"   value="0.08" />    <!-- sigma_a -->
        //   <param name="gyroscope_noise_density"      type="double"   value="0.004" />   <!-- sigma_g -->
        //   <param name="accelerometer_random_walk"    type="double"   value="0.00004" /> <!-- sigma_wa -->
        //   <param name="gyroscope_random_walk"        type="double"   value="2.0e-6" />  <!-- sigma_wg -->

        //   <!-- Noise Values for Initialization -->
        //   <param name="sigma_prior_rotation"         type="double"   value="0.1" />
        //   <param name="sigma_prior_translation"      type="double"   value="0.3" />
        //   <param name="sigma_velocity"               type="double"   value="0.1" />
        //   <param name="sigma_bias"                   type="double"   value="0.15" />
        //   <param name="sigma_pose_rotation"          type="double"   value="0.1" />
        //   <param name="sigma_pose_translation"       type="double"   value="0.2" />

        Self {
            graph_new: NonlinearFactorGraph::default(),
            values_new: Values::default(),
            graph_main: NonlinearFactorGraph::default(),
            values_initial: Values::default(),
            isam2: ISAM2::default(),
            preint_gtsam: PreintegratedCombinedMeasurements::default(),
            initialized: false,

            // TODO SOFIYA PARAMS
            sigma_prior_rotation: [0.1, 0.1, 0.1],
            sigma_prior_translation: [0.3, 0.3, 0.3],
            sigma_velocity: 0.1,
            sigma_bias: 0.15,
            sigma_camera: 0.306555403,
            accel_noise_density: SETTINGS.get::<f64>(IMU, "noise_acc"),
            gyro_noise_density: SETTINGS.get::<f64>(IMU, "noise_gyro"),
            accel_random_walk: SETTINGS.get::<f64>(IMU, "acc_walk"),
            gyro_random_walk: SETTINGS.get::<f64>(IMU, "gyro_walk"),

            // TODO SOFIYA IS THIS RIGHT?
            // prior_q_g_to_i: Quaternion::new(0.716147, 0.051158, 0.133778, 0.683096),
            // prior_p_i_in_g: Vector3::new(0.925493, -6.214668, 0.422872),
            // prior_ba: [0.00489771688759235973,  0.00800897351104824969, 0.03020588299505782420],
            // prior_bg: [-0.00041196751646696610, 0.00992948457018005999, 0.02188282212555122189],
            // prior_v_i_in_g: [1.128364, -2.280640, 0.213326],
            prior_q_g_to_i: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            prior_p_i_in_g: Vector3::new(0.0, 0.0, 0.0),
            prior_ba: [0.0, 0.0, 0.0],
            prior_bg: [0.0, 0.0, 0.0],
            prior_v_i_in_g: [0.0 ,0.0, 0.0],

            ct_state: 0,
            ct_state_lookup: HashMap::new(),
            timestamp_lookup: HashMap::new(),
            measurement_smart_lookup_left: HashMap::new(),
        }
    }

    fn solve(&mut self, current_frame: &mut Frame, last_frame: &Frame, imu_measurements: &mut ImuMeasurements) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("solve");

        let timestamp = (current_frame.timestamp * 1e14) as i64; // Convert to int just so we can hash it
        // Return if the node already exists in the graph
        if self.ct_state_lookup.contains_key(&timestamp) {
            println!("NODE WITH TIMESTAMP {} ALREADY EXISTS, {}", timestamp, current_frame.timestamp);
            return Ok(false);
        }

        if !self.initialized {
            self.add_initials_and_priors(timestamp);
            self.initialized = true;
        } else {
            self.create_imu_factor(imu_measurements, &current_frame, & last_frame)?;

            // Original models
            let new_state = self.get_predicted_state();
     
            // Move node count forward in time
            self.ct_state += 1;

            // Append to our node vectors
            self.values_new.insert_pose3(
                &Symbol::new(b'x', self.ct_state),
                &new_state.pose
            );
            self.values_new.insert_vector3(
                &Symbol::new(b'v', self.ct_state),
                &new_state.velocity
            );
            self.values_new.insert_constant_bias(
                &Symbol::new(b'b', self.ct_state),
                &new_state.bias
            );
            self.values_initial.insert_pose3(
                &Symbol::new(b'x', self.ct_state),
                &new_state.pose
            );
            self.values_initial.insert_vector3(
                &Symbol::new(b'v', self.ct_state),
                &new_state.velocity
            );
            self.values_initial.insert_constant_bias(
                &Symbol::new(b'b', self.ct_state),
                &new_state.bias
            );

            // Add ct state to map
            self.ct_state_lookup.insert(timestamp, self.ct_state);
            self.timestamp_lookup.insert(self.ct_state, timestamp);
        }

        self.process_smart_features(& current_frame);

        let optimized_pose = self.optimize();

        if optimized_pose {
            // Update frame with optimized values
            // TODO there's probably a better way to clean up all this conversion than this
            let updated_pose: Isometry3<f64> = self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into();
            current_frame.pose = Some(Pose::new_from_isometry(updated_pose));
            let velocity: gtsam::base::vector::Vector3 = self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into();
            let vel_raw = velocity.get_raw();
            current_frame.imu_data.velocity = Some(DVVector3::new_with(vel_raw[0], vel_raw[1], vel_raw[2]));
            let bias_ref = self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap();
            let accel_bias = bias_ref.accel_bias().get_raw();
            let gyro_bias = bias_ref.gyro_bias().get_raw();

            // TODO SOFIYA Should this be set_new_bias?
            current_frame.imu_data.imu_bias = ImuBias {
                bax: accel_bias[0],
                bay: accel_bias[1],
                baz: accel_bias[2],
                bwx: gyro_bias[0],
                bwy: gyro_bias[1],
                bwz: gyro_bias[2]
            };

            println!("!!!!!! FINAL FRAME POSE: {:?}", current_frame.pose);
        }


        Ok(optimized_pose)
    }

    fn add_initials_and_priors(&mut self, timestamp: i64) {
        let _span = tracy_client::span!("add_initials_and_priors");

        // Create prior factor and add it to the graph
        let prior_state = GtsamState {
            pose: gtsam::geometry::pose3::Pose3::from_parts(
                gtsam::geometry::point3::Point3::new(self.prior_p_i_in_g[0], self.prior_p_i_in_g[1], self.prior_p_i_in_g[2]),
                gtsam::geometry::rot3::Rot3::from(self.prior_q_g_to_i)
            ),
            velocity: gtsam::base::vector::Vector3::new(self.prior_v_i_in_g[0], self.prior_v_i_in_g[1], self.prior_v_i_in_g[2]),
            bias: gtsam::imu::imu_bias::ConstantBias::new(
                &gtsam::base::vector::Vector3::new(self.prior_ba[0], self.prior_ba[1], self.prior_ba[2]),
                &gtsam::base::vector::Vector3::new(self.prior_bg[0], self.prior_bg[1], self.prior_bg[2])
            )
        };

        debug!("Add initials and priors, prior state: {:?}", prior_state);

        let pose_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
            self.sigma_prior_rotation[0], self.sigma_prior_rotation[1], self.sigma_prior_rotation[2],
            self.sigma_prior_translation[0], self.sigma_prior_translation[1], self.sigma_prior_translation[2]
        ));
        let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.sigma_velocity);
        let b_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(6, self.sigma_bias);

        self.graph_new.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
        self.graph_new.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
        self.graph_new.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);
        self.graph_main.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
        self.graph_main.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
        self.graph_main.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);

        // Add initial state to the graph
        self.values_new.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_new.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_new.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);
        self.values_initial.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_initial.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_initial.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);

        // Add ct state to map
        self.ct_state_lookup.insert(timestamp, self.ct_state);
        self.timestamp_lookup.insert(self.ct_state, timestamp);

        // Create GTSAM preintegration parameters for use with Foster's version
        let mut params = PreintegrationCombinedParams::makesharedu();  // Z-up navigation frame: gravity points along negative Z-axis !!!
        params.set_params(
            self.accel_noise_density * self.accel_noise_density, // acc white noise in continuous
            self.gyro_noise_density * self.gyro_noise_density, // gyro white noise in continuous
            self.accel_random_walk * self.accel_random_walk, // acc bias in continuous
            self.gyro_random_walk * self.gyro_random_walk, // gyro bias in continuous
            0.1, // error committed in integrating position from velocities
            1e-5 // error in the bias used for preintegration
        );

        // Actually create the GTSAM preintegration
        self.preint_gtsam = PreintegratedCombinedMeasurements::new(params, &prior_state.bias);
    }

    fn get_predicted_state(&self) -> GtsamState {
        let _span = tracy_client::span!("get_predicted_state");
        // This function will get the predicted state based on the IMU measurement

        // Get the current state (t=k)
        let state_k = GtsamState {
            pose: self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into(),
            velocity: self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into(),
            bias: self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
        };

        // debug!("... current state: {:?}", state_k);

        // From this we should predict where we will be at the next time (t=K+1)
        let state_k1 = self.preint_gtsam.predict(
            &gtsam::navigation::navstate::NavState::new(
                &state_k.pose,
                &state_k.velocity
            ),
            &state_k.bias
        );

        let predicted = GtsamState {
            pose: state_k1.get_pose().into(),
            velocity: state_k1.get_velocity().into(),
            bias: state_k.bias
        };
        
        println!("... predicted pose: {:?}", predicted.pose);

        return predicted;
    }

    fn create_imu_factor(&mut self, imu_measurements: &mut ImuMeasurements, current_frame: &Frame, previous_frame: &Frame) -> Result<(), Box<dyn std::error::Error>> {
        // This function will create a discrete IMU factor using the GTSAM preintegrator class
        // This will integrate from the current state time up to the new update time
        let _span = tracy_client::span!("create_imu_factor");

        let mut imu_from_last_frame = VecDeque::with_capacity(imu_measurements.len()); // mvImuFromLastFrame
        let imu_per = 0.000000000001; // 0.001 in orbslam, adjusted here for different timestamp units

        while !imu_measurements.is_empty() {
            if imu_measurements.front().unwrap().timestamp < previous_frame.timestamp - imu_per {
                imu_measurements.pop_front();
            } else if imu_measurements.front().unwrap().timestamp < current_frame.timestamp - imu_per {
                imu_from_last_frame.push_back(imu_measurements.pop_front().unwrap());
            } else {
                imu_from_last_frame.push_back(imu_measurements.pop_front().unwrap());
                break;
            }
        }
        let n = imu_from_last_frame.len() - 1;

        for i in 0..n {
            let mut tstep = 0.0;
            let mut acc: Vector3<f64> = Vector3::zeros(); // acc
            let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel

            if i == 0 && i < (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tini = imu_from_last_frame[i].timestamp - previous_frame.timestamp;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tini/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tini/tab)
                ) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - previous_frame.timestamp;
            } else if i < (n - 1) {
                acc = (imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc) * 0.5;
                ang_vel = (imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
            } else if i > 0 && i == (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tend = imu_from_last_frame[i + 1].timestamp - current_frame.timestamp;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tend/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tend/tab)
                ) * 0.5;
                tstep = current_frame.timestamp - imu_from_last_frame[i].timestamp;
            } else if i == 0 && i == (n - 1) {
                acc = imu_from_last_frame[i].acc;
                ang_vel = imu_from_last_frame[i].ang_vel;
                tstep = current_frame.timestamp - previous_frame.timestamp;
            }
            tstep = tstep * 1e9; 

            self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);
        }

        let imu_factor = CombinedImuFactor::new(
            &Symbol::new(b'x', self.ct_state),
            &Symbol::new(b'v', self.ct_state),
            &Symbol::new(b'x', self.ct_state + 1),
            &Symbol::new(b'v', self.ct_state + 1),
            &Symbol::new(b'b', self.ct_state),
            &Symbol::new(b'b', self.ct_state + 1),
            & self.preint_gtsam
        );
        self.graph_new.add_combined_imu_factor(&imu_factor);
        self.graph_main.add_combined_imu_factor(&imu_factor);

        Ok(())
    }

    fn process_smart_features(&mut self, frame: &Frame) {
        let _span = tracy_client::span!("process_smart_features");
        // Loop through LEFT features
        // let features: VectorOfPoint2f = frame.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();
        let matches = & frame.mappoint_matches.matches;
        for index in 0..matches.len() - 1 {
            let mappoint = matches[index];
            match mappoint {
                Some((mp_id, _is_outlier)) => {
                    // Check to see if it is already in the graph
                    match self.measurement_smart_lookup_left.get_mut(&mp_id) {
                        Some(smartfactor) => {
                            // Insert measurements to a smart factor
                            let kp = frame.features.get_keypoint(index).0;
                            smartfactor.add(
                                & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y as f64),
                                &Symbol::new(b'x', self.ct_state)
                            );
                            continue;
                        },
                        None => {
                            // If we know it is not in the graph
                            // Create a smart factor for the new feature
                            let measurement_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(2, self.sigma_camera);
                            let k = gtsam::geometry::cal3_s2::Cal3S2::default();

                            // Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
                            let sensor_p_body = ImuCalib::new().tbc;

                            let mut smartfactor_left = gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2::new(
                                &measurement_noise,
                                &k,
                                & sensor_p_body.inverse().into()
                            );

                            let kp = frame.features.get_keypoint(index).0;

                            // Insert measurements to a smart factor
                            smartfactor_left.add(
                                & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y  as f64),
                                &Symbol::new(b'x', self.ct_state)
                            );

                            // Add smart factor to FORSTER2 model
                            self.graph_new.add_smartfactor(&smartfactor_left);
                            self.graph_main.add_smartfactor(&smartfactor_left);

                            self.measurement_smart_lookup_left.insert(mp_id, smartfactor_left);
                        }
                    }
                },
                None => {}
            }
        }

    }

    fn optimize(&mut self) -> bool {
        let _span = tracy_client::span!("optimize");

        // Return if not initialized
        if !self.initialized && self.ct_state < 2 {
            return false;
        }

        // Perform smoothing update
        self.isam2.update_noresults(& self.graph_new, & self.values_new);
        self.values_initial = self.isam2.calculate_estimate().into();

        // Remove the used up nodes
        self.values_new.clear();

        // Remove the used up factors
        self.graph_new.resize(0);

        self.reset_imu_integration();

        return true;
    }

    fn reset_imu_integration(&mut self) {
        // Use the optimized bias to reset integration
        if self.values_initial.exists(&Symbol::new(b'b', self.ct_state)) {
            self.preint_gtsam.reset_integration_and_set_bias(
                & self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
            );
        }
    }
}


#[derive(Debug)]
struct GtsamState {
    pub pose: gtsam::geometry::pose3::Pose3,
    pub velocity: gtsam::base::vector::Vector3,
    pub bias: gtsam::imu::imu_bias::ConstantBias,
}
