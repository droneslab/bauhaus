extern crate g2o;
use log::{warn, info, debug, error};
use nalgebra::Matrix;
use std::{collections::{BTreeMap, BTreeSet, HashMap}, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{prelude::*, types::VectorOfKeyPoint,types::VectorOfPoint2f, types::VectorOfu8, types::VectorOfMat};
use gtsam::{
    inference::symbol::Symbol,
    linear::noise_model::{DiagonalNoiseModel, IsotropicNoiseModel},
    nonlinear::{
        levenberg_marquardt_optimizer::LevenbergMarquardtOptimizer,
        levenberg_marquardt_params::LevenbergMarquardtParams,
        nonlinear_factor_graph::NonlinearFactorGraph, values::Values,
    },
};

use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{
        messages::{ImageMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    }, map::{features::Features, frame::Frame, keyframe::MapPointMatches, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image, imu::{ImuBias, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, ImuModule}, optimizer, orbslam_extractor::ORBExtractor, relocalization::Relocalization}, registered_actors::{CAMERA_MODULE, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}, MAP_INITIALIZED
};

use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{ImagePathMsg, InitKeyFrameMsg, NewKeyFrameMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackedMapPointData};
use crate::modules::module_definitions::MapInitializationModule;

pub struct TrackingBackendGTSAM {
    system: System,
    sensor: Sensor,

    /// Graphs
    graph: NonlinearFactorGraph,
    initials: Values,

    /// Factor graph optimization params
    bias_covariance: (f32, f32),
    max_iterations: u32,
    lambda_upper_bound: f64,
    lambda_lower_bound: f64,
    diagonal_damping: f64,
    relative_error_tol: f64,
    absolute_error_tol: f64,
    // IMU Param
    accel_covariance: nalgebra::Matrix3::<f64>,
    gyro_covariance: nalgebra::Matrix3::<f64>,
    integration_covariance: nalgebra::Matrix3::<f64>,

    /// Frontend
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    // _orb_extractor_right: Option<Box<dyn FeatureExtractionModule>>,
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    init_id: Id,

    // Frames
    last_frame: Option<Frame>,
    curr_frame_id: i32,
    current_frame: Frame,

    /// Backend
    state: TrackingState,
    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_timestamp: Option<Timestamp>,

    // Modules 
    imu: IMU,
    // relocalization: Relocalization,

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
        let i = nalgebra::Matrix3::<f64>::identity();

        let mut actor = TrackingBackendGTSAM {
            graph: NonlinearFactorGraph::default(),
            initials: Values::default(),

            bias_covariance: (6.0, 0.4),
            max_iterations: 1000,
            lambda_upper_bound: 1.0e6,
            lambda_lower_bound: 0.1,
            diagonal_damping: 1000.0,
            relative_error_tol: 1.0e-9,
            absolute_error_tol: 1.0e-9,

            // IMU preintegration parameters
            // Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
            accel_covariance: i * 0.2,
            gyro_covariance: i * 0.2,
            integration_covariance: i * 0.2,

            system,
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

            // match self.state {
            //     TrackingState::Ok | TrackingState::RecentlyLost => {
            //         self.update_trajectory_in_logs(created_kf).expect("Could not save trajectory")
            //     },
            //     _ => {},
            // };
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

        // Preintegrate
        self.imu.preintegrate(&mut imu_measurements, &mut self.current_frame, self.last_frame.as_mut().unwrap(), self.map.read()?.last_kf_id);

        // Solve VIO graph
        self.add_imu_measurements(); // measured_poses, measured_acc, measured_omega, measured_vel, delta_t, 10
        self.add_keypoints(); // vision_data, measured_poses, 10, depth, axs
        self.estimate();

        // Create new keyframe
        if self.need_new_keyframe()? {
            created_keyframe = true;
            self.extract_features_and_add_to_existing()?;
            self.create_new_keyframe()?;
        }

        Ok(created_keyframe)
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
        let map = self.map.read()?;
        let ref_kf = map.get_keyframe(self.ref_kf_id.unwrap());
        let mut added = 0;
        for idx1 in 0..self.current_frame.features.get_all_keypoints().len() {
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

    fn calculate_3d_points(&mut self) {
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

    fn add_imu_measurements(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // measured_poses, measured_acc, measured_omega, measured_vel, delta_t, 10

        // Pose prior
        let pose_key = 0;
        self.graph.add_prior_factor_pose3(
            pose_key,
            &self.last_frame.as_ref().unwrap().pose.unwrap().into(),
            &DiagonalNoiseModel::from_sigmas(nalgebra::Vector6::new(0.2, 0.2, 0.2, 0.2, 0.2, 0.2)));
        self.initials.insert_pose3(0, &self.last_frame.as_ref().unwrap().pose.unwrap().into());
        //Reference:
        // pose_key = X(0)
        // pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]))
        // pose_0 = gtsam.Pose3(measured_poses[0])
        // self.graph.push_back(gtsam.PriorFactorPose3(pose_key, pose_0, pose_noise))
        // self.initial_estimate.insert(pose_key, gtsam.Pose3(measured_poses[0]))


        // IMU prior
        let bias_key = 1;
        let constantbias = gtsam::imu::imu_bias::ConstantBias::default();
        self.graph.add_prior_factor_constant_bias(
            bias_key,
            &constantbias,
            &DiagonalNoiseModel::from_sigmas(nalgebra::Vector6::new(0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
        );
        self.initials.insert_constant_bias(bias_key, &constantbias);
        //Reference:
        // bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.5)
        // self.graph.push_back(gtsam.PriorFactorConstantBias(bias_key, gtsam.imuBias.ConstantBias(), bias_noise))
        // self.initial_estimate.insert(bias_key, gtsam.imuBias.ConstantBias())


        // Velocity prior
        // this should update velocity in current frame but needs last frame to have imu position and rotation
        if self.map.read()?.imu_initialized {
            self.imu.predict_state_last_frame(&mut self.current_frame, &mut self.last_frame.as_mut().unwrap());
        } else {
            // but what here???
        }
        let velocity_key = 2;
        let velocity = gtsam::base::vector::Vector3::new(0.0, 0.0, 0.0); // TODO update this with real velocity
        self.graph.add_prior_factor_vector3(
            velocity_key,
            &velocity,
            &IsotropicNoiseModel::from_dim_and_sigma(3, 0.5)
        );
        self.initials.insert_vector3(velocity_key, &velocity);
        //Reference:
        // velocity_key = V(0)
        // velocity_noise = gtsam.noiseModel.Isotropic.Sigma(3, .5)
        // velocity_0 = measured_vel[0]
        // self.graph.push_back(gtsam.PriorFactorVector(velocity_key, velocity_0, velocity_noise))
        // self.initial_estimate.insert(velocity_key, velocity_0)



        // # Preintegrator

        // curr_frame.imu_data.imu_preintegrated

        // accum = gtsam.PreintegratedImuMeasurements(self.IMU_PARAMS)

        // # Add measurements to factor graph
        // for i in range(1, n_frames):
        //     accum.integrateMeasurement(measured_acc[i], measured_omega[i], delta_t[i-1])
        //     if i % n_skip == 0:
        //         pose_key += 1
        //         DELTA = gtsam.Pose3(gtsam.Rot3.Rodrigues(0, 0, 0.1 * np.random.randn()),
        //                             gtsam.Point3(4 * np.random.randn(), 4 * np.random.randn(), 4 * np.random.randn()))
        //         self.initial_estimate.insert(pose_key, gtsam.Pose3(measured_poses[i]).compose(DELTA))

        //         velocity_key += 1
        //         self.initial_estimate.insert(velocity_key, measured_vel[i])

        //         bias_key += 1
        //         self.graph.add(gtsam.BetweenFactorConstantBias(bias_key - 1, bias_key, gtsam.imuBias.ConstantBias(), self.BIAS_COVARIANCE))
        //         self.initial_estimate.insert(bias_key, gtsam.imuBias.ConstantBias())

        //         # Add IMU Factor
        //         self.graph.add(gtsam.ImuFactor(pose_key - 1, velocity_key - 1, pose_key, velocity_key, bias_key, accum))

        //         # Reset preintegration
        //         accum.resetIntegration()

        Ok(())
    }

    fn add_keypoints(&mut self) {
        let mut points1: VectorOfPoint2f = self.last_frame.as_ref().unwrap().features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();


    //     // vision_data, measured_poses, args.n_skip, depth, axs
    //     let R_rect = nalgebra::Matrix4::<f64>::new(
    //         9.999239e-01, 9.837760e-03, -7.445048e-03, 0.0,
    //         -9.869795e-03, 9.999421e-01, -4.278459e-03, 0.0,
    //         7.402527e-03, 4.351614e-03, 9.999631e-01, 0.0,
    //         0.0, 0.0, 0.0, 1.0
    //     );
    //     let R_cam_velo = nalgebra::Matrix3::<f64>::new(
    //         7.533745e-03, -9.999714e-01, -6.166020e-04,
    //         1.480249e-02, 7.280733e-04, -9.998902e-01,
    //         9.998621e-01, 7.523790e-03, 1.480755e-02
    //     );
    //     let R_velo_imu = nalgebra::Matrix3::<f64>::new(
    //         9.999976e-01, 7.553071e-04, -2.035826e-03,
    //         -7.854027e-04, 9.998898e-01, -1.482298e-02,
    //         2.024406e-03, 1.482454e-02, 9.998881e-01
    //     );
    //     let t_cam_velo = nalgebra::Vector3::<f64>::new(-4.069766e-03, -7.631618e-02, -2.717806e-01);
    //     let t_velo_imu = nalgebra::Vector3::<f64>::new(-8.086759e-01, 3.195559e-01, -7.997231e-01);
    //     let mut T_velo_imu = nalgebra::Matrix4::<f64>::identity();
    //     let T_cam_velo = nalgebra::Matrix4::<f64>::identity();
    // //   T_velo_imu[3,3] = 1.
    // //   T_cam_velo[3,3] = 1.
    // //   T_velo_imu[:3,:3] = R_velo_imu
    // //   T_velo_imu[:3,3] = t_velo_imu
    // //   T_cam_velo[:3,:3] = R_cam_velo
    // //   T_cam_velo[:3,3] = t_cam_velo
    // //   cam_to_imu = R_rect @ T_cam_velo @ T_velo_imu
    // //   CAM_TO_IMU_POSE = gtsam.Pose3(cam_to_imu)
    // //   imu_to_cam = np.linalg.inv(cam_to_imu)
    // //   IMU_TO_CAM_POSE = gtsam.Pose3(imu_to_cam)

    //     let K_np = nalgebra::Matrix3::<f64>::new(
    //         9.895267e+02, 0.000000e+00, 7.020000e+02,
    //         0.000000e+00, 9.878386e+02, 2.455590e+02,
    //         0.000000e+00, 0.000000e+00, 1.000000e+00
    //     );
    //     // K = gtsam.Cal3_S2(K_np[0,0], K_np[1,1], 0., K_np[0,2], K_np[1,2])

    //     // valid_track = np.zeros(vision_data.shape[0], dtype=bool)
    //     // N = vision_data.shape[1]
    //     // for i in range(vision_data.shape[0]):
    //     //     track_length = N - np.sum(vision_data[i,:,0] == -1)
    //     //     if track_length > 1 and track_length < 0.5*N:
    //     //         valid_track[i] = True

    //     // count = 0
    //     // measurement_noise = gtsam.noiseModel.Isotropic.Sigma(2, 10.0) 
    //     // for i in range(0, vision_data.shape[0], 20):
    //     //     if not valid_track[i]:
    //     //         continue
    //     //     key_point_initialized=False 
    //     //     for j in range(vision_data.shape[1]-1):
    //     //     if vision_data[i,j,0] >= 0:
    //     //         zp = float(depth[j * n_skip][vision_data[i,j,1], vision_data[i,j,0], 2])
    //     //         if zp == 0:
    //     //             continue
    //     //         self.graph.push_back(gtsam.GenericProjectionFactorCal3_S2(
    //     //         vision_data[i,j,:], measurement_noise, X(j), L(i), K, IMU_TO_CAM_POSE))
    //     //         if not key_point_initialized:
    //     //             count += 1

    //     //             # Initialize landmark 3D coordinates
    //     //             fx = K_np[0,0]
    //     //             fy = K_np[1,1]
    //     //             cx = K_np[0,2]
    //     //             cy = K_np[1,2]

    //     //             # Depth:
    //     //             zp = float(depth[j * n_skip][vision_data[i,j,1], vision_data[i,j,0], 2])
    //     //             xp = float(vision_data[i,j,0] - cx) / fx * zp
    //     //             yp = float(vision_data[i,j,1] - cy) / fy * zp

    //     //             # Convert to global
    //     //             Xg = measured_poses[j*n_skip] @ imu_to_cam @ np.array([xp, yp, zp, 1])
    //     //             axs.scatter(Xg[0], Xg[1], s=10)
    //     //             self.initial_estimate.insert(L(i), Xg[:3])

    //     //             key_point_initialized = True

    }

    fn estimate(&mut self) {
    //     // self.optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate, SOLVER_PARAMS)
    //     // self.result = self.optimizer.optimize()

        let mut opt_param = LevenbergMarquardtParams::default();
        opt_param.set_max_iterations(self.max_iterations);
        opt_param.set_lambda_upper_bound(self.lambda_upper_bound);
        opt_param.set_lambda_lower_bound(self.lambda_lower_bound);
        opt_param.set_diagonal_damping(true);
        opt_param.set_relative_error_to_l(self.relative_error_tol);
        opt_param.set_absolute_error_to_l(self.absolute_error_tol);

        let mut opt = LevenbergMarquardtOptimizer::new(&self.graph, &self.initials, &opt_param);

        let result = opt.optimize_safely();
        // for i in 1..=5 {
        //     let pose: nalgebra::Isometry3<f64> = result.get_pose3(i).unwrap().into();
        //     println!("[{i}] {pose}");
        // }

    //     // return self.result
    }


    // fn update_trajectory_in_logs(
    //     &mut self, created_new_kf: bool,
    // ) -> Result<(), Box<dyn std::error::Error>> {
    //     let relative_pose = {
    //         let reference_pose = if created_new_kf {
    //             // If we created a new kf this round, the relative keyframe pose is the same as the current frame pose.
    //             self.current_frame.pose.unwrap()
    //         } else if self.last_frame.as_ref().unwrap().pose.is_none() {
    //             // This condition should only happen for the first frame after initialization
    //             self.map.read()?.get_keyframe(self.ref_kf_id.unwrap()).get_pose().inverse()
    //         } else {
    //             self.last_frame.as_ref().unwrap().pose.unwrap()
    //         };

    //         self.current_frame.pose.unwrap() * reference_pose.inverse()
    //     };

    //     self.trajectory_poses.push(relative_pose);

    //     info!("Frame {} pose: {:?}", self.current_frame.frame_id, self.current_frame.pose);

    //     self.system.send(
    //         SHUTDOWN_ACTOR, 
    //         Box::new(TrajectoryMsg{
    //             pose: self.current_frame.pose.unwrap().inverse(),
    //             ref_kf_id: self.current_frame.ref_kf_id.unwrap(),
    //             timestamp: self.current_frame.timestamp,
    //             map_version: self.map.read()?.version
    //         })
    //     );

    //     let map = self.map.read()?;
    //     let bla = map.get_keyframe(self.ref_kf_id.unwrap()).get_mp_matches().iter().filter_map(|v| match v { 
    //        Some((id, _is_outlier)) => Some(*id),
    //        None => None
    //     });
    //     self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
    //         pose: self.current_frame.pose.unwrap(),
    //         mappoint_matches: vec![],
    //         nontracked_mappoints: HashMap::new(),
    //         mappoints_in_tracking: bla.collect(),
    //         timestamp: self.current_frame.timestamp,
    //         map_version: self.map.read()?.version
    //     }));

    //     Ok(())
    // }

    fn need_new_keyframe(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        let num_kfs = self.map.read()?.num_keyframes();

        if self.sensor.is_imu() && !self.map.read()?.imu_initialized {
            if (self.current_frame.timestamp - self.last_kf_timestamp.unwrap()) * 1e9 >= 0.25 { // 250 milliseconds
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

