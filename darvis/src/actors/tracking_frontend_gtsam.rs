extern crate g2o;
use gtsam::{imu::imu_bias::ConstantBias, navigation::combined_imu_factor::{PreintegratedCombinedMeasurements, PreintegrationCombinedParams}};
use log::{warn, info, debug, error};
use nalgebra::{Matrix3, Matrix6, SMatrix, Vector3};
use std::{collections::{BTreeMap, BTreeSet, HashMap, VecDeque}, fmt::{self, Formatter}, mem, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{core::{no_array, KeyPoint, Point, Point2f, Scalar, CV_8U, CV_8UC1}, imgproc::circle, prelude::*, types::{VectorOfKeyPoint, VectorOfMat, VectorOfPoint2f, VectorOfu8}};
use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{
        messages::{ImageMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg},
    }, map::{features::Features, frame::Frame, keyframe::MapPointMatches, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{good_features_to_track::GoodFeaturesExtractor, image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, ImuModule}, optimizer, orbslam_extractor::ORBExtractor, relocalization::Relocalization}, registered_actors::{new_feature_extraction_module, CAMERA_MODULE, FEATURE_DETECTION, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}, MAP_INITIALIZED
};
use crate::registered_actors::IMU;

use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{FeatureTracksAndIMUMsg, ImagePathMsg, InitKeyFrameMsg, NewKeyFrameMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackedMapPointData};
use crate::modules::module_definitions::MapInitializationModule;

pub struct TrackingFrontendGTSAM {
    system: System,
    sensor: Sensor,
    state: GtsamFrontendTrackingState,

    /// Frontend
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    imu_measurements_since_last_kf: ImuMeasurements,
    prev_points: VectorOfPoint2f,
    prev_points_ids: Vec<Id>,

    // Frames
    last_frame: Option<Frame>,
    curr_frame_id: i32,
    current_frame: Frame,

    // IMU 
    // I know this is hacky but I dont' want to figure out how to merge the gtsam imu preintegration object with the orbslam imu object
    // Imu_for_init object needed for map initialization but that's it, otherwise should use the GtsamIMUModule
    imu: GtsamIMUModule,
    imu_for_init: IMU,
    current_bias: ImuBias, // TODO SOFIYA NEED TO READ THIS VALUE FROM BACKEND AFTER IT IS OPTIMIZED

    /// References to map
    // map_initialized: bool,
    map: ReadWriteMap,
    initialization: Option<MapInitialization>, // data sent to map actor to initialize new map
    map_scale: f64,

    /// Global defaults
    first_image_time: f64,
    max_features: i32,
    min_distance: f64,
}

impl Actor for TrackingFrontendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let orb_extractor_ini = match sensor.is_mono() {
            true => Some(new_feature_extraction_module(true)),
            false => None
        };

        let mut actor = TrackingFrontendGTSAM {
            system,
            orb_extractor_left: new_feature_extraction_module(false),
            orb_extractor_ini,
            sensor,
            map,
            initialization: Some(MapInitialization::new()),
            state: GtsamFrontendTrackingState::NotInitialized,
            imu: GtsamIMUModule::new(
                SETTINGS.get::<f64>(IMU, "noise_gyro"), SETTINGS.get::<f64>(IMU, "gyro_walk"),
                SETTINGS.get::<f64>(IMU, "noise_acc"), SETTINGS.get::<f64>(IMU, "acc_walk"),
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
            ),
            imu_for_init: IMU::new(),
            last_frame: None,
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            curr_frame_id: 0,
            prev_points_ids: vec![],
            first_image_time: 0.0,
            max_features: SETTINGS.get::<i32>(TRACKING_FRONTEND, "max_features"),
            min_distance: SETTINGS.get::<f64>(TRACKING_FRONTEND, "min_distance"),
            imu_measurements_since_last_kf: ImuMeasurements::new(),
            prev_points: VectorOfPoint2f::new(),
            map_scale: 1.0,
            current_bias: ImuBias::new(),
        };
        tracy_client::set_thread_name!("tracking frontend gtsam");

        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message) {
                break;
            }
            actor.map.match_map_version();
        }
    }

}

impl TrackingFrontendGTSAM {
    fn handle_message(&mut self, message: MessageBox) -> bool {
        if message.is::<ImagePathMsg>() || message.is::<ImageMsg>() {
            if self.system.queue_full() {
                // Abort additional work if there are too many frames in the msg queue.
                info!("Tracking gtsam dropped 1 frame");
                return false;
            }
            let _span = tracy_client::span!("track");

            let (image, timestamp, mut imu_measurements) = if message.is::<ImagePathMsg>() {
                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (image::read_image_file(&msg.image_path), msg.timestamp, msg.imu_measurements)
            } else {
                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (msg.image, msg.timestamp, msg.imu_measurements)
            };
            self.imu_measurements_since_last_kf.append(&mut imu_measurements);

            debug!("Tracking frontend working on frame {}", self.curr_frame_id);

            let mut pub_this_frame = false;
            if matches!(self.state, GtsamFrontendTrackingState::NotInitialized) {
                // If map is not initialized yet, just extract features and try to initialize
                self.initialize_map(&image, timestamp).unwrap();

                if matches!(self.state, GtsamFrontendTrackingState::Ok) {
                    // If initialized successsfully,
                    // publish this frame so backend has a reference to the frame associated with the initialization
                    pub_this_frame = true;
                }
            } else {
                // If this is not the first image, track features from the last image
                self.current_frame = Frame::new_no_features(
                    self.curr_frame_id, 
                    Some(image.clone()),
                    timestamp,
                    Some(& self.last_frame.as_mut().unwrap())
                ).expect("Could not create frame!");

                // Preintegration
                self.imu.preintegrate(&mut self.imu_measurements_since_last_kf, &mut self.current_frame, & self.last_frame.as_ref().unwrap());

                // Tracking
                let (tracked_kp, total_kp) = self.optical_flow().unwrap();

                // determine if frame should be a keyframe
                let need_new_kf = self.need_new_keyframe();

                if need_new_kf {
                    // If we need a new keyframe, publish this frame
                    pub_this_frame = true;
                }

                // let transform = self.calculate_transform().unwrap();
                // let new_trans = *transform.get_translation() * (self.map_scale);
                // let new_pose = Pose::new(new_trans, * transform.get_rotation()) * self.last_frame.as_ref().unwrap().pose.unwrap();

                // self.current_frame.pose = Some(new_pose);
                // debug!("Optical flow tracked {} from original {}... pose prediction: {:?}", tracked_kp, total_kp, new_pose);

                debug!("SOFIYA FEATURES. After optical flow, frame has N {}, features {}, mappoint matches {}", self.current_frame.features.num_keypoints, self.current_frame.features.get_all_keypoints().len(), self.current_frame.mappoint_matches.len());
            }

            self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
                keypoints: DVVectorOfKeyPoint::empty(),
                image,
                timestamp,
            }));

            if pub_this_frame {
                // TODO SOFIYA double check that these commented out portions are ok...
                // self.outlier_rejection_mono(); // Can get away without doing this?
                self.extract_features_and_add_to_existing();
                // self.undistort_keypoints(); // Is this even used?
                // self.get_smart_mono_measurements(); // THIS IS THE SAME AS the feature vectors matching to mappoint IDs in the current frame

                tracy_client::Client::running()
                    .expect("message! without a running Client")
                    .message("Publish frame!", 2);

                self.imu_measurements_since_last_kf = ImuMeasurements::new();
                let preintegration_results = self.imu.reset_preintegration(self.current_bias);

                self.system.send(TRACKING_BACKEND, Box::new(FeatureTracksAndIMUMsg {
                    frame: self.current_frame.clone(),
                    preintegration_results,
                    mappoint_ids: self.prev_points_ids.clone()
                }));
            }

            // Swap current and last frame to avoid cloning current frame into last frame
            // At next iteration, current frame will be immediately overwritten with the real current frame
            std::mem::swap(&mut self.last_frame, &mut Some(self.current_frame));
            self.curr_frame_id += 1;
        } else if message.is::<ShutdownMsg>() {
            return true;
        } else {
            warn!("Tracking backend received unknown message type!");
        }
        return false;
    }

    fn initialize_map(&mut self, curr_img: &opencv::core::Mat, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("initialize_map");
        let (keypoints, descriptors) = match self.sensor {
            Sensor(FrameSensor::Mono, _) => {
                if !matches!(self.state, GtsamFrontendTrackingState::NotInitialized) || (self.current_frame.frame_id < SETTINGS.get::<f64>(SYSTEM, "fps") as i32) {
                    self.orb_extractor_ini.as_mut().unwrap().extract(& curr_img).unwrap()
                } else {
                    self.orb_extractor_left.extract(& curr_img).unwrap()
                }
            },
            _ => { 
                // See GrabImageMonocular, GrabImageStereo, GrabImageRGBD in Tracking.cc
                todo!("Stereo, RGBD")
            }
        };

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
        let init_success = self.initialization.as_mut().unwrap().try_initialize(&self.current_frame, &mut self.imu_for_init.imu_preintegrated_from_last_kf)?;
        if init_success {
            {
                let _ = match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map,  &mut self.imu_for_init.imu_preintegrated_from_last_kf)? {
                    Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, _curr_kf_timestamp, map_scale)) => {
                        // Map needs to be initialized before tracking can begin. Received from map actor
                        if self.sensor.is_imu() {
                            self.imu.bias = self.map.read()?.get_keyframe( curr_kf_id).imu_data.imu_preintegrated.as_ref().unwrap().get_updated_bias();
                        }

                        {
                            // Set current frame's updated info from map initialization
                            self.current_frame.pose = Some(curr_kf_pose);
                            self.current_frame.ref_kf_id = Some(curr_kf_id);
                            self.map_scale = map_scale;
                        }
                        self.state = GtsamFrontendTrackingState::Ok;

                        {
                            let map = self.map.read()?;
                            let curr_kf = map.get_keyframe(curr_kf_id);
                            for mp_id in local_mappoints {
                                let index = curr_kf.get_mp_match_index(&mp_id).unwrap();
                                let kp = curr_kf.features.get_keypoint(index).0;
                                self.prev_points.push(kp.pt());
                                self.prev_points_ids.push(mp_id);
                            }
                        }

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

            // self.map_initialized = true;

            self.state = GtsamFrontendTrackingState::Ok;
        } else {
            self.state = GtsamFrontendTrackingState::NotInitialized;
        }
        Ok(())
    }

    fn optical_flow(&mut self) -> Result<(u32, usize), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("optical_flow");

        let mut status = VectorOfu8::new();

        let frame1 = self.last_frame.as_mut().unwrap();

        // let mut points1: VectorOfPoint2f = frame1.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();
        // let mut points1 = &mut self.prev_points;
        let mut points2 = VectorOfPoint2f::new();

        //this function automatically gets rid of points for which tracking fails
        let mut err = opencv::types::VectorOff32::default();
        let win_size = opencv::core::Size::new(24, 24);
        let termcrit = opencv::core::TermCriteria {
          typ: 3,
          max_count: 30,
          epsilon: 0.01,
        };
        let max_level = 3;
        opencv::video::calc_optical_flow_pyr_lk(
          & frame1.image.as_ref().unwrap(), & self.current_frame.image.as_ref().unwrap(), &mut self.prev_points, &mut points2, &mut status, &mut err, win_size, max_level, termcrit, 0, 0.001,
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
            self.prev_points.remove(i - indez_correction)?;
            self.prev_points_ids.remove(i - indez_correction);
            points2.remove(i - indez_correction)?;
            frame1.features.remove_keypoint(i - indez_correction)?;

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

    fn need_new_keyframe(&mut self) -> bool {
        let kf_diff_ns = (self.current_frame.timestamp - self.last_frame.as_ref().unwrap().timestamp) * 1e9;
        let nr_valid_features = self.current_frame.features.num_keypoints;

        let max_time_elapsed = kf_diff_ns >= (10.0 * 10e6);
        let min_time_elapsed = kf_diff_ns >= (0.2 * 10e6);
        let nr_features_low = nr_valid_features <= 0;

        let matches_ref_cur = self.find_matching_keypoints();

        // check for large enough disparity
        let disparity = self.compute_median_disparity(matches_ref_cur);

        let is_disparity_low = disparity < 0.5;
        let disparity_low_first_time = is_disparity_low && !matches!(self.state, GtsamFrontendTrackingState::LowDisparity);
        let enough_disparity = !is_disparity_low;

        let max_disparity_reached = disparity > 200.0;
        let disparity_flipped = (enough_disparity || disparity_low_first_time) && min_time_elapsed;

        max_time_elapsed || max_disparity_reached || disparity_flipped || nr_features_low
    }

    fn find_matching_keypoints(&mut self) -> Vec<(i32, i32)> {
        // Find keypoints that observe the same landmarks in both frames

        let mut curr_frame_map: HashMap<Id, i32> = HashMap::new();
        for idx1 in 0..self.current_frame.features.get_all_keypoints().len() - 1 {
            let mp_id = * self.prev_points_ids.get(idx1 as usize).unwrap();
            curr_frame_map.insert(mp_id, idx1);
            // While we're here, add the mappoint to the frame's mappoint matches. Backend will need this
            self.current_frame.mappoint_matches.add(idx1 as u32, mp_id, false);

        }

        // Map of position of landmark j in ref frame to position of landmark j in cur_frame
        let last_frame = self.last_frame.as_ref().unwrap();
        let mut matches = vec![];
        for idx_in_last_frame in 0..last_frame.mappoint_matches.len() - 1 {
            let mp_id = last_frame.mappoint_matches.get(idx_in_last_frame as usize);
            match mp_id {
                None => continue,
                Some((mp_id, _is_outlier)) => {
                    match curr_frame_map.get(&mp_id) {
                        None => continue,
                        Some(idx_in_curr_frame) => {
                            matches.push((idx_in_last_frame as i32, * idx_in_curr_frame as i32));
                        }
                    }
                }
            }
        }
        return matches;
    }

    fn compute_median_disparity(&mut self, matches: Vec<(i32, i32)>) -> f32 {
        // Compute disparity
        let mut disparity_sq: Vec<f32> = vec![];
        for (idx_in_last_frame, idx_in_curr_frame) in matches {
            let px_diff = self.current_frame.features.get_keypoint(idx_in_curr_frame as usize).0.pt() - self.last_frame.as_ref().unwrap().features.get_keypoint(idx_in_last_frame as usize).0.pt();
            let px_dist = px_diff.x * px_diff.x + px_diff.y * px_diff.y;
            disparity_sq.push(px_dist);
        }

        if disparity_sq.is_empty() {
            warn!("Have no matches for disparity computation.");
            return 0.0;
        }

        // Compute median:
        let center = disparity_sq.len() / 2;
        // nth element sorts the array partially until it finds the median.
        pdqselect::select_by(&mut disparity_sq, center, |a, b| a.partial_cmp(b).unwrap());
        return disparity_sq[center].sqrt();
    }

    fn outlier_rejection_mono(&mut self) {
        let matches_ref_cur = self.find_matching_keypoints();

        // NOTE: versors are already in the rectified left camera frame.
        // No further rectification needed.
        //* Get bearing vectors for opengv.

        // BearingVectors f_ref;
        // BearingVectors f_cur;
        // const size_t& n_matches = matches_ref_cur.size();
        // f_ref.reserve(n_matches);
        // f_cur.reserve(n_matches);
        // for (const KeypointMatch& it : matches_ref_cur) {
        //     //! Reference bearing vector
        //     CHECK_LT(it.first, ref_bearings.size());
        //     const auto& ref_bearing = ref_bearings.at(it.first);
        //     f_ref.push_back(ref_bearing);

        //     //! Current bearing vector
        //     CHECK_LT(it.second, cur_bearings.size());
        //     const auto& cur_bearing = cur_bearings.at(it.second);
        //     f_cur.push_back(cur_bearing);
        // }

        // //! Setup adapter.
        // CHECK_GT(f_ref.size(), 0);
        // CHECK_EQ(f_ref.size(), f_cur.size());
        // CHECK_EQ(f_ref.size(), n_matches);
        // Adapter2d2d adapter(f_ref, f_cur);
        // if (tracker_params_.ransac_use_2point_mono_) {
        //     adapter.setR12(cam_lkf_Pose_cam_kf.rotation().matrix());
        //     adapter.sett12(cam_lkf_Pose_cam_kf.translation().matrix());
        // }

        // //! Solve problem.
        // gtsam::Pose3 best_pose = gtsam::Pose3();
        // bool success = false;
        // if (tracker_params_.ransac_use_2point_mono_) {
        //     success = runRansac(std::make_shared<Problem2d2dGivenRot>(
        //                             adapter, tracker_params_.ransac_randomize_),
        //                         tracker_params_.ransac_threshold_mono_,
        //                         tracker_params_.ransac_max_iterations_,
        //                         tracker_params_.ransac_probability_,
        //                         tracker_params_.optimize_2d2d_pose_from_inliers_,
        //                         &best_pose,
        //                         inliers);
        // } else {
        //     success = runRansac(
        //         std::make_shared<Problem2d2d>(adapter,
        //                                     tracker_params_.pose_2d2d_algorithm_,
        //                                     tracker_params_.ransac_randomize_),
        //         tracker_params_.ransac_threshold_mono_,
        //         tracker_params_.ransac_max_iterations_,
        //         tracker_params_.ransac_probability_,
        //         tracker_params_.optimize_2d2d_pose_from_inliers_,
        //         &best_pose,
        //         inliers);
        // }

        // if (!success) {
        //     status_pose = std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
        // } else {
        //     // TODO(Toni): it seems we are not removing outliers if we send an invalid
        //     // tracking status (above), but the backend calls addLandmarksToGraph even
        //     // when we have an invalid status!

        //     // TODO(Toni): check quality of tracking
        //     //! Check enough inliers.
        //     TrackingStatus status = TrackingStatus::VALID;
        //     if (inliers->size() <
        //         static_cast<size_t>(tracker_params_.minNrMonoInliers_)) {
        //     CHECK(!inliers->empty());
        //     status = TrackingStatus::FEW_MATCHES;
        //     }

        //     // NOTE: 2-point always returns the identity rotation, hence we have to
        //     // substitute it:
        //     if (tracker_params_.ransac_use_2point_mono_) {
        //     CHECK(cam_lkf_Pose_cam_kf.rotation().equals(best_pose.rotation()));
        //     }

        //     //! Fill debug info.
        //     debug_info_.nrMonoPutatives_ = adapter.getNumberCorrespondences();
        //     debug_info_.nrMonoInliers_ = inliers->size();
        //     debug_info_.monoRansacIters_ = 0;  // no access to ransac from here
        //     // debug_info_.monoRansacIters_ = ransac->iterations_;

        //     status_pose = std::make_pair(status, best_pose);
        // }

        // VLOG(5) << "2D2D tracking " << (success ? " success " : " failure ") << ":\n"
        //         << "- Tracking Status: "
        //         << TrackerStatusSummary::asString(status_pose.first) << '\n'
        //         << "- Total Correspondences: " << f_ref.size() << '\n'
        //         << "\t- # inliers: " << inliers->size() << '\n'
        //         << "\t- # outliers: " << f_ref.size() - inliers->size() << '\n'
        //         << "- Best pose: \n"
        //         << status_pose.second;

        // return status_pose;

    

    }


    fn extract_features_and_add_to_existing(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("extract features");

        let num_features_to_find = 200 - self.current_frame.features.num_keypoints as i32;

        if num_features_to_find < 0 {
            warn!("Already have enough features ({}), not extracting more", self.current_frame.features.num_keypoints);
            return Ok(());
        }

        let mut keypoints = opencv::types::VectorOfPoint2f::new();

        let mut mask = opencv::core::Mat::new_rows_cols_with_default(
            self.current_frame.image.as_ref().unwrap().rows(),
            self.current_frame.image.as_ref().unwrap().cols(),
            CV_8U,
            Scalar::all(255.0)
        ).unwrap();
        for i in 0..self.current_frame.features.num_keypoints {
            let (kp, _) = self.current_frame.features.get_keypoint(i as usize);
            circle(&mut mask, Point::new(kp.pt().x as i32, kp.pt().y as i32), 20, Scalar::all(0.0), -1, 8, 0).unwrap();
        }

        opencv::imgproc::good_features_to_track(
            & self.current_frame.image.as_ref().unwrap(), &mut keypoints, num_features_to_find as i32, 0.01, self.min_distance, &mut mask, 3, false, 0.04
        ).unwrap();
        println!("Tracked features NEW: {}", keypoints.len());

        let mut new_keypoints = VectorOfKeyPoint::new();
        let mut new_descriptor = Mat::default();
        let mut new_descriptor_vec = VectorOfMat::new();

        for i in 0..self.current_frame.features.num_keypoints {
            new_keypoints.push(self.current_frame.features.get_keypoint(i as usize).0);
            new_descriptor_vec.push((* self.current_frame.features.descriptors.row(i as u32)).clone());
        }

        for i in 0..keypoints.len() {
            let kp = keypoints.get(i)?;
            new_keypoints.push(KeyPoint::new_coords(kp.x, kp.y, 31.0, -1.0, 0.0, 0, -1).expect("Failed to create keypoint"));
            // Note... creating a fake descriptor here because good features to track doesn't give descriptors back
            new_descriptor_vec.push(Mat::new_rows_cols_with_default(1, 32, CV_8UC1, Scalar::all(0.0)).expect("Failed to create descriptor"));
            self.prev_points.push(kp);
            self.prev_points_ids.push(-1);
        }

        opencv::core::vconcat(&new_descriptor_vec, &mut new_descriptor).expect("Failed to concatenate");

        println!("Re-extracting features, before: {}, after: {}", self.current_frame.features.num_keypoints, new_keypoints.len());

        self.current_frame.replace_features(DVVectorOfKeyPoint::new(new_keypoints), DVMatrix::new(new_descriptor))?;

        Ok(())
    }

    fn calculate_transform(&self) -> Result<Pose, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("calculate_transform");
        // recovering the pose and the essential matrix
        let prev_features: VectorOfPoint2f = self.prev_points.clone();
        let curr_features: VectorOfPoint2f = self.current_frame.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();

        let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
        let essential_mat = opencv::calib3d::find_essential_mat(
          &prev_features,
          &curr_features,
          &CAMERA_MODULE.k_matrix.mat(),
          opencv::calib3d::RANSAC,
          0.999,
          1.0,
          1000,
          &mut mask,
        )?;
        opencv::calib3d::recover_pose_estimated(
          &essential_mat,
          &prev_features,
          &curr_features,
          &CAMERA_MODULE.k_matrix.mat(),
          &mut recover_r,
          &mut recover_t,
          &mut mask,
        )?;

        let recover_t = nalgebra::Vector3::<f64>::new(
            *recover_t.at_2d::<f64>(0, 0)?,
            *recover_t.at_2d::<f64>(1, 0)?,
            *recover_t.at_2d::<f64>(2, 0)?
        );
        let recover_r = nalgebra::Matrix3::<f64>::new(
            *recover_r.at_2d::<f64>(0, 0)?, *recover_r.at_2d::<f64>(0, 1)?, *recover_r.at_2d::<f64>(0, 2)?,
            *recover_r.at_2d::<f64>(1, 0)?, *recover_r.at_2d::<f64>(1, 1)?, *recover_r.at_2d::<f64>(1, 2)?,
            *recover_r.at_2d::<f64>(2, 0)?, *recover_r.at_2d::<f64>(2, 1)?, *recover_r.at_2d::<f64>(2, 2)?
        );

        Ok(Pose::new(recover_t,recover_r))
    }

}



struct GtsamIMUModule {
    preint_gtsam: PreintegratedCombinedMeasurements, // IMU preintegration
    bias: ImuBias,
}
impl GtsamIMUModule {

    fn new(
        gyro_noise_density: f64, gyro_random_walk: f64,
        accel_noise_density: f64, accel_random_walk: f64, 
        prior_ba: [f64; 3], prior_bg: [f64; 3],
    ) -> GtsamIMUModule {
        // Create GTSAM preintegration parameters for use with Foster's version
        let prior_state_bias = gtsam::imu::imu_bias::ConstantBias::new(
            &gtsam::base::vector::Vector3::new(prior_ba[0], prior_ba[1], prior_ba[2]),
            &gtsam::base::vector::Vector3::new(prior_bg[0], prior_bg[1], prior_bg[2])
        );
        let mut params = PreintegrationCombinedParams::makesharedu();  // Z-up navigation frame: gravity points along negative Z-axis !!!
        params.set_params(
            accel_noise_density * accel_noise_density, // acc white noise in continuous
            gyro_noise_density * gyro_noise_density, // gyro white noise in continuous
            accel_random_walk * accel_random_walk, // acc bias in continuous
            gyro_random_walk * gyro_random_walk, // gyro bias in continuous
            0.1, // error committed in integrating position from velocities
            1e-5 // error in the bias used for preintegration
        );


        Self {
            preint_gtsam: PreintegratedCombinedMeasurements::new(params, &prior_state_bias),
            bias: ImuBias::new(),
        }
    }

    fn reset_preintegration(&mut self, new_bias: ImuBias) -> PreintegratedCombinedMeasurementsResults {
        let bias_convert = ConstantBias::new(
            &gtsam::base::vector::Vector3::new(new_bias.bax, new_bias.bay, new_bias.baz),
            &gtsam::base::vector::Vector3::new(new_bias.bwx, new_bias.bwy, new_bias.bwz)
        );
        let old_preint_gtsam: PreintegratedCombinedMeasurementsResults = self.preint_gtsam.into();
        self.preint_gtsam.reset_integration_and_set_bias(& bias_convert);
        old_preint_gtsam
    }

    fn preintegrate(&mut self, imu_measurements: &mut ImuMeasurements, current_frame: &Frame, last_frame: &Frame) -> Result<(), Box<dyn std::error::Error>> {
        // This function will create a discrete IMU factor using the GTSAM preintegrator class
        // This will integrate from the current state time up to the new update time
        let _span = tracy_client::span!("create_imu_factor");

        let mut imu_from_last_frame = VecDeque::with_capacity(imu_measurements.len()); // mvImuFromLastFrame
        let imu_per = 0.000000000001; // Used to be 0.000000000001 to adjust for different timestamp units, not sure why it needs to be reverted now. 0.001 in orbslam. 

        while !imu_measurements.is_empty() {
            if imu_measurements.front().unwrap().timestamp < last_frame.timestamp - imu_per {
                imu_measurements.pop_front();
            } else if imu_measurements.front().unwrap().timestamp < current_frame.timestamp - imu_per {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
            } else {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
                break;
            }
        }
        let n = imu_from_last_frame.len() - 1;

        // let other_imu = IMU::new();
        // let mut imu_preintegrated_from_last_frame = ImuPreIntegrated::new(previous_frame.imu_data.imu_bias);

        for i in 0..n {
            let mut tstep = 0.0;
            let mut acc: Vector3<f64> = Vector3::zeros(); // acc
            let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel

            if i == 0 && i < (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tini = imu_from_last_frame[i].timestamp - last_frame.timestamp;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tini/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tini/tab)
                ) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - last_frame.timestamp;
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
                tstep = current_frame.timestamp - last_frame.timestamp;
            }
            tstep = tstep * 1e9;

            self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum GtsamFrontendTrackingState {
    #[default] NotInitialized,
    LowDisparity,
    Ok,
}


// We need to be able to send the results from preintegrating in PreintegratedCombinedMeasurements
// to the backend, but cannot send pointer in a message. So, pull out necessary data and reconstruct
// the new PreintegratedCombinedMeasurements object on the backend.
pub struct PreintegratedCombinedMeasurementsResults {
    pub bias_acc_covariance: Matrix3<f64>,
    pub bias_omega_covariance: Matrix3<f64>,
    pub bias_acc_omega_int_covariance: Matrix6<f64>,
    pub preint_meas_cov: SMatrix<f64, 15, 15>,
}

impl Into<PreintegratedCombinedMeasurementsResults> for PreintegratedCombinedMeasurements {
    fn into(self) -> PreintegratedCombinedMeasurementsResults {
        let preint_meas_cov = self.get_preint_meas_cov();
        
        let bias_acc_covariance = self.get_bias_acc_covariance();
        let bias_omega_covariance = self.get_bias_omega_covariance();
        let bias_acc_omega_int_covariance = self.get_bias_acc_omega_int_covariance();
        let preint_meas_cov = self.get_preint_meas_covariance();
        PreintegratedCombinedMeasurementsResults {
            bias_acc_covariance,
            bias_omega_covariance,
            bias_acc_omega_int_covariance,
            preint_meas_cov,
        }
    }
}
