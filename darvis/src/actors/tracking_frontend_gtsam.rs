extern crate g2o;
use ahash::{HashMap, HashMapExt};
// use arrsac::Arrsac;
use gtsam::{imu::imu_bias::ConstantBias, inference::symbol::Symbol, navigation::combined_imu_factor::{PreintegratedCombinedMeasurements, PreintegrationCombinedParams}, sys::Rot3};
use log::{debug, error, info, warn};
use nalgebra::{UnitQuaternion, Vector3};
use std::{cmp::max, collections::VecDeque, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{core::{Point, Point2f, Scalar, CV_8U}, imgcodecs, imgproc::circle, prelude::*, types::{VectorOfKeyPoint, VectorOfPoint2f, VectorOff32, VectorOfu8}};
use core::{
    config::*, matrix::*, system::{Actor, MessageBox, System, Timestamp}
};
use std::fmt::Debug;
use num_traits::Pow;
use crate::{ImuInitializationData, actors::messages::ImuInitializationMsg, map::pose::{DVRotation, DVTranslation}, modules::{imu::{ImuBias, ImuCalib, PreintegrationGTSAM}, opengv_translation_only_sac::{CentralRelativeAdapter, Ransac, TranslationOnlySacProblem}}, registered_actors::IMU};

use crate::{
    actors::{local_mapping::LOCAL_MAPPING_IDLE, messages::{FeatureTracksAndIMUMsg, ImageMsg, ImagePathMsg, InitKeyFrameMsg, ShutdownMsg, TrajectoryMsg, VisFeaturesMsg}}, map::{frame::Frame, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image::{self, draw_optical_flow}, imu::{ImuMeasurements, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, MapInitializationModule}}, registered_actors::{new_feature_extraction_module, CAMERA_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}
};


pub struct TrackingFrontendGTSAM {
    system: System,
    state: GtsamFrontendTrackingState,
    map: ReadWriteMap,

    initialization_data: Option<ImuInitializationData>,

    /// Feature extractors
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    gftt: opencv::core::Ptr<opencv::features2d::GFTTDetector>,

    // IMU
    imu_measurements_since_last_kf: ImuMeasurements,

    // Feature IDs
    tracked_features_last_frame: TrackedFeatures,
    tracked_features_last_keyframe: TrackedFeatures,
    tracked_features: TrackedFeatures,
    removed_features: Vec<u64>,

    // Frames
    last_frame: Frame,
    curr_frame_id: i32,
    current_frame: Frame,
    last_keyframe: Frame,
    frames_since_last_kf: i32,
    last_kf_timestamp: Timestamp,

    preintegration: PreintegrationGTSAM,
    tbc: Pose, // Transform from camera frame to body frame (IMU)
}

impl Actor for TrackingFrontendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let feature_detector = opencv::features2d::GFTTDetector::create(
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "nonmaxsuppression__max_nr_keypoints_before_anms"),
            SETTINGS.get::<f64>(TRACKING_FRONTEND, "gftt_quality_level"),
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "gftt_min_distance_btw_tracked_and_detected_features") as f64,
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "gftt_block_size"),
            SETTINGS.get::<bool>(TRACKING_FRONTEND, "gftt_use_harris_corner_detector"),
            SETTINGS.get::<f64>(TRACKING_FRONTEND, "gftt_k"),
        ).unwrap();

        let mut actor = TrackingFrontendGTSAM {
            system,
            orb_extractor_ini: Some(new_feature_extraction_module(true)),
            gftt: feature_detector,
            map,
            state: GtsamFrontendTrackingState::FirstFrame,
            tracked_features: TrackedFeatures::default(),
            tracked_features_last_frame: TrackedFeatures::default(),
            tracked_features_last_keyframe: TrackedFeatures::default(),
            last_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            curr_frame_id: 0,
            imu_measurements_since_last_kf: ImuMeasurements::new(),
            frames_since_last_kf: 0,
            removed_features: vec![],
            last_kf_timestamp: 0.0,
            preintegration: PreintegrationGTSAM::default(),
            tbc: ImuCalib::new().tbc,
            last_keyframe: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            initialization_data: None,
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
            let _span = tracy_client::span!("track");

            // Abort additional work if there are too many frames in the msg queue.
            if self.system.queue_full() {
                info!("Tracking gtsam dropped 1 frame");
                return false;
            }

            let (image, image_color, timestamp, mut imu_measurements, imu_initialization) = if message.is::<ImagePathMsg>() {
                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (image::read_image_file(&msg.image_path, imgcodecs::IMREAD_GRAYSCALE), None, msg.timestamp, msg.imu_measurements, msg.imu_initialization)
            } else {
                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (msg.image, msg.color_image, msg.timestamp, msg.imu_measurements, msg.imu_initialization)
            };

            debug!("Tracking frontend working on frame {} at timestamp {}", self.curr_frame_id, timestamp);

            match self.state {
                GtsamFrontendTrackingState::FirstFrame => {
                    // For first frame, save initialization data
                    println!("SOFIYA INITIALIZATION, publish first frame at timestamp {}", timestamp);
                    self.initialization_data = imu_initialization;
                    self.state = GtsamFrontendTrackingState::NotInitialized;
                },
                GtsamFrontendTrackingState::NotInitialized => {
                    // Perform first feature extraction
                    let (keypoints, descriptors) = self.orb_extractor_ini.as_mut().unwrap().extract(& image).unwrap();
                    let init_pose = imu_initialization.as_ref().unwrap().pose;

                    self.current_frame = Frame::new( 
                        self.curr_frame_id, 
                        keypoints,
                        descriptors,
                        image.cols() as u32,
                        image.rows() as u32,
                        Some(image.clone()),
                        Some(& self.last_frame),
                        false,
                        timestamp,
                    ).expect("Could not create frame!");
                    self.current_frame.pose = Some(init_pose);
                    self.state = GtsamFrontendTrackingState::Ok;

                    self.preintegration.initialize(ImuBias::new());

                    println!("SOFIYA INITIALIZATION, publish initial frame at timestamp {}", timestamp);
                    self.publish_frame(None, self.initialization_data.clone());
                },
                GtsamFrontendTrackingState::Ok | GtsamFrontendTrackingState::LowDisparity | GtsamFrontendTrackingState::FewMatches | GtsamFrontendTrackingState::Invalid => {
                    // TODO Kimera not sure all four conditions should be here

                    // Regular tracking
                    self.current_frame = Frame::new_no_features(
                        self.curr_frame_id, 
                        Some(image.clone()),
                        timestamp,
                        Some(& self.last_frame)
                    ).expect("Could not create frame!");

                    self.preintegration.preintegrate_kimera(&mut imu_measurements);
                    let body_r_cam = self.tbc.get_rotation();
                    let cam_r_body = body_r_cam.try_inverse().unwrap();
                    let delta_rij = self.preintegration.get_delta_rij();
                    let kf_r_ref_frame = Some(DVRotation::new(cam_r_body * delta_rij * *body_r_cam));

                    // Optical flow
                    self.optical_flow().unwrap();

                    // Determine if frame should be a keyframe
                    if self.need_new_keyframe() {
                        self.publish_frame(kf_r_ref_frame, imu_initialization);
                    } else {
                        self.frames_since_last_kf += 1;
                    }
                }
            };

            self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
                keypoints: DVVectorOfKeyPoint::empty(),
                image,
                timestamp,
            }));

            // BOOKKEEPING TO SET LAST_FRAME = CURRENT_FRAME
            // Swap current and last frame to avoid cloning current frame into last frame
            // At next iteration, current frame will be immediately overwritten with the real current frame
            std::mem::swap(&mut self.last_frame, &mut self.current_frame);
            // Keep track of features from last frame
            self.tracked_features_last_frame = self.tracked_features.clone();
            self.curr_frame_id += 1;
        } else if message.is::<ShutdownMsg>() {
            // Sleep a little to allow other threads to finish
            sleep(Duration::from_millis(100));
            return true;
        } else {
            warn!("Tracking frontend GTSAM received unknown message type!");
        }
        return false;
    }

    fn publish_frame(&mut self, kf_r_ref_frame: Option<DVMatrix3<f64>>, imu_initialization: Option<ImuInitializationData>){
        tracy_client::Client::running()
            .expect("message! without a running Client")
            .message("Publish frame!", 2);

        debug!("Publish frame");

        if let Some(rot) = kf_r_ref_frame {
            let (state, tracking_pose) = self.geometric_outlier_rejection(rot);
            self.state = state;
            println!("AFTER OUTLIER REJECTION, STATE IS {:?}", self.state);
        }
        self.extract_new_features().expect("Couldn't extract good features to track?");

        // SEND TO BACKEND!
        self.system.send(TRACKING_BACKEND, Box::new(FeatureTracksAndIMUMsg {
            tracker_status: self.state,
            frame: self.current_frame.clone(),
            feature_tracks: self.tracked_features.clone(),
            imu_initialization,
            preintegration: self.preintegration.get_preintegration_clone(),
        }));

        self.last_kf_timestamp = self.current_frame.timestamp;
        self.removed_features.clear();
        self.tracked_features_last_keyframe = self.tracked_features.clone();

        // Draw optical flow for debugging
        // if self.last_frame.image.is_some() {
            // draw_optical_flow(
            //     self.last_frame.image.as_ref().unwrap(),
            //     self.current_frame.image.as_ref().unwrap(),
            //     & self.tracked_features_last_frame.get_points_as_vector_of_point2f(),
            //     & self.tracked_features.get_points_as_vector_of_point2f(),
            //     &format!("results/flow/front{}.png", self.curr_frame_id),
            // ).unwrap();
            // debug!("Frontend, tracked features last: {:?}", self.tracked_features_last_frame);
            // debug!("Frontend, tracked features now: {:?}", self.tracked_features);
        // }

        // Reset preintegration to result of latest optimization
        let latest_imu_bias = {
            let map = self.map.read().unwrap();
            if map.last_kf_id != -1 {
                let last_kf = map.get_keyframe(map.last_kf_id);
                println!("RESET PREINTEGRATION! LAST KF IS {}, BIAS IS: {:?}", last_kf.id, last_kf.imu_data.imu_bias);
                last_kf.imu_data.imu_bias.clone()
            } else {
                // NO KFs yet, set to default
                ImuBias::new()
            }
        };
        self.preintegration.reset_integration_and_set_bias(& latest_imu_bias);
        self.frames_since_last_kf = 0;
        self.last_keyframe = self.current_frame.clone();
    }

    fn optical_flow(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("optical_flow");
        let mut status = VectorOfu8::new();
        let mut points2 = VectorOfPoint2f::new();

        // Optical flow
        let window_size = SETTINGS.get::<i32>(TRACKING_FRONTEND, "opticalflow_winsize");
        opencv::video::calc_optical_flow_pyr_lk(
            & self.last_frame.image.as_ref().unwrap(),
            & self.current_frame.image.as_ref().unwrap(),
            & self.tracked_features.get_points_as_vector_of_point2f(),
            &mut points2,
            &mut status,
            &mut opencv::types::VectorOff32::default(),
            opencv::core::Size::new(window_size, window_size), 
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "opticalflow_maxlevel"),
            opencv::core::TermCriteria {
                typ: SETTINGS.get::<i32>(TRACKING_FRONTEND, "opticalflow_termcriteriatype"),
                max_count: SETTINGS.get::<i32>(TRACKING_FRONTEND, "opticalflow_maxiter"),
                epsilon: SETTINGS.get::<f64>(TRACKING_FRONTEND, "opticalflow_epsilon"),
            },
            0,
            1e-4,
        )?;

        let mut index_correction = 0; // We mutate self.tracked_features vectors while status and points2 lengths remain the same
        let mut total_tracked = 0;

        // TODO KIMERA: Delete features if they are older than the opticalflow_maxfeatureage: https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/frontend/Tracker.cpp#L174C23-L174C30
        // println!("Optical flow...");
        for i in 0..status.len() {
            let index_in_mutated = i - index_correction;
            let pt = points2.get(i)?;
            let status_ok = status.get(i)? == 1;

            if !status_ok || pt.x < 0.0 || pt.y < 0.0 {
                // FEATURE IS NOT TRACKED! Remove from tracked_features
                let removed_id = self.tracked_features.remove(index_in_mutated);
                self.tracked_features_last_frame.remove(index_in_mutated);
                self.tracked_features_last_keyframe.remove(index_in_mutated);
                index_correction = index_correction + 1;
                self.removed_features.push(removed_id as u64);
            } else {
                // FEATURE IS FOUND! Update the point in tracked_features
                // Not updating current_frame's features yet, we will do this if this frame becomes a new keyframe during the feature extraction step.
                let bearing_vector = self.get_bearing_vector(&pt);
                self.tracked_features.update(index_in_mutated, pt, bearing_vector);

                // println!("{:?}, {:?}", pt, bearing_vector);
                total_tracked += 1;
            }
        }

        debug!("Optical flow tracked {} from original {}", total_tracked, status.len());
        // debug!("Removed: {:?}", self.removed_features);
        Ok(())
    }

    fn geometric_outlier_rejection(
        &mut self, 
        rotation: DVRotation
    ) -> (GtsamFrontendTrackingState, Pose) {
        // Begin void VisionImuFrontend::outlierRejectionMono
        // Checks inside this are not relevant to euroc monocular

        // Begin TrackingStatusPose Tracker::geometricOutlierRejection2d2d(
        //    Frame* ref_frame, Frame* cur_frame, const gtsam::Pose3& cam_lkf_Pose_cam_kf)

        let cam_lkf_pose_cam_kf = Pose::new_with_default_trans(*rotation);

        // println!("Current keypoints: {:?}", self.tracked_features);
        // println!("Last keyframe keypoints: {:?}", self.tracked_features_last_keyframe);

        // Sofiya... think I don't have to do this because of the way I keep track of features
        // let matches_ref_cur = {
        //     // Begin void Tracker::findMatchingKeypoints(const Frame& ref_frame,
        //     //                             const Frame& cur_frame,
        //     //                             KeypointMatches* matches_ref_cur)
        //     // Find keypoints that observe the same landmarks in both frames:
        //     let mut matches: Vec<(usize, usize)> = vec![];
        //     let ref_lm_index_map: HashMap<i32, usize> = HashMap::new();

        //     for i in 0..self.tracked_features_last_keyframe.len() {
        //         let kf_point = self.tracked_features_last_keyframe.get_point(i);
        //         let curr_f_point = self.tracked_features.get_point(i);
        //         matches.push((kf_point as usize, curr_f_point as usize));
        //     }

        //     // for (size_t i = 0; i < ref_frame.landmarks_.size(); ++i) {
        //     //     const LandmarkId& ref_id = ref_frame.landmarks_.at(i);
        //     //     if (ref_id != -1) {
        //     //     // Map landmark id -> position in ref_frame.landmarks_
        //     //     ref_lm_index_map[ref_id] = i;
        //     //     }
        //     // }

        //     // // Map of position of landmark j in ref frame to position of landmark j in
        //     // // cur_frame
        //     // matches_ref_cur->reserve(ref_lm_index_map.size());
        //     // for (size_t i = 0; i < cur_frame.landmarks_.size(); ++i) {
        //     //     const LandmarkId& cur_id = cur_frame.landmarks_.at(i);
        //     //     if (cur_id != -1) {
        //     //     auto it = ref_lm_index_map.find(cur_id);
        //     //     if (it != ref_lm_index_map.end()) {
        //     //         matches_ref_cur->push_back(std::make_pair(it->second, i));
        //     //     }
        //     //     }
        //     // }
        //     matches
        // };

        if self.tracked_features_last_keyframe.len() == 0 {
            error!("No matching keypoints from frame {} to frame {}. Mono Tracking Status = INVALID.", self.last_frame.frame_id, self.current_frame.frame_id);
            return (GtsamFrontendTrackingState::Invalid, Pose::default());
        } else {
            let (inliers, state, pose) = self.geometric_outlier_rejection_inner(
                &cam_lkf_pose_cam_kf
            );

            // Remove correspondences classified as outliers
            if !matches!(state, GtsamFrontendTrackingState::FewMatches) {
                self.remove_outliers(&inliers);
            }

            // THIS IS ONLY USEFUL if we are completely still...
            if matches!(state, GtsamFrontendTrackingState::Ok) {
                // Check enough disparity.
                if let Some(disparity) = self.compute_median_disparity(&self.tracked_features_last_keyframe, &self.tracked_features) {
                    println!("Disparity: {} / {}", disparity, SETTINGS.get::<f64>(TRACKING_FRONTEND, "disparity_threshold") as f32);
                    if disparity < SETTINGS.get::<f64>(TRACKING_FRONTEND, "disparity_threshold") as f32 {
                        info!("Low mono disparity.");
                        return (GtsamFrontendTrackingState::LowDisparity, pose);
                    }
                } else {
                    error!("Median disparity calculation failed...");
                }
            }
            return (state, pose);
        }
    }

    fn remove_outliers(
        &mut self,
        inliers: & Vec<usize>,
    ) {
        // void Tracker::removeOutliersMono(const std::vector<int>& inliers,
        //                          Frame* ref_frame,
        //                          Frame* cur_frame,
        //                          KeypointMatches* matches_ref_cur)


        // println!("Inliers: {:?}", inliers);
        // println!("Matches ref cur size: {}", self.tracked_features_last_keyframe.len());

        // Find indices of outliers in current frame.
        let outliers: Vec<usize> = {
            // void Tracker::findOutliers(const KeypointMatches& matches_ref_cur,
            //                std::vector<int> inliers,
            //                std::vector<int>* outliers)

            // Get outlier indices from inlier indices.
            // std::sort(inliers.begin(), inliers.end(), std::less<size_t>());

            let mut outliers = vec![];
                // outliers->reserve(matches_ref_cur.size() - inliers.size());

            // The following is a complicated way of computing a set difference
            let mut k = 0;
            for i in 0..self.tracked_features_last_keyframe.len() {
                if k < inliers.len() // If we haven't exhaused inliers
                    && i > inliers[k] // If we are after the inlier[k]
                {
                    k += 1; // Check the next inlier
                }

                if k >= inliers.len() || i != inliers[k] { // If i is not an inlier 
                    outliers.push(i);
                }
            }
            println!("Outliers: {:?}", outliers);
            outliers

                // for (size_t i = 0u; i < matches_ref_cur.size(); ++i) {
                //     if (k < inliers.size()                    // If we haven't exhaused inliers
                //         && static_cast<int>(i) > inliers[k])  // If we are after the inlier[k]
                //     ++k;                                    // Check the next inlier
                //     if (k >= inliers.size() ||
                //         static_cast<int>(i) != inliers[k])  // If i is not an inlier
                //     outliers->push_back(i);
                // }
                // }
        };

        // Remove outliers.
        // outliers cannot be a vector of size_t because opengv uses a vector of
        // int.
            // for out in outliers {
            //         // const auto& ref_kp_cur_kp = (*matches_ref_cur)[out];
            //         // ref_frame->landmarks_.at(ref_kp_cur_kp.first) = -1;
            //         // cur_frame->landmarks_.at(ref_kp_cur_kp.second) = -1;
            // }

        // Store only inliers from now on.
        let mut outlier_free_tracked_last_kf = TrackedFeatures::default();
        let mut outlier_free_tracked_current = TrackedFeatures::default();
        let mut outlier_free_tracked_last_frame = TrackedFeatures::default();

        for inlier in inliers {
            outlier_free_tracked_last_kf.add(
                self.tracked_features_last_keyframe.get_point(*inlier),
                self.tracked_features_last_keyframe.get_bearing_vector(*inlier),
            );
            outlier_free_tracked_current.add(
                self.tracked_features.get_point(*inlier),
                self.tracked_features.get_bearing_vector(*inlier),
            );
            outlier_free_tracked_last_frame.add(
                self.tracked_features_last_frame.get_point(*inlier),
                self.tracked_features_last_frame.get_bearing_vector(*inlier),
            );
        }
        self.tracked_features_last_keyframe = outlier_free_tracked_last_kf;
        self.tracked_features = outlier_free_tracked_current;
        self.tracked_features_last_frame = outlier_free_tracked_last_frame;

        println!("Matches after removing outliers: {}", self.tracked_features.len());

            // KeypointMatches outlier_free_matches_ref_cur;
            // outlier_free_matches_ref_cur.reserve(inliers.size());
            // for (const size_t& in : inliers) {
            //     outlier_free_matches_ref_cur.push_back((*matches_ref_cur)[in]);
            // }
            // *matches_ref_cur = outlier_free_matches_ref_cur;
    }

    fn geometric_outlier_rejection_inner(
        &self,
        cam_lkf_pose_cam_kf: &Pose,
    ) -> (Vec<usize>, GtsamFrontendTrackingState, Pose) {
        // TrackingStatusPose Tracker::geometricOutlierRejection2d2d(
        //     const BearingVectors& ref_bearings,
        //     const BearingVectors& cur_bearings,
        //     const KeypointMatches& matches_ref_cur,
        //     std::vector<int>* inliers,
        //     const gtsam::Pose3& cam_lkf_Pose_cam_kf)

        let mut status;

        // NOTE: versors are already in the rectified left camera frame.
        // No further rectification needed.
        // Get bearing vectors for opengv.
        let mut f_ref = vec![];
        let mut f_cur = vec![];
        for i in 0..self.tracked_features_last_keyframe.len() {
            // Reference bearing vector
            let ref_bearing = self.tracked_features_last_keyframe.get_bearing_vector(i);
            f_ref.push(*ref_bearing);
            // Current bearing vector
            let cur_bearing = self.tracked_features.get_bearing_vector(i);
            f_cur.push(*cur_bearing);
        }

        // println!("Ransac.... ref bearings: {}, cur bearings: {}", f_ref.len(), f_cur.len());
        // println!("Ransac... ref bearings: {:?}", f_ref);
        // println!("Ransac... cur bearings: {:?}", f_cur);

        // Solve problem.
        let (success, inliers, mut best_pose) = {
            // Begin bool runRansac(
            //   std::shared_ptr<SampleConsensusProblem> sample_consensus_problem_ptr,
            //   const double& threshold,
            //   const int& max_iterations,
            //   const double& probability,
            //   const bool& do_nonlinear_optimization,
            //   gtsam::Pose3* best_pose,
            //   std::vector<int>* inliers)


            println!("cam_lkf_pose_cam_kf: {:?}", cam_lkf_pose_cam_kf.get_rotation());

            // Setup adaptor
            let adapter = CentralRelativeAdapter::new(
                f_ref,
                f_cur,
                cam_lkf_pose_cam_kf.get_rotation(), 
                cam_lkf_pose_cam_kf.get_translation()
            );
                // Adapter2d2d adapter(f_ref, f_cur);
                // if (tracker_params_.ransac_use_2point_mono_) {
                //     adapter.setR12(cam_lkf_Pose_cam_kf.rotation().matrix());
                //     adapter.sett12(cam_lkf_Pose_cam_kf.translation().matrix());
                // }


            // Create ransac
            let max_iterations = SETTINGS.get::<i32>(TRACKING_FRONTEND, "ransac_max_iterations") as usize;
            let problem = TranslationOnlySacProblem::new(adapter, false);
            let mut ransac = Ransac::new(
                problem,
                SETTINGS.get::<f64>(TRACKING_FRONTEND, "ransac_threshold_mono"),
                max_iterations as u32,
                SETTINGS.get::<f64>(TRACKING_FRONTEND, "ransac_probability"),
                true,
                vec![] // Sofiya note, this is empty in kimera too
            );

            // Run ransac
            if ransac.compute_model() {
                if ransac.inliers.is_empty() {
                    (false, ransac.inliers, Pose::default())
                } else {
                    let best_pose = Pose::new(
                        Vector3::new(
                            ransac.model_coefficients.translation[0],
                            ransac.model_coefficients.translation[1],
                            ransac.model_coefficients.translation[2],
                        ),
                        *cam_lkf_pose_cam_kf.get_rotation(),
                    );
                    (true, ransac.inliers, best_pose)
                }
            } else {
                (false, vec![], Pose::default())
            }
        };

        println!("RANSAC success? {}, inliers: {}", success, inliers.len());

        if !success {
            status = GtsamFrontendTrackingState::Invalid;
            best_pose = Pose::default();
        } else {
            // Check enough inliers.
            status = GtsamFrontendTrackingState::Ok;
            if inliers.len() < SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_nr_mono_inliers") as usize {
                status = GtsamFrontendTrackingState::FewMatches;
            }
        }

        (inliers, status, best_pose)
    }

    fn compute_median_disparity(& self, last_features: &TrackedFeatures, current_features: &TrackedFeatures) -> Option<f32> {
        let mut disparity_sq = vec![];
        for i in 0..last_features.len() {
            let px_diff = current_features.get_point(i) - last_features.get_point(i);
            let px_dist = px_diff.x * px_diff.x + px_diff.y * px_diff.y;
            disparity_sq.push(px_dist);
        }

        if disparity_sq.len() == 0 {
            warn!("No matches for disparity calculation");
            return None;
        }

        // Compute median:
        let center = disparity_sq.len() / 2;
        // nth element sorts the array partially until it finds the median.
        let (_, median, _) = disparity_sq.select_nth_unstable_by(
            center,
            |a, b| a.partial_cmp(b).unwrap()
        );
        let median_disparity = median.sqrt();
        return Some(median_disparity);
    }

    fn extract_new_features(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("extract features");

        let num_features_to_find = max(SETTINGS.get::<i32>(TRACKING_FRONTEND, "gftt_max_features") - self.tracked_features.points.len() as i32, 0);
        if num_features_to_find <= 0 {
            warn!("Have enough features ({}), not extracting more", self.tracked_features.points.len());
            return Ok(());
        }

        println!("Feature detector, need {}, max features per frame: {}", num_features_to_find, SETTINGS.get::<i32>(TRACKING_FRONTEND, "gftt_max_features") );

        // Mask tracked features
        let mut keypoints = opencv::types::VectorOfKeyPoint::new();
        let image = self.current_frame.image.as_ref().unwrap();
        let mut mask = opencv::core::Mat::new_rows_cols_with_default(
            image.rows(),
            image.cols(),
            CV_8U,
            Scalar::all(255.0)
        ).unwrap();
        for point in self.tracked_features.points.iter() {
            circle(
                &mut mask,
                Point::new(point.x as i32, point.y as i32),
                SETTINGS.get::<i32>(TRACKING_FRONTEND, "gftt_min_distance_btw_tracked_and_detected_features"),
                Scalar::all(0.0),
                -1,
                -1,
                0
            ).unwrap();
        }

        // Raw feature detection
        self.gftt.detect(&image, &mut keypoints, &mut mask)?;

        println!("Raw number of points detected: {}", keypoints.len());

        // for kp in keypoints.iter() {
        //     println!("Extracted kp: {:?} {:?}", kp.pt(), kp.response());
        // }

        // Non-max suppression
        let max_keypoints = self.non_max_suppression(
            &keypoints,
            num_features_to_find as i32,
            image.cols(),
            image.rows(),
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "nonmaxsuppression__nr_horizontal_bins"),
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "nonmaxsuppression__nr_vertical_bins"),
        )?;

        // for kp in max_keypoints.iter() {
        //     println!("Nonmax: {:?}", kp.pt());
        // }

        // Corner sub-pix
        let mut new_corners = opencv::types::VectorOfPoint2f::new();
        opencv::core::KeyPoint::convert(&max_keypoints, &mut new_corners, &opencv::core::Vector::<i32>::new())?;

        if new_corners.len() > 0 {
            let window_size = SETTINGS.get::<i32>(TRACKING_FRONTEND, "subpix_window_size");
            let zero_zone = SETTINGS.get::<i32>(TRACKING_FRONTEND, "subpix_zero_zone");

            opencv::imgproc::corner_sub_pix(
                image,
                &mut new_corners,
                opencv::core::Size::new(window_size, window_size),
                opencv::core::Size::new(zero_zone, zero_zone),
                opencv::core::TermCriteria {
                    typ: SETTINGS.get::<i32>(TRACKING_FRONTEND, "subpix_termcriteriatype"),
                    max_count: SETTINGS.get::<i32>(TRACKING_FRONTEND, "subpix_max_iters"),
                    epsilon: SETTINGS.get::<f64>(TRACKING_FRONTEND, "subpix_epsilon_error"),
                },
            )?;
        }

        for point in new_corners.iter() {
            let bearing_vector = self.get_bearing_vector(&point);
            self.tracked_features.add(Point2f::new(point.x, point.y), bearing_vector);
        }

        debug!("Extracted {} new features", new_corners.len());

        Ok(())
    }

    fn get_bearing_vector(&self, px: & Point2f) -> DVVector3<f64> {
        // Calibrate pixel.
        // matrix of px with a single entry, i.e., a single pixel
        let mut undistorted_keypoint = opencv::types::VectorOfPoint2f::new();
        opencv::calib3d::undistort_points(
            &opencv::types::VectorOfPoint2f::from(vec![*px]),
            &mut undistorted_keypoint,
            &CAMERA_MODULE.k_matrix.mat(),
            &VectorOff32::from_iter((*CAMERA_MODULE.dist_coef.as_ref().unwrap()).clone()),
            &opencv::core::Mat::default(),
            &opencv::core::Mat::default(),
        ).unwrap();

        // Transform to unit vector.
        let versor: Vector3<f64> = Vector3::new(
            undistorted_keypoint.get(0).unwrap().x as f64,
            undistorted_keypoint.get(0).unwrap().y  as f64,
            1.0
        );

        // Return unit norm vector
        DVVector3::new(versor.normalize())
    }

    fn non_max_suppression(
        &self, keypoints: & opencv::types::VectorOfKeyPoint, need_n_corners: i32,
        image_cols: i32, image_rows: i32,
        nr_horizontal_bins: i32, nr_vertical_bins: i32
    ) -> Result<opencv::types::VectorOfKeyPoint, Box<dyn std::error::Error>> {
        // Note... results here aren't the same as Kimera because somehow the response for every keypoint in kimera is 0, and then they sort by this... To get around the difference, just removed the sort in both for now.
        // Sorting keypoints by deacreasing order of strength
        // let mut response_vector = vec![];
        // for i in 0..keypoints.len() {
        //     response_vector.push(keypoints.get(i)?.response());
        //     println!("Response: {}", keypoints.get(i)?.response());
        // }
        // let mut indx: Vec<usize> = (0..response_vector.len()).collect(); // C++ std::iota
        // indx.sort_by(|&a, &b| b.cmp(&a));

        // let mut keypoints_sorted = vec![];
        // for i in 0..keypoints.len() {
        //     keypoints_sorted.push(keypoints.get(indx[i] as usize));
        //     println!("Sorted keypoint: {:?}", keypoints.get(indx[i] as usize)?.pt());
        // }
        let keypoints_sorted = keypoints;

        if need_n_corners as usize > keypoints.len() {
            return Ok(keypoints.clone());
        }
        let bin_row_size = image_rows as f32 / nr_vertical_bins as f32;
        let bin_col_size = image_cols  as f32 / nr_horizontal_bins as f32;

        // Binning
        // 0. count the number of valid bins (as specified by the user in the yaml
        // initialize mask such that all bins are considered
        let binning_mask = nalgebra::DMatrix::<i32>::from_element(nr_vertical_bins as usize, nr_horizontal_bins as usize, 1);
        let nr_active_bins = binning_mask.sum(); // sum of 1's in binary mask

        // 1. compute how many features we want to retain in each bin
        let num_ret_points_per_bin = (need_n_corners as f64 / nr_active_bins as f64).round() as i32;

        // 2. assign keypoints to bins and retain top numRetPointsPerBin for each bin
        let mut binned_keypoints = opencv::types::VectorOfKeyPoint::new(); // binned keypoints we want to output
        let mut nr_keypoints_in_bin = nalgebra::DMatrix::<i32>::zeros(
            nr_vertical_bins as usize,
            nr_horizontal_bins as usize);  // store number of kpts for each bin
        for i in 0..keypoints_sorted.len() {
            let current_kp = keypoints_sorted.get(i).unwrap();
            let bin_row_ind = (current_kp.pt().y / bin_row_size) as usize;
            let bin_col_ind = (current_kp.pt().x / bin_col_size) as usize;

            // if bin is active and needs more keypoints
            if binning_mask[(bin_row_ind, bin_col_ind)] == 1 && nr_keypoints_in_bin[(bin_row_ind, bin_col_ind)] < num_ret_points_per_bin {
                binned_keypoints.push(current_kp.clone());
                nr_keypoints_in_bin[(bin_row_ind, bin_col_ind)] += 1;
            }
        }
        Ok(binned_keypoints)
    }

    fn calculate_transform(&self, new_tracked_features: & TrackedFeatures) 
        -> Result<Pose, Box<dyn std::error::Error>> 
    {
        let _span = tracy_client::span!("calculate_transform");
        // recovering the pose and the essential matrix
        let prev_features: VectorOfPoint2f = self.tracked_features.get_points_as_vector_of_point2f();
        let curr_features: VectorOfPoint2f = new_tracked_features.get_points_as_vector_of_point2f();

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

    fn need_new_keyframe(&mut self) -> bool {
        let kf_diff_ns = (self.current_frame.timestamp - self.last_kf_timestamp) * 1e9;
        let nr_valid_features = self.tracked_features.len(); //frame.getNrValidKeypoints();

        let min_time_elapsed = kf_diff_ns >= SETTINGS.get::<f64>(TRACKING_FRONTEND, "min_intra_keyframe_time_ns");
        let max_time_elapsed = kf_diff_ns >= SETTINGS.get::<f64>(TRACKING_FRONTEND, "max_intra_keyframe_time_ns");
        let nr_features_low = nr_valid_features as i32 <= SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_num_features");

        // KeypointMatches matches_ref_cur;
        // tracker_->findMatchingKeypoints(frame_lkf, frame, &matches_ref_cur);

        // check for large enough disparity
        let disparity = self.compute_median_disparity(
            &self.tracked_features_last_keyframe,
            &self.tracked_features
        ).expect("There should already be a keyframe in the map if need_new_keyframe is called.");

        let is_disparity_low = disparity < SETTINGS.get::<f64>(TRACKING_FRONTEND, "disparity_threshold") as f32;
        let disparity_low_first_time = is_disparity_low && !matches!(self.state, GtsamFrontendTrackingState::LowDisparity);
        let enough_disparity = !is_disparity_low;

        let max_disparity_reached = disparity > SETTINGS.get::<f64>(TRACKING_FRONTEND, "max_disparity_since_lkf") as f32;
        let disparity_flipped = (enough_disparity || disparity_low_first_time) && min_time_elapsed;

        // println!("Max time elapsed: {}, {}", kf_diff_ns, SETTINGS.get::<f64>(TRACKING_FRONTEND, "max_intra_keyframe_time_ns"));
        // println!("Max disparity reached: {}, {}", disparity, SETTINGS.get::<f64>(TRACKING_FRONTEND, "max_disparity_since_lkf"));
        // println!("Disparity flipped: {}, {}", enough_disparity, disparity_low_first_time);
        // println!("Nr features low: {}, {}", nr_valid_features, SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_num_features"));

        // println!("Disparity flipped: {} {} {} {} {}", enough_disparity,disparity_low_first_time, min_time_elapsed, kf_diff_ns, SETTINGS.get::<f64>(TRACKING_FRONTEND, "min_intra_keyframe_time_ns"));
        // println!(
        //     "Need new KF? Max time elapsed: {}, max disparity reached: {}, disparity flipped: {}, nr features low: {}",
        //     max_time_elapsed, max_disparity_reached, disparity_flipped, nr_features_low
        // );

        // println!("SOFIYA! Min intra keyframe time: {}", SETTINGS.get::<f64>(TRACKING_FRONTEND, "min_intra_keyframe_time_ns"));
        // println!("SOFIYA! Max intra keyframe time: {}", SETTINGS.get::<f64>(TRACKING_FRONTEND, "max_intra_keyframe_time_ns"));
        // println!("SOFIYA! Min num features: {}", SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_num_features"));
        // println!("SOFIYA! Disparity threshold: {}", SETTINGS.get::<f64>(TRACKING_FRONTEND, "disparity_threshold"));
        // println!("SOFIYA! MAX disparity since lkf: {}", SETTINGS.get::<f64>(TRACKING_FRONTEND, "max_disparity_since_lkf"));

        // println!("Timestamps: {} {}", self.current_frame.timestamp, self.last_kf_timestamp);

        return max_time_elapsed || max_disparity_reached || disparity_flipped || nr_features_low;

    }

}



#[derive(Debug, Clone, Copy, Default)]
pub enum GtsamFrontendTrackingState {
    #[default] FirstFrame,
    NotInitialized,
    Ok,
    LowDisparity,
    Invalid,
    FewMatches,
    // Disabled
}

#[derive(Clone)]
pub struct TrackedFeatures {
    points: Vec<Point2f>,
    feature_ids: Vec<i32>,
    last_feature_id: i32,
    versors: Vec<DVVector3<f64>>
}
impl TrackedFeatures {
    pub fn default() -> Self {
        TrackedFeatures {
            points: vec![],
            feature_ids: vec![],
            last_feature_id: 0,
            versors: vec![],
        }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn get_points_as_vector_of_point2f(&self) -> VectorOfPoint2f {
        let mut vec = VectorOfPoint2f::new();
        self.points.iter().for_each(|pt| vec.push(*pt));
        vec
    }

    pub fn get_feature_id(&self, index: usize) -> i32 {
        self.feature_ids[index]
    }

    pub fn get_point(&self, index: usize) -> Point2f {
        self.points[index]
    }

    pub fn get_bearing_vector(&self, index: usize) -> DVVector3<f64> {
        self.versors[index]
    }

    pub fn update(&mut self, index: usize, point: Point2f, bearing_vector: DVVector3<f64>) {
        self.points[index] = point;
        self.versors[index] = bearing_vector;
    }

    pub fn add(&mut self, point: Point2f, bearing_vector: DVVector3<f64>) -> i32 {
        self.points.push(point);
        self.feature_ids.push(self.last_feature_id);
        self.versors.push(bearing_vector);
        self.last_feature_id += 1;

        return self.last_feature_id - 1;
    }

    pub fn remove(&mut self, index: usize) -> i32 {
        let id = self.feature_ids.remove(index);
        self.points.remove(index);
        self.versors.remove(index);
        id
    }
}

impl Debug for TrackedFeatures {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TrackedFeatures with {} points: ; ", self.points.len())?;
        for i in 0..self.points.len() {
            write!(f, "ID {}: ({}, {}), ", self.feature_ids[i], self.points[i].x, self.points[i].y)?;
        }
        Ok(())
    }
}