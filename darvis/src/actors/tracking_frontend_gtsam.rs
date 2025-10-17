extern crate g2o;
use ahash::HashMap;
use log::{warn, info, debug};
use std::{cmp::max, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{core::{Point, Point2f, Scalar, CV_8U}, imgcodecs, imgproc::circle, prelude::*, types::{VectorOfKeyPoint, VectorOfPoint2f, VectorOfu8}};
use core::{
    config::*, matrix::*, system::{Actor, MessageBox, System, Timestamp}
};
use std::fmt::Debug;
use crate::{
    actors::{local_mapping::LOCAL_MAPPING_IDLE, messages::{FeatureTracksAndIMUMsg, ImageMsg, ImagePathMsg, InitKeyFrameMsg, ShutdownMsg, TrajectoryMsg, VisFeaturesMsg}}, map::{frame::Frame, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image::{self, draw_optical_flow}, imu::{ImuMeasurements, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, MapInitializationModule}}, registered_actors::{new_feature_extraction_module, CAMERA_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}
};


pub struct TrackingFrontendGTSAM {
    system: System,
    state: GtsamFrontendTrackingState,
    map: ReadWriteMap,

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
    frames_since_last_kf: i32,
    last_kf_timestamp: Timestamp,

    // ORBSLAM map initialization 
    // I know this is hacky but I dont' want to figure out how to merge the gtsam imu preintegration object with the orbslam imu object
    // Imu_for_init object needed for map initialization but that's it, otherwise should use the GtsamIMUModule
    imu_for_init: IMU,
    initialization: Option<MapInitialization>, // data sent to map actor to initialize new map
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
            // orb_extractor_left: new_feature_extraction_module(false),
            orb_extractor_ini: Some(new_feature_extraction_module(true)),
            gftt: feature_detector,
            map,
            initialization: Some(MapInitialization::new()),
            state: GtsamFrontendTrackingState::NotInitialized,
            tracked_features: TrackedFeatures::default(),
            tracked_features_last_frame: TrackedFeatures::default(),
            tracked_features_last_keyframe: TrackedFeatures::default(),
            imu_for_init: IMU::new(),
            last_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            curr_frame_id: 0,
            imu_measurements_since_last_kf: ImuMeasurements::new(),
            frames_since_last_kf: 0,
            removed_features: vec![],
            last_kf_timestamp: 0.0,
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

            let (image, image_color, timestamp, mut imu_measurements, mut imu_initialization) = if message.is::<ImagePathMsg>() {
                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (image::read_image_file(&msg.image_path, imgcodecs::IMREAD_GRAYSCALE), None, msg.timestamp, msg.imu_measurements, msg.imu_initialization)
            } else {
                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (msg.image, msg.color_image, msg.timestamp, msg.imu_measurements, msg.imu_initialization)
            };
            self.imu_measurements_since_last_kf.append(&mut imu_measurements);

            debug!("Tracking frontend working on frame {} at timestamp {}", self.curr_frame_id, timestamp);

            let pub_this_frame = match self.state {
                GtsamFrontendTrackingState::NotInitialized => {
                    // If map is not initialized yet, just extract features and try to initialize
                    // If initialized successsfully,
                    // publish this frame so backend has a reference to the frame associated with the initialization

                    // SOFIYA TURN OFF MAP INITIALIZATION
                        // let initialized = self.initialize_map(&image, timestamp).unwrap();
                        // println!("Timestamp {}. Initialized map? {}. GT Imu init is: {:?}", timestamp, initialized, imu_initialization);

                        // if initialized {
                        //     let kf1_pose = self.map.read().unwrap().get_keyframe(1).get_pose();
                        //     imu_initialization.as_mut().unwrap().pose = kf1_pose;
                        //     println!("Set imu initialization pose to: {:?}", kf1_pose);
                        // }
                        
                        // initialized

                    // When turning back on, comment all this out:
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

                        println!("INITIALIZED IN FRONTEND!");

                        true
                },
                GtsamFrontendTrackingState::Ok | GtsamFrontendTrackingState::LowDisparity => {
                    // Regular tracking
                    self.current_frame = Frame::new_no_features(
                        self.curr_frame_id, 
                        Some(image.clone()),
                        timestamp,
                        Some(& self.last_frame)
                    ).expect("Could not create frame!");

                    // Optical flow
                    self.optical_flow().unwrap();

                    // Calculate transform from optical flow
                    // let transform = self.calculate_transform(& new_tracked_features).unwrap();
                    // let new_trans = *transform.get_translation() * (self.map_scale);
                    // let new_pose = Pose::new(new_trans, * transform.get_rotation()) * self.last_frame.pose.unwrap();
                    // self.current_frame.pose = Some(new_pose);
                    // debug!("OPTICAL FLOW POSE ESTIMATE... {}, {:?}", timestamp * 1e9, new_pose);

                    // Determine if frame should be a keyframe
                    self.need_new_keyframe()
                    // true
                }
            };

            // println!("IMU MEASUREMENTS ARE: {:?}", self.imu_measurements_since_last_kf);

            // self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            //     pose: Pose::default(),
            //     mappoint_matches: vec![],
            //     mappoints_in_tracking: BTreeSet::new(),
            //     timestamp: self.current_frame.timestamp,
            //     map_version: 1
            // }));


            println!("Pub this frame? {}", pub_this_frame);
            if pub_this_frame {
                tracy_client::Client::running()
                    .expect("message! without a running Client")
                    .message("Publish frame!", 2);

                // TODO Kimera
                // self.outlier_rejection();
                // TODO kimera... this stuff is actually inside the outlier rejection function
                if let Some(disparity) = self.compute_median_disparity(&self.tracked_features_last_frame, &self.tracked_features) {
                    if disparity < SETTINGS.get::<f64>(TRACKING_FRONTEND, "disparity_threshold") as f32 {
                        debug!("Low mono disparity.");
                        self.state = GtsamFrontendTrackingState::LowDisparity;
                    }
                }

                self.extract_new_features().expect("Couldn't extract good features to track?");

                // Send current imu measurements to backend, replace with empty ones
                let mut imu_measurements = ImuMeasurements::new();
                std::mem::swap(&mut self.imu_measurements_since_last_kf, &mut imu_measurements);


                println!("Huh? Send to backend");
                // SEND TO BACKEND!
                self.system.send(TRACKING_BACKEND, Box::new(FeatureTracksAndIMUMsg {
                    tracker_status: self.state,
                    frame: self.current_frame.clone(),
                    imu_measurements,
                    feature_tracks: self.tracked_features.clone(),
                    removed_feature_ids: self.removed_features.clone(),
                    imu_initialization,
                }));

                self.last_kf_timestamp = self.current_frame.timestamp;
                self.removed_features.clear();
                self.tracked_features_last_keyframe = self.tracked_features.clone();

                if self.last_frame.image.is_some() {
                    // draw_optical_flow(
                    //     self.last_frame.image.as_ref().unwrap(),
                    //     self.current_frame.image.as_ref().unwrap(),
                    //     & self.tracked_features_last_frame.get_points_as_vector_of_point2f(),
                    //     & self.tracked_features.get_points_as_vector_of_point2f(),
                    //     &format!("results/flow/front{}.png", self.curr_frame_id),
                    // ).unwrap();
                    // debug!("Frontend, tracked features last: {:?}", self.tracked_features_last_frame);
                    // debug!("Frontend, tracked features now: {:?}", self.tracked_features);
                    
                }

                self.frames_since_last_kf = 0;
            } else {
                self.frames_since_last_kf += 1;
            }

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

    fn initialize_map(&mut self, curr_img: &opencv::core::Mat, timestamp: Timestamp) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("initialize_map");
        let (keypoints, descriptors) = self.orb_extractor_ini.as_mut().unwrap().extract(& curr_img).unwrap();

        self.current_frame = Frame::new(
            self.curr_frame_id, 
            keypoints,
            descriptors,
            curr_img.cols() as u32,
            curr_img.rows() as u32,
            Some(curr_img.clone()),
            Some(& self.last_frame),
            self.map.read()?.imu_initialized,
            timestamp,
        ).expect("Could not create frame!");

        if self.initialization.is_none() {
            self.initialization = Some(MapInitialization::new());
        }
        // TODO SOFIYA Should the self.imu_for_init values be used somewhere after initialization?
        let init_success = self.initialization.as_mut().unwrap().try_initialize(&self.current_frame, &mut self.imu_for_init.imu_preintegrated_from_last_kf)?;
        if init_success {
            println!("Map initialized successfully with frames {:?} and {}", self.initialization.as_ref().unwrap().initial_frame.as_ref().unwrap().frame_id, self.curr_frame_id);

            match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map,  &mut self.imu_for_init.imu_preintegrated_from_last_kf)? {
                Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, _curr_kf_timestamp, map_scale)) => {
                    // Map needs to be initialized before tracking can begin
                    // Set current frame's updated info from map initialization
                    self.current_frame.pose = Some(curr_kf_pose);
                    self.current_frame.ref_kf_id = Some(curr_kf_id);
                    self.state = GtsamFrontendTrackingState::Ok;

                    // Log initial pose in shutdown actor
                    self.system.send(SHUTDOWN_ACTOR, 
                    Box::new(TrajectoryMsg{
                            pose: self.map.read()?.get_keyframe(ini_kf_id).get_pose(),
                            ref_kf_id: ini_kf_id,
                            timestamp: self.map.read()?.get_keyframe(ini_kf_id).timestamp,
                            map_version: self.map.read()?.version
                        })
                    );

                    // SOFIYA TURN OFF LOCAL MAPPING
                    // Send first two keyframes to local mapping
                    self.system.send(LOCAL_MAPPING, Box::new(
                        InitKeyFrameMsg { kf_id: ini_kf_id, map_version: self.map.read()?.version }
                    ));
                    self.system.send(LOCAL_MAPPING,Box::new(
                        InitKeyFrameMsg { kf_id: curr_kf_id, map_version: self.map.read()?.version } 
                    ));
                },
                None => {
                    panic!("Could not create initial map");
                }
            };

            self.state = GtsamFrontendTrackingState::Ok;
            Ok(true)
        } else {
            self.state = GtsamFrontendTrackingState::NotInitialized;
            Ok(false)
        }
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
                self.tracked_features.update(index_in_mutated, pt);
                total_tracked += 1;
            }
        }

        debug!("Optical flow tracked {} from original {}", total_tracked, status.len());
        // debug!("Removed: {:?}", self.removed_features);
        Ok(())
    }

    fn outlier_rejection(&mut self, rotation: Option<nalgebra::Matrix3<f64>>) -> Result<(), Box<dyn std::error::Error>> {
        // TODO Kimera... outlier rejection needs preintegration in frontend.
        // keyframe_R_cur_frame comes from preintegration!
    //     // auto tic_full_preint = utils::Timer::tic();
    //     const ImuFrontend::PimPtr& pim = imu_frontend_->preintegrateImuMeasurements(
    //         input->getImuStamps(), input->getImuAccGyrs());
    //     CHECK(pim);
    //     const gtsam::Rot3 body_R_cam = mono_camera_->getBodyPoseCam().rotation();
    //     const gtsam::Rot3 cam_R_body = body_R_cam.inverse();
    //     gtsam::Rot3 camLrectLkf_R_camLrectK_imu =
    //   cam_R_body * pim->deltaRij() * body_R_cam;


        // void VisionImuFrontend::outlierRejectionMono(
        //     const gtsam::Rot3& keyframe_R_cur_frame,
        //     Frame* frame_lkf,
        //     Frame* frame_k,
        //     TrackingStatusPose* status_pose_mono) const {

        let given_rot = rotation.is_some();
        // const bool time_aligned =
        //     frontend_state_ != FrontendState::InitialTimeAlignment;
        // const bool imu_ok = given_rot && time_aligned;
        // let imu_ok = given_rot && time_aligned;

        // let pose = if imu_ok {
        //     // 2-point RANSAC.
        //     self.geometric_outlier_rejection_2d2d(frame_lkf, frame_k, & Pose::new_with_default_trans(rotation.unwrap()));
        // } else {
        //     // 5-point RANSAC.
        //     //     *status_pose_mono =
        //     //         tracker_->geometricOutlierRejection2d2d(frame_lkf, frame_k);
        // };

        Ok(())
    }

    fn geometric_outlier_rejection_2d2d(
        &mut self, frame_lkf: &Frame, frame_k: &Frame, pose: &Pose
    ) -> Result<(), Box<dyn std::error::Error>> {
        // TODO Kimera

        // KeypointMatches matches_ref_cur;
        // findMatchingKeypoints(*ref_frame, *cur_frame, &matches_ref_cur);

        // TrackingStatusPose result;

        // if (matches_ref_cur.empty()) {
        //     LOG(ERROR) << "No matching keypoints from frame " << ref_frame->id_
        //             << " to frame " << cur_frame->id_ << ".\n"
        //             << "Mono Tracking Status = INVALID.";
        //     result = std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
        // } else {
        //     std::vector<int> inliers;
        //     result = geometricOutlierRejection2d2d(ref_frame->versors_,
        //                                         cur_frame->versors_,
        //                                         matches_ref_cur,
        //                                         &inliers,
        //                                         cam_lkf_Pose_cam_kf);

        //     // TODO(Toni): should we remove outliers if few matches?
        //     //! Remove correspondences classified as outliers
        //     if (result.first != TrackingStatus::FEW_MATCHES) {
        //     removeOutliersMono(inliers, ref_frame, cur_frame, &matches_ref_cur);
        //     }

        //     // THIS IS ONLY USEFUL if we are completely still...
        //     if (result.first == TrackingStatus::VALID) {
        //     // TODO(TONI): unrotate due to optical flow before calculating
        //     // disparity...
        //     // TODO(TONI): this has no place here... should be somewhere else...
        //     //! Check enough disparity.
        //     double disparity;
        //     if (computeMedianDisparity(ref_frame->keypoints_,
        //                                 cur_frame->keypoints_,
        //                                 matches_ref_cur,
        //                                 &disparity)) {
        //         if (disparity < tracker_params_.disparityThreshold_) {
        //         LOG(INFO) << "Low mono disparity.";
        //         result.first = TrackingStatus::LOW_DISPARITY;
        //         }
        //     } else {
        //         LOG(ERROR) << "Median disparity calculation failed...";
        //     }
        //     }
        // }

        // debug_info_.monoRansacTime_ = utils::Timer::toc(start_time_tic).count();
        // return result;
        Ok(())
    }

    fn remove_outliers(&mut self) {
        // TODO Kimera

        // removeOutliersMono
            // // Find indices of outliers in current frame.
            // std::vector<int> outliers;
            // findOutliers(*matches_ref_cur, inliers, &outliers);
            // // Remove outliers.
            // // outliers cannot be a vector of size_t because opengv uses a vector of
            // // int.
            // for (const size_t& out : outliers) {
            //     const auto& ref_kp_cur_kp = (*matches_ref_cur)[out];
            //     ref_frame->landmarks_.at(ref_kp_cur_kp.first) = -1;
            //     cur_frame->landmarks_.at(ref_kp_cur_kp.second) = -1;
            // }

            // // Store only inliers from now on.
            // KeypointMatches outlier_free_matches_ref_cur;
            // outlier_free_matches_ref_cur.reserve(inliers.size());
            // for (const size_t& in : inliers) {
            //     outlier_free_matches_ref_cur.push_back((*matches_ref_cur)[in]);
            // }
            // *matches_ref_cur = outlier_free_matches_ref_cur;
    }

    fn compute_median_disparity(& self, last_features: &TrackedFeatures, current_features: &TrackedFeatures) -> Option<f32> {
        // computeMedianDisparity
        let mut disparity_sq = vec![];
        // print!("PX diffs: ");
        for i in 0..last_features.len() {
            let px_diff = current_features.get_point(i) - last_features.get_point(i);
            let px_dist = px_diff.x * px_diff.x + px_diff.y * px_diff.y;
            // print!("{:?} {}, ", px_diff, px_dist);
            disparity_sq.push(px_dist);
        }

        if disparity_sq.len() == 0 {
            warn!("No matches for disparity calculation");
            return None;
        }
        // println!("Disparity_sq: {:?}", disparity_sq);

        // Compute median:
        let center = disparity_sq.len() / 2;
        // nth element sorts the array partially until it finds the median.
        let (_, median, _) = disparity_sq.select_nth_unstable_by(
            center,
            |a, b| a.partial_cmp(b).unwrap()
        );
        let median_disparity = median.sqrt();
        // println!("Median disparity: {} {}", median, median_disparity);
        return Some(median_disparity);
    }

    fn find_matching_keypoints(&mut self) {
        // TODO Kimera

        //FindMatchingKeypoints
            // // Find keypoints that observe the same landmarks in both frames:
            // std::map<LandmarkId, size_t> ref_lm_index_map;
            // for (size_t i = 0; i < ref_frame.landmarks_.size(); ++i) {
            //     const LandmarkId& ref_id = ref_frame.landmarks_.at(i);
            //     if (ref_id != -1) {
            //     // Map landmark id -> position in ref_frame.landmarks_
            //     ref_lm_index_map[ref_id] = i;
            //     }
            // }

            // // Map of position of landmark j in ref frame to position of landmark j in
            // // cur_frame
            // matches_ref_cur->reserve(ref_lm_index_map.size());
            // for (size_t i = 0; i < cur_frame.landmarks_.size(); ++i) {
            //     const LandmarkId& cur_id = cur_frame.landmarks_.at(i);
            //     if (cur_id != -1) {
            //     auto it = ref_lm_index_map.find(cur_id);
            //     if (it != ref_lm_index_map.end()) {
            //         matches_ref_cur->push_back(std::make_pair(it->second, i));
            //     }
            //     }
            // }
    }

    fn geometric_outlier_rejection_inner() {
        // TODO Kimera

        //   TrackingStatusPose status_pose;
        // // NOTE: versors are already in the rectified left camera frame.
        // // No further rectification needed.
        // //! Get bearing vectors for opengv.
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

    fn extract_new_features(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("extract features");

        print!("max features: {}", SETTINGS.get::<i32>(TRACKING_FRONTEND, "gftt_max_features") as usize);

        let num_features_to_find = max(SETTINGS.get::<i32>(TRACKING_FRONTEND, "gftt_max_features") - self.tracked_features.points.len() as i32, 0);
        if num_features_to_find <= 0 {
            warn!("Have enough features ({}), not extracting more", self.tracked_features.points.len());
            return Ok(());
        }

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
            // println!("Masking feature: {:?}", Point::new(point.x as i32, point.y as i32));
        }

        // Raw feature detection
        self.gftt.detect(&image, &mut keypoints, &mut mask)?;
        // opencv::imgproc::good_features_to_track(
        //     & image, &mut points, num_features_to_find as i32,
        //     quality_level, min_distance, &mut mask, block_size, false, k
        // ).unwrap();

        println!("Current image size: {} x {}", image.cols(), image.rows());
        println!("Raw number of points detected: {}", keypoints.len());

        // Non-max suppression
        let tolerance = 0.1;
        let max_keypoints = self.non_max_suppression(
            &keypoints,
            num_features_to_find as i32,
            tolerance,
            image.cols(),
            image.rows(),
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "nonmaxsuppression__nr_horizontal_bins"),
            SETTINGS.get::<i32>(TRACKING_FRONTEND, "nonmaxsuppression__nr_vertical_bins"),
        )?;

        // Corner sub-pix
        let mut new_corners = opencv::types::VectorOfPoint2f::new();
        opencv::core::KeyPoint::convert(&max_keypoints, &mut new_corners, &opencv::core::Vector::<i32>::new())?;

        // TODO(Toni) this takes a ton of time 27ms each time...
        // Change window_size, and term_criteria to improve timing
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

        // println!("Extracted features:");
        for point in new_corners.iter() {
            self.tracked_features.add(Point2f::new(point.x, point.y));
        }

        debug!("Extracted {} new features", new_corners.len());

        Ok(())
    }

    fn non_max_suppression(
        &self, keypoints: & opencv::types::VectorOfKeyPoint, need_n_corners: i32,
        tolerance: f64, image_cols: i32, image_rows: i32,
        nr_horizontal_bins: i32, nr_vertical_bins: i32
    ) -> Result<opencv::types::VectorOfKeyPoint, Box<dyn std::error::Error>> {
        // Sorting keypoints by deacreasing order of strength
        let mut response_vector = vec![];
        for i in 0..keypoints.len() {
            response_vector.push(keypoints.get(i)?.response());
        }
        let mut indx: Vec<usize> = (0..response_vector.len()).collect(); // C++ std::iota
        indx.sort_by(|&a, &b| b.cmp(&a));

        // TODO Kimera need to do this sort
        // std::vector<int> Indx(responseVector.size());
        // std::iota(std::begin(Indx), std::end(Indx), 0);
        // cv::sortIdx(responseVector, Indx, cv::SortFlags::SORT_DESCENDING);
        let mut keypoints_sorted = vec![];
        for i in 0..keypoints.len() {
            keypoints_sorted.push(keypoints.get(indx[i] as usize));
        }

        
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
            let current_kp = keypoints_sorted.get(i).as_ref().unwrap().as_ref().unwrap();
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

    fn calculate_transform(&self, new_tracked_features: & TrackedFeatures) -> Result<Pose, Box<dyn std::error::Error>> {
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
        // ORB-SLAM3
        // // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        // let c1a = self.frames_since_last_kf >= (SETTINGS.get::<i32>(TRACKING_FRONTEND, "max_frames_to_insert_kf") as i32);
        // // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // let c1b = self.frames_since_last_kf >= (SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_frames_to_insert_kf") as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
        // //Condition 1c: tracking is weak
        // let c1c = (self.tracked_features.len() as u32) < (SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_num_features") as u32);

        // // let c1c = ((self.matches_inliers as f32) < tracked_mappoints * 0.5 || need_to_insert_close) ;
        // // // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        // // let c2 = (((self.matches_inliers as f32) < (tracked_mappoints * th_ref_ratio) || need_to_insert_close)) && self.matches_inliers > 15;
        // // (c1a||c1b||c1c) && c2

        // debug!("Need new keyframe? {} {} {}", c1a, c1b, c1c);
        // c1a || c1b || c1c

        // Kimera:
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
        // tracker_->computeMedianDisparity(
        //     frame_lkf.keypoints_, frame.keypoints_, matches_ref_cur, &disparity);

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
    #[default] NotInitialized,
    Ok,
    LowDisparity,
    // FewMatches,
    // Invalid,
    // Disabled
}

pub type TrackedFeaturesIndexMap = HashMap<i32, usize>; // feature_id -> index in a frame's features

#[derive(Clone)]
pub struct TrackedFeatures {
    points: Vec<Point2f>,
    feature_ids: Vec<i32>,
    last_feature_id: i32,
}
impl TrackedFeatures {
    pub fn default() -> Self {
        TrackedFeatures {
            points: vec![],
            feature_ids: vec![],
            last_feature_id: 0,
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

    pub fn update(&mut self, index: usize, point: Point2f) {
        self.points[index] = point;
    }

    pub fn add(&mut self, point: Point2f) -> i32 {
        self.points.push(point);
        self.feature_ids.push(self.last_feature_id);
        self.last_feature_id += 1;

        return self.last_feature_id - 1;
    }

    pub fn remove(&mut self, index: usize) -> i32 {
        let id = self.feature_ids.remove(index);
        self.points.remove(index);
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