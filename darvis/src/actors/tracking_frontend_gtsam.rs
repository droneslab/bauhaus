extern crate g2o;
use gtsam::{imu::imu_bias::ConstantBias, navigation::combined_imu_factor::{PreintegratedCombinedMeasurements, PreintegrationCombinedParams}};
use log::{warn, info, debug, error};
use nalgebra::{Matrix3, Matrix6, SMatrix, Vector3};
use std::{collections::{BTreeMap, BTreeSet, HashMap, VecDeque}, fmt::{self, Formatter}, mem, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{core::{no_array, KeyPoint, Point, Point2f, Scalar, CV_8U, CV_8UC1}, features2d::{FastFeatureDetector, FastFeatureDetector_DetectorType, FastFeatureDetector_TYPE_9_16}, imgproc::circle, prelude::*, types::{VectorOfKeyPoint, VectorOfMat, VectorOfPoint2f, VectorOfu8}};
use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::messages::{ImageMsg, InitKeyFrameMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg, VisTrajectoryMsg, VisTrajectoryTrackingMsg}, map::{features::Features, frame::Frame, keyframe::MapPointMatches, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{good_features_to_track::GoodFeaturesExtractor, image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, ImuModule}, opencv_extractor::OpenCVExtractor, optimizer, orbslam_extractor::ORBExtractor, relocalization::Relocalization}, registered_actors::{new_feature_extraction_module, CAMERA_MODULE, FEATURE_DETECTION, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}, MAP_INITIALIZED
};

use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{FeatureTracksAndIMUMsg, ImagePathMsg, TrajectoryMsg, InitKeyFrameMsgGTSAM}, tracking_backend::TrackedMapPointData};
use crate::modules::module_definitions::MapInitializationModule;



pub struct TrackingFrontendGTSAM {
    system: System,
    sensor: Sensor,
    state: GtsamFrontendTrackingState,

    /// Frontend
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    imu_measurements_since_last_kf: ImuMeasurements,

    // Feature IDs
    tracked_features_last_kf: TrackedFeatures,


    // Frames
    last_frame: Frame,
    curr_frame_id: i32,
    current_frame: Frame,
    frames_since_last_kf: i32,

    // Initialization 
    // I know this is hacky but I dont' want to figure out how to merge the gtsam imu preintegration object with the orbslam imu object
    // Imu_for_init object needed for map initialization but that's it, otherwise should use the GtsamIMUModule
    imu_for_init: IMU,
    initialization: Option<MapInitialization>, // data sent to map actor to initialize new map

    /// References to map
    map: ReadWriteMap,
    map_scale: f64,
}

impl Actor for TrackingFrontendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");

        let mut actor = TrackingFrontendGTSAM {
            system,
            orb_extractor_left: new_feature_extraction_module(false),
            orb_extractor_ini: Some(new_feature_extraction_module(true)),
            sensor,
            map,
            initialization: Some(MapInitialization::new()),
            state: GtsamFrontendTrackingState::NotInitialized,
            tracked_features_last_kf: TrackedFeatures::default(),
            imu_for_init: IMU::new(),
            last_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            curr_frame_id: 0,
            imu_measurements_since_last_kf: ImuMeasurements::new(),
            map_scale: 1.0,
            frames_since_last_kf: 0,
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

            let (image, timestamp, mut imu_measurements) = if message.is::<ImagePathMsg>() {
                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (image::read_image_file(&msg.image_path), msg.timestamp, msg.imu_measurements)
            } else {
                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (msg.image, msg.timestamp, msg.imu_measurements)
            };
            self.imu_measurements_since_last_kf.append(&mut imu_measurements);

            debug!("Tracking frontend working on frame {}", self.curr_frame_id);

            let pub_this_frame = match self.state {
                GtsamFrontendTrackingState::NotInitialized => {
                    // If map is not initialized yet, just extract features and try to initialize
                    // If initialized successsfully,
                    // publish this frame so backend has a reference to the frame associated with the initialization
                    self.initialize_map(&image, timestamp).unwrap()
                },
                GtsamFrontendTrackingState::Ok => {
                    // Regular tracking
                    self.current_frame = Frame::new_no_features(
                        self.curr_frame_id, 
                        Some(image.clone()),
                        timestamp,
                        Some(& self.last_frame)
                    ).expect("Could not create frame!");

                    // Optical flow
                    let new_tracked_features = self.optical_flow().unwrap();

                    // Calculate transform from optical flow
                    let transform = self.calculate_transform(& new_tracked_features).unwrap();
                    let new_trans = *transform.get_translation() * (self.map_scale);
                    let new_pose = Pose::new(new_trans, * transform.get_rotation()) * self.last_frame.pose.unwrap();
                    self.current_frame.pose = Some(new_pose);
                    debug!("Pose prediction from pure optical flow: {:?}", new_pose);

                    self.tracked_features_last_kf = new_tracked_features;

                    // Determine if frame should be a keyframe
                    let pub_this_frame = self.need_new_keyframe();
                    if pub_this_frame {
                        self.extract_good_features_to_track().unwrap();
                    };

                    // self.system.try_send(VISUALIZER, Box::new(VisTrajectoryTrackingMsg{
                    //     pose: new_pose,
                    //     timestamp: self.current_frame.timestamp,
                    //     map_version: self.map.read().unwrap().version
                    // }));

                    pub_this_frame
                }
            };

            if pub_this_frame {
                tracy_client::Client::running()
                    .expect("message! without a running Client")
                    .message("Publish frame!", 2);

                // Send current imu measurements to backend, replace with empty ones
                let mut imu_measurements = ImuMeasurements::new();
                std::mem::swap(&mut self.imu_measurements_since_last_kf, &mut imu_measurements);

                // SEND TO BACKEND!
                self.system.send(TRACKING_BACKEND, Box::new(FeatureTracksAndIMUMsg {
                    frame: self.current_frame.clone(),
                    imu_measurements,
                    feature_tracks: self.tracked_features_last_kf.clone(),
                }));

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
        let (keypoints, descriptors) = match self.sensor {
            Sensor(FrameSensor::Mono, _) => {
                if matches!(self.state, GtsamFrontendTrackingState::NotInitialized) || (self.current_frame.frame_id < SETTINGS.get::<f64>(SYSTEM, "fps") as i32) {
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
            match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map,  &mut self.imu_for_init.imu_preintegrated_from_last_kf)? {
                Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, _curr_kf_timestamp, map_scale)) => {
                    // Map needs to be initialized before tracking can begin
                    // Set current frame's updated info from map initialization
                    self.current_frame.pose = Some(curr_kf_pose);
                    self.current_frame.ref_kf_id = Some(curr_kf_id);
                    self.map_scale = map_scale;
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

            self.extract_good_features_to_track()?;

            self.state = GtsamFrontendTrackingState::Ok;
            Ok(true)
        } else {
            self.state = GtsamFrontendTrackingState::NotInitialized;
            Ok(false)
        }
    }

    fn optical_flow(&mut self) -> Result<TrackedFeatures, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("optical_flow");
        let mut status = VectorOfu8::new();
        let mut points2 = VectorOfPoint2f::new();

        // Optical flow
        opencv::video::calc_optical_flow_pyr_lk(
            & self.last_frame.image.as_ref().unwrap(),
            & self.current_frame.image.as_ref().unwrap(),
            & self.tracked_features_last_kf.get_points_as_vector_of_point2f(),
            &mut points2,
            &mut status,
            &mut opencv::types::VectorOff32::default(),
            opencv::core::Size::new(24, 24), 
            3,
            opencv::core::TermCriteria {
                typ: 3,
                max_count: 30,
                epsilon: 0.01,
            },
            0,
            0.001,
        )?;

        // Get rid of points for which the KLT tracking failed or those who have gone outside the frame
        let mut new_tracked_features = TrackedFeatures::default();
        new_tracked_features.last_feature_id = self.tracked_features_last_kf.last_feature_id;

        // Len(Status) == Len(points2) == Len(self.tracked_features)

        let mut index_correction = 0; // We mutate self.tracked_features vectors while status and points2 lengths remain the same
        let mut total_tracked = 0;
        for i in 0..status.len() {
            let index_in_mutated = i - index_correction;
            let pt = points2.get(i)?;
            let status_ok = status.get(i)? == 1;

            if !status_ok || pt.x < 0.0 || pt.y < 0.0 {
                // FEATURE IS NOT TRACKED! Remove from tracked_features
                let _removed_id = self.tracked_features_last_kf.remove(index_in_mutated);
                index_correction = index_correction + 1;
            } else {
                // FEATURE IS FOUND! Update the point in new_tracked_features
                // Not updating current_frame's features yet, we will do this if this frame becomes a new keyframe during the feature extraction step.

                new_tracked_features.add_with_id(pt, self.tracked_features_last_kf.feature_ids[index_in_mutated]);
                total_tracked += 1;
            }
        }

        debug!("Optical flow tracked {} from original {}", total_tracked, status.len());
        Ok(new_tracked_features)
    }

    fn extract_good_features_to_track(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("extract features");

        let num_features_to_find = 200 - self.tracked_features_last_kf.points.len();

        if num_features_to_find <= 0 {
            warn!("Have enough features ({}), not extracting more", self.tracked_features_last_kf.points.len());
            return Ok(());
        }

        let mut points = opencv::types::VectorOfPoint2f::new();
        let image = self.current_frame.image.as_ref().unwrap();

        let mut mask = opencv::core::Mat::new_rows_cols_with_default(
            image.rows(),
            image.cols(),
            CV_8U,
            Scalar::all(255.0)
        ).unwrap();
        for point in self.tracked_features_last_kf.points.iter() {
            circle(&mut mask, Point::new(point.x as i32, point.y as i32), 20, Scalar::all(0.0), -1, 8, 0).unwrap();
        }
        let min_distance = SETTINGS.get::<f64>(TRACKING_FRONTEND, "min_distance") as f64;

        opencv::imgproc::good_features_to_track(
            & image, &mut points, num_features_to_find as i32, 0.01, min_distance, &mut mask, 3, false, 0.04
        ).unwrap();


        for point in points.iter() {
            self.tracked_features_last_kf.add(Point2f::new(point.x as f32, point.y as f32));
        }

        debug!("Extracted {} new features", points.len());

        Ok(())
    }


    fn need_new_keyframe(&mut self) -> bool {
        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        let c1a = self.frames_since_last_kf >= (SETTINGS.get::<i32>(TRACKING_FRONTEND, "max_frames_to_insert_kf") as i32);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        let c1b = self.frames_since_last_kf >= (SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_frames_to_insert_kf") as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
        //Condition 1c: tracking is weak
        let c1c = (self.tracked_features_last_kf.len() as u32) < (SETTINGS.get::<i32>(TRACKING_FRONTEND, "min_num_features") as u32);

        // let c1c = ((self.matches_inliers as f32) < tracked_mappoints * 0.5 || need_to_insert_close) ;
        // // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        // let c2 = (((self.matches_inliers as f32) < (tracked_mappoints * th_ref_ratio) || need_to_insert_close)) && self.matches_inliers > 15;
        // (c1a||c1b||c1c) && c2

        debug!("Need new keyframe? {} {} {}, {}", c1a, c1b, c1c, self.tracked_features_last_kf.len());

        c1a || c1b || c1c
    }


    fn calculate_transform(&self, new_tracked_features: & TrackedFeatures) -> Result<Pose, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("calculate_transform");
        // recovering the pose and the essential matrix
        let prev_features: VectorOfPoint2f = self.tracked_features_last_kf.get_points_as_vector_of_point2f();
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
}


#[derive(Debug, Clone, Copy, Default)]
enum GtsamFrontendTrackingState {
    #[default] NotInitialized,
    Ok,
}

pub type TrackedFeaturesIndexMap = HashMap<i32, usize>; // feature_id -> index in a frame's features

#[derive(Debug, Clone)]
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

    pub fn add(&mut self, point: Point2f) -> i32 {
        self.points.push(point);
        self.feature_ids.push(self.last_feature_id);
        self.last_feature_id += 1;

        return self.last_feature_id - 1;
    }

    pub fn add_with_id(&mut self, point: Point2f, id: i32) {
        self.points.push(point);
        self.feature_ids.push(id);
    }

    pub fn remove(&mut self, index: usize) -> i32 {
        let id = self.feature_ids.remove(index);
        self.points.remove(index);
        id
    }
}