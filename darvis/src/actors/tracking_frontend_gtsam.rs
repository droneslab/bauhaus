extern crate g2o;
use gtsam::navigation::combined_imu_factor::{PreintegratedCombinedMeasurements, PreintegrationCombinedParams};
use log::{warn, info, debug, error};
use nalgebra::Vector3;
use std::{collections::{BTreeMap, BTreeSet, HashMap, VecDeque}, fmt::{self, Formatter}, mem, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{core::{no_array, Point2f, Scalar, CV_8UC1}, prelude::*, types::{VectorOfKeyPoint, VectorOfMat, VectorOfPoint2f, VectorOfu8}};
use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{
        messages::{ImageMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    }, map::{features::Features, frame::Frame, keyframe::MapPointMatches, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{good_features_to_track::GoodFeaturesExtractor, image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, ImuModule}, optimizer, orbslam_extractor::ORBExtractor, relocalization::Relocalization}, registered_actors::{new_feature_extraction_module, CAMERA_MODULE, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}, MAP_INITIALIZED
};
use crate::registered_actors::IMU;

use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{FeatureTracksAndIMUMsg, ImagePathMsg, InitKeyFrameMsg, NewKeyFrameMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackedMapPointData};
use crate::modules::module_definitions::MapInitializationModule;

pub struct TrackingFrontendGTSAM {
    system: System,
    sensor: Sensor,
    state: TrackingState,

    /// Frontend
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    imu_measurements_since_last_kf: ImuMeasurements,
    prev_points: VectorOfPoint2f,
    prev_points_ids: Vec<Id>,
    // ids: Vec<i32>,

    // Frames
    last_frame: Option<Frame>,
    curr_frame_id: i32,
    current_frame: Frame,

    // KeyFrames
    frames_since_last_kf: i32,

    // Modules 
    imu: IMU,

    /// References to map
    // map_initialized: bool,
    map: ReadWriteMap,
    initialization: Option<MapInitialization>, // data sent to map actor to initialize new map
    map_scale: f64,

    /// Global defaults
    // localization_only_mode: bool,
    first_image_time: f64,
    publish_frequency: f64,
    max_features: i32,
    min_distance: f64,

    // preint_gtsam: PreintegratedCombinedMeasurements,
}

impl Actor for TrackingFrontendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let imu = IMU::new();
        // let preint_gtsam = initialize_preintegration(
        //     SETTINGS.get::<f64>(IMU, "noise_gyro"), SETTINGS.get::<f64>(IMU, "gyro_walk"),
        //     SETTINGS.get::<f64>(IMU, "noise_acc"), SETTINGS.get::<f64>(IMU, "acc_walk"),
        //     [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        // );
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
            state: TrackingState::NotInitialized,
            imu,
            last_frame: None,
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            curr_frame_id: 0,
            // preint_gtsam,
            prev_points_ids: vec![],
            frames_since_last_kf: 0,
            first_image_time: 0.0,
            publish_frequency: SETTINGS.get::<f64>(TRACKING_FRONTEND, "publish_frequency"),
            max_features: SETTINGS.get::<i32>(TRACKING_FRONTEND, "max_features"),
            min_distance: SETTINGS.get::<f64>(TRACKING_FRONTEND, "min_distance"),
            imu_measurements_since_last_kf: ImuMeasurements::new(),
            prev_points: VectorOfPoint2f::new(),
            map_scale: 1.0,
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
            if matches!(self.state, TrackingState::NotInitialized) {
                // If map is not initialized yet, just extract features and try to initialize
                self.initialize_map(&image, timestamp).unwrap();

                if matches!(self.state, TrackingState::Ok) {
                    // If initialized successsfully,
                    // publish this frame so backend has a reference to the frame associated with the initialization
                    pub_this_frame = true;
                }
            } else {
                // If this is not the first image, track features from the last image
                let min_kfs = 3;
                let max_kfs = 6;
                if self.frames_since_last_kf >= min_kfs && self.frames_since_last_kf <= max_kfs {
                    pub_this_frame = true;
                    self.frames_since_last_kf = 0;
                    // self.first_image_time = timestamp;
                }
                self.current_frame = Frame::new_no_features(
                    self.curr_frame_id, 
                    Some(image.clone()),
                    timestamp,
                    Some(& self.last_frame.as_mut().unwrap())
                ).expect("Could not create frame!");

                // Track features
                let (tracked_kp, total_kp) = self.optical_flow().unwrap();

                let transform = self.calculate_transform().unwrap();
                let new_trans = *transform.get_translation() * (self.map_scale);
                let new_pose = Pose::new(new_trans, * transform.get_rotation()) * self.last_frame.as_ref().unwrap().pose.unwrap();

                self.current_frame.pose = Some(new_pose);
                debug!("Optical flow tracked {} from original {}... pose prediction: {:?}", tracked_kp, total_kp, new_pose);

                debug!("SOFIYA FEATURES. After optical flow, frame has N {}, features {}, mappoint matches {}", self.current_frame.features.num_keypoints, self.current_frame.features.get_all_keypoints().len(), self.current_frame.mappoint_matches.len());

                // let num_tracked_mappoints = self.map_feature_tracks_to_mappoints().unwrap();
                // debug!("Tracked {} mappoints", num_tracked_mappoints);

                if pub_this_frame {
                    if (total_kp as i32) < 500 { // TODO SOFIYA PUT THIS INTO CONFIG FILE
                        debug!("PUB THIS FRAME! Extracting new features! Total kp {}", total_kp);
                        // self.extract_features_and_add_to_existing().unwrap();
                    } else {
                        debug!("PUB THIS FRAME! Not extracting new features! Total kp {}", total_kp);
                    }
                }
            }

            self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
                keypoints: DVVectorOfKeyPoint::empty(),
                image,
                timestamp,
            }));

            if pub_this_frame {
                tracy_client::Client::running()
                    .expect("message! without a running Client")
                    .message("Publish frame!", 2);

                let mut old_imu_msmts = ImuMeasurements::new();
                mem::swap(&mut old_imu_msmts, &mut self.imu_measurements_since_last_kf);

                self.system.send(TRACKING_BACKEND, Box::new(FeatureTracksAndIMUMsg {
                    frame: self.current_frame.clone(),
                    imu_measurements: old_imu_msmts,
                    mappoint_ids: self.prev_points_ids.clone()
                }));
            }

            self.last_frame = Some(self.current_frame.clone()); // TODO SOFIYA AVOID THE CLONE?
            self.curr_frame_id += 1;
            self.frames_since_last_kf += 1;
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
                if !matches!(self.state, TrackingState::NotInitialized) || (self.current_frame.frame_id < SETTINGS.get::<f64>(SYSTEM, "fps") as i32) {
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
        let init_success = self.initialization.as_mut().unwrap().try_initialize(&self.current_frame, &mut self.imu.imu_preintegrated_from_last_kf)?;
        if init_success {
            {
                let _ = match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map,  &mut self.imu.imu_preintegrated_from_last_kf)? {
                    Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, _curr_kf_timestamp, map_scale)) => {
                        // Map needs to be initialized before tracking can begin. Received from map actor
                        if self.sensor.is_imu() {
                            self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(self.map.read()?.get_keyframe( curr_kf_id).imu_data.imu_preintegrated.as_ref().unwrap().get_updated_bias());
                        }

                        {
                            // Set current frame's updated info from map initialization
                            self.current_frame.pose = Some(curr_kf_pose);
                            self.current_frame.ref_kf_id = Some(curr_kf_id);
                            self.map_scale = map_scale;
                        }
                        self.state = TrackingState::Ok;

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

            self.state = TrackingState::Ok;
        } else {
            self.state = TrackingState::NotInitialized;
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
          epsilon: 0.1,
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

    fn extract_features_and_add_to_existing(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("extract features");

        // let mut corners = opencv::types::VectorOfPoint2f::new();
        // opencv::imgproc::good_features_to_track(
        //     & self.current_frame.image.as_ref().unwrap(), &mut corners, self.max_features, 0.01, self.min_distance, & no_array(), 3, false, 0.00
        // ).unwrap();

        // println!("Tracked features NEW: {}", corners.len());

        // self.prev_points = corners;


        // OLD:

        let (keypoints, descriptors) = //match self.sensor {
            // Sensor(FrameSensor::Mono, _) => {
            //     if !self.map_initialized || (self.current_frame.frame_id < SETTINGS.get::<f64>(SYSTEM, "fps") as i32) {
            //         self.orb_extractor_ini.as_mut().unwrap().extract(& self.current_frame.image.as_ref().unwrap()).unwrap()
            //     } else {
                    self.orb_extractor_left.extract(& self.current_frame.image.as_ref().unwrap()).unwrap();
        //         }
        //     },
        //     _ => { 
        //         // See GrabImageMonocular, GrabImageStereo, GrabImageRGBD in Tracking.cc
        //         todo!("Stereo, RGBD")
        //     }
        // };


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
            self.prev_points.push(keypoints.get(i as usize)?.pt());
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

    // fn need_new_keyframe(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
    //     // if self.sensor.is_imu() && !self.map.read()?.imu_initialized {
    //     //     if (self.current_frame.timestamp - self.last_kf_timestamp.unwrap()) * 1e9 >= 0.25 { // 250 milliseconds
    //     //         return Ok(true);
    //     //     } else {
    //     //         return Ok(false);
    //     //     }
    //     // }

    //     // Tracked MapPoints in the reference keyframe
    //     let min_observations = match num_kfs <= 2 {
    //         true => 2,
    //         false => 3
    //     };

    //     let map_lock = self.map.read()?;
    //     let tracked_mappoints = map_lock.get_keyframe(self.ref_kf_id.unwrap()).get_tracked_mappoints(&*map_lock, min_observations) as f32;

    //     // Check how many "close" points are being tracked and how many could be potentially created.
    //     let (tracked_close, non_tracked_close) = self.current_frame.check_close_tracked_mappoints();
    //     let need_to_insert_close = (tracked_close<100) && (non_tracked_close>70);

    //     // Thresholds
    //     let th_ref_ratio = match self.sensor {
    //         Sensor(FrameSensor::Mono, ImuSensor::None) => 0.9,
    //         Sensor(FrameSensor::Mono, ImuSensor::Some) => {
    //             // Points tracked from the local map
    //             if self.matches_inliers > 350 { 0.75 } else { 0.90 }
    //         },
    //         Sensor(FrameSensor::Stereo, _) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => 0.75,
    //         Sensor(FrameSensor::Rgbd, ImuSensor::None) => { if num_kfs < 2 { 0.4 } else { 0.75 } }
    //     };

    //     // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //     let c1a = self.frames_since_last_kf >= (self.max_frames_to_insert_kf as i32);
    //     // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //     let c1b = self.frames_since_last_kf >= (self.min_frames_to_insert_kf as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
    //     //Condition 1c: tracking is weak
    //     let sensor_is_right = match self.sensor {
    //         Sensor(FrameSensor::Mono, _) | Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => false,
    //         _ => true
    //     }; // I do not know why they just select for RGBD or Stereo without IMU
    //     let c1c = sensor_is_right && ((self.matches_inliers as f32) < tracked_mappoints * 0.25 || need_to_insert_close) ;
    //     // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    //     let c2 = (((self.matches_inliers as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_inliers > 15;

    //     // println!("c2: {}, inliers: {}, tracked mappoints: {}, th ref ratio: {}", c2, self.matches_inliers, tracked_mappoints, th_ref_ratio);

    //     // Temporal condition for Inertial cases
    //     let close_to_last_kf = match self.last_kf_timestamp {
    //         Some(timestamp) => self.current_frame.timestamp - timestamp >= 0.5 * 1e-9, // 500 milliseconds
    //         None => false
    //     };

    //     let c3 = self.sensor.is_imu() && close_to_last_kf;

    //     let recently_lost = match self.state {
    //         TrackingState::RecentlyLost => true,
    //         _ => false
    //     };
    //     let sensor_is_imumono = match self.sensor {
    //         Sensor(FrameSensor::Rgbd, ImuSensor::Some) => true,
    //         _ => false
    //     };
    //     let c4 = ((self.matches_inliers < 75 && self.matches_inliers > 15) || recently_lost) && sensor_is_imumono;

    //     // Note: removed code here about checking for idle local mapping and/or interrupting bundle adjustment
    //     let create_new_kf =  ((c1a||c1b||c1c) && c2)||c3 ||c4;

    //     tracy_client::Client::running()
    //         .expect("message! without a running Client")
    //         .message(format!("need new kf: {} {}", create_new_kf, LOCAL_MAPPING_IDLE.load(Ordering::SeqCst)).as_str(), 2);

    //     if LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) && create_new_kf {
    //         self.frames_since_last_kf = 0;
    //         return Ok(true);
    //     } else {
    //         self.frames_since_last_kf += 1;
    //         return Ok(false);
    //     }
    // }

    // fn preintegrate(&mut self, imu_measurements: &mut ImuMeasurements) {
    //     // This function will create a discrete IMU factor using the GTSAM preintegrator class
    //     // This will integrate from the current state time up to the new update time
    //     let _span = tracy_client::span!("create_imu_factor");

    //     let last_frame_timestamp = self.last_frame.as_ref().unwrap().timestamp;
    //     let mut imu_from_last_frame = VecDeque::with_capacity(imu_measurements.len()); // mvImuFromLastFrame
    //     let imu_per = 0.000000000001; // 0.001 in orbslam, adjusted here for different timestamp units

    //     while !imu_measurements.is_empty() {
    //         if imu_measurements.front().unwrap().timestamp < last_frame_timestamp - imu_per {
    //             imu_measurements.pop_front();
    //         } else if imu_measurements.front().unwrap().timestamp < self.current_frame.timestamp - imu_per {
    //             imu_from_last_frame.push_back(imu_measurements.pop_front().unwrap());
    //         } else {
    //             imu_from_last_frame.push_back(imu_measurements.pop_front().unwrap());
    //             break;
    //         }
    //     }
    //     let n = imu_from_last_frame.len() - 1;

    //     for i in 0..n {
    //         let mut tstep = 0.0;
    //         let mut acc: Vector3<f64> = Vector3::zeros(); // acc
    //         let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel

    //         if i == 0 && i < (n - 1) {
    //             let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
    //             let tini = imu_from_last_frame[i].timestamp - last_frame_timestamp;
    //             acc = (
    //                 imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
    //                 (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tini/tab)
    //             ) * 0.5;
    //             ang_vel = (
    //                 imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
    //                 (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tini/tab)
    //             ) * 0.5;
    //             tstep = imu_from_last_frame[i + 1].timestamp - last_frame_timestamp;
    //         } else if i < (n - 1) {
    //             acc = (imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc) * 0.5;
    //             ang_vel = (imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel) * 0.5;
    //             tstep = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
    //         } else if i > 0 && i == (n - 1) {
    //             let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
    //             let tend = imu_from_last_frame[i + 1].timestamp - self.current_frame.timestamp;
    //             acc = (
    //                 imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
    //                 (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tend/tab)
    //             ) * 0.5;
    //             ang_vel = (
    //                 imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
    //                 (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tend/tab)
    //             ) * 0.5;
    //             tstep = self.current_frame.timestamp - imu_from_last_frame[i].timestamp;
    //         } else if i == 0 && i == (n - 1) {
    //             acc = imu_from_last_frame[i].acc;
    //             ang_vel = imu_from_last_frame[i].ang_vel;
    //             tstep = self.current_frame.timestamp - last_frame_timestamp;
    //         }
    //         tstep = tstep * 1e9; 

    //         self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);
    //     }
    // }
}

// fn initialize_preintegration(
//     gyro_noise_density: f64, gyro_random_walk: f64,
//     accel_noise_density: f64, accel_random_walk: f64, 
//     prior_ba: [f64; 3], prior_bg: [f64; 3],

// ) -> PreintegratedCombinedMeasurements {
//     // Create GTSAM preintegration parameters for use with Foster's version
//     let prior_state_bias = gtsam::imu::imu_bias::ConstantBias::new(
//         &gtsam::base::vector::Vector3::new(prior_ba[0], prior_ba[1], prior_ba[2]),
//         &gtsam::base::vector::Vector3::new(prior_bg[0], prior_bg[1], prior_bg[2])
//     );
//     let mut params = PreintegrationCombinedParams::makesharedu();  // Z-up navigation frame: gravity points along negative Z-axis !!!
//     params.set_params(
//         accel_noise_density * accel_noise_density, // acc white noise in continuous
//         gyro_noise_density * gyro_noise_density, // gyro white noise in continuous
//         accel_random_walk * accel_random_walk, // acc bias in continuous
//         gyro_random_walk * gyro_random_walk, // gyro bias in continuous
//         0.1, // error committed in integrating position from velocities
//         1e-5 // error in the bias used for preintegration
//     );

//     // Actually create the GTSAM preintegration
//     PreintegratedCombinedMeasurements::new(params, &prior_state_bias)
// }
