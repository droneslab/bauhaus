extern crate g2o;
use log::{warn, info, debug, error};
use std::{collections::{BTreeMap, BTreeSet, HashMap}, fmt::{self, Debug}, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{prelude::*, types::VectorOfKeyPoint,types::VectorOfPoint2f, types::VectorOfu8, types::VectorOfMat};

use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, System, Timestamp}
};
use crate::{
    actors::{
        messages::{IMUInitializedMsg, ImageMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    }, map::{frame::Frame, keyframe::MapPointMatches, map::Id, pose::Pose}, modules::{image, imu::DVImu, map_initialization::DVInitialization, optimizer::{self, INV_LEVEL_SIGMA2}, orbextractor::DVORBextractor, orbmatcher::{self, descriptor_distance, SCALE_FACTORS}, relocalization::DVRelocalization}, registered_actors::{CAMERA, CAMERA_MODULE, FEATURE_DETECTION, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}, MapLock, MAP_INITIALIZED
};

use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{ImagePathMsg, InitKeyFrameMsg, NewKeyFrameMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackedMapPointData};
use crate::modules::module::FeatureExtractionModule;
use crate::modules::module::MapInitializationModule;
use crate::modules::module::CameraModule;

// Like the Frame object, but with fewer 
struct FlowFrame {
    frame_id: i32,
    timestamp: Timestamp,

    // Vision
    image: opencv::core::Mat,
    keypoints: VectorOfKeyPoint,
    descriptors: opencv::core::Mat,

    // Map
    pose: Option<Pose>,
    ref_kf_id: Option<Id>,
    // frame: Option<Frame>,
}
impl FlowFrame {
    fn create_frame_from_self(&self) -> Frame {
        let mut frame = Frame::new(
            self.frame_id, 
            DVVectorOfKeyPoint::new(self.keypoints.clone()),
            DVMatrix::new(self.descriptors.clone()),
            self.image.cols() as u32,
            self.image.rows() as u32,
            self.timestamp
        ).expect("Could not create frame!");
        frame.pose = self.pose;
        frame.ref_kf_id = self.ref_kf_id;
        frame
    }
}

pub struct TrackingOpticalFlow {
    system: System,
    sensor: Sensor,

    /// Frontend
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    _orb_extractor_right: Option<Box<dyn FeatureExtractionModule>>,
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    init_id: Id,

    /// Backend
    state: TrackingState,
    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_timestamp: Option<Timestamp>,

    // IMU 
    imu: DVImu,
    // Relocalization
    relocalization: DVRelocalization,
    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

    /// References to map
    map_initialized: bool,
    map_updated : bool,  // TODO (mvp) I'm not sure we want to use this
    map: MapLock,
    initialization: Option<DVInitialization>, // data sent to map actor to initialize new map

    // Local data used for different stages of tracking
    matches_inliers : i32, // mnMatchesInliers ... Current matches in frame
    local_keyframes: BTreeSet<Id>, //mvpLocalKeyFrames 
    local_mappoints: BTreeSet<Id>, //mvpLocalMapPoints
    track_in_view: HashMap::<Id, TrackedMapPointData>, // mbTrackInView , member variable in Mappoint
    track_in_view_r: HashMap::<Id, TrackedMapPointData>, // mbTrackInViewR, member variable in Mappoint
    kf_track_reference_for_frame: HashMap::<Id, Id>, // mnTrackReferenceForFrame, member variable in Keyframe
    mp_track_reference_for_frame: HashMap::<Id, Id>,  // mnTrackReferenceForFrame, member variable in Mappoint
    last_frame_seen: HashMap::<Id, Id>, // mnLastFrameSeen, member variable in Mappoint
    non_tracked_mappoints: HashMap<Id, i32>, // just for testing


    // Specific to optical flow module
    last_keyframe_mappoint_matches: Option<MapPointMatches>, // mnLastFrameMatches

    /// Global defaults
    localization_only_mode: bool,
    max_frames_to_insert_kf : i32 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
    min_frames_to_insert_kf: i32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
    frames_to_reset_imu: u32, //mnFramesToResetIMU

    min_num_features: u32, // Optical flow re-extraction of features if less than this number
}

impl Actor for TrackingOpticalFlow {
    type MapRef = MapLock;

    fn new_actorstate(system: System, map: Self::MapRef) -> TrackingOpticalFlow {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let _orb_extractor_right: Option<Box<dyn FeatureExtractionModule>> = match sensor.frame() {
            FrameSensor::Stereo => Some(Box::new(DVORBextractor::new(false))),
            FrameSensor::Mono | FrameSensor::Rgbd => None,
        };
        let orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>> = match sensor.is_mono() {
            true => Some(Box::new(DVORBextractor::new(true))), // sofiya orbslam2 loop closing
            false => None
        };

        TrackingOpticalFlow {
            system,
            orb_extractor_left: Box::new(DVORBextractor::new(false)),
            _orb_extractor_right,
            orb_extractor_ini,
            map_initialized: false,
            init_id: 0,
            sensor,
            map,
            initialization: Some(DVInitialization::new()),
            localization_only_mode: SETTINGS.get::<bool>(SYSTEM, "localization_only_mode"),
            max_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "max_frames_to_insert_kf"),
            min_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_frames_to_insert_kf"),
            frames_to_reset_imu: SETTINGS.get::<i32>(TRACKING_BACKEND, "frames_to_reset_IMU") as u32,
            state: TrackingState::NotInitialized,
            ref_kf_id: None,
            frames_since_last_kf: 0,
            last_kf_timestamp: None,
            imu: DVImu::new(None, None, sensor, false, false),
            relocalization: DVRelocalization{last_reloc_frame_id: 0, timestamp_lost: None},
            map_updated: false,
            trajectory_poses: Vec::new(),
            min_num_features: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_num_features")  as u32,
            matches_inliers: 0,
            local_keyframes: BTreeSet::new(),
            local_mappoints: BTreeSet::new(),
            track_in_view: HashMap::new(),
            track_in_view_r: HashMap::new(),
            kf_track_reference_for_frame: HashMap::new(),
            mp_track_reference_for_frame: HashMap::new(),
            last_frame_seen: HashMap::new(),
            non_tracked_mappoints: HashMap::new(),
            last_keyframe_mappoint_matches: None,
        }
    }

    fn spawn(system: System, map: Self::MapRef) {
        tracy_client::set_thread_name!("tracking optical flow");

        let mut actor = TrackingOpticalFlow::new_actorstate(system, map);
        let max_queue_size = actor.system.receiver_bound.unwrap_or(100);

        let mut curr_frame_id: Id = -1;
        let mut last_frame: Option<FlowFrame> = None;

        'outer: loop {
            let message = actor.system.receive().unwrap();

            if message.is::<ImagePathMsg>() || message.is::<ImageMsg>() {
                if actor.system.queue_len() > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Tracking optical flow dropped 1 frame");
                    continue;
                }

                let (image, timestamp) = if message.is::<ImagePathMsg>() {
                    let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                    (image::read_image_file(&msg.image_path), msg.timestamp)
                } else {
                    let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                    (msg.image, msg.timestamp)
                };

                match actor.track(image, &mut curr_frame_id, timestamp, &mut last_frame) {
                    Ok((curr_frame, created_kf)) => { 
                        match actor.state {
                            TrackingState::Ok | TrackingState::RecentlyLost => {
                                actor.update_trajectory_in_logs(&curr_frame, last_frame.as_mut().unwrap(), created_kf).expect("Could not save trajectory")
                            },
                            _ => {},
                        };
                        last_frame = Some(curr_frame);
                        curr_frame_id += 1;
                        actor.frames_since_last_kf = if created_kf { 0 } else { actor.frames_since_last_kf + 1 };
                     },
                    Err(e) => {
                        error!("Error in tracking optical flow: {}", e);
                        continue;
                    }
                };
            } else if message.is::<InitKeyFrameMsg>() {
                // Received from local mapping after it inserts a keyframe
                let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

                actor.ref_kf_id = Some(msg.kf_id);
            } else if message.is::<LastKeyFrameUpdatedMsg>() {
                // Received from local mapping after it culls and creates new MPs for the last inserted KF
                actor.state = TrackingState::Ok;
            } else if message.is::<IMUInitializedMsg>() {
                // TODO (IMU) process message from local mapping!
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Tracking backend received unknown message type!");
            }
        }
    }
}

impl TrackingOpticalFlow {
    fn track(
        &mut self,
        curr_img: opencv::core::Mat, curr_frame_id: &mut i32, timestamp: Timestamp,
        last_frame: &mut Option<FlowFrame>
    ) -> Result<(FlowFrame, bool), Box<dyn std::error::Error>> {
        let mut curr_flow_frame: FlowFrame;

        if matches!(self.state, TrackingState::NotInitialized) {
            // If map is not initialized yet, just extract features and try to initialize
            let (keypoints, descriptors) = self.extract_features(curr_img.clone(), *curr_frame_id);

            curr_flow_frame = FlowFrame {
                keypoints,
                descriptors,
                image: curr_img.clone(),
                pose: None,
                ref_kf_id: None,
                timestamp,
                frame_id: *curr_frame_id
            };

            if self.initialization.is_none() {
                self.initialization = Some(DVInitialization::new());
            }
            let current_frame = curr_flow_frame.create_frame_from_self();
            let init_success = self.initialization.as_mut().unwrap().try_initialize(&current_frame)?;
            if init_success {
                let (ini_kf_id, curr_kf_id);
                {
                    (ini_kf_id, curr_kf_id) = match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map) {
                        Some((curr_kf_pose, curr_kf_id, ini_kf_id, _local_mappoints, curr_kf_timestamp)) => {
                            // Map needs to be initialized before tracking can begin. Received from map actor
                            self.frames_since_last_kf = 0;
                            self.ref_kf_id = Some(curr_kf_id);
                            self.last_kf_timestamp = Some(curr_kf_timestamp);

                            {
                                // Set current frame's updated info from map initialization
                                curr_flow_frame.pose = Some(curr_kf_pose);
                                curr_flow_frame.ref_kf_id = Some(curr_kf_id);
                                self.last_keyframe_mappoint_matches = Some(self.map.read().keyframes.get(&curr_kf_id).unwrap().clone_matches());
                            }
                            self.state = TrackingState::Ok;

                            // Log initial pose in shutdown actor
                            self.system.send(SHUTDOWN_ACTOR, 
                            Box::new(TrajectoryMsg{
                                    pose: self.map.read().keyframes.get(&ini_kf_id).unwrap().pose,
                                    ref_kf_id: ini_kf_id,
                                    timestamp: self.map.read().keyframes.get(&ini_kf_id).unwrap().timestamp
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

                // Send first two keyframes to local mapping
                self.system.send(LOCAL_MAPPING, Box::new(
                    InitKeyFrameMsg { kf_id: ini_kf_id }
                ));
                self.system.send(LOCAL_MAPPING,Box::new(
                    InitKeyFrameMsg { kf_id: curr_kf_id }
                ));

                self.state = TrackingState::Ok;
                sleep(Duration::from_millis(50)); // Sleep just a little to allow local mapping to process first keyframe
            } else {
                self.state = TrackingState::NotInitialized;
            }
        } else {
            // If this is not the first image, track features from the last image

            let last_flow_frame = last_frame.as_mut().unwrap();
            curr_flow_frame = FlowFrame {
                keypoints: VectorOfKeyPoint::new(),
                descriptors: opencv::core::Mat::default(),
                image: curr_img.clone(),
                pose: None,
                ref_kf_id: last_flow_frame.ref_kf_id,
                timestamp,
                frame_id: *curr_frame_id
            };

            let mut status = VectorOfu8::new();

            Self::feature_tracking_kps(last_flow_frame, &mut curr_flow_frame, &mut status)?;

            debug!("Tracked {} from original {}, desc rows {}", curr_flow_frame.keypoints.len(), status.len(), curr_flow_frame.descriptors.rows());

            let points1 = last_flow_frame.keypoints.iter().map(|kp| kp.pt()).collect();
            let points2 = curr_flow_frame.keypoints.iter().map(|kp| kp.pt()).collect();

            curr_flow_frame.pose = Some(Self::calculate_transform(&points1, &points2)? * last_flow_frame.pose.unwrap());

            debug!("Current pose: {:?}", curr_flow_frame.pose);
            println!("Keypoints len: {}", curr_flow_frame.keypoints.len());

            if curr_flow_frame.keypoints.len() < self.min_num_features as usize {
                debug!("Re-extracting features");
                self.extract_features_and_add_to_existing(&mut curr_flow_frame, *curr_frame_id)?;
            }

            // if self.need_new_keyframe(&mut curr_flow_frame, &last_flow_frame) {
            //     debug!("Need new keyframe!");

            //     self.create_new_keyframe(&mut curr_flow_frame, *curr_frame_id, & last_flow_frame.pose.unwrap())?;
            // }
        };

        self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
            keypoints: DVVectorOfKeyPoint::empty(),
            image: curr_img,
            timestamp,
        }));


        Ok((curr_flow_frame, false))
    }

    fn extract_features(&mut self, image: opencv::core::Mat, curr_frame_id: i32) -> (VectorOfKeyPoint, Mat) {
        let _span = tracy_client::span!("extract features");


        let (keypoints, descriptors) = if self.map_initialized && (curr_frame_id - self.init_id > self.max_frames_to_insert_kf) {
            self.orb_extractor_left.extract(image).unwrap()
        } else if self.sensor.is_mono() {
            self.orb_extractor_ini.as_mut().unwrap().extract(image).unwrap()
        } else {
            self.orb_extractor_left.extract(image).unwrap()
        };
        match self.sensor.frame() {
            FrameSensor::Stereo => todo!("Stereo"), //Also call extractor_right, see Tracking::GrabImageStereo,
            _ => {}
        }
        (keypoints, descriptors)
    }


    fn extract_features_and_add_to_existing(&mut self, frame: &mut FlowFrame, curr_frame_id: i32) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("extract features");

        let (keypoints, descriptors) = self.extract_features(frame.image.clone(), curr_frame_id);

        let mut new_keypoints = VectorOfKeyPoint::new();
        let mut new_descriptor = Mat::default();
        let mut new_descriptor_vec = VectorOfMat::new();

        for i in 0..frame.keypoints.len() {
            new_keypoints.push(frame.keypoints.get(i)?);
            new_descriptor_vec.push(frame.descriptors.row(i as i32)?);
        }

        for i in 0..keypoints.len() {
            new_keypoints.push(keypoints.get(i)?);
            new_descriptor_vec.push(descriptors.row(i as i32)?);
        }

        opencv::core::vconcat(&new_descriptor_vec, &mut new_descriptor).expect("Failed to concatenate");

        frame.keypoints = new_keypoints;
        frame.descriptors = new_descriptor;
        Ok(())
    }

    fn feature_tracking_kps(frame1: &mut FlowFrame, frame2: &mut FlowFrame, status: &mut VectorOfu8) -> Result<(), Box<dyn std::error::Error>> {
        let mut points1: VectorOfPoint2f = frame1.keypoints.iter().map(|kp| kp.pt()).collect();
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
          &frame1.image, &frame2.image, &mut points1, &mut points2, status, &mut err, win_size, max_level, termcrit, 0, 0.001,
        )?;

        //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
        let mut indez_correction = 0;
        let mut new_desc2_vec = VectorOfMat::new();

        for i in 0..status.len() {
          let pt = points2.get(i - indez_correction)?;
          if (status.get(i)? == 0) || pt.x < 0.0 || pt.y < 0.0 {
            if pt.x < 0.0 || pt.y < 0.0 {
              status.set(i, 0)?;
            }
            points1.remove(i - indez_correction)?;
            points2.remove(i - indez_correction)?;

            frame1.keypoints.remove(i - indez_correction)?;
            indez_correction = indez_correction + 1;
            } else {
                let curr_kp = frame1.keypoints.get(i - indez_correction)?;
                frame2.keypoints.push(opencv::core::KeyPoint::new_coords(pt.x, pt.y, curr_kp.size(), curr_kp.angle(), curr_kp.response(), curr_kp.octave(), curr_kp.class_id()).expect("Failed to create keypoint"));

                new_desc2_vec.push(frame1.descriptors.row(i as i32)?);
            }
        }

        let mut new_descriptor = opencv::core::Mat::default();

        opencv::core::vconcat(&new_desc2_vec, &mut new_descriptor).expect("Failed to concatenate");

        frame2.descriptors = new_descriptor;

        Ok(())
    }

    pub fn calculate_transform(prev_features: &VectorOfPoint2f, curr_features: &VectorOfPoint2f) -> Result<Pose, Box<dyn std::error::Error>> {
        //recovering the pose and the essential matrix
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

    fn update_trajectory_in_logs(
        &mut self, current_frame: &FlowFrame, last_frame: &FlowFrame, created_new_kf: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let relative_pose = {
            if last_frame.pose.is_none() {
                // This should only happen for the first frame after initialization
                current_frame.pose.unwrap() * self.map.read().keyframes.get(&self.ref_kf_id.unwrap()).expect("Can't get ref kf from map").pose.inverse()
            } else {
                current_frame.pose.unwrap() * last_frame.pose.unwrap().inverse()
            }
        };

        let relative_pose = {
            let ref_kf_pose = if created_new_kf {
                // If we created a new kf this round, the relative keyframe pose is the same as the current frame pose.
                current_frame.pose.unwrap()
            } else if last_frame.pose.is_none() {
                // This condition should only happen for the first frame after initialization
                self.map.read().keyframes.get(&self.ref_kf_id.unwrap()).expect("Can't get ref kf from map").pose.inverse()
            } else {
                last_frame.pose.unwrap().inverse()
            };

            current_frame.pose.unwrap() * ref_kf_pose.inverse()
        };

        self.trajectory_poses.push(relative_pose);

        info!("Frame {} pose: {:?}", current_frame.frame_id, relative_pose);

        self.system.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: current_frame.pose.unwrap().inverse(),
                ref_kf_id: current_frame.ref_kf_id.unwrap(),
                timestamp: current_frame.timestamp
            })
        );

        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            pose: current_frame.pose.unwrap(),
            mappoint_matches: vec![],
            nontracked_mappoints: HashMap::new(),
            mappoints_in_tracking: self.local_mappoints.clone(),
            timestamp: current_frame.timestamp
        }));

        Ok(())
    }


     fn associate_keypoints_with_map(&mut self, current_frame: &mut Frame) -> Result<bool, Box<dyn std::error::Error>> {
        let lock = self.map.read();
        let ref_kf = lock.keyframes.get(&self.ref_kf_id.unwrap()).expect("Could not get ref kf");

        println!("Ref kf is: {:?}", ref_kf.id);

        // FROM FUSE:

        let tcw = current_frame.pose.unwrap().get_translation();
        let rcw = current_frame.pose.unwrap().get_rotation();
        let ow = current_frame.get_camera_center().unwrap();


        println!("Current frame translation: {:?}", tcw);

        let th = 3.0;
        const  TH_LOW: i32 = 50;

        let mut skipping_no_mp = 0;
        let mut skipping_mp_deleted =0;
        let mut skip_depth = 0;
        let mut skip_image = 0;
        let mut skip_pyramid = 0;
        let mut skip_viewing = 0;
        let mut skip_indices = 0;
        let mut skip_levels = 0;
        let mut skip_e2 = 0;
        let mut skip_threshold = 0;

        for mappoint in ref_kf.get_mp_matches() {
            let mp_id = match mappoint {
                Some((mp_id, _)) => mp_id,
                None => {
                    skipping_no_mp +=1;
                    continue
                }
            };

            let (mut best_dist, mut best_idx);
            {
                let mappoint = match lock.mappoints.get(&mp_id) {
                    Some(mp) => mp,
                    None => {
                        skipping_mp_deleted += 1;
                        continue
                    }
                };
                let p_3d_w = mappoint.position;
                let p_3d_c = (*rcw) * (*p_3d_w) + (*tcw);

                // Depth must be positive
                if p_3d_c[2] < 0.0 {
                    skip_depth += 1;
                    continue;
                }

                let _inv_z = 1.0 / p_3d_c[2]; // Used by stereo below
                let uv = CAMERA_MODULE.project(DVVector3::new(p_3d_c));

                // Point must be inside the image
                if !current_frame.features.is_in_image(uv.0, uv.1) {
                    skip_image += 1;
                    continue;
                }

                let max_distance = mappoint.get_max_distance_invariance();
                let min_distance = mappoint.get_min_distance_invariance();
                let po = *p_3d_w - *ow;
                let dist_3d = po.norm();

                // Depth must be inside the scale pyramid of the image
                if dist_3d < min_distance || dist_3d > max_distance {
                    skip_pyramid += 1;
                    continue;
                }

                // Viewing angle must be less than 60 deg
                let pn = mappoint.normal_vector;
                if po.dot(&pn) < 0.5 * dist_3d {
                    skip_viewing += 1;
                    continue;
                }

                // Search in a radius
                let predicted_level = mappoint.predict_scale(&dist_3d);
                let radius = th * SCALE_FACTORS[predicted_level as usize];
                let indices = current_frame.features.get_features_in_area(&uv.0, &uv.1, radius as f64, None);
                if indices.is_empty() {
                    skip_indices += 1;
                    continue;
                }

                // Match to the most similar keypoint in the radius
                best_dist = 256;
                best_idx = -1;

                for idx2 in indices {
                    let (kp, is_right) = current_frame.features.get_keypoint(idx2 as usize);
                    let kp_level = kp.octave();

                    if kp_level < predicted_level - 1 || kp_level > predicted_level {
                        skip_levels += 1;
                        continue;
                    }

                    let (kpx, kpy, ex, ey, e2);
                    kpx = kp.pt().x as f64;
                    kpy = kp.pt().y as f64;
                    ex = uv.0 - kpx;
                    ey = uv.1 - kpy;
                    e2 = ex * ex + ey * ey;
                    if e2 * INV_LEVEL_SIGMA2[kp_level as usize] as f64 > 5.99 {
                        skip_e2 += 1;
                        continue;
                    }

                    let desc_kf = current_frame.features.descriptors.row(idx2);
                    let dist = descriptor_distance(&lock.mappoints.get(mp_id).unwrap().best_descriptor, &desc_kf);

                    if dist < best_dist {
                        best_dist = dist;
                        best_idx = idx2 as i32;
                    }
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            if best_dist <= TH_LOW {
                current_frame.mappoint_matches.add(best_idx as u32, *mp_id, false);
            } else {
                skip_threshold += 1;
            }
        }


        println!("SKIPPING: no mp: {}, mp deleted: {}, depth: {}, image: {}, pyramid: {}, viewing: {}, indices: {}, levels: {}, e2: {}, threshold: {}", skipping_no_mp, skipping_mp_deleted, skip_depth, skip_image, skip_pyramid, skip_viewing, skip_indices, skip_levels, skip_e2, skip_threshold);

        println!("Current frame mappoint matches: {:?}", current_frame.mappoint_matches.debug_count);
        Ok(true)

        // let mut matches = orbmatcher::search_by_projection_with_threshold(
        //     current_frame,
        //     last_frame,
        //     th,
        //     true,
        //     &self.map,
        //     self.sensor
        // )?;

        // if matches < 20 {
        //     info!("tracking_backend::track_with_motion_model;not enough matches, wider window search");
        //     current_frame.mappoint_matches.clear();
        //     matches = orbmatcher::search_by_projection_with_threshold(
        //         current_frame,
        //         last_frame,
        //         2 * th,
        //         self.sensor.is_mono(),
        //         &self.map,
        //         self.sensor
        //     )?;
        //     debug!("Tracking search by projection with previous frame: {} matches / {} mappoints in last frame. ({} total mappoints in map)", matches, last_frame.mappoint_matches.debug_count, self.map.read().mappoints.len());
        // }

        // if matches < 20 {
        //     warn!("tracking_backend::track_with_motion_model;not enough matches!!");
        //     return Ok(self.sensor.is_imu());
        // }

        // optimizer::optimize_pose(current_frame, &self.map);

        // Discard outliers
        // let nmatches_map = self.discard_outliers(current_frame);

        // for (idx, _kp) in current_frame.features.get_all_keypoints().iter().enumerate() {
        //     match ref_kf.get_mp_match(&(idx as u32)) {
        //         Some((mp_id, _outlier)) => {
        //             // let mappoint = lock.mappoints.get(&mp_id).expect("Could not get mappoint");
        //             current_frame.mappoint_matches.add(idx as u32, mp_id, false);
        //         },
        //         None => {}
        //     }
        // }
        // match self.sensor.is_imu() {
        //     true => { return Ok(true); },
        //     false => { return Ok(nmatches_map >= 10); }
        // };
    }

    fn track_reference_keyframe(&mut self, current_frame: &mut Frame, last_frame_pose: &Pose) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track_reference_keyframe");
        // Tracking::TrackReferenceKeyFrame()
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver

        current_frame.compute_bow();
        let nmatches;
        {
            let map_read_lock = self.map.read();
            self.ref_kf_id = Some(map_read_lock.last_kf_id);
            let ref_kf = map_read_lock.keyframes.get(&self.ref_kf_id.unwrap()).unwrap();

            println!("Reference kf is {}", ref_kf.id);

            nmatches = orbmatcher::search_by_bow_f(ref_kf, current_frame, true, 0.7)?;
            debug!("Tracking search by bow: {} matches / {} matches in keyframe {}. ({} total mappoints in map)", nmatches, ref_kf.get_mp_matches().iter().filter(|item| item.is_some()).count(), ref_kf.id, map_read_lock.mappoints.len());
        }

        if nmatches < 15 {
            warn!("track_reference_keyframe has fewer than 15 matches = {}!!\n", nmatches);
            return Ok(false);
        }

        optimizer::optimize_pose(current_frame, &self.map);

        // Discard outliers
        let nmatches_map = self.discard_outliers(current_frame);

        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn track_local_map(&mut self, current_frame: &mut Frame) -> (bool, i32) {
        let _span = tracy_client::span!("track_local_map");
        // bool Tracking::TrackLocalMap()
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        self.update_local_keyframes(current_frame);
        self.update_local_points(current_frame);
        match self.search_local_points(current_frame) {
            Ok(res) => res,
            Err(e) => warn!("Error in search_local_points: {}", e)
        }

        // optimizer::optimize_pose(current_frame, &self.map);

        self.matches_inliers = 0;
        // Update MapPoints Statistics
        for index in 0..current_frame.mappoint_matches.len() {
            if let Some((mp_id, is_outlier)) = current_frame.mappoint_matches.get(index) {
                if !is_outlier {
                    if let Some(mp) = self.map.read().mappoints.get(&mp_id) {
                        mp.increase_found(1);
                    }

                    if self.localization_only_mode {
                        let map_read_lock = self.map.read();
                        if let Some(mp) = map_read_lock.mappoints.get(&mp_id) {
                            if mp.get_observations().len() > 0 {
                                self.matches_inliers+=1;
                            }
                        } else {
                            current_frame.mappoint_matches.delete_at_indices((index as i32, -1));
                        }
                    } else {
                        self.matches_inliers += 1;
                    }

                } else if !self.sensor.is_mono() {
                    todo!("Stereo");
                    //mCurrentFrame.mappoint_matches[i] = static_cast<MapPoint*>(NULL);
                    // current_frame.as_mut().unwrap().mappoint_matches.remove(&index);
                }
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames_to_insert_kf as i32) && self.matches_inliers<50 {
            warn!("track_local_map unsuccessful; matches in frame < 50 : {}",self.matches_inliers);
            return (false, self.matches_inliers);
        }

        match self.state {
            TrackingState::RecentlyLost => { 
                return (self.matches_inliers > 10, self.matches_inliers);
            },
            _ => {}
        }

        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => { 
                return (
                    !(self.matches_inliers<15 && self.imu.is_initialized) && !(self.matches_inliers<50 && !self.imu.is_initialized),
                    self.matches_inliers
                )
            },
            Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => {
                return (self.matches_inliers >= 15, self.matches_inliers);
            },
            _ => { return (self.matches_inliers > 30, self.matches_inliers) }
        }
    }

    fn update_local_keyframes(&mut self, current_frame: &mut Frame) -> Option<()> {
        let _span = tracy_client::span!("update_local_keyframes");
        // void Tracking::UpdateLocalKeyFrames()
        // Each map point votes for the keyframes in which it has been observed

        println!("BEGIN UPDATE LOCAL KEYFRAMES");

        // println!("Current frame mappoint matches: {:?}", current_frame.mappoint_matches);

        // BTreeMap so items are sorted, so we have the same output as orbslam for testing. Can probably revert to regular hashmap later.
        let mut kf_counter = BTreeMap::<Id, i32>::new();
        let lock = self.map.read();
        // TODO (IMU) ... Check below should be !self.imu.is_initialized || current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2
        for i in 0..current_frame.mappoint_matches.len() {
            if let Some((mp_id, _is_outlier)) = current_frame.mappoint_matches.get(i as usize) {
                if let Some(mp) = lock.mappoints.get(&mp_id) {
                    for kf_id in mp.get_observations().keys() {
                        *kf_counter.entry(*kf_id).or_insert(0) += 1;
                    }
                } else {
                    current_frame.mappoint_matches.delete_at_indices((i as i32, -1));
                }
            }
        }

        println!("kf counter: {:?}", kf_counter);
        
        let (mut max, mut max_kf_id) = (0, 0);
        self.local_keyframes.clear();
        let mut local_kf_vec = Vec::<Id>::new();

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (kf_id, count) in kf_counter {
            if count > max {
                max = count;
                max_kf_id = kf_id;
            }
            local_kf_vec.push(kf_id);
            self.kf_track_reference_for_frame.insert(kf_id, current_frame.frame_id);
        }

        // Also include some keyframes that are neighbors to already-included keyframes
        let mut i = 0;
        while local_kf_vec.len() <= 80 && i < local_kf_vec.len() { // Limit the number of keyframes
            let next_kf_id = local_kf_vec[i];
            let kf = lock.keyframes.get(&next_kf_id);
            match kf {
                Some(kf) => {
                    for kf_id in &kf.get_covisibility_keyframes(10) {
                        if self.kf_track_reference_for_frame.get(kf_id) != Some(&current_frame.frame_id) {
                            local_kf_vec.push(*kf_id);
                            self.kf_track_reference_for_frame.insert(*kf_id, current_frame.frame_id);
                            break;
                        }
                    }
                    for kf_id in &kf.children {
                        if self.kf_track_reference_for_frame.get(kf_id) != Some(&current_frame.frame_id) {
                            local_kf_vec.push(*kf_id);
                            self.kf_track_reference_for_frame.insert(*kf_id, current_frame.frame_id);
                            break;
                        }
                    }

                    if let Some(parent_id) = kf.parent {
                        if self.kf_track_reference_for_frame.get(&parent_id) != Some(&current_frame.frame_id) {
                            local_kf_vec.push(parent_id);
                            self.kf_track_reference_for_frame.insert(parent_id, current_frame.frame_id);
                        }
                    };
                },
                None => {
                    // KF could have been deleted since constructing local_kf_vec
                    local_kf_vec.remove(i);
                },
            };

            i += 1;
        }
        self.local_keyframes = local_kf_vec.iter().map(|x| *x).collect();

        // Add 10 last temporal KFs (mainly for IMU)
        if self.sensor.is_imu() && self.local_keyframes.len() < 80 {
            todo!("IMU");
            // For the last 20 created keyframes, save into new_local_keyframes
            // until one of them was not in self.local_keyframes. Then, quit.
        }

        if max_kf_id != 0 {
            current_frame.ref_kf_id = Some(max_kf_id);
            self.ref_kf_id = Some(max_kf_id);
        }
        None
    }

    fn update_local_points(&mut self, current_frame: &mut Frame) {
        let _span = tracy_client::span!("update_local_points");
        // void Tracking::UpdateLocalPoints()
        self.local_mappoints.clear();
        let lock = self.map.read();
        let mut kfs_to_remove = vec![];
        for kf_id in self.local_keyframes.iter().rev() {
            match lock.keyframes.get(kf_id) {
                Some(kf) => {
                    let mp_ids = kf.get_mp_matches();
                    for item in mp_ids {
                        if let Some((mp_id, _)) = item {
                            if self.mp_track_reference_for_frame.get(mp_id) != Some(&current_frame.frame_id) {
                                self.local_mappoints.insert(*mp_id);
                                self.mp_track_reference_for_frame.insert(*mp_id, current_frame.frame_id);
                            }
                        }
                    }
                },
                None => kfs_to_remove.push(*kf_id),
            }
        }

        // KF could have been deleted since construction of local_keyframes
        for kf_id in kfs_to_remove {
            self.local_keyframes.remove(&kf_id);
        }
    }

    fn search_local_points(&mut self, current_frame: &mut Frame) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("search_local_points");
        //void Tracking::SearchLocalPoints()

        // Do not search map points already matched
        {
            let mut increased_visible = vec![];
            let lock = self.map.read();
            for index in 0..current_frame.mappoint_matches.len() {
                if let Some((id, _)) = current_frame.mappoint_matches.get(index as usize) {
                    if let Some(mp) = lock.mappoints.get(&id) {
                        mp.increase_visible(1);
                        increased_visible.push(id);

                        self.last_frame_seen.insert(id, current_frame.frame_id);
                        self.track_in_view.remove(&id);
                        self.track_in_view_r.remove(&id);
                    } else {
                        current_frame.mappoint_matches.delete_at_indices((index as i32, -1));
                    }
                }
            }
        }

        // Project points in frame and check its visibility
        let mut to_match = 0;
        {
            let lock = self.map.read();
            for mp_id in &self.local_mappoints {
                if self.last_frame_seen.get(mp_id) == Some(&current_frame.frame_id) {
                    continue;
                }

                match lock.mappoints.get(mp_id) {
                    Some(mp) => {
                        // Project (this fills MapPoint variables for matching)
                        let (tracked_data_left, tracked_data_right) = current_frame.is_in_frustum(mp, 0.5);
                        if tracked_data_left.is_some() || tracked_data_right.is_some() {
                            lock.mappoints.get(&mp_id).unwrap().increase_visible(1);
                            to_match += 1;
                        }
                        if let Some(d) = tracked_data_left {
                            self.track_in_view.insert(*mp_id, d);
                        } else {
                            self.track_in_view.remove(mp_id);
                        }
                        if let Some(d) = tracked_data_right {
                            self.track_in_view_r.insert(*mp_id, d);
                        } else {
                            self.track_in_view_r.remove(mp_id);
                        }
                    },
                    None => {
                        continue;
                    }
                };
            }
        }

        debug!("Local points: {:?}", self.local_mappoints);
        debug!("Tracl in view: {:?}", self.track_in_view);
        if to_match > 0 {
            let mut th = match self.sensor.frame() {
                FrameSensor::Rgbd => 3,
                _ => 1
            };
            if self.imu.is_initialized {
                if self.imu.imu_ba2 {
                    th = 2;
                } else {
                    th = 6;
                }
            } else if !self.imu.is_initialized && self.sensor.is_imu() {
                th = 10;
            }

            // If the camera has been relocalised recently, perform a coarser search
            if current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
                th = 5;
            }
            match self.state {
                TrackingState::RecentlyLost | TrackingState::Lost => th = 15,
                _ => {}
            }

            let (non_tracked_points, matches) = orbmatcher::search_by_projection(
                current_frame,
                &mut self.local_mappoints,
                th, 0.8,
                &self.track_in_view, &self.track_in_view_r,
                &self.map, self.sensor
            )?;
            self.non_tracked_mappoints = non_tracked_points;
            debug!("Tracking search local points: {} matches / {} local points. ({} total mappoints in map)", matches, self.local_mappoints.len(), self.map.read().mappoints.len());

        }

        Ok(())
    }

    fn create_new_keyframe(&mut self, current_frame: &mut FlowFrame, curr_frame_id: i32, last_frame_pose: &Pose) -> Result<(), Box<dyn std::error::Error>>{
        let _span = tracy_client::span!("create_new_keyframe");

        // let mut status = VectorOfu8::new();
        // current_frame.descriptors = opencv::core::Mat::default();
        // current_frame.keypoints = VectorOfKeyPoint::new();

        // Self::feature_tracking_kps(last_frame, current_frame, &mut status)?;

        // debug!("Tracked {} from original {}, desc rows {}", current_frame.keypoints.len(), status.len(), current_frame.descriptors.rows());

        // debug!("Flow tracking keypoints extracted {} from original {}, desc rows {}", current_frame.keypoints.len(), status.len(), current_frame.descriptors.rows());

        let mut new_kf = current_frame.create_frame_from_self();

        // self.associate_keypoints_with_map(&mut new_kf)?;
        self.track_reference_keyframe(&mut new_kf, last_frame_pose)?;
        // self.track_local_map(&mut new_kf);

        tracy_client::Client::running()
        .expect("message! without a running Client")
        .message("create new keyframe", 2);

        // KeyFrame created here and inserted into map
        self.system.send(
            LOCAL_MAPPING,
            Box::new( NewKeyFrameMsg{  keyframe: new_kf, } )
        );

        Ok(())
    }

    fn need_new_keyframe(&mut self, current_frame: &mut FlowFrame, last_frame: & FlowFrame) -> bool {
        let num_kfs = self.map.read().keyframes.len();
        let not_enough_frames_since_last_reloc = (current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames_to_insert_kf as i32)) && (num_kfs as i32 > self.max_frames_to_insert_kf);
        let imu_not_initialized = self.sensor.is_imu() && !self.imu.is_initialized;

        let too_close_to_last_kf = match self.last_kf_timestamp {
            Some(timestamp) => current_frame.timestamp - timestamp < 0.25, // 250 milliseconds
            None => false
        };

        if self.localization_only_mode || not_enough_frames_since_last_reloc || (imu_not_initialized && too_close_to_last_kf) {
            return false;
        } else if imu_not_initialized && !too_close_to_last_kf {
            return true;
        }

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        let c1a = self.frames_since_last_kf >= (self.max_frames_to_insert_kf as i32);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        let c1b = self.frames_since_last_kf >= (self.min_frames_to_insert_kf as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
        //Condition 1c: tracking is weak
        let c1c = current_frame.keypoints.len() < self.min_num_features as usize;

        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        // let c2 = (((self.matches_inliers as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_inliers > 15;

        // let create_new_kf =  ((c1a||c1b||c1c) && c2)||c3 ||c4;

        println!("Need new kf? ({} || {} || {}) && {}", c1a, c1b, c1c, LOCAL_MAPPING_IDLE.load(Ordering::SeqCst));

        // tracy_client::Client::running()
        //     .expect("message! without a running Client")
        //     .message(format!("need new kf: {} {}", c1a || c1b || c1c, LOCAL_MAPPING_IDLE.load(Ordering::SeqCst)).as_str(), 2);

        return (c1a || c1b || c1c) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
    }

    fn discard_outliers(&mut self, current_frame: &mut Frame) -> i32 {
        let discarded = current_frame.delete_mappoint_outliers();
        for (mp_id, _index) in discarded {
            self.last_frame_seen.insert(mp_id, current_frame.frame_id);
        }
        current_frame.mappoint_matches.tracked_mappoints(&*self.map.read(), 1)
    }

}
