extern crate g2o;
use cxx::UniquePtr;
use log::{warn, info, debug, error};
use std::{fmt, fmt::Debug, collections::{HashSet, HashMap}, sync::atomic::Ordering};
use opencv::{prelude::*, types::VectorOfKeyPoint,};

use core::{
    actor::Actor,
    sensor::{Sensor, FrameSensor, ImuSensor},
    matrix::*,
    config::*
};
use crate::{
    registered_actors::{FEATURE_DETECTION, CAMERA, VISUALIZER, TRACKING_BACKEND, LOCAL_MAPPING, SHUTDOWN_ACTOR},
    actors::{
        messages::{ShutdownMsg, ImageMsg, VisFeaturesMsg, LastKeyFrameUpdatedMsg, IMUInitializedMsg},
        tracking_backend::TrackingState,
    },
    modules::{imu::ImuModule, relocalization::Relocalization, map_initialization::Initialization, orbmatcher, optimizer},
    ActorChannels,
    map::{map::Id, misc::Timestamp, pose::Pose, frame::Frame}, MapLock,
};

use super::{tracking_backend::TrackedMapPointData, local_mapping::LOCAL_MAPPING_IDLE, messages::{NewKeyFrameMsg, VisTrajectoryMsg, TrajectoryMsg, InitKeyFrameMsg}};

#[derive(Debug)]
pub struct TrackingFull {
    actor_channels: ActorChannels,
    sensor: Sensor,

    /// Frontend
    orb_extractor_left: DVORBextractor,
    orb_extractor_right: Option<DVORBextractor>,
    orb_extractor_ini: Option<DVORBextractor>,
    init_id: Id,

    /// Backend
    state: TrackingState,
    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_timestamp: Option<Timestamp>,
    // Local data used for different stages of tracking
    matches_inliers : i32, // mnMatchesInliers ... Current matches in frame
    local_keyframes: Vec<Id>, //mvpLocalKeyFrames 
    local_mappoints: HashSet<Id>, //mvpLocalMapPoints
    track_in_view: HashMap::<Id, TrackedMapPointData>, // mbTrackInView , member variable in Mappoint
    track_in_view_r: HashMap::<Id, TrackedMapPointData>, // mbTrackInViewR, member variable in Mappoint
    kf_track_reference_for_frame: HashMap::<Id, Id>, // mnTrackReferenceForFrame, member variable in Keyframe
    mp_track_reference_for_frame: HashMap::<Id, Id>,  // mnTrackReferenceForFrame, member variable in Mappoint
    last_frame_seen: HashMap::<Id, Id>, // mnLastFrameSeen, member variable in Mappoint
    // IMU 
    imu: ImuModule,
    // Relocalization
    relocalization: Relocalization,
    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

    /// References to map
    map_initialized: bool,
    map_updated : bool,  // TODO (mvp) I'm not sure we want to use this
    map: MapLock,
    initialization: Option<Initialization>, // data sent to map actor to initialize new map

    /// Global defaults
    localization_only_mode: bool,
    frames_to_reset_imu: u32, //mnFramesToResetIMU
    insert_kfs_when_lost: bool,
    max_frames : i32 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
    min_frames: u32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
}

impl Actor for TrackingFull {
    type MapRef = MapLock;

    fn new_actorstate(actor_channels: ActorChannels, map: Self::MapRef) -> TrackingFull {
        let max_features = SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features");
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let orb_extractor_right = match sensor.frame() {
            FrameSensor::Stereo => Some(DVORBextractor::new(max_features)),
            FrameSensor::Mono | FrameSensor::Rgbd => None,
        };
        let orb_extractor_ini = match sensor.is_mono() {
            true => Some(DVORBextractor::new(max_features*5)),
            false => None
        };
        TrackingFull {
            actor_channels,
            orb_extractor_left: DVORBextractor::new(max_features),
            orb_extractor_right,
            orb_extractor_ini,
            map_initialized: false,
            init_id: 0,
            // TODO(MVP) after we are done testing, this should be set to:  SETTINGS.get::<f64>(SYSTEM, "fps") as i32,
            // orbslam sets this to 10 regardless of what the actual fps is
            max_frames: 10,
            sensor,
            map,
            initialization: Some(Initialization::new()),
            localization_only_mode: SETTINGS.get::<bool>(SYSTEM, "localization_only_mode"),
            frames_to_reset_imu: SETTINGS.get::<i32>(TRACKING_BACKEND, "frames_to_reset_IMU") as u32,
            insert_kfs_when_lost: SETTINGS.get::<bool>(TRACKING_BACKEND, "insert_KFs_when_lost"),
            min_frames: 0,
            state: TrackingState::NotInitialized,
            ref_kf_id: None,
            frames_since_last_kf: 0,
            last_kf_timestamp: None,
            matches_inliers: 0,
            local_keyframes: vec![],
            local_mappoints: HashSet::new(),
            track_in_view: HashMap::new(),
            track_in_view_r: HashMap::new(),
            kf_track_reference_for_frame: HashMap::new(),
            mp_track_reference_for_frame: HashMap::new(),
            last_frame_seen: HashMap::new(),
            imu: ImuModule::new(None, None, sensor, false, false),
            relocalization: Relocalization{last_reloc_frame_id: 0, timestamp_lost: None},
            map_updated: false,
            trajectory_poses: Vec::new(),

        }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = TrackingFull::new_actorstate(actor_channels, map);
        let max_queue_size = actor.actor_channels.receiver_bound.unwrap_or(100);
        let mut last_frame = None; // Keep last_frame here instead of in tracking object to avoid putting it in an option and doing a ton of unwraps
        let mut curr_frame_id = 0;

        tracy_client::set_thread_name!("tracking full");

        'outer: loop {
            let message = actor.actor_channels.receive().unwrap();

            if message.is::<ImageMsg>() {
                if actor.actor_channels.queue_len() > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Tracking dropped 1 frame");
                    continue;
                }

                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));

                // Extract features
                let image_cols = msg.image.cols() as u32;
                let image_rows = msg.image.rows() as u32;
                let (keypoints, descriptors) = match actor.actor_channels.actors.get(VISUALIZER).is_some() {
                    true => {
                        // TODO (timing) ... cloned if visualizer running. maybe make global shared object?
                        let (keypoints, descriptors) = actor.extract_features(msg.image.clone(), curr_frame_id);
                        actor.send_to_visualizer(keypoints.clone(), msg.image, msg.timestamp);
                        (keypoints, descriptors)
                    },
                    false => {
                        let (keypoints, descriptors) = actor.extract_features(msg.image, curr_frame_id);
                        (keypoints, descriptors)
                    }
                };

                debug!("Tracking working on frame {}", msg.frame_id);
                let mut current_frame = Frame::new(
                    curr_frame_id, 
                    DVVectorOfKeyPoint::new(keypoints),
                    DVMatrix::new(descriptors),
                    image_cols,
                    image_rows,
                    msg.timestamp
                ).expect("Could not create frame!");
                match actor.track(&mut current_frame, &mut last_frame) {
                    Ok(()) => {
                        if current_frame.ref_kf_id.is_none() {
                            current_frame.ref_kf_id = actor.ref_kf_id;
                        }
                        last_frame = Some(current_frame);
                        curr_frame_id += 1;

                        match actor.state {
                            TrackingState::Ok | TrackingState::RecentlyLost => {
                                actor.update_trajectory_in_logs(last_frame.as_mut().unwrap()).expect("Could not save trajectory")
                            },
                            _ => {},
                        };
                    },
                    Err(e) => {
                        panic!("Error in Tracking Backend: {}", e);
                    }
                };
            } else if message.is::<InitKeyFrameMsg>() {
                // Received from the map actor after it inserts a keyframe
                let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));
                last_frame.as_mut().unwrap().ref_kf_id = Some(msg.kf_id);
            } else if message.is::<LastKeyFrameUpdatedMsg>() {
                // Received from local mapping after it culls and creates new MPs for the last inserted KF
                actor.state = TrackingState::Ok;
            } else if message.is::<IMUInitializedMsg>() {
                // TODO (IMU) process message from local mapping!
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Tracking frontend received unknown message type!");
            }
        }
    }
}

impl TrackingFull {
    fn extract_features(&mut self, image: opencv::core::Mat, curr_frame_id: i32) -> (VectorOfKeyPoint, Mat) {
        let _span = tracy_client::span!("extract features");

        let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image)).into();
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();

        // TODO (C++ and Rust optimizations) ... this takes ~70 ms which is way high compared to ORB-SLAM3. I think this is because the rust and C++ bindings are not getting optimized together.
        if self.map_initialized && (curr_frame_id - self.init_id < self.max_frames) {
            self.orb_extractor_left.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
        } else if self.sensor.is_mono() {
            self.orb_extractor_ini.as_mut().unwrap().extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
        } else {
            self.orb_extractor_left.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
        }
        match self.sensor.frame() {
            FrameSensor::Stereo => todo!("Stereo"), //Also call extractor_right, see Tracking::GrabImageStereo,
            _ => {}
        }
        (keypoints.kp_ptr.kp_ptr, descriptors.mat_ptr.mat_ptr)
    }

    fn track(&mut self, current_frame: &mut Frame, last_frame: &mut Option<Frame>) -> Result<(), Box<dyn std::error::Error>>  {
        let _span = tracy_client::span!("track");

        // TODO (reset): Reset map because local mapper set the bad imu flag
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1808

        // TODO (multimaps): Create new map if timestamp older than previous frame arrives
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1820

        // TODO (reset) TODO (multimaps): Timestamp jump detected, either reset active map or create new map
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1828
        // (entire block)

        if self.sensor.is_imu() {
            self.imu.preintegrate();
            todo!("IMU");
            // set bias of new frame = to bias of last
            // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1860
        }

        // Initial estimation of camera pose and matching
        match (self.localization_only_mode, self.state) {
            (true, _) => {
                todo!("Localization only");
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1933
                // Look for "mbOnlyTracking" in Track() function
            },
            (false, TrackingState::NotInitialized) => {
                if self.initialization.is_none() {
                    self.initialization = Some(Initialization::new());
                }
                let init_success = self.initialization.as_mut().unwrap().try_initialize(&current_frame)?;
                if init_success {
                    // Give up ownership of self.initialization to avoid clone, we don't need it from now on
                    let mut init = None;
                    std::mem::swap(&mut init, &mut self.initialization);

                    let (ini_kf_id, curr_kf_id);
                    {
                        // TODO (stereo) - Add option to make the map monocular or stereo
                        let mut write_lock = self.map.write();
                        let init_unwrap = init.unwrap();
                        (ini_kf_id, curr_kf_id) = match write_lock.create_initial_map_monocular(init_unwrap.mp_matches, init_unwrap.p3d, init_unwrap.initial_frame.unwrap(), init_unwrap.current_frame.unwrap()) {
                                Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, curr_kf_timestamp)) => {
                                    // Map needs to be initialized before tracking can begin. Received from map actor
                                    self.frames_since_last_kf = 0;
                                    self.local_keyframes.push(curr_kf_id);
                                    self.local_keyframes.push(ini_kf_id);
                                    self.local_mappoints = local_mappoints;
                                    self.ref_kf_id = Some(curr_kf_id);
                                    self.last_kf_timestamp = Some(curr_kf_timestamp);
                                    {
                                        // Set current frame's updated info from map initialization
                                        current_frame.ref_kf_id = Some(curr_kf_id);
                                        current_frame.pose = Some(curr_kf_pose);
                                    }
                                    self.state = TrackingState::Ok;
                                    (ini_kf_id, curr_kf_id)
                                },
                                None => {
                                    panic!("Could not create initial map");
                                }
                            };
                    }

                    self.map_initialized = true;
                    self.init_id = ini_kf_id;

                    // Send first two keyframes to local mapping
                    self.actor_channels.send(LOCAL_MAPPING, Box::new(
                        InitKeyFrameMsg { kf_id: ini_kf_id }
                    ));
                    self.actor_channels.send(LOCAL_MAPPING,Box::new(
                        InitKeyFrameMsg { kf_id: curr_kf_id }
                    ));


                    self.state = TrackingState::Ok;
                    return Ok(());
                } else {
                    self.state = TrackingState::NotInitialized;
                    return Ok(());
                }
            },
            (false, TrackingState::Ok) => {
                let no_motion_model = (self.imu.velocity.is_none() && !self.imu.is_initialized) || self.relocalization.frames_since_lost(&current_frame) < 2;
                // SOFIYA...CheckReplacedInLastFrame. this might be important?

                let track_success = match no_motion_model {
                    true => self.track_reference_keyframe(current_frame, last_frame.as_mut().unwrap())?,
                    false => {
                        match self.track_with_motion_model(current_frame, last_frame.as_mut().unwrap())? {
                            true => true,
                            false => self.track_reference_keyframe(current_frame, last_frame.as_mut().unwrap())?
                        }
                    }
                };
                self.state = match track_success {
                    true => TrackingState::Ok,
                    false => {
                        self.relocalization.timestamp_lost = Some(current_frame.timestamp);
                        match self.map.read().keyframes.len() > 10 {
                            true =>{warn!("State recently lost 271"); TrackingState::RecentlyLost},
                            false => TrackingState::Lost
                        }
                    },
                }
            },
            (false, TrackingState::RecentlyLost) => {
                warn!("State recently lost");
                let relocalize_success = match self.imu.ready() {
                    true => {
                        let _ok = self.imu.predict_state(); // TODO (IMU): I guess this should be used somewhere?
                        self.relocalization.past_cutoff(&current_frame)
                    },
                    false => {
                        // TODO (relocalization): remove the call to shutdown actor and uncomment line of code below.
                        // This is just to gracefully shut down instead of panicking at the to do 
                        error!("Relocalization! Shutting down for now.");
                        self.actor_channels.send(SHUTDOWN_ACTOR, Box::new(ShutdownMsg{}));
                        self.state = TrackingState::Lost;
                        return Ok(());
                        // !self.relocalization.run() && self.relocalization.sec_since_lost(&current_frame) > 3
                    }
                };
                self.state = match relocalize_success {
                    true => {
                        warn!("Relocalization unsuccessful...");
                        TrackingState::Lost
                    },
                    false => {warn!("State recently lost"); TrackingState::RecentlyLost}
                }
            },
            (false, TrackingState::Lost) => {
                match self.map.read().keyframes.len() < 10 {
                    true => {
                        warn!("Reseting current map...");
                        error!("Resetting current map! Shutting down for now.");
                        self.actor_channels.send(SHUTDOWN_ACTOR, Box::new(ShutdownMsg{}));
                        self.state = TrackingState::Lost;
                        return Ok(());

                        // TODO (RESET)
                    },
                    false => {
                        info!("Creating new map...");
                        self.create_new_map();
                    }
                }

                self.state = TrackingState::Ok;
                return Ok(());
            }
        };


        // Only 3 valid states now
        // TrackingState::Ok | TrackingState::RecentlyLost | TrackingState::Lost

        // Track Local Map
        let (enough_matches, _matches_in_frame) = self.track_local_map(current_frame, last_frame.as_mut().unwrap());
        if enough_matches {
            self.state = TrackingState::Ok;
        } else if matches!(self.state, TrackingState::Ok) {
            if self.sensor.is_imu() {
                warn!("Track lost for less than 1 second,");
                warn!("IMU, Reset... Reset map because local mapper set the bad imu flag");
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2149
            }
            self.relocalization.timestamp_lost = Some(current_frame.timestamp);
            self.state = TrackingState::RecentlyLost;
        }

        if self.sensor.is_imu() {
            todo!("IMU");
            // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
        }

        if enough_matches || matches!(self.state, TrackingState::RecentlyLost) {
            // Update motion model
            let last_frame = last_frame.as_ref();
            if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !current_frame.pose.is_none() {
                let last_pose = last_frame.expect("No last frame in tracking?").pose.as_ref().expect("Can't get last frame's pose?");
                let last_twc = last_pose.inverse();
                self.imu.velocity = Some(*current_frame.pose.as_ref().expect("Can't get current frame?") * last_twc);
            } else {
                self.imu.velocity = None;
            }

            // Clean VO matches
            current_frame.delete_mappoints_without_observations(&self.map.read());

            // Check if we need to insert a new keyframe
            let insert_if_lost_anyway = self.insert_kfs_when_lost && matches!(self.state, TrackingState::RecentlyLost) && self.sensor.is_imu();
            let need_new_kf = self.need_new_keyframe(current_frame);
            if need_new_kf && (matches!(self.state, TrackingState::Ok) || insert_if_lost_anyway) {
                self.create_new_keyframe(current_frame);
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            let _ = current_frame.delete_mappoint_outliers();
        }

        // Reset if the camera get lost soon after initialization
        if matches!(self.state, TrackingState::Lost) {
            if self.map.read().keyframes.len() <= 10  || (self.sensor.is_imu() && !self.imu.is_initialized) {
                warn!("tracking_backend::handle_message;Track lost soon after initialization, resetting...",);
                // TODO (RESET)
            } else {
                self.create_new_map();
            }
        }

        Ok(())
    }

    fn update_trajectory_in_logs(
        &mut self, current_frame: &Frame,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let relative_pose;
        {
            let map = self.map.read();
            relative_pose = current_frame.get_pose_relative(&map);
            self.trajectory_poses.push(relative_pose);
        }
        self.actor_channels.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: relative_pose,
                ref_kf_id: current_frame.ref_kf_id.unwrap(),
                timestamp: current_frame.timestamp
            })
        );

        if self.actor_channels.actors.get(VISUALIZER).is_some() {
            // TODO (timing) ... cloned if visualizer running. maybe make global shared object?
            self.actor_channels.send(VISUALIZER, Box::new(VisTrajectoryMsg{
                pose: current_frame.pose.unwrap(),
                mappoint_matches: current_frame.mappoint_matches.matches.clone(),
                timestamp: current_frame.timestamp
            }));
        }

        Ok(())
    }

    fn track_reference_keyframe(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track_reference_keyframe");
        // Tracking::TrackReferenceKeyFrame()
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver

        current_frame.compute_bow();
        let nmatches;
        {
            let map_read_lock = self.map.read();
            let ref_kf = map_read_lock.keyframes.get(&self.ref_kf_id.unwrap()).unwrap();

            nmatches = orbmatcher::search_by_bow_f(ref_kf, current_frame, true, 0.7)?;
        }

        // debug!("track reference keyframe matches {}", current_frame.mappoint_matches.matches.len());

        if nmatches < 15 {
            warn!("track_reference_keyframe has fewer than 15 matches = {}!!\n", nmatches);
            return Ok(false);
        }

        current_frame.pose = Some(last_frame.pose.unwrap());

        optimizer::optimize_pose(current_frame, &self.map);

        // Discard outliers
        let nmatches_map = self.delete_mappoint_outliers(current_frame);
        // debug!("track reference keyframe matches after outliers {}", nmatches_map);

        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn track_with_motion_model(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track_with_motion_model");
        // Tracking::TrackWithMotionModel()
        // If tracking was successful for last frame, we use a constant
        // velocity motion model to predict the camera pose and perform
        // a guided search of the map points observed in the last frame. If
        // not enough matches were found (i.e. motion model is clearly
        // violated), we use a wider search of the map points around
        // their position in the last frame. The pose is then optimized
        // with the found correspondences.

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        self.update_last_frame(last_frame);

        let enough_frames_to_reset_imu = current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32);
        if self.imu.is_initialized && enough_frames_to_reset_imu {
            // Predict state with IMU if it is initialized and it doesnt need reset
            todo!("IMU");
            self.imu.predict_state();
        } else {
            current_frame.pose = Some(self.imu.velocity.unwrap() * last_frame.pose.unwrap());
        }

        current_frame.mappoint_matches.clear();

        // Project points seen in previous frame
        let th = match self.sensor.frame() {
            FrameSensor::Mono => 15,
            _ => 7
        };

        let mut matches = orbmatcher::search_by_projection_with_threshold(
            current_frame,
            last_frame,
            th,
            self.sensor.is_mono(),
            &self.map,
            self.sensor
        )?;

        // debug!("motion model matches {}", current_frame.mappoint_matches.matches.len());

        // If few matches, uses a wider window search
        if matches < 20 {
            info!("tracking_backend::track_with_motion_model;not enough matches, wider window search");
            current_frame.mappoint_matches.clear();
            matches = orbmatcher::search_by_projection_with_threshold(
                current_frame,
                last_frame,
                2 * th,
                self.sensor.is_mono(),
                &self.map,
                self.sensor
            )?;
        }

        // debug!("motion model matches with wider search {}", current_frame.mappoint_matches.matches.len());

        if matches < 20 {
            warn!("tracking_backend::track_with_motion_model;not enough matches!!");
            return Ok(self.sensor.is_imu());
        }

        // Optimize frame pose with all matches
        optimizer::optimize_pose(current_frame, &self.map);

        // Discard outliers
        let nmatches_map = self.delete_mappoint_outliers(current_frame);

        if self.localization_only_mode {
            todo!("Localization only");
            // mbVO = nmatchesMap<10;
            // return nmatches>20;
        }

        // debug!("motion model matches after outliers {}", nmatches_map);
        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };

    }

    fn update_last_frame(&mut self, last_frame: &mut Frame) {
        // Tracking::UpdateLastFrame()

        // Update pose according to reference keyframe
        let ref_kf_id = last_frame.ref_kf_id.unwrap();
        let map_lock = self.map.read();
        let ref_kf_pose = map_lock.keyframes.get(&ref_kf_id).expect("Reference kf should be in map").pose;
        last_frame.pose = Some(*self.trajectory_poses.last().unwrap() * ref_kf_pose);

        if self.sensor.is_mono() || self.frames_since_last_kf == 0 {
            return;
        }

        match self.sensor.is_mono() {
            true => return,
            false => {
                todo!("Stereo, RGBD");
                // Create "visual odometry" MapPoints
                // We sort points according to their measured depth by the stereo/RGB-D sensor
                // vector<pair<float,int> > vDepthIdx;
                // const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;
                // vDepthIdx.reserve(Nfeat);
                // for(int i=0; i<Nfeat;i++)
                // {
                //     float z = mLastFrame.mvDepth[i];
                //     if(z>0)
                //     {
                //         vDepthIdx.push_back(make_pair(z,i));
                //     }
                // }

                // if(vDepthIdx.empty())
                //     return;

                // sort(vDepthIdx.begin(),vDepthIdx.end());

                // // We insert all close points (depth<mThDepth)
                // // If less than 100 close points, we insert the 100 closest ones.
                // int nPoints = 0;
                // for(size_t j=0; j<vDepthIdx.size();j++)
                // {
                //     int i = vDepthIdx[j].second;

                //     bool bCreateNew = false;

                //     MapPoint* pMP = mLastFrame.mvpMapPoints[i];

                //     if(!pMP)
                //         bCreateNew = true;
                //     else if(pMP->Observations()<1)
                //         bCreateNew = true;

                //     if(bCreateNew)
                //     {
                //         Eigen::Vector3f x3D;

                //         if(mLastFrame.Nleft == -1){
                //             mLastFrame.UnprojectStereo(i, x3D);
                //         }
                //         else{
                //             x3D = mLastFrame.UnprojectStereoFishEye(i);
                //         }

                //         MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
                //         mLastFrame.mvpMapPoints[i]=pNewMP;

                //         mlpTemporalPoints.push_back(pNewMP);
                //         nPoints++;
                //     }
                //     else
                //     {
                //         nPoints++;
                //     }

                //     if(vDepthIdx[j].first>mThDepth && nPoints>100)
                //         break;

                // }
            }
        }
    }

    fn track_local_map(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> (bool, i32) {
        let _span = tracy_client::span!("track_local_map");
        // bool Tracking::TrackLocalMap()
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        self.update_local_keyframes(current_frame, last_frame);
        self.update_local_points(current_frame);
        self.search_local_points(current_frame);

        if !self.imu.is_initialized || (current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32)) {
            optimizer::optimize_pose(current_frame, &self.map);
        } else if !self.map_updated {
            optimizer::pose_inertial_optimization_last_frame(current_frame, &self.map);
        } else {
            optimizer::pose_inertial_optimization_last_keyframe(current_frame);
        }

        self.matches_inliers = 0;
        // Update MapPoints Statistics
        for index in 0..current_frame.mappoint_matches.matches.len() {
            if let Some((mp_id, is_outlier)) = current_frame.mappoint_matches.matches[index as usize] {
                if !is_outlier {
                    if let Some(mp) = self.map.read().mappoints.get(&mp_id) {
                        mp.increase_found(1);
                        // println!("Increase found {}", mp_id);
                    }

                    if self.localization_only_mode {
                        let map_read_lock = self.map.read();
                        if let Some(mp) = map_read_lock.mappoints.get(&mp_id) {
                            if mp.get_observations().len() > 0 {
                                self.matches_inliers+=1;
                            }
                        } else {
                            current_frame.mappoint_matches.matches[index] = None;
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

        // Sofiya matches
        //     mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
        // send this in local mapper message?

        // debug!("Matches in track local map {}", self.matches_inliers);

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32) && self.matches_inliers<50 {
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

    fn update_local_keyframes(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> Option<()> {
        let _span = tracy_client::span!("update_local_keyframes");
        // void Tracking::UpdateLocalKeyFrames()
        // Each map point votes for the keyframes in which it has been observed
        let mut kf_counter = HashMap::<Id, i32>::new();

        {
            let lock = self.map.read();
            for i in 0..current_frame.mappoint_matches.matches.len() {
                match !self.imu.is_initialized || current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
                    true => {
                        if current_frame.mappoint_matches.has(&(i as u32)) {
                            let mp_id = current_frame.mappoint_matches.get(&(i as u32));
                            if let Some(mp) = lock.mappoints.get(&mp_id) {
                                for kf_id in mp.get_observations().keys() {
                                    *kf_counter.entry(*kf_id).or_insert(0) += 1;
                                }
                            } else {
                                current_frame.mappoint_matches.matches[i as usize] = None;
                            }
                        }
                    },
                    false => {
                        // Using lastframe since current frame has no matches yet
                        if last_frame.mappoint_matches.has(&(i as u32)) {
                            let mp_id = last_frame.mappoint_matches.get(&(i as u32));
                            if let Some(mp) = lock.mappoints.get(&mp_id) {
                                for kf_id in mp.get_observations().keys() {
                                    *kf_counter.entry(*kf_id).or_insert(0) += 1;
                                }
                            } else {
                                last_frame.mappoint_matches.matches[i as usize] = None;
                            }
                        }
                    }
                }
            }
        }

        let (mut max, mut max_kf_id) = (0, 0);
        self.local_keyframes.clear();

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (kf_id, count) in kf_counter {
            if count > max {
                max = count;
                max_kf_id = kf_id;
            }
            self.local_keyframes.push(kf_id);
            self.kf_track_reference_for_frame.insert(kf_id, current_frame.frame_id);
        }

        // Also include some keyframes that are neighbors to already-included keyframes
        let mut i = 0;
        {
            let lock = self.map.read();
            while self.local_keyframes.len() <= 80 && i < self.local_keyframes.len() { // Limit the number of keyframes
                let next_kf_id = self.local_keyframes[i];

                for kf_id in &lock.keyframes.get(&next_kf_id)?.get_covisibility_keyframes(10) {
                    if self.kf_track_reference_for_frame.get(kf_id) != Some(&current_frame.frame_id) {
                        self.local_keyframes.push(*kf_id);
                        self.kf_track_reference_for_frame.insert(*kf_id, current_frame.frame_id);
                        break;
                    }
                }
                for kf_id in &lock.keyframes.get(&next_kf_id)?.children {
                    if self.kf_track_reference_for_frame.get(kf_id) != Some(&current_frame.frame_id) {
                        self.local_keyframes.push(*kf_id);
                        self.kf_track_reference_for_frame.insert(*kf_id, current_frame.frame_id);
                        break;
                    }
                }
        
                if let Some(parent_id) = lock.keyframes.get(&next_kf_id)?.parent {
                    if self.kf_track_reference_for_frame.get(&parent_id) != Some(&current_frame.frame_id) {
                        self.local_keyframes.push(parent_id);
                        self.kf_track_reference_for_frame.insert(parent_id, current_frame.frame_id);
                    }
                };
                i += 1;
            }
        }

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

    fn update_local_points(&mut self, current_frame: &mut Frame) -> Option<()> {
        let _span = tracy_client::span!("update_local_points");
        // void Tracking::UpdateLocalPoints()
        self.local_mappoints.clear();
        let lock = self.map.read();
        for kf_id in &self.local_keyframes {
            let mp_ids = lock.keyframes.get(kf_id)?.get_mp_matches();
            for item in mp_ids {
                if let Some((mp_id, _)) = item {
                    if self.mp_track_reference_for_frame.get(mp_id) != Some(&current_frame.frame_id) {
                        self.local_mappoints.insert(*mp_id);
                        self.mp_track_reference_for_frame.insert(*mp_id, current_frame.frame_id);
                    }
                }
            }
        }

        None
    }

    fn search_local_points(&mut self, current_frame: &mut Frame) {
        let _span = tracy_client::span!("search_local_points");
        //void Tracking::SearchLocalPoints()

        // Do not search map points already matched
        {
            let lock = self.map.read();
            for index in 0..current_frame.mappoint_matches.matches.len() {
                if let Some((id, _)) = current_frame.mappoint_matches.matches[index as usize] {
                    if let Some(mp) = lock.mappoints.get(&id) {
                        mp.increase_visible(1);
                        // println!("Increase visible {}", id);

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
                let (tracked_data_left, tracked_data_right) = match lock.mappoints.get(mp_id) {
                    Some(mp) => {
                        if self.last_frame_seen.get(mp_id) == Some(&current_frame.frame_id) {
                            continue;
                        }
                        // Project (this fills MapPoint variables for matching)
                        current_frame.is_in_frustum(mp, 0.5)
                    },
                    None => {
                        continue;
                    }
                };

                if tracked_data_left.is_some() || tracked_data_right.is_some() {
                    lock.mappoints.get(&mp_id).unwrap().increase_visible(1);
                    // println!("Increase visible {}", mp_id);
                    to_match += 1;
                }
                if let Some(d) = tracked_data_left {
                    self.track_in_view.insert(*mp_id, d);
                }
                if let Some(d) = tracked_data_right {
                    self.track_in_view_r.insert(*mp_id, d);
                }
            }
        }

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

            let _matches = orbmatcher::search_by_projection(
                current_frame,
                &mut self.local_mappoints,
                th, 0.8,
                &self.track_in_view, &self.track_in_view_r,
                &self.map, self.sensor
            );
            // debug!("search local points matches {:?}", matches);
        }

    }

    fn create_new_keyframe(&mut self, current_frame: &mut Frame) {
        let _span = tracy_client::span!("create_new_keyframe");
        //CreateNewKeyFrame
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216
        let new_kf = Frame::new_clone(&current_frame);

        if self.sensor.is_imu() {
            todo!("IMU"); //Reset preintegration from last KF (Create new object)
        //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
        }

        if !self.sensor.is_mono() {
            todo!("Stereo");
        //     mCurrentFrame.UpdatePoseMatrices();
        //     // cout << "create new MPs" << endl;
        //     // We sort points by the measured depth by the stereo/RGBD sensor.
        //     // We create all those MapPoints whose depth < mThDepth.
        //     // If there are less than 100 close points we create the 100 closest.
        //     int maxPoint = 100;
        //     if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        //         maxPoint = 100;

        //     vector<pair<float,int> > vDepthIdx;
        //     int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        //     vDepthIdx.reserve(mCurrentFrame.N);
        //     for(int i=0; i<N; i++)
        //     {
        //         float z = mCurrentFrame.mvDepth[i];
        //         if(z>0)
        //         {
        //             vDepthIdx.push_back(make_pair(z,i));
        //         }
        //     }

        //     if(!vDepthIdx.empty())
        //     {
        //         sort(vDepthIdx.begin(),vDepthIdx.end());

        //         int nPoints = 0;
        //         for(size_t j=0; j<vDepthIdx.size();j++)
        //         {
        //             int i = vDepthIdx[j].second;

        //             bool bCreateNew = false;

        //             MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        //             if(!pMP)
        //                 bCreateNew = true;
        //             else if(pMP->Observations()<1)
        //             {
        //                 bCreateNew = true;
        //                 mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        //             }

        //             if(bCreateNew)
        //             {
        //                 Eigen::Vector3f x3D;

        //                 if(mCurrentFrame.Nleft == -1){
        //                     mCurrentFrame.UnprojectStereo(i, x3D);
        //                 }
        //                 else{
        //                     x3D = mCurrentFrame.UnprojectStereoFishEye(i);
        //                 }

        //                 MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
        //                 pNewMP->AddObservation(pKF,i);

        //                 //Check if it is a stereo observation in order to not
        //                 //duplicate mappoints
        //                 if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
        //                     mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
        //                     pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
        //                     pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
        //                 }

        //                 pKF->AddMapPoint(pNewMP,i);
        //                 pNewMP->ComputeDistinctiveDescriptors();
        //                 pNewMP->UpdateNormalAndDepth();
        //                 mpAtlas->AddMapPoint(pNewMP);

        //                 mCurrentFrame.mvpMapPoints[i]=pNewMP;
        //                 nPoints++;
        //             }
        //             else
        //             {
        //                 nPoints++;
        //             }

        //             if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
        //             {
        //                 break;
        //             }
        //         }
        //     }
        }

        // mnLastKeyFrameId = mCurrentFrame.mnId;
        // mpLastKeyFrame = pKF;

        self.last_kf_timestamp = Some(new_kf.timestamp);

        // KeyFrame created here and inserted into map
        self.actor_channels.send(
            LOCAL_MAPPING,
            Box::new( NewKeyFrameMsg{  keyframe: new_kf, } )
        );
    }

    fn need_new_keyframe(&self, current_frame: &mut Frame) -> bool {
        let num_kfs = self.map.read().keyframes.len();
        let not_enough_frames_since_last_reloc = (current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32)) && (num_kfs as i32 > self.max_frames);
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

        // Tracked MapPoints in the reference keyframe
        let min_observations = match num_kfs <= 2 {
            true => 2,
            false => 3
        };

        let map_lock = self.map.read();
        let tracked_mappoints = map_lock.keyframes.get(&self.ref_kf_id.unwrap())
            .map(|kf| kf.get_tracked_mappoints(&*map_lock, min_observations) as f32)
            .unwrap_or(0.0);

        // Check how many "close" points are being tracked and how many could be potentially created.
        let (tracked_close, non_tracked_close) = current_frame.check_close_tracked_mappoints();
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
        let c1a = self.frames_since_last_kf >= (self.max_frames as i32);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        let c1b = self.frames_since_last_kf >= (self.min_frames as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
        //Condition 1c: tracking is weak
        let sensor_is_right = match self.sensor {
            Sensor(FrameSensor::Mono, _) | Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => false,
            _ => true
        }; // I do not know why they just select for RGBD or Stereo without IMU
        let c1c = sensor_is_right && ((self.matches_inliers as f32) < tracked_mappoints * 0.25 || need_to_insert_close) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        let c2 = (((self.matches_inliers as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_inliers > 15;

        // Temporal condition for Inertial cases
        let close_to_last_kf = match self.last_kf_timestamp {
            Some(timestamp) => current_frame.timestamp - timestamp < 0.5, // 500 milliseconds
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
        return ((c1a||c1b||c1c) && c2)||c3 ||c4;
    }

    //* Helper functions */
    fn delete_mappoint_outliers(&mut self, current_frame: &mut Frame) -> i32 {
        let discarded = current_frame.delete_mappoint_outliers();
        for mp_id in discarded {
            // self.map.read().mappoints.get(&mp_id).unwrap().increase_found();
            self.track_in_view.remove(&mp_id);
            self.last_frame_seen.insert(mp_id, current_frame.frame_id);
            // TODO (Stereo) ... need to remove this from track_in_view_r if the mp is seen in the right camera
        }
        current_frame.mappoint_matches.tracked_mappoints(&*self.map.read(), 1)
    }

    //* Next steps */
    fn create_new_map(&self) -> bool {
        todo!("Multimaps: Atlas::CreateNewMap");
    }

    fn send_to_visualizer(&mut self, keypoints: VectorOfKeyPoint, image: Mat, timestamp: Timestamp) {
        // Send image and features to visualizer
        self.actor_channels.find(VISUALIZER).send(Box::new(VisFeaturesMsg {
            keypoints: DVVectorOfKeyPoint::new(keypoints),
            image,
            timestamp,
        })).unwrap();
    }
}


pub struct DVORBextractor {
    pub extractor: UniquePtr<dvos3binding::ffi::ORBextractor>,
    pub max_features: i32
}
impl DVORBextractor {
    pub fn new(max_features: i32) -> Self {
        DVORBextractor{
            max_features,
            extractor: dvos3binding::ffi::new_orb_extractor(
                max_features,
                SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "min_th_fast"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_begin"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_end")
            )
        }
    }
}
impl Clone for DVORBextractor {
    fn clone(&self) -> Self {
        DVORBextractor::new(self.max_features)
    }
}
impl Debug for DVORBextractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVORBextractor")
         .finish()
    }
}