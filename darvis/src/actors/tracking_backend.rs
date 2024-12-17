use std::{collections::{BTreeMap, BTreeSet, HashMap}, sync::atomic::Ordering, thread::sleep, time::Duration};
use log::{warn, info, debug, error};
use core::{config::*, matrix::DVMatrix, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, Timestamp}};
use crate::{
    actors::{local_mapping::LOCAL_MAPPING_IDLE, loop_closing::KeyFrameAndPose, messages::{FeatureMsg, InitKeyFrameMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, TrackingStateMsg, TrajectoryMsg, UpdateFrameIMUMsg, VisTrajectoryMsg}}, map::{
        frame::Frame, map::Id, pose::Pose
    }, modules::{imu::{normalize_rotation, ImuBias, ImuMeasurements, ImuPreIntegrated, GRAVITY_VALUE, IMU}, map_initialization::MapInitialization, module_definitions::FeatureMatchingModule, optimizer, orbslam_matcher, relocalization::Relocalization}, registered_actors::{self, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, LOOP_CLOSING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}, MapLock, System, MAP_INITIALIZED
};
use crate::modules::module_definitions::ImuModule;
use super::{local_mapping::LOCAL_MAPPING_ABORT, messages::{NewKeyFrameMsg, Reset}};
use crate::modules::module_definitions::MapInitializationModule;
use crate::modules::module_definitions::RelocalizationModule;

pub struct TrackingBackend {
    system: System,
    state: TrackingState,

    // Frames
    last_frame: Option<Frame>,
    current_frame: Frame,

    // Map
    map: MapLock,
    initialization: Option<MapInitialization>, // data sent to map actor to initialize new map
    last_map_change_index: i32,
    map_updated: bool,

    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_timestamp: Option<Timestamp>,

    // Local map data used for different stages of tracking
    matches_inliers : i32, // mnMatchesInliers ... Current matches in frame
    local_keyframes: BTreeSet<Id>, //mvpLocalKeyFrames 
    local_mappoints: BTreeSet<Id>, //mvpLocalMapPoints
    track_in_view: HashMap::<Id, TrackedMapPointData>, // mbTrackInView , member variable in Mappoint
    track_in_view_r: HashMap::<Id, TrackedMapPointData>, // mbTrackInViewR, member variable in Mappoint
    kf_track_reference_for_frame: HashMap::<Id, Id>, // mnTrackReferenceForFrame, member variable in Keyframe
    mp_track_reference_for_frame: HashMap::<Id, Id>,  // mnTrackReferenceForFrame, member variable in Mappoint
    last_frame_seen: HashMap::<Id, Id>, // mnLastFrameSeen, member variable in Mappoint
    non_tracked_mappoints: HashMap<Id, i32>, // just for testing

    // Modules 
    imu: IMU,
    relocalization: Relocalization,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
    reference_kfs_for_trajectory: Vec<(Id, Option<Id>)>, //mlpReferences ... First is Id of reference kf, second is that kf's parent ID (in case kf is deleted but we need to know its parent later)

    // Global defaults
    localization_only_mode: bool,
    frames_to_reset_imu: u32, //mnFramesToResetIMU
    insert_kfs_when_lost: bool,
    max_frames_to_insert_kf : i32 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
    min_frames_to_insert_kf: i32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
    sensor: Sensor,
}

impl Actor for TrackingBackend {
    type MapRef = MapLock;

    fn new_actorstate(system: System, map: Self::MapRef) -> TrackingBackend {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        TrackingBackend {
            system,
            map,
            sensor,
            initialization: Some(MapInitialization::new()),
            localization_only_mode: SETTINGS.get::<bool>(SYSTEM, "localization_only_mode"),
            frames_to_reset_imu: SETTINGS.get::<i32>(TRACKING_BACKEND, "frames_to_reset_IMU") as u32,
            insert_kfs_when_lost: SETTINGS.get::<bool>(TRACKING_BACKEND, "insert_KFs_when_lost"),
            max_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "max_frames_to_insert_kf"),
            min_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_frames_to_insert_kf"),
            state: TrackingState::NotInitialized,
            ref_kf_id: None,
            frames_since_last_kf: 0,
            last_kf_timestamp: None,
            matches_inliers: 0,
            local_keyframes: BTreeSet::new(),
            local_mappoints: BTreeSet::new(),
            track_in_view: HashMap::new(),
            track_in_view_r: HashMap::new(),
            kf_track_reference_for_frame: HashMap::new(),
            mp_track_reference_for_frame: HashMap::new(),
            last_frame_seen: HashMap::new(),
            imu: IMU::new(),
            relocalization: Relocalization{last_reloc_frame_id: 0, timestamp_lost: None},
            map_updated: false,
            last_map_change_index: 0,
            trajectory_poses: Vec::new(),
            reference_kfs_for_trajectory: Vec::new(),
            non_tracked_mappoints: HashMap::new(),
            last_frame: None,
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
        }
    }

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = TrackingBackend::new_actorstate(system, map);
        let max_queue_size = actor.system.receiver_bound.unwrap_or(100);

        tracy_client::set_thread_name!("tracking backend");

        'outer: loop {
            let message = actor.system.receive().unwrap();

            if message.is::<FeatureMsg>() {
                // Regular tracking. Received from tracking frontend
                let queue_len = actor.system.queue_len();
                if queue_len > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Tracking backend dropped 1 frame, queue len: {}", queue_len);
                    continue;
                }

                let msg = message.downcast::<FeatureMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));
                debug!("Tracking working on frame {}. State: {:?}, Reference kf: {:?}", msg.frame_id, actor.state, actor.ref_kf_id);
                actor.current_frame = Frame::new(
                    msg.frame_id, 
                    msg.keypoints,
                    msg.descriptors,
                    msg.image_width,
                    msg.image_height,
                    None,
                    actor.last_frame.as_ref(),
                    msg.timestamp,
                ).expect("Could not create frame!");
                let mut imu_measurements = msg.imu_measurements;
                match actor.track(&mut imu_measurements) {
                    Ok((created_kf, reset_map)) => {
                        if reset_map {
                            actor.last_frame = None;
                            actor.ref_kf_id = None;
                        } else {
                            if actor.current_frame.ref_kf_id.is_none() {
                                actor.current_frame.ref_kf_id = actor.ref_kf_id;
                            }
                            actor.last_frame = Some(actor.current_frame.clone());
                        }
                        println!("after tracking done, reset map is {}, last frame is {}", reset_map, actor.last_frame.is_some());

                        match actor.state {
                            TrackingState::Ok | TrackingState::RecentlyLost => {
                                actor.update_trajectory_in_logs(created_kf).expect("Could not save trajectory");
                            },
                            _ => {},
                        };
                    },
                    Err(e) => {
                        panic!("Error in Tracking Backend: {}", e);
                    }
                };
            } else if message.is::<InitKeyFrameMsg>() {
                // Received from local mapping after it inserts a keyframe
                let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

                if matches!(actor.state, TrackingState::NotInitialized) {
                    debug!("Map just reset, so ignore message");
                } else {
                    actor.last_frame.as_mut().unwrap().ref_kf_id = Some(msg.kf_id);
                    actor.ref_kf_id = Some(msg.kf_id);
                }

            } else if message.is::<LastKeyFrameUpdatedMsg>() {
                // Received from local mapping after it culls and creates new MPs for the last inserted KF
                if matches!(actor.state, TrackingState::NotInitialized) {
                    debug!("LastKFUpdatedMsg sent from local mapping but we just reset");
                } else {
                    actor.state = TrackingState::Ok;
                }
            } else if message.is::<UpdateFrameIMUMsg>() {
                let msg = message.downcast::<UpdateFrameIMUMsg>().unwrap_or_else(|_| panic!("Could not downcast IMU initialized message!"));

                actor.update_frame_imu(msg.scale, msg.imu_bias, msg.current_kf_id, msg.imu_initialized);
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Tracking backend received unknown message type!");
            }
        }
    }
}

impl TrackingBackend {
    fn track(&mut self, imu_measurements: &mut ImuMeasurements) -> Result<(bool, bool), Box<dyn std::error::Error>>  {
        let _span = tracy_client::span!("track");

        // TODO (reset): Reset map because local mapper set the bad imu flag
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1808

        // TODO (multimaps): Create new map if timestamp older than previous frame arrives
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1820

        // TODO (reset) TODO (multimaps): Timestamp jump detected, either reset active map or create new map
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1828
        // (entire block)

        if self.sensor.is_imu() {
            if let Some(ref_kf_id) = self.ref_kf_id {
                if let Some(ref_kf) = self.map.read().keyframes.get(&ref_kf_id) {
                    self.current_frame.imu_data.set_new_bias(ref_kf.imu_data.imu_bias);
                    println!("Tracking, set current frame bias to last KF ({}) bias {}", ref_kf.frame_id, ref_kf.imu_data.imu_bias);
                }
            }
            if self.last_frame.is_some() {
                self.imu.preintegrate(imu_measurements, &mut self.current_frame, self.last_frame.as_mut().unwrap(), self.map.read().last_kf_id);
            }
        }

        self.map_updated = self.map.read().map_change_index > self.last_map_change_index;
        self.last_map_change_index = self.map.read().map_change_index;

        // Initial estimation of camera pose and matching
        match (self.localization_only_mode, self.state) {
            (true, _) => {
                todo!("Localization only");
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1933
                // Look for "mbOnlyTracking" in Track() function
            },
            (false, TrackingState::NotInitialized) => {
                if self.initialization.is_none() {
                    self.initialization = Some(MapInitialization::new());
                }
                let init_success = self.initialization.as_mut().unwrap().try_initialize(&self.current_frame)?;
                if init_success {

                    let (ini_kf_id, curr_kf_id);
                    {
                        // TODO (stereo) - Add option to make the map monocular or stereo
                        (ini_kf_id, curr_kf_id) = match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map, & self.imu.imu_preintegrated_from_last_kf) {
                                Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, curr_kf_timestamp, _inverse_scene_median_depth)) => {
                                    // Map needs to be initialized before tracking can begin. Received from map actor
                                    self.frames_since_last_kf = 0;
                                    self.local_keyframes.insert(curr_kf_id);
                                    self.local_keyframes.insert(ini_kf_id);
                                    self.local_mappoints = local_mappoints;
                                    self.ref_kf_id = Some(curr_kf_id);
                                    self.last_kf_timestamp = Some(curr_kf_timestamp);
                                    if self.sensor.is_imu() {
                                        self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(self.map.read().keyframes.get(& curr_kf_id).unwrap().imu_data.imu_preintegrated.as_ref().unwrap().get_updated_bias());
                                    }

                                    {
                                        // Set current frame's updated info from map initialization
                                        self.current_frame.ref_kf_id = Some(curr_kf_id);
                                        self.current_frame.pose = Some(curr_kf_pose);
                                        self.current_frame.mappoint_matches = self.map.read().keyframes.get(&curr_kf_id).unwrap().clone_matches();
                                    }
                                    self.state = TrackingState::Ok;

                                    // Log initial pose in shutdown actor
                                    self.system.send(SHUTDOWN_ACTOR, 
                                    Box::new(TrajectoryMsg{
                                            pose: self.map.read().keyframes.get(&ini_kf_id).unwrap().get_pose(),
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

                    // Let tracking frontend know the map is initialized
                    self.system.send(TRACKING_FRONTEND, Box::new(
                        TrackingStateMsg {
                            state: TrackingState::Ok,
                            init_id: curr_kf_id
                        }
                    ));

                    // Send first two keyframes to local mapping
                    self.system.send(LOCAL_MAPPING, Box::new(
                        InitKeyFrameMsg { kf_id: ini_kf_id }
                    ));
                    self.system.send(LOCAL_MAPPING,Box::new(
                        InitKeyFrameMsg { kf_id: curr_kf_id }
                    ));


                    self.state = TrackingState::Ok;
                    sleep(Duration::from_millis(50)); // Sleep just a little to allow local mapping to process first keyframe

                    return Ok((false, false));
                } else {
                    self.state = TrackingState::NotInitialized;
                    return Ok((false, false));
                }
            },
            (false, TrackingState::Ok) => {
                let no_motion_model = (self.imu.velocity.is_none() && !self.map.read().imu_initialized) || self.relocalization.frames_since_lost(&self.current_frame) < 2;

                let track_success = match no_motion_model {
                    true => self.track_reference_keyframe()?,
                    false => {
                        match self.track_with_motion_model()? {
                            true => true,
                            false => self.track_reference_keyframe()?
                        }
                    }
                };
                self.state = match track_success {
                    true => TrackingState::Ok,
                    false => {
                        self.relocalization.timestamp_lost = Some(self.current_frame.timestamp);
                        match self.map.read().keyframes.len() > 10 {
                            true =>{warn!("State recently lost!"); TrackingState::RecentlyLost},
                            false => TrackingState::Lost
                        }
                    },
                }
            },
            (false, TrackingState::RecentlyLost) => {
                let relocalize_success = match self.imu.ready(&self.map) {
                    true => {
                        todo!("Relocalization, shutting down for now.");
                        // let _ok = self.imu.predict_state(&mut self.map, & self.current_frame);
                        // self.relocalization.past_cutoff(&self.current_frame)
                    },
                    false => {
                        // TODO (relocalization): remove the call to shutdown actor and uncomment line of code below.
                        // This is just to gracefully shut down instead of panicking at the to do 
                        todo!("Relocalization! Shutting down for now.");
                        self.system.send(SHUTDOWN_ACTOR, Box::new(ShutdownMsg{}));
                        self.state = TrackingState::Lost;
                        return Ok((false, false));
                        // !self.relocalization.run() && self.relocalization.sec_since_lost(&self.current_frame) > 3
                    }
                };
                self.state = match relocalize_success {
                    true => {
                        warn!("Relocalization unsuccessful...");
                        TrackingState::Lost
                    },
                    false => {
                        warn!("Setting state to recently lost!");
                        TrackingState::RecentlyLost
                    }
                }
            },
            (false, TrackingState::Lost) => {
                match self.map.read().keyframes.len() < 10 {
                    true => {
                        warn!("Reseting current map...");
                        todo!("Multi-maps, Can use the reset_active_map_code but there is no multi-map reasoning.");
                        self.system.send(SHUTDOWN_ACTOR, Box::new(ShutdownMsg{}));
                        self.state = TrackingState::Lost;
                        return Ok((false, false));
                    },
                    false => {
                        info!("Creating new map...");
                        self.create_new_map();
                    }
                }

                self.state = TrackingState::Ok;
                return Ok((false, false));
            }
        };

        // Only 3 valid states now
        // TrackingState::Ok | TrackingState::RecentlyLost | TrackingState::Lost

        // Track Local Map
        let (enough_matches, _matches_in_frame) = self.track_local_map();
        if enough_matches {
            self.state = TrackingState::Ok;
        } else if matches!(self.state, TrackingState::Ok) {
            if self.sensor.is_imu() {
                warn!("Track lost for less than 1 second, resetting active map...");
                self.reset_active_map();
                return Ok((false, true));
            }
            self.relocalization.timestamp_lost = Some(self.current_frame.timestamp);
            self.state = TrackingState::RecentlyLost;
            warn!("Setting state to recently lost!");
        }

        if self.sensor.is_imu() {
            // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
            if self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + self.frames_to_reset_imu as i32 && self.current_frame.frame_id > self.frames_to_reset_imu as i32 && self.map.read().imu_initialized {
                todo!("Relocalization.");
                // Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
                // Frame* pF = new Frame(mCurrentFrame);
                // pF->mpPrevFrame = new Frame(mLastFrame);

                // // Load preintegration
                // pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
            }
        }

        let mut created_new_kf = false;
        if enough_matches || matches!(self.state, TrackingState::RecentlyLost) {
            // Update motion model
            let last_frame = self.last_frame.as_ref();
            if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !self.current_frame.pose.is_none() {
                let last_twc = last_frame.expect("No last frame in tracking?").pose.as_ref().expect("Can't get last frame's pose?").inverse();
                // println!("Last frame pose: {:?}", last_twc);
                // println!("Current frame pose: {:?}", self.current_frame.pose.unwrap());
                self.imu.velocity = Some(*self.current_frame.pose.as_ref().expect("Can't get current frame?") * last_twc);
                // println!("Setting velocity for frame {} to {:?}", self.current_frame.frame_id, self.imu.velocity.unwrap());
            } else {
                self.imu.velocity = None;
            }

            // Clean VO matches
            self.current_frame.delete_mappoints_without_observations(&self.map.read());

            // Check if we need to insert a new keyframe
            let insert_if_lost_anyway = self.insert_kfs_when_lost && matches!(self.state, TrackingState::RecentlyLost) && self.sensor.is_imu();
            let need_new_kf = self.need_new_keyframe();
            if need_new_kf && (matches!(self.state, TrackingState::Ok) || insert_if_lost_anyway) {
                self.create_new_keyframe();
                created_new_kf = true;
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            let _ = self.current_frame.delete_mappoint_outliers();
        }

        // Reset if the camera get lost soon after initialization
        if matches!(self.state, TrackingState::Lost) {
            if self.map.read().keyframes.len() <= 10  || (self.sensor.is_imu() && !self.map.read().imu_initialized) {
                warn!("tracking_backend::handle_message;Track lost soon after initialization, resetting...",);
                // TODO (RESET)
            } else {
                self.create_new_map();
            }
        }

        Ok((created_new_kf, false))
    }

    fn update_trajectory_in_logs(
        &mut self, created_new_kf: bool
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        let last_frame = self.last_frame.as_ref().expect("No last frame in tracking?");

        let relative_pose = {
            let ref_kf_pose = if created_new_kf {
                // If we created a new kf this round, the relative keyframe pose is the same as the current frame pose.
                // We have to do this because ORBSLAM3 creates a new keyframe in tracking whereas we wait until beginning
                // of local mapping. This code is fine, but we might want to make an atomic int in the map containing the
                // latest keyframe inserted in local mapping, rather than sending that as a message from LM to T (current
                // implementation). The way we have it now, we are always one reference keyframe behind because local mapping
                // will insert the kf in the middle of the tracking thread loop, and then tracking will only know about it in
                // the next iteration.
                last_frame.pose.unwrap()
            } else {
                let map = self.map.read();
                let ref_kf = map.keyframes.get(&self.ref_kf_id.unwrap()).expect("Can't get ref kf from map");
                ref_kf.get_pose()
            };

            last_frame.pose.unwrap() * ref_kf_pose.inverse()
        };
        self.trajectory_poses.push(relative_pose);
        self.reference_kfs_for_trajectory.push((last_frame.ref_kf_id.unwrap(), self.map.read().keyframes.get(&last_frame.ref_kf_id.unwrap()).unwrap().parent));

        info!("Frame {} pose: {:?}", last_frame.frame_id, last_frame.pose.unwrap());

        self.system.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: last_frame.pose.unwrap().inverse(),
                ref_kf_id: last_frame.ref_kf_id.unwrap(),
                timestamp: last_frame.timestamp
            })
        );

        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            pose: self.current_frame.pose.unwrap(),
            mappoint_matches: self.current_frame.mappoint_matches.matches.clone(),
            nontracked_mappoints: self.non_tracked_mappoints.clone(),
            mappoints_in_tracking: self.local_mappoints.clone(),
            timestamp: self.current_frame.timestamp
        }));

        Ok(())
    }

    fn track_reference_keyframe(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track_reference_keyframe");
        // Tracking::TrackReferenceKeyFrame()
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver

        println!("TRACK REFERENCE KF");
        self.current_frame.compute_bow();
        let nmatches;
        {
            let map_read_lock = self.map.read();
            self.ref_kf_id = Some(map_read_lock.last_kf_id);
            println!("(1) TRACK REF KF, SET REF KF TO: {}", self.ref_kf_id.unwrap());
            let ref_kf = map_read_lock.keyframes.get(&self.ref_kf_id.unwrap()).unwrap();

            nmatches = FEATURE_MATCHING_MODULE.search_by_bow_with_frame(ref_kf, &mut self.current_frame, true, 0.7)?;
            // debug!("Tracking search by bow: {} matches / {} matches in keyframe {}. ({} total mappoints in map)", nmatches, ref_kf.get_mp_matches().iter().filter(|item| item.is_some()).count(), ref_kf.id, map_read_lock.mappoints.len());

        }

        if nmatches < 15 {
            warn!("track_reference_keyframe has fewer than 15 matches = {}!!\n", nmatches);
            return Ok(false);
        }

        self.current_frame.pose = Some(self.last_frame.as_ref().unwrap().pose.unwrap());
        println!("Track reference keyframe, set current frame pose to last frame pose: {:?}", self.last_frame.as_ref().unwrap().pose.unwrap());

        optimizer::optimize_pose(&mut self.current_frame, &self.map);

        // Discard outliers
        let nmatches_map = self.discard_outliers();

        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn track_with_motion_model(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
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
        // println!("TRACK MOTION MODEL");

        self.update_last_frame();

        let enough_frames_to_reset_imu = self.current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32);
        if self.map.read().imu_initialized && enough_frames_to_reset_imu {
            // Predict state with IMU if it is initialized and it doesnt need reset
            if self.map_updated && self.map.read().last_kf_id > 0 {
                self.imu.predict_state_last_keyframe(& self.map, &mut self.current_frame, self.map.read().last_kf_id);
            } else {
                self.imu.predict_state_last_frame(&mut self.current_frame, self.last_frame.as_mut().unwrap());
            }
            return Ok(true);
        } else {
            println!("self.imu.velocity: {:?}", self.imu.velocity);
            self.current_frame.pose = Some(self.imu.velocity.unwrap() * self.last_frame.as_ref().unwrap().pose.unwrap());
            println!("Track motion model, set current frame pose to last frame pose * imu velocity: {:?}", self.current_frame.pose.unwrap());

        }

        self.current_frame.mappoint_matches.clear();

        // Project points seen in previous frame
        let th = match self.sensor.frame() {
            FrameSensor::Mono => 15,
            _ => 7
        };

        let mut matches = FEATURE_MATCHING_MODULE.search_by_projection_with_threshold(
            &mut self.current_frame,
            self.last_frame.as_mut().unwrap(),
            th,
            true,
            &self.map,
            self.sensor
        )?;
        // debug!("Tracking search by projection with previous frame: {} matches / {} mappoints in last frame. ({} total mappoints in map)", matches, last_frame.mappoint_matches.debug_count, self.map.read().mappoints.len());

        // If few matches, uses a wider window search
        if matches < 20 {
            info!("tracking_backend::track_with_motion_model;not enough matches, wider window search");
            self.current_frame.mappoint_matches.clear();
            matches = FEATURE_MATCHING_MODULE.search_by_projection_with_threshold(
                &mut self.current_frame,
                self.last_frame.as_mut().unwrap(),
                2 * th,
                self.sensor.is_mono(),
                &self.map,
                self.sensor
            )?;
            // debug!("Tracking search by projection with previous frame: {} matches / {} mappoints in last frame. ({} total mappoints in map)", matches, last_frame.mappoint_matches.debug_count, self.map.read().mappoints.len());
        }

        if matches < 20 {
            warn!("tracking_backend::track_with_motion_model;not enough matches!!");
            return Ok(self.sensor.is_imu());
        }

        // Optimize frame pose with all matches
        optimizer::optimize_pose(&mut self.current_frame, &self.map);

        // Discard outliers
        let nmatches_map = self.discard_outliers();

        if self.localization_only_mode {
            todo!("Localization only");
            // mbVO = nmatchesMap<10;
            // return nmatches>20;
        }

        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };

    }

    fn update_last_frame(&mut self) {
        // Tracking::UpdateLastFrame()

        // Update pose according to reference keyframe
        let ref_kf_id = self.last_frame.as_ref().unwrap().ref_kf_id.unwrap();
        let map_lock = self.map.read();
        // println!("KFs in map: {:?}", map_lock.keyframes.keys());
        let ref_kf_pose = map_lock.keyframes.get(&ref_kf_id).expect(format!("Reference kf {} should be in map", ref_kf_id).as_str()).get_pose();
        self.last_frame.as_mut().unwrap().pose = Some(*self.trajectory_poses.last().unwrap() * ref_kf_pose);

        if self.sensor.is_mono() || self.frames_since_last_kf == 0 {
            return;
        }

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

    fn track_local_map(&mut self) -> (bool, i32) {
        let _span = tracy_client::span!("track_local_map");
        // bool Tracking::TrackLocalMap()
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        
        {
            let _span = tracy_client::span!("update_local_map");
            self.update_local_keyframes();
            self.update_local_points();
        }
        match self.search_local_points() {
            Ok(res) => res,
            Err(e) => warn!("Error in search_local_points: {}", e)
        }

        if !self.map.read().imu_initialized || (self.current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32)) {
            optimizer::optimize_pose(&mut self.current_frame, &self.map);
        } else if !self.map_updated {
            optimizer::pose_inertial_optimization_last_frame(&mut self.current_frame, &mut self.last_frame.as_mut().unwrap(), & self.track_in_view, &self.map, &self.sensor);
        } else {
            optimizer::pose_inertial_optimization_last_keyframe(&mut self.current_frame, & self.track_in_view, &self.map, &self.sensor);
        }

        self.matches_inliers = 0;
        // Update MapPoints Statistics
        for index in 0..self.current_frame.mappoint_matches.len() {
            if let Some((mp_id, is_outlier)) = self.current_frame.mappoint_matches.get(index) {
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
                            self.current_frame.mappoint_matches.delete_at_indices((index as i32, -1));
                        }
                    } else {
                        self.matches_inliers += 1;
                    }

                } else if !self.sensor.is_mono() {
                    todo!("Stereo");
                    //mCurrentFrame.mappoint_matches[i] = static_cast<MapPoint*>(NULL);
                    // self.current_frame.as_mut().unwrap().mappoint_matches.remove(&index);
                }
            }
        }
        println!("Matches in track local map: {}", self.matches_inliers);

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames_to_insert_kf as i32) && self.matches_inliers<50 {
            warn!("track_local_map unsuccessful; matches in frame < 50 : {}",self.matches_inliers);
            return (false, self.matches_inliers);
        }

        match self.state {
            TrackingState::RecentlyLost => { 
                return (self.matches_inliers > 10, self.matches_inliers);
            },
            _ => {}
        }

        println!("Track local map... matches inliers: {}, imu initialized: {}", self.matches_inliers, self.map.read().imu_initialized);
        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => { 
                return (
                    !(self.matches_inliers<15 && self.map.read().imu_initialized) && !(self.matches_inliers<40 && !self.map.read().imu_initialized),
                    self.matches_inliers
                )
            },
            Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => {
                return (self.matches_inliers >= 15, self.matches_inliers);
            },
            _ => { return (self.matches_inliers >= 30, self.matches_inliers) }
        }
    }

    fn update_local_keyframes(&mut self) -> Option<()> {
        // void Tracking::UpdateLocalKeyFrames()
        // Each map point votes for the keyframes in which it has been observed

        // BTreeMap so items are sorted, so we have the same output as orbslam for testing. Can probably revert to regular hashmap later.
        let mut kf_counter = BTreeMap::<Id, i32>::new();
        let lock = self.map.read();
        if !lock.imu_initialized || self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
            for i in 0..self.current_frame.mappoint_matches.len() {
                if let Some((mp_id, _is_outlier)) = self.current_frame.mappoint_matches.get(i as usize) {
                    if let Some(mp) = lock.mappoints.get(&mp_id) {
                        for kf_id in mp.get_observations().keys() {
                            *kf_counter.entry(*kf_id).or_insert(0) += 1;
                        }
                    } else {
                        self.current_frame.mappoint_matches.delete_at_indices((i as i32, -1));
                    }
                }
            }

        } else {
            // Using lastframe since current frame has no matches yet
            for i in 0..self.last_frame.as_ref().unwrap().mappoint_matches.len() {
                if let Some((mp_id, _is_outlier)) = self.last_frame.as_ref().unwrap().mappoint_matches.get(i as usize) {
                    if let Some(mp) = lock.mappoints.get(&mp_id) {
                        for kf_id in mp.get_observations().keys() {
                            *kf_counter.entry(*kf_id).or_insert(0) += 1;
                        }
                    } else {
                        self.last_frame.as_mut().unwrap().mappoint_matches.delete_at_indices((i as i32, -1));
                    }
                }
            }
        }


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
            self.kf_track_reference_for_frame.insert(kf_id, self.current_frame.frame_id);
        }

        // Also include some keyframes that are neighbors to already-included keyframes
        let mut i = 0;
        while local_kf_vec.len() <= 80 && i < local_kf_vec.len() { // Limit the number of keyframes
            let next_kf_id = local_kf_vec[i];
            let kf = lock.keyframes.get(&next_kf_id);
            match kf {
                Some(kf) => {
                    for kf_id in &kf.get_covisibility_keyframes(10) {
                        if self.kf_track_reference_for_frame.get(kf_id) != Some(&self.current_frame.frame_id) {
                            local_kf_vec.push(*kf_id);
                            self.kf_track_reference_for_frame.insert(*kf_id, self.current_frame.frame_id);
                            break;
                        }
                    }
                    for kf_id in &kf.children {
                        if self.kf_track_reference_for_frame.get(kf_id) != Some(&self.current_frame.frame_id) {
                            local_kf_vec.push(*kf_id);
                            self.kf_track_reference_for_frame.insert(*kf_id, self.current_frame.frame_id);
                            break;
                        }
                    }

                    if let Some(parent_id) = kf.parent {
                        if self.kf_track_reference_for_frame.get(&parent_id) != Some(&self.current_frame.frame_id) {
                            local_kf_vec.push(parent_id);
                            self.kf_track_reference_for_frame.insert(parent_id, self.current_frame.frame_id);
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
            // For the last 20 created keyframes, save into new_local_keyframes
            // until one of them was not in self.local_keyframes. Then, quit.
            let mut temp_kf_id = lock.last_kf_id;
            for _ in 0..20 {
                if self.kf_track_reference_for_frame.get(&temp_kf_id) != Some(&self.current_frame.frame_id) {
                    self.local_keyframes.insert(temp_kf_id);
                    self.kf_track_reference_for_frame.insert(temp_kf_id, self.current_frame.frame_id);
                    match lock.keyframes.get(& temp_kf_id) {
                        Some(temp_kf) => {
                            match temp_kf.prev_kf_id {
                                Some(prev_kf_id) => {
                                    temp_kf_id = prev_kf_id;
                                },
                                None => break
                            }
                        },
                        None => break
                    }
                }
            }
        }

        if max_kf_id != 0 {
            self.current_frame.ref_kf_id = Some(max_kf_id);
            self.ref_kf_id = Some(max_kf_id);
            println!("(2) TRACK REF KF, SET REF KF TO: {}", self.ref_kf_id.unwrap());
        }
        None
    }

    fn update_local_points(&mut self) {
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
                            if self.mp_track_reference_for_frame.get(mp_id) != Some(&self.current_frame.frame_id) {
                                self.local_mappoints.insert(*mp_id);
                                self.mp_track_reference_for_frame.insert(*mp_id, self.current_frame.frame_id);
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

    fn search_local_points(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("search_local_points");
        //void Tracking::SearchLocalPoints()

        // Do not search map points already matched
        {
            let mut increased_visible = vec![];
            let lock = self.map.read();
            for index in 0..self.current_frame.mappoint_matches.len() {
                if let Some((id, _)) = self.current_frame.mappoint_matches.get(index as usize) {
                    if let Some(mp) = lock.mappoints.get(&id) {
                        mp.increase_visible(1);
                        increased_visible.push(id);

                        self.last_frame_seen.insert(id, self.current_frame.frame_id);
                        self.track_in_view.remove(&id);
                        self.track_in_view_r.remove(&id);
                    } else {
                        self.current_frame.mappoint_matches.delete_at_indices((index as i32, -1));
                    }
                }
            }
        }

        // Project points in frame and check its visibility
        let mut to_match = 0;
        {
            let lock = self.map.read();
            for mp_id in &self.local_mappoints {
                if self.last_frame_seen.get(mp_id) == Some(&self.current_frame.frame_id) {
                    continue;
                }

                match lock.mappoints.get(mp_id) {
                    Some(mp) => {
                        // Project (this fills MapPoint variables for matching)
                        let (tracked_data_left, tracked_data_right) = self.current_frame.is_in_frustum(mp, 0.5);
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

        if to_match > 0 {
            let mut th = match self.sensor.frame() {
                FrameSensor::Rgbd => 3,
                _ => 1
            };
            if self.map.read().imu_initialized {
                if self.map.read().imu_ba2 {
                    th = 2;
                } else {
                    th = 6;
                }
            } else if !self.map.read().imu_initialized && self.sensor.is_imu() {
                th = 6;
            }

            // If the camera has been relocalised recently, perform a coarser search
            if self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
                th = 5;
            }
            match self.state {
                TrackingState::RecentlyLost | TrackingState::Lost => th = 15,
                _ => {}
            }

            let (non_tracked_points, matches) = FEATURE_MATCHING_MODULE.search_by_projection(
                &mut self.current_frame,
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

    fn create_new_keyframe(&mut self) {
        let _span = tracy_client::span!("create_new_keyframe");
        //CreateNewKeyFrame
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216
        let new_kf = Frame::new_clone(& self.current_frame);

        //Reset preintegration from last KF (Create new object)
        self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(self.current_frame.imu_data.imu_bias);

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

        // tracy_client::Client::running()
        // .expect("message! without a running Client")
        // .message("create new keyframe", 2);

        // KeyFrame created here and inserted into map
        self.system.send(
            LOCAL_MAPPING,
            Box::new( NewKeyFrameMsg{
                keyframe: new_kf,
                tracking_state: self.state,
                matches_in_tracking: self.matches_inliers,
                tracked_mappoint_depths: self.track_in_view.iter().map(|(k, v)| (*k, v.track_depth)).collect(),
            } )
        );
    }

    fn need_new_keyframe(&mut self) -> bool {
        let num_kfs = self.map.read().keyframes.len();
        let not_enough_frames_since_last_reloc = (self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames_to_insert_kf as i32)) && (num_kfs as i32 > self.max_frames_to_insert_kf);

        if self.sensor.is_imu() && !self.map.read().imu_initialized {
            if (self.current_frame.timestamp - self.last_kf_timestamp.unwrap()) * 1e9 >= 0.25 { // 250 milliseconds
                return true;
            } else {
                return false;
            }
        }

        if self.localization_only_mode {
            return false;
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
            Some(timestamp) => self.current_frame.timestamp - timestamp < 0.5, // 500 milliseconds
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
            return true;
        } else {
            self.frames_since_last_kf += 1;
            return false;
        }
    }

    fn update_frame_imu(&mut self, scale: f64, imu_bias: ImuBias, current_kf_id: Id, imu_initialized: bool) {
        println!("UpdateFrameIMU, Set bias for KF {}", current_kf_id);

        // Map * pMap = pCurrentKeyFrame->GetMap();
        // unsigned int index = mnFirstFrameId;
        // list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
        // list<bool>::iterator lbL = mlbLost.begin(); // todo this is always true??

        for i in 0..self.trajectory_poses.len() {
            let mut pose = self.trajectory_poses[i];
            let (mut kf, mut parent) = self.reference_kfs_for_trajectory[i];

            while self.map.read().keyframes.get(& kf).is_none() {
                println!("KF deleted, finding parent {} {:?}", kf, parent);
                if parent.is_none() {
                    break;
                } else {
                    let lock = self.map.read();
                    let parent_kf = lock.keyframes.get(& parent.unwrap()).unwrap();
                    kf = parent_kf.id;
                    parent = parent_kf.parent;
                }

            }

            if self.map.read().keyframes.get(& kf).unwrap().origin_map_id == self.map.read().id {
                pose.set_translation(*pose.get_translation() * (scale));
            }
        }
        self.imu.last_bias = Some(imu_bias);
        self.last_kf_timestamp = Some(self.map.read().keyframes.get(&current_kf_id).unwrap().timestamp);
        self.last_frame.as_mut().unwrap().imu_data.set_new_bias(imu_bias);
        self.current_frame.imu_data.set_new_bias(imu_bias);

        while !self.current_frame.imu_data.imu_preintegrated.is_some() {
            println!("Waiting for bias to be set");
            std::thread::sleep(std::time::Duration::from_millis(500));
        }

        // if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
        let lock = self.map.read();
        let last_kf = lock.keyframes.get(&self.map.read().last_kf_id).unwrap();
        if self.last_frame.as_ref().unwrap().frame_id == self.map.read().last_kf_id {
            self.last_frame.as_mut().unwrap().set_imu_pose_velocity(
            Pose::new(
                * last_kf.get_imu_position(),
                * last_kf.get_imu_rotation(),
            ),
            * last_kf.imu_data.get_velocity()
            );
        } else {
            let gz = nalgebra::Vector3::new(0.0, 0.0, -GRAVITY_VALUE);
            let twb1 = * last_kf.get_imu_position();
            let rwb1 = * last_kf.get_imu_rotation();
            let vwb1 = * last_kf.imu_data.get_velocity();
            let t12 = last_kf.imu_data.imu_preintegrated.as_ref().unwrap().d_t;

            let delta_rot = last_kf.imu_data.imu_preintegrated.as_ref().unwrap().get_updated_delta_rotation();
            let delta_pos = last_kf.imu_data.imu_preintegrated.as_ref().unwrap().get_updated_delta_position();
            let delta_vel = last_kf.imu_data.imu_preintegrated.as_ref().unwrap().get_updated_delta_velocity();
            self.last_frame.as_mut().unwrap().set_imu_pose_velocity(
                Pose::new(
                    twb1 + vwb1 * t12 + 0.5 * t12 * t12 * gz + rwb1 * delta_pos,
                    * normalize_rotation(rwb1 * delta_rot),
                ),
                vwb1 + gz * t12 + rwb1 * delta_vel
            );

        }

        if self.current_frame.imu_data.imu_preintegrated.is_some() {
            let last_kf = lock.keyframes.get(&self.map.read().last_kf_id).unwrap();
            let gz = nalgebra::Vector3::new(0.0, 0.0, -GRAVITY_VALUE);
            let twb1 = * last_kf.get_imu_position();
            let rwb1 = * last_kf.get_imu_rotation();
            let vwb1 = * last_kf.imu_data.get_velocity();
            let t12 = last_kf.imu_data.imu_preintegrated.as_ref().unwrap().d_t;

            let delta_rot = self.current_frame.imu_data.imu_preintegrated.as_ref().unwrap().get_updated_delta_rotation();
            let delta_pos = self.current_frame.imu_data.imu_preintegrated.as_ref().unwrap().get_updated_delta_position();
            let delta_vel = self.current_frame.imu_data.imu_preintegrated.as_ref().unwrap().get_updated_delta_velocity();

            self.current_frame.set_imu_pose_velocity(
                Pose::new(
                    twb1 + vwb1 * t12 + 0.5 * t12 * t12 * gz + rwb1 * delta_pos,
                    * normalize_rotation(rwb1 * delta_rot),
                ),
                vwb1 + gz * t12 + rwb1 * delta_vel
            );

        }
    }

    fn reset_active_map(&mut self) {
        println!("Reset active map!!!!!");

        // Reset loop closing
        self.system.send(
            LOOP_CLOSING,
            Box::new( Reset{ } )
        );

        // Reset local mapping
        self.system.send(
            LOCAL_MAPPING,
            Box::new( Reset{ } )
        );
        LOCAL_MAPPING_ABORT.store(true, Ordering::SeqCst);

        self.map.write().reset_active_map();

        self.initialization = Some(MapInitialization::new());
        self.state = TrackingState::NotInitialized;
        self.ref_kf_id = None;
        self.frames_since_last_kf = 0;
        self.last_kf_timestamp = None;
        self.matches_inliers = 0;
        self.local_keyframes = BTreeSet::new();
        self.local_mappoints = BTreeSet::new();
        self.track_in_view = HashMap::new();
        self.track_in_view_r = HashMap::new();
        self.kf_track_reference_for_frame = HashMap::new();
        self.mp_track_reference_for_frame = HashMap::new();
        self.last_frame_seen = HashMap::new();
        self.relocalization = Relocalization{last_reloc_frame_id: 0, timestamp_lost: None};
        self.map_updated = false;
        self.last_map_change_index = 0;
        self.trajectory_poses = Vec::new();
        self.non_tracked_mappoints = HashMap::new();

        tracy_client::Client::running()
            .expect("message! without a running Client")
            .message("Reset active map", 2);
    }

    //* Helper functions */
    fn discard_outliers(&mut self) -> i32 {
        let discarded = self.current_frame.delete_mappoint_outliers();
        // Note: orbslam removes mappoint from track_in_view if the index < mCurrentFrame.Nleft but Nleft is always -1!
        // So, commenting out for now.
        for (mp_id, _index) in discarded {
            // if (index as u32) < self.current_frame.features.num_keypoints {
            //     self.track_in_view.remove(&mp_id);
            //     // TODO (Stereo) ... need to remove this from track_in_view_r if the mp is seen in the right camera
            // }
            self.last_frame_seen.insert(mp_id, self.current_frame.frame_id);
        }
        self.current_frame.mappoint_matches.tracked_mappoints(&*self.map.read(), 1)
    }

    //* Next steps */
    fn create_new_map(&self) -> bool {
        todo!("Multimaps: Atlas::CreateNewMap");
    }
}

#[derive(Default, Debug, Clone)]
pub struct TrackedMapPointData {
    pub predicted_level: i32,
    pub view_cos: f64,
    pub proj_x: f64,
    pub proj_y: f64,
    pub track_depth: f64
    // Note: orbslam also has "right" versions of each of the above fields
    // and sets either the normal (left) versions or the right versions, depending on which camera it is.
    // When writing the stereo code, don't duplicate all the fields like they did.
}

#[derive(Debug, Clone, Copy, Default)]
pub enum TrackingState {
    #[default] NotInitialized,
    Lost,
    RecentlyLost,
    Ok
}