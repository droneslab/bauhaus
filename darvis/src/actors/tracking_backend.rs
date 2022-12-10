use std::{sync::Arc, collections::{HashSet, HashMap}};
use axiom::prelude::*;
use chrono::{prelude::*, Duration};
use derivative::Derivative;
use log::{warn, info, debug, error};
use opencv::core::Point2f;
use dvcore::{lockwrap::ReadOnlyWrapper, plugin_functions::Function, config::*};
use crate::{
    actors::messages::{FeatureMsg, KeyFrameMsg, MapInitializedMsg, TrajectoryMessage, TrackingStateMsg},
    registered_modules::{LOCAL_MAPPING, TRACKING_BACKEND, SHUTDOWN},
    dvmap::{
        map::Map, map_actor::MapWriteMsg, map_actor::{MAP_ACTOR}, map::Id,
        keyframe::KeyFrame, frame::Frame, pose::Pose, mappoint::{MapPoint, FullMapPoint},
    },
    modules::{camera::Camera, camera::CameraType, imu::{ImuModule}, optimizer::{self}, orbmatcher, map_initialization::Initialization, relocalization::Relocalization},
};

use super::messages::{InitialMapMsg, KeyFrameIdMsg};

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
impl TrackedMapPointData {
    pub fn new() -> Self {
        Self { ..Default::default() }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum TrackingState {
    #[default] NotInitialized,
    Lost,
    RecentlyLost,
    WaitForMapResponse,
    Ok
}

#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct DarvisTrackingBack {
    state: TrackingState,

    // Map
    map: ReadOnlyWrapper<Map>,
    initialization: Initialization, // data sent to map actor to initialize new map

    // Feature matching
    temporal_points: Vec<MapPoint<FullMapPoint>>,
    matches_in_frame : i32, // Current matches in frame
    prev_matched: Vec<Point2f>, // mvbPrevMatched

    // Frames
    last_frame_id: Id,
    current_frame: Frame,
    last_frame: Option<Frame>,

    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_ts: Option<DateTime<Utc>>,

    // Local map
    // TODO (design) Can we get rid of the local_ variables and pipe them through the map instead?
    local_keyframes: HashSet<Id>, //mvpLocalKeyFrames 
    local_mappoints: HashSet<Id>, //mvpLocalMapPoints
    track_in_view: HashMap::<Id, TrackedMapPointData>,
    track_in_view_r: HashMap::<Id, TrackedMapPointData>,

    // IMU 
    imu: ImuModule,

    // Relocalization
    relocalization: Relocalization,

    // Idk where to put these
    camera: Camera,
    map_updated : bool,  // TODO (design) I'm not sure we want to use this

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

    // Global defaults
    localization_only_mode: bool,
    frames_to_reset_imu: u32, //mnFramesToResetIMU
    insert_kfs_when_lost: bool,
    min_num_features: i32,
    max_frames : i64 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
    min_frames: u32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
    sensor: Sensor,
}

impl DarvisTrackingBack {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisTrackingBack {
        // Note: This is suuuuuch a specific bug. Default::default() calls the default function 
        // for every single item in the struct, even if it is explicitly overriden (such as
        // map, camera, sensor, global_defaults, and optimizer in the below code). This means that
        // a second map is created, DISCARDED, and then DarvisTrackingBack.map is set to the first map
        // that we wanted in the first place. So it looks like it duplicates the map, but it doesn't actually
        // and you shouldn't worry if you see this.
        DarvisTrackingBack {
            map,
            camera: Camera::new(CameraType::Pinhole).unwrap(),
            sensor: GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor"),
            initialization: Initialization::new(),
            localization_only_mode: GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "localization_only_mode"),
            frames_to_reset_imu: GLOBAL_PARAMS.get::<i32>(TRACKING_BACKEND, "frames_to_reset_IMU") as u32,
            insert_kfs_when_lost: GLOBAL_PARAMS.get::<bool>(TRACKING_BACKEND, "insert_KFs_when_lost"),
            min_num_features: GLOBAL_PARAMS.get::<i32>(TRACKING_BACKEND, "min_num_features"),
            max_frames: GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "fps") as i64,
            min_frames: 0,
            last_frame_id: -1,
            ..Default::default()
        }
    }

    fn handle_message(&mut self, context: axiom::prelude::Context, msg: Arc<FeatureMsg>) {
        let map_actor = context.system.find_aid_by_name(MAP_ACTOR).unwrap();
        let tracking_actor = context.system.find_aid_by_name(TRACKING_BACKEND).unwrap();

        // Sofiya interface: creating new frame
        self.last_frame_id += 1;
        self.current_frame = match Frame::new(
            self.last_frame_id, 
            msg.keypoints.clone(), // TODO (msg copy)
            msg.descriptors.clone(), // TODO (msg copy)
            msg.image_width,
            msg.image_height,
            &self.camera,
        ) {
            Ok(frame) => frame,
            Err(e) => panic!("Problem creating a frame: {:?}", e),
        };

        // TODO (reset): Reset map because local mapper set the bad imu flag
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1808

        // TODO (multimaps): Create new map if timestamp older than previous frame arrives
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1820

        // TODO (reset) TODO (multimaps): Timestamp jump detected, either reset active map or create new map
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1828
        // (entire block)

        // TODO (IMU) set bias of new frame = to bias of last
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1860

        if self.sensor.is_imu() {
            self.imu.preintegrate();
        }

        // TODO: update map change index. Used by mbMapUpdated
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1890

        debug!("Tracking state is {:?}", self.state);
        // Initial estimation of camera pose and matching
        let initial_success = match self.state {
            TrackingState::WaitForMapResponse => { return; },
            TrackingState::NotInitialized => {
                match self.initialization.try_initialize(&self.current_frame, &mut self.camera) {
                    Ok(ready) => {
                        if ready {
                            let msg = match self.sensor.frame() {
                                FrameSensor::Mono => MapWriteMsg::create_initial_map_monocular(self.initialization.clone(), tracking_actor),
                                _ => MapWriteMsg::create_initial_map_stereo(self.initialization.clone(), tracking_actor)
                            };
                            map_actor.send_new(msg).unwrap();
                            info!("Sent initialization info to map");
                            self.state = TrackingState::WaitForMapResponse;
                        }
                    },
                    Err(msg) => info!("{}", msg)
                }

                return;
            },
            TrackingState::Ok => {
                let mut ok = false;
                if self.imu.ready(&self.map) && self.relocalization.frames_since_lost(&self.current_frame) >= 2 {
                    ok = self.track_with_motion_model().unwrap();
                    debug!("Track with motion model result, {}", ok);
                };
                if !ok {
                    ok = self.track_reference_keyframe(&map_actor).unwrap();
                    debug!("Track reference keyframe result, {}", ok);
                }
                if !ok {
                    self.relocalization.timestamp_lost = Some(self.current_frame.timestamp);
                    self.state = match self.kfs_in_map() > 10 {
                        true => TrackingState::RecentlyLost,
                        false => TrackingState::Lost
                    };
                }
                ok
            },
            TrackingState::RecentlyLost => {
                warn!("tracking_backend::handle_message;State recently lost");
                let (ok, is_lost);
                if self.imu.ready(&self.map) {
                    ok = self.imu.predict_state();
                    is_lost = self.relocalization.past_cutoff(&self.current_frame);
                } else {
                    ok = self.relocalization.run();
                    is_lost = !ok && self.relocalization.sec_since_lost(&self.current_frame) > Duration::seconds(3);
                }
                if is_lost {
                    self.state = TrackingState::Lost;
                    warn!("tracking_backend::handle_message;Tracking lost...");
                }
                ok
            },
            TrackingState::Lost => {
                info!("tracking_backend::handle_message;New map...");
                if self.kfs_in_map() < 10 {
                    info!("tracking_backend::handle_message;Reseting current map...");
                    map_actor.send_new(MapWriteMsg::reset_active_map()).unwrap();
                } else {
                    self.create_new_map();
                }

                return;
            }
        };

        // TODO (localization-only): Above match should check for localization-only mode
        // and only run the above code if localization-only is NOT enabled. Need to implement
        // localization-only version.
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1933
        // Look for "mbOnlyTracking" in Track() function

        // Track Local Map
        let success = initial_success && self.track_local_map(&map_actor);
        self.state = match success {
            true => TrackingState::Ok,
            false => {
                if self.sensor.is_imu() {
                    info!("tracking_backend::handle_message;Track lost for less than 1 second");

                    // TODO (reset) TODO (IMU): Reset map because local mapper set the bad imu flag
                    // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2149
                }
                self.relocalization.timestamp_lost = Some(self.current_frame.timestamp);
                TrackingState::RecentlyLost
            }
        };

        // TODO (IMU)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
        // This code not done, so double check with C++ reference.
        // {
        //     // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        //     let enough_frames_to_reset_imu = self.current_frame.as_ref().unwrap().id <= self.last_reloc_frame_id + self.frames_to_reset_imu;
        //     if enough_frames_to_reset_imu && self.current_frame.as_ref().unwrap().id > self.frames_to_reset_imu && self.sensor.is_imu() && self.self.map.read().imu_initialized() {
        //         // Load preintegration
        //         // This code directly copied from C++, obviously needs to be changed
        //         // pF->mpImuPreintegratedFrame = new IMU::Preintegrated(self.current_frame.mpImuPreintegratedFrame);
        //     }
        //     if self.self.map.read().imu_initialized() && success {
        //         if self.current_frame.as_ref().unwrap().id == self.last_reloc_frame_id + self.frames_to_reset_imu {
        //             println!("TRACK: RESETING FRAME!!!");
        //             self.reset_frame_IMU();
        //         }
        //         else if self.current_frame.as_ref().unwrap().id > self.last_reloc_frame_id + 30 {
        //             self.last_bias = self.current_frame.as_ref().unwrap().imu_bias;
        //         }
        //     }
        // }

        if success || matches!(self.state, TrackingState::RecentlyLost) {
            // Update motion model
            let last_frame = self.last_frame.as_ref();
            if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !self.current_frame.pose.is_none() {
                let last_pose = last_frame.unwrap().pose.as_ref().unwrap();
                let last_twc = last_pose.inverse();
                // TODO (IMU)
                self.imu.velocity = Some(self.current_frame.pose.as_ref().unwrap().clone() * last_twc);
            } else {
                self.imu.velocity = None;
            }

            self.current_frame.delete_mappoints_without_observations(&self.map.read());
            self.temporal_points.clear();

            // Check if we need to insert a new keyframe
            let insert_if_lost_anyway = self.insert_kfs_when_lost && matches!(self.state, TrackingState::RecentlyLost) && self.sensor.is_imu();
            if self.need_new_keyframe() && (success || insert_if_lost_anyway) {
                self.create_new_keyframe(&context);
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            let _ = self.current_frame.discard_outliers();
        }

        // Reset if the camera get lost soon after initialization
        if matches!(self.state, TrackingState::Lost) {
            if self.kfs_in_map() <= 10  || (self.sensor.is_imu() && !self.map.read().imu_initialized) {
                warn!("tracking_backend::handle_message;Track lost before IMU initialisation, resetting...",);
                let map_msg = MapWriteMsg::reset_active_map();
                map_actor.send_new(map_msg).unwrap();
                return;
            }

            self.create_new_map();
            return;
        }

        if self.current_frame.ref_kf_id.is_none() {
            self.current_frame.ref_kf_id = self.ref_kf_id;
        }

        debug!(" state_check : Middle {:?}", self.state);
        let trajectory_msg = match self.state {
            TrackingState::Ok | TrackingState::RecentlyLost => {
                match self.current_frame.pose {
                    Some(current_pose) => {
                        match self.current_frame.ref_kf_id {
                            Some(ref_kf_id) => {
                                TrajectoryMessage::new(
                                    current_pose * self.map.read().get_keyframe(&ref_kf_id).unwrap().pose.inverse(),
                                    self.current_frame.ref_kf_id.unwrap(),
                                    self.current_frame.timestamp
                                )
                            },
                            None => {error!("Current frame has pose but ref kf is still none.Probably a concurrency issue with map actor sending pose info back to here."); TrajectoryMessage::empty()}
                        }
                    },
                    None => {error!("Tracking state is {:?} but current frame does not have a pose.", self.state); TrajectoryMessage::empty()}
                }
            },
            _ => TrajectoryMessage::empty()
        };
        context.system.find_aid_by_name(SHUTDOWN).unwrap().send_new(trajectory_msg).unwrap();

        debug!(" state_check : self.state {:?}", self.state);
    }

    //* MVP */
    fn track_reference_keyframe(&mut self, map_actor: &Aid) -> Result<bool, Box<dyn std::error::Error>> {
        // Tracking::TrackReferenceKeyFrame()
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        self.current_frame.compute_bow();
        let mut vp_mappoint_matches = HashMap::<u32, (Id, bool)>::new();
        let nmatches;
        {
            let map_read_lock = self.map.read();
            let ref_kf = map_read_lock.get_keyframe(&self.ref_kf_id.unwrap()).unwrap();
            let mut increase_found = Vec::new();

            match orbmatcher::search_by_bow_f(ref_kf, &mut self.current_frame,true, 0.7) {
                Ok(matches) => {
                    nmatches = matches.len();
                    debug!("search_by_bow_f : num_matches {}", nmatches);

                    self.current_frame.clear_mappoints();
                    for (index, mp_id) in matches {
                        self.current_frame.add_mappoint(index, mp_id, false);
                        vp_mappoint_matches.insert(index, (mp_id, false)); // adding only the matches from orbmatcher bow search
                        increase_found.push((mp_id, 1));
                    }

                    let map_msg = MapWriteMsg::increase_found(increase_found);
                    map_actor.send_new(map_msg).unwrap();
                    // TODO (MVP_Done): Pranay :map needs to be updated with kf_match_edits after calling this!!!
                },
                Err(err) => panic!("Problem with search_by_bow_f {}", err)
            }
        }
        if nmatches < 15 {
            warn!("tracking_backend::track_reference_keyframe;Less than 15 matches = {}!!\n", nmatches);
            return Ok(false);
        }

        self.current_frame.mappoint_matches = vp_mappoint_matches.clone();
        self.current_frame.pose = Some(self.last_frame.as_ref().unwrap().pose.unwrap());

        if let Some((_, pose)) = optimizer::optimize_pose(&mut self.current_frame, &self.map) {
            self.current_frame.pose = Some(pose);
        }

        // Discard outliers
        let deleted_mps = self.current_frame.discard_outliers();

        let nmatches_map = -deleted_mps + self.current_frame.get_num_mappoints_with_observations(&*self.map.read());

        info!("nmatches_map {}, mappoints_with_obs {}",nmatches_map, self.current_frame.get_num_mappoints_with_observations(&*self.map.read()));
        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn track_with_motion_model(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
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
        self.update_last_frame();

        let enough_frames_to_reset_imu = self.current_frame.id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32);
        if self.map.read().imu_initialized && enough_frames_to_reset_imu {
            // Predict state with IMU if it is initialized and it doesnt need reset
            self.imu.predict_state();
            return Ok(true);
        } else {
            self.current_frame.pose = Some(self.imu.velocity.unwrap() * self.last_frame.as_ref().unwrap().pose.unwrap());
        }

        self.current_frame.clear_mappoints();

        // Project points seen in previous frame
        let th = match self.sensor.frame() {
            FrameSensor::Mono => 15,
            _ => 7
        };

        let mut matches = orbmatcher::search_by_projection_with_threshold(
            &mut self.current_frame, self.last_frame.as_ref().unwrap(),
            th,
            self.sensor.is_mono(),
            0.6,
            &self.track_in_view,
            &self.track_in_view_r,
            &self.map,
            self.sensor
        )?;

        // If few matches, uses a wider window search
        if matches < 20 {
            info!("tracking_backend::track_with_motion_model;not enough matches, wider window search");
            self.current_frame.clear_mappoints();
            matches = orbmatcher::search_by_projection_with_threshold(
                &mut self.current_frame,
                self.last_frame.as_ref().unwrap(),
                2 * th,
                self.sensor.is_mono(),
                0.6,
                &self.track_in_view,
                &self.track_in_view_r,
                &self.map,
                self.sensor
            )?;
        }

        if matches < 20 {
            warn!("tracking_backend::track_with_motion_model;not enough matches!!");
            return Ok(self.sensor.is_imu());
        }

        // Optimize frame pose with all matches
        optimizer::optimize_pose(&mut self.current_frame, &self.map)
            .map(|(_,pose)| self.current_frame.pose = Some(pose) );

        // Discard outliers
        let deleted_mps = self.current_frame.discard_outliers();
        let nmatches_map = -deleted_mps + self.current_frame.get_num_mappoints_with_observations(&*self.map.read());

        if self.localization_only_mode {
            // TODO (localization-only)
            // mbVO = nmatchesMap<10;
            // return nmatches>20;
        }

        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn search_local_points(&mut self) {
        //void Tracking::SearchLocalPoints()
        // Sofiya: not sure what the point of this function is?
        let mut mps_to_increase_visible = Vec::new();

        // Do not search map points already matched
        let mut seen_this_frame = HashSet::<Id>::new();
        for (_, (id, _)) in &self.current_frame.mappoint_matches {
            mps_to_increase_visible.push(id);
            seen_this_frame.insert(*id); 
        }

        // Project points in frame and check its visibility
        let mut to_match = 0;
        for mp_id in &self.local_mappoints {
            match seen_this_frame.get(mp_id) {
                Some(_) => continue,
                None => {}
            }
            // Project (this fills MapPoint variables for matching)
            let map_read_lock = self.map.read();
            let (tracked_data_left, tracked_data_right) = self.current_frame.is_in_frustum(*mp_id, 0.5, &*map_read_lock);
            if tracked_data_left.is_some() || tracked_data_right.is_some() {
                mps_to_increase_visible.push(&mp_id);
                to_match += 1;
            }
            if let Some(d) = tracked_data_left {
                self.track_in_view.insert(*mp_id, d);
            }
            if let Some(d) = tracked_data_right {
                self.track_in_view_r.insert(*mp_id, d);
            }
            // TODO 10/17 .. framedrawer thing
            // self.current_frame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }

        if to_match > 0 {
            let mut th = match self.sensor.frame() {
                FrameSensor::Rgbd => 3,
                _ => 1
            };
            // TODO (IMU)
            if self.map.read().imu_initialized {
                if self.map.read().imu_ba2 {
                    th = 2;
                } else {
                    th = 6;
                }
            } else if !self.map.read().imu_initialized && self.sensor.is_imu() {
                th = 10;
            }

            // If the camera has been relocalised recently, perform a coarser search
            if self.current_frame.id < self.relocalization.last_reloc_frame_id + 2 {
                th = 5;
            }
            match self.state {
                TrackingState::RecentlyLost | TrackingState::Lost => th = 15,
                _ => {}
            }

            // TODO (MVP) send mps_to_increase_visible to map

            let matches = orbmatcher::search_by_projection(
                &mut self.current_frame,
                &self.local_mappoints,
                th, true, 0.8,
                &self.track_in_view, &self.track_in_view_r,
                &self.map
            );
        }
    }

    fn track_local_map(&mut self, map_actor: &Aid) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2949
        // This is for visualization
        // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        self.update_local_keyframes();
        self.update_local_points();
        self.search_local_points();

        info!(" {{{{optimize_pose}}}}");
        let mut inliers = 0;
        if !self.map.read().imu_initialized || (self.current_frame.id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32)) {
            optimizer::optimize_pose(&mut self.current_frame, &self.map)
                .map(|(_,pose)| self.current_frame.pose = Some(pose) );
        } else if !self.map_updated {
            let _inliers = optimizer::pose_inertial_optimization_last_frame(&mut self.current_frame, &self.map);
        } else {
            let _inliers = optimizer::pose_inertial_optimization_last_keyframe(&mut self.current_frame);
        }

        self.matches_in_frame = 0;
        // Update MapPoints Statistics
        let mut increase_found = Vec::new();
        for (index, (mp_id, _)) in &self.current_frame.mappoint_matches {
            if !self.current_frame.is_mp_outlier(&index) {
                increase_found.push((*mp_id, 1));

                if self.localization_only_mode {
                    let map_read_lock = self.map.read();
                    let mappoint = map_read_lock.get_mappoint(&mp_id).unwrap();

                    if mappoint.get_observations().len() > 0 {
                        self.matches_in_frame+=1;
                    }
                } else {
                    self.matches_in_frame += 1;
                }

            } else if !self.sensor.is_mono() {
                //TODO (stereo)
                //mCurrentFrame.mappoint_matches[i] = static_cast<MapPoint*>(NULL);
                // Following probably needs to be inside map actor:
                // self.current_frame.as_mut().unwrap().mappoint_matches.remove(&index);
                // But why delete these points at all? and why here?
            }
        }

        let map_msg = MapWriteMsg::increase_found(increase_found);
        map_actor.send_new(map_msg).unwrap();

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if self.current_frame.id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32) && self.matches_in_frame<50 {
            warn!("track_local_map unsuccessful; matches in frame < 50 : {}",self.matches_in_frame);
            return false;
        }

        match self.state {
            TrackingState::RecentlyLost => { return self.matches_in_frame > 10; },
            _ => {}
        }

        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => { 
                return !(self.matches_in_frame<15 && self.map.read().imu_initialized) && !(self.matches_in_frame<50 && !self.map.read().imu_initialized)
            },
            Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => {
                return self.matches_in_frame >= 15;
            },
            _ => { return self.matches_in_frame > 30 }
        }
    }

    fn update_last_frame(&mut self) {
        // Tracking::UpdateLastFrame()

        // Update pose according to reference keyframe
        let last_saved_pose = self.trajectory_poses.last().unwrap();
        let reference_kf_pose;
        {
            let map_lock = self.map.read();
            let reference_kf = self.last_frame.as_ref().unwrap().ref_kf_id;
            match reference_kf {
                Some(_id) => {
                    reference_kf_pose = map_lock.get_keyframe(&reference_kf.unwrap()).unwrap().pose;
                },
                None => { return; }
            }
        }
        self.last_frame.as_mut().unwrap().pose = Some(reference_kf_pose);

        if self.sensor.is_mono() || self.frames_since_last_kf == 0 {
            return;
        }

        {
            // TODO (RGBD) TODO (Stereo)
            // the code below in rust is good but needs to be fixed for the compiler
            // rest of the C++ code isn't implemented yet

            // Create "visual odometry" MapPoints
            // We sort points according to their measured depth by the stereo/RGB-D sensor
            // let depth_idx = Vec::new();
            // let num_features = match self.last_frame.unwrap().Nleft {
            //     0 => self.last_frame.unwrap().N,
            //     _ => self.last_frame.unwrap().Nleft
            // };

            // for i in 0..num_features {
            //     let z = self.last_frame.unwrap().mv_depth.get(i);
            //     if z > 0 {
            //         depth_idx.push((z,i));
            //     }
            // }

            // if depth_idx.is_empty() {
            //     return;
            // }


            // sort(vDepthIdx.begin(),vDepthIdx.end());

            // We insert all close points (depth<mThDepth)
            // If less than 100 close points, we insert the 100 closest ones.
            // int nPoints = 0;
            // for(size_t j=0; j<vDepthIdx.size();j++)
            // {
            //     int i = vDepthIdx[j].second;

            //     bool bCreateNew = false;

            //     MapPoint* pMP = mLastFrame.mappoint_matches[i];

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
            //         mLastFrame.mappoint_matches[i]=pNewMP;

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

    fn update_local_keyframes(&mut self) {
        // void Tracking::UpdateLocalKeyFrames()

        // Each map point votes for the keyframes in which it has been observed
        let mut kf_counter = HashMap::<Id, i32>::new();
        let frame = match !self.map.read().imu_initialized || self.current_frame.id < self.relocalization.last_reloc_frame_id + 2 {
            true => &self.current_frame,
            false => self.last_frame.as_ref().unwrap() // Using lastframe since current frame has no matches yet
        };
        for (_, (mp_id, _)) in &frame.mappoint_matches {
            let map_read_lock = self.map.read();
            let mp = map_read_lock.get_mappoint(&mp_id).unwrap();
            for kf_id in mp.get_observations().keys() {
                *kf_counter.entry(*kf_id).or_insert(0) += 1;
            }
        }

        let (mut max, mut max_kf_id) = (0, 0);
        let mut new_local_keyframes = HashSet::<Id>::new();
        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (kf_id, count) in kf_counter {
            if count > max {
                max = count;
                max_kf_id = kf_id;
            }
            new_local_keyframes.insert(kf_id);
        }

        // Also include some keyframes that are neighbors to already-included keyframes
        fn get_first_new(local_keyframes: &HashSet<Id>, kf_ids: &Vec<Id>) -> Id {
            for kf_id in kf_ids {
                if local_keyframes.get(kf_id).is_none() { return *kf_id; }
            }
            -1 
        }
        let mut neighbors_to_add = HashSet::<Id>::new();
        for kf_id in new_local_keyframes.iter() {
            // Limit the number of keyframes
            if new_local_keyframes.len() + neighbors_to_add.len() > 80 { break; }

            let map_read_lock = self.map.read();
            let kf = map_read_lock.get_keyframe(&kf_id).unwrap();

            neighbors_to_add.insert(
                get_first_new(&self.local_keyframes, &kf.get_connections(10))
            );
            neighbors_to_add.insert(
                get_first_new(&self.local_keyframes, &kf.full_kf_info.children)
            );

            kf.full_kf_info.parent.map(|parent_id| {
                if self.local_keyframes.get(&parent_id).is_none() {
                    neighbors_to_add.insert(parent_id);
                }
            });
        }
        new_local_keyframes.extend(neighbors_to_add); // TODO (MVP) I'm pretty sure I have to add these in the keyframe

        // Add 10 last temporal KFs (mainly for IMU)
        if self.sensor.is_imu() && self.local_keyframes.len() < 80 {
            // TODO (IMU)
            // For the last 20 created keyframes, save into new_local_keyframes
            // until one of them was not in self.local_keyframes. Then, quit.
        }

        if max_kf_id != 0 {
            self.current_frame.ref_kf_id = Some(max_kf_id);
            self.ref_kf_id = Some(max_kf_id);
        }

        self.local_keyframes.clear();
        self.local_keyframes = new_local_keyframes;
    }

    fn update_local_points(&mut self) {
        // void Tracking::UpdateLocalPoints()
        let mut new_mappoints = HashSet::<Id>::new();

        // Note: a way to get around using mnTrackReferenceForFrame inside each KF
        // Basically, ORBSLAM uses these variables on a kf/mp to keep track of the 
        // frame that last saved it in local_keyframes or local_mappoints..
        // If it was last saved by this frame, they remove it from local_mappoints.
        // So, just directly look up if it was saved in local_mappoints instead.
        for kf_id in &self.local_keyframes {
            let map_read_lock = self.map.read();
            if map_read_lock.get_keyframe(kf_id).is_some()
            {
                let mp_ids = &map_read_lock.get_keyframe(kf_id).unwrap().mappoint_matches;
                for (_, (mp_id, _)) in mp_ids {
                    if self.local_mappoints.get(mp_id).is_none() {
                        new_mappoints.insert(*mp_id);
                    }
                }
            }

        }

        self.local_mappoints.clear();
        self.local_mappoints = new_mappoints;
    }

    fn create_new_keyframe(&mut self, context: & axiom::prelude::Context) {
        //CreateNewKeyFrame
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216

        let new_kf = KeyFrame::new(&self.current_frame);

        // TODO (IMU) Reset preintegration from last KF (Create new object)
        // if sensor.is_imu() {
        //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
        // }

        // TODO (IMU)
        // if !sensor.is_mono() {
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
        // }

        // mnLastKeyFrameId = mCurrentFrame.mnId;
        // mpLastKeyFrame = pKF;

        self.last_kf_ts = Some(new_kf.timestamp);

        // KeyFrame created here and sent to LM, but not inserted into map yet
        let localmapping = context.system.find_aid_by_name(LOCAL_MAPPING).unwrap();
        let actor_msg = GLOBAL_PARAMS.get::<String>(LOCAL_MAPPING, "actor_message");
        match actor_msg.as_ref() {
            "KeyFrameMsg" => {
                localmapping.send_new(KeyFrameMsg{ keyframe: new_kf }).unwrap();
            },
            _ => {
                warn!("tracking_backend::create_new_keyframe;invalid message type, selecting KeyFrameMsg");
                localmapping.send_new(KeyFrameMsg{ keyframe: new_kf }).unwrap();
            },
        }
    }

    fn need_new_keyframe(&self) -> bool {
        let kfs_in_map = self.kfs_in_map();
        let not_enough_frames_since_last_reloc = (self.current_frame.id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32)) && (kfs_in_map > (self.max_frames as u32));
        let imu_not_initialized = self.sensor.is_imu() && !self.map.read().imu_initialized;

        let too_close_to_last_kf = match self.last_kf_ts {
            Some(timestamp) => self.current_frame.timestamp - timestamp < Duration::milliseconds(250),
            None => false
        };

        if self.localization_only_mode || not_enough_frames_since_last_reloc || (imu_not_initialized && too_close_to_last_kf) {
            return false;
        } else if imu_not_initialized && !too_close_to_last_kf {
            return true;
        }

        // Tracked MapPoints in the reference keyframe
        let min_observations = match kfs_in_map <= 2 {
            true => 2,
            false => 3
        };

        let map_lock = self.map.read();
        let tracked_mappoints = map_lock.get_keyframe(&self.ref_kf_id.unwrap())
            .map(|kf| kf.tracked_mappoints(&*map_lock, min_observations) as f32)
            .unwrap_or(0.0);

        // Check how many "close" points are being tracked and how many could be potentially created.
        let (tracked_close, non_tracked_close) = self.current_frame.check_close_tracked_mappoints();
        let need_to_insert_close = (tracked_close<100) && (non_tracked_close>70);

        // Thresholds
        let th_ref_ratio = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::None) => 0.9,
            Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                // Points tracked from the local map
                if self.matches_in_frame > 350 { 0.75 } else { 0.90 }
            },
            Sensor(FrameSensor::Stereo, _) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => 0.75,
            Sensor(FrameSensor::Rgbd, ImuSensor::None) => { if kfs_in_map < 2 { 0.4 } else { 0.75 } }
        }; // Sofiya: is Rgbd right? See ORBSLAM3 code https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3142

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        let c1a = self.frames_since_last_kf >= (self.max_frames as i32);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // Note: removed localmappingidle from here
        let c1b = self.frames_since_last_kf >= (self.min_frames as i32);
        //Condition 1c: tracking is weak
        let sensor_is_right = match self.sensor {
            Sensor(FrameSensor::Mono, _) | Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => false,
            _ => true
        }; // Sofiya: why would they just be selecting for RGBD or Stereo without IMU??
        let c1c = sensor_is_right && ((self.matches_in_frame as f32) < tracked_mappoints * 0.25 || need_to_insert_close) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        let c2 = (((self.matches_in_frame as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_in_frame > 15;

        // Temporal condition for Inertial cases
        let close_to_last_kf = match self.last_kf_ts {
            Some(timestamp) => self.current_frame.timestamp - timestamp < Duration::milliseconds(500),
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
        let c4 = ((self.matches_in_frame < 75 && self.matches_in_frame > 15) || recently_lost) && sensor_is_imumono;

        // Note: removed code here about checking for idle local mapping and/or interrupting bundle adjustment
        return ((c1a||c1b||c1c) && c2)||c3 ||c4;
    }

    //* Helper functions */
    fn kfs_in_map(&self) -> u32 {
        let map_read_lock = self.map.read();
        map_read_lock.num_keyframes() as u32
    }

    //* Next steps */
    fn create_new_map(&self) -> bool {
        todo!("Multimaps: Atlas::CreateNewMap");
    }
}
impl Function for DarvisTrackingBack {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        if let Some(msg) = message.content_as::<FeatureMsg>() {
            self.handle_message(context, msg)
        } else if let Some(msg) = message.content_as::<MapInitializedMsg>() {
            debug!("Received map initialized message {:?}", msg.curr_kf_pose);
            self.frames_since_last_kf = 0;
            self.local_keyframes.insert(msg.curr_kf_id);
            self.local_keyframes.insert(msg.ini_kf_id);
            self.local_mappoints = msg.local_mappoints.clone();
            self.ref_kf_id = Some(msg.curr_kf_id);
            self.current_frame.pose = Some(msg.curr_kf_pose);
            self.state = TrackingState::Ok;
            // Sofiya: ORBSLAM also sets lastkeyframeid, but I think we can get away without it

            context.system.find_aid_by_name("TRACKING_FRONTEND").unwrap().send_new(
                TrackingStateMsg { state: TrackingState::Ok }
            ).unwrap();
            context.system.find_aid_by_name("LOCAL_MAPPING").unwrap().send_new(
                InitialMapMsg { kf: msg.ini_kf_id }
            ).unwrap();
        } else if let Some(msg) = message.content_as::<KeyFrameIdMsg>() {
            self.current_frame.ref_kf_id = Some(msg.keyframe_id);
        }

        match self.state {
            TrackingState::Lost => self.last_frame = None,
            _ => self.last_frame = Some(self.current_frame.clone())
        }

        Ok(Status::done(()))
    }
}