use std::collections::{HashSet, HashMap};
use derivative::Derivative;
use log::{warn, info, debug, error};
use opencv::core::Point2f;
use dvcore::{lockwrap::ReadOnlyWrapper, config::*, sensor::{Sensor, FrameSensor, ImuSensor}, base::Actor};
use crate::{
    actors::messages::{FeatureMsg, MapInitializedMsg, TrajectoryMsg, TrackingStateMsg, KeyFrameIdMsg, LastKeyFrameUpdatedMsg, ShutdownMsg},
    actors::map_actor::MapWriteMsg,
    registered_actors::{LOCAL_MAPPING, TRACKING_BACKEND, SHUTDOWN_ACTOR, TRACKING_FRONTEND, VISUALIZER, MAP_ACTOR},
    dvmap::{
        map::Map, map::Id,
        keyframe::{Frame, InitialFrame, PrelimKeyFrame}, pose::Pose, mappoint::{MapPoint, FullMapPoint},
    },
    modules::{imu::ImuModule, optimizer::{self}, orbmatcher, map_initialization::Initialization, relocalization::Relocalization}, ActorChannels,
};


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
impl TrackedMapPointData {}

#[derive(Debug, Clone, Copy, Default)]
pub enum TrackingState {
    #[default] NotInitialized,
    Lost,
    RecentlyLost,
    WaitForMapResponse,
    Ok
}

#[derive(Debug, Derivative)]
#[derivative(Default)]
pub struct DarvisTrackingBack {
    actor_channels: ActorChannels,
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
    // current_frame: Frame<InitialFrame>,
    last_frame: Option<Frame<InitialFrame>>,

    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_timestamp: Option<u64>,

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
    map_updated : bool,  // TODO (design) I'm not sure we want to use this

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

    // Global defaults
    localization_only_mode: bool,
    frames_to_reset_imu: u32, //mnFramesToResetIMU
    insert_kfs_when_lost: bool,
    max_frames : i64 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
    min_frames: u32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
    sensor: Sensor,
}

impl Actor for DarvisTrackingBack {
    fn run(&mut self) {
        loop {
            let message = self.actor_channels.receive().unwrap();

            if let Some(msg) = message.downcast_ref::<FeatureMsg>() {
                // Regular tracking. Received from tracking frontend
                match self.tracking_backend(msg) {
                    Ok((frame, new_state)) => {
                        self.last_frame = Some(frame);
                        self.state = new_state;
                    },
                    Err(e) => {
                        panic!("Error in Tracking Backend: {}", e);
                    }
                };
            } else if let Some(msg) = message.downcast_ref::<MapInitializedMsg>() {
                // Map needs to be initialized before tracking can begin. Received from map actor
                self.frames_since_last_kf = 0;
                self.local_keyframes.insert(msg.curr_kf_id);
                self.local_keyframes.insert(msg.ini_kf_id);
                self.local_mappoints = msg.local_mappoints.clone();
                self.ref_kf_id = Some(msg.curr_kf_id);
                self.last_frame.as_mut().unwrap().pose = Some(msg.curr_kf_pose); 
                self.state = TrackingState::Ok;
                // Sofiya: ORBSLAM also sets lastkeyframeid, but I think we can get away without it

                self.actor_channels.find(TRACKING_FRONTEND).unwrap().send(Box::new(
                    TrackingStateMsg { 
                        state: TrackingState::Ok,
                        init_id: msg.ini_kf_id
                    }
                )).expect("Could not send to tracking frontend");
                self.actor_channels.find(LOCAL_MAPPING).unwrap().send(Box::new(
                    KeyFrameIdMsg { keyframe_id: msg.ini_kf_id }
                )).expect("Could not send to local mapping");
                self.actor_channels.find(LOCAL_MAPPING).unwrap().send(Box::new(
                    KeyFrameIdMsg { keyframe_id: msg.curr_kf_id }
                )).unwrap();

            } else if let Some(msg) = message.downcast_ref::<KeyFrameIdMsg>() {
                // Received from the map actor after it inserts a keyframe
                info!("Sending keyframe ID {} to local mapping", msg.keyframe_id);
                // TODO... do we need to set this?
                self.last_frame.as_mut().unwrap().ref_kf_id = Some(msg.keyframe_id);
                self.actor_channels.find(LOCAL_MAPPING).unwrap().send(Box::new(
                    KeyFrameIdMsg { keyframe_id: msg.keyframe_id }
                )).unwrap();

            } else if let Some(_) = message.downcast_ref::<LastKeyFrameUpdatedMsg>() {
                // Received from local mapping after it culls and creates new MPs for the last inserted KF
                self.state = TrackingState::Ok;
            } else if let Some(_) = message.downcast_ref::<ShutdownMsg>() {
                break;
            } else {
                warn!("Tracking backend received unknown message type!");
            }
        }
    }
}

impl DarvisTrackingBack {
    pub fn new(map: ReadOnlyWrapper<Map>, actor_channels: ActorChannels) -> DarvisTrackingBack {
        // Note: This is suuuuuch a specific bug. Default::default() calls the default function 
        // for every single item in the struct, even if it is explicitly overriden (such as
        // map, camera, sensor, global_defaults, and optimizer in the below code). This means that
        // a second map is created, DISCARDED, and then DarvisTrackingBack.map is set to the first map
        // that we wanted in the first place. So it looks like it duplicates the map, but it doesn't actually
        // and you shouldn't worry if you see this.
        DarvisTrackingBack {
            actor_channels,
            map,
            sensor: GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor"),
            initialization: Initialization::new(),
            localization_only_mode: GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "localization_only_mode"),
            frames_to_reset_imu: GLOBAL_PARAMS.get::<i32>(TRACKING_BACKEND, "frames_to_reset_IMU") as u32,
            insert_kfs_when_lost: GLOBAL_PARAMS.get::<bool>(TRACKING_BACKEND, "insert_KFs_when_lost"),
            max_frames: GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "fps") as i64,
            min_frames: 0,
            last_frame_id: -1,
            ..Default::default()
        }
    }


    fn tracking_backend(
        &mut self, msg: &FeatureMsg
    ) -> Result<(Frame::<InitialFrame>, TrackingState), Box<dyn std::error::Error>>  {
        self.last_frame_id += 1;
        let mut current_frame = Frame::<InitialFrame>::new(
            self.last_frame_id, 
            msg.keypoints.clone(), // TODO (msg copy)
            msg.descriptors.clone(), // TODO (msg copy)
            msg.image_width,
            msg.image_height,
            msg.timestamp
        )?;

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

        // TODO: update map change index. Used by mbMapUpdated
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1890

        // Initial estimation of camera pose and matching
        let mut new_state = match (self.localization_only_mode, self.state) {
            (true, _) => {
                todo!("Localization only");
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1933
                // Look for "mbOnlyTracking" in Track() function
            },
            (false, TrackingState::WaitForMapResponse) => {
                return Ok((current_frame, TrackingState::WaitForMapResponse));
            },
            (false, TrackingState::NotInitialized) => {
                match self.initialization.try_initialize(&current_frame)? {
                    true => {
                        let msg = match self.sensor.frame() {
                            FrameSensor::Mono => MapWriteMsg::create_initial_map_monocular(self.initialization.clone(), TRACKING_BACKEND),
                            _ => MapWriteMsg::create_initial_map_stereo(self.initialization.clone(), TRACKING_BACKEND)
                        };
                        self.actor_channels.find(MAP_ACTOR).unwrap().send(Box::new(msg))?;
                        return Ok((current_frame, TrackingState::WaitForMapResponse));
                    },
                    false => {
                        return Ok((current_frame, TrackingState::NotInitialized));
                    }
                }
            },
            (false, TrackingState::Ok) => {
                let can_use_motion_model = (self.imu.velocity.is_none() && !self.map.read().imu_initialized) || self.relocalization.frames_since_lost(&current_frame) < 2;
                let track_success = match can_use_motion_model {
                    true => self.track_reference_keyframe(&mut current_frame)?,
                    false => {
                        match self.track_with_motion_model(&mut current_frame)? {
                            true => true,
                            false => self.track_reference_keyframe(&mut current_frame)?
                        }
                    }
                };
                match track_success {
                    true => TrackingState::Ok,
                    false => {
                        self.relocalization.timestamp_lost = Some(current_frame.timestamp);
                        match self.kfs_in_map() > 10 {
                            true =>TrackingState::RecentlyLost,
                            false => TrackingState::Lost
                        }
                    },
                }
            },
            (false, TrackingState::RecentlyLost) => {
                warn!("State recently lost");
                let relocalize_success = match self.imu.ready(&self.map) {
                    true => {
                        let ok = self.imu.predict_state(); // TODO (IMU): I guess this should be used somewhere?
                        self.relocalization.past_cutoff(&current_frame)
                    },
                    false => {
                        !self.relocalization.run() && self.relocalization.sec_since_lost(&current_frame) > 3
                    }
                };
                match relocalize_success {
                    true => {
                        warn!("Relocalization unsuccessful...");
                        TrackingState::Lost
                    },
                    false => TrackingState::RecentlyLost
                }
            },
            (false, TrackingState::Lost) => {
                match self.kfs_in_map() < 10 {
                    true => {
                        info!("Reseting current map...");
                        self.actor_channels.find(MAP_ACTOR).unwrap().send(Box::new(MapWriteMsg::reset_active_map()))?;
                    },
                    false => {
                        info!("Creating new map...");
                        self.create_new_map();
                    }
                }

                return Ok((current_frame, TrackingState::Ok));
            }
        };

        // Only 3 valid states now
        // TrackingState::Ok | TrackingState::RecentlyLost | TrackingState::Lost

        // Track Local Map
        let (enough_matches, matches_in_frame) = self.track_local_map(&mut current_frame, &new_state);
        new_state = match (enough_matches, new_state) {
            (true, TrackingState::Ok) => {
                TrackingState::Ok
            },
            (true, TrackingState::RecentlyLost) | (false, TrackingState::Ok) => {
                if self.sensor.is_imu() {
                    info!("tracking_backend::handle_message;Track lost for less than 1 second");
                    todo!("IMU, Reset... Reset map because local mapper set the bad imu flag");
                    // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2149
                }
                self.relocalization.timestamp_lost = Some(current_frame.timestamp);
                TrackingState::RecentlyLost
            },
            (_, TrackingState::Lost) => {
                // Reset if the camera get lost soon after initialization
                if self.kfs_in_map() <= 10  || (self.sensor.is_imu() && !self.map.read().imu_initialized) {
                    warn!("tracking_backend::handle_message;Track lost before IMU initialisation, resetting...",);
                    let map_msg = MapWriteMsg::reset_active_map();
                    self.actor_channels.find(MAP_ACTOR).unwrap().send(Box::new(map_msg))?;
                } else {
                    self.create_new_map();
                }
                return Ok((current_frame, TrackingState::Lost));
            }
            _ => {
                return Ok((current_frame, TrackingState::RecentlyLost));
            }
        };

        if self.sensor.is_imu() {
            todo!("IMU");
            // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
        }

        match new_state {
            TrackingState::Ok | TrackingState::RecentlyLost => {
                // Update motion model
                let last_frame = self.last_frame.as_ref();
                if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !current_frame.pose.is_none() {
                    let last_pose = last_frame.expect("No last frame in tracking?").pose.as_ref().expect("Can't get last frame's pose?");
                    let last_twc = last_pose.inverse();
                    self.imu.velocity = Some(current_frame.pose.as_ref().expect("Can't get current frame?").clone() * last_twc);
                } else {
                    self.imu.velocity = None;
                }

                current_frame.delete_mappoints_without_observations(&self.map.read());
                self.temporal_points.clear();

                // Check if we need to insert a new keyframe
                let insert_if_lost_anyway = self.insert_kfs_when_lost && matches!(new_state, TrackingState::RecentlyLost) && self.sensor.is_imu();
                if self.need_new_keyframe(&current_frame, matches_in_frame, &new_state) && insert_if_lost_anyway {
                    self.create_new_keyframe(&current_frame);
                }

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame. Only has effect if lastframe is tracked
                let _ = current_frame.discard_outliers();

                if current_frame.ref_kf_id.is_none() {
                    current_frame.ref_kf_id = self.ref_kf_id;
                }

                let current_pose = current_frame.pose.expect(concat!("Could not parse '", stringify!($new_state), "'"));
                let ref_kf_id = current_frame.ref_kf_id.expect("Current frame has pose but ref kf is still none.Probably a concurrency issue with map actor sending pose info back to here.");

                // TODO (vis): Why is this not just current_pose?
                let new_pose = current_pose * self.map.read().get_keyframe(&ref_kf_id).expect("Can't get ref kf").pose.expect("Can't get ref kf pose").inverse();
                self.trajectory_poses.push(new_pose);

                let traj_msg = TrajectoryMsg::new(
                    new_pose,
                    ref_kf_id,
                    current_frame.timestamp
                );
                self.actor_channels.find(SHUTDOWN_ACTOR).unwrap().send(Box::new(traj_msg.clone()))?;

                if let Some(vis) = self.actor_channels.find(VISUALIZER) {
                    vis.send(Box::new(traj_msg))?;
                }
            },
            TrackingState::NotInitialized |  TrackingState::WaitForMapResponse | TrackingState::Lost => {
                return Err("Should have handled these cases by this point!".into());
            }
        }

        Ok((current_frame, new_state))
    }

    //* MVP */
    fn track_reference_keyframe(&mut self, current_frame: &mut Frame<InitialFrame>) -> Result<bool, Box<dyn std::error::Error>> {
        // Tracking::TrackReferenceKeyFrame()
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        current_frame.compute_bow();
        let mut vp_mappoint_matches = HashMap::<u32, (Id, bool)>::new();
        let nmatches;
        {
            let map_read_lock = self.map.read();
            let ref_kf = map_read_lock.get_keyframe(&self.ref_kf_id.unwrap()).unwrap();
            let mut increase_found = Vec::new();

            match orbmatcher::search_by_bow_f(ref_kf, current_frame,true, 0.7) {
                Ok(matches) => {
                    nmatches = matches.len();

                    current_frame.clear_mappoints();
                    for (index, mp_id) in matches {
                        // debug!("Adding mappoint in trk");
                        current_frame.add_mappoint(index, mp_id, false);
                        vp_mappoint_matches.insert(index, (mp_id, false)); // adding only the matches from orbmatcher bow search
                        increase_found.push((mp_id, 1));
                    }

                    let map_msg = MapWriteMsg::increase_found(increase_found);
                    self.actor_channels.find(MAP_ACTOR).unwrap().send(Box::new(map_msg))?;
                },
                Err(err) => panic!("Problem with search_by_bow_f {}", err)
            }
        }
        if nmatches < 15 {
            warn!("track_reference_keyframe;Fewer than 15 matches = {}!!\n", nmatches);
            return Ok(false);
        }

        current_frame.pose = Some(self.last_frame.as_ref().unwrap().pose.unwrap());

        if let Some((_, pose)) = optimizer::optimize_pose(current_frame, &self.map) {
            current_frame.pose = Some(pose);
        }

        // Discard outliers
        let _ = current_frame.discard_outliers();
        let mappoints_with_obs = current_frame.get_num_mappoints_with_observations(&*self.map.read());
        let nmatches_map = mappoints_with_obs;

        debug!("TRACK REFERENCE KEYFRAME matches {}", nmatches_map);

        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn track_with_motion_model(&mut self, current_frame: &mut Frame<InitialFrame>) -> Result<bool, Box<dyn std::error::Error>> {
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

        let enough_frames_to_reset_imu = current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32);
        if self.map.read().imu_initialized && enough_frames_to_reset_imu {
            // Predict state with IMU if it is initialized and it doesnt need reset
            self.imu.predict_state();
            return Ok(true);
        } else {
            current_frame.pose = Some(self.imu.velocity.unwrap() * self.last_frame.as_ref().unwrap().pose.unwrap());
        }

        current_frame.clear_mappoints();

        // Project points seen in previous frame
        let th = match self.sensor.frame() {
            FrameSensor::Mono => 15,
            _ => 7
        };

        let mut matches = orbmatcher::search_by_projection_with_threshold(
            current_frame,
            self.last_frame.as_ref().unwrap(),
            th,
            self.sensor.is_mono(),
            &self.map,
            self.sensor
        )?;
        debug!("MOTION MODEL initial matches {}", matches);

        // If few matches, uses a wider window search
        if matches < 20 {
            info!("tracking_backend::track_with_motion_model;not enough matches, wider window search");
            current_frame.clear_mappoints();
            matches = orbmatcher::search_by_projection_with_threshold(
                current_frame,
                self.last_frame.as_ref().unwrap(),
                2 * th,
                self.sensor.is_mono(),
                &self.map,
                self.sensor
            )?;
        }

        if matches < 20 {
            warn!("tracking_backend::track_with_motion_model;not enough matches!!");
            return Ok(self.sensor.is_imu());
        }

        // Optimize frame pose with all matches
        optimizer::optimize_pose(current_frame, &self.map)
            .map(|(_,pose)| current_frame.pose = Some(pose) );

        // Discard outliers
        let _ = current_frame.discard_outliers();
        let nmatches_map = current_frame.get_num_mappoints_with_observations(&*self.map.read());

        if self.localization_only_mode {
            todo!("Localization only");
            // mbVO = nmatchesMap<10;
            // return nmatches>20;
        }

        debug!("TRACK MOTION MODEL matches {}", nmatches_map);
        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn update_last_frame(&mut self) {
        // Tracking::UpdateLastFrame()

        // Update pose according to reference keyframe
        let reference_kf_pose;
        {
            let map_lock = self.map.read();
            match self.last_frame.as_ref().unwrap().ref_kf_id {
                Some(id) => {
                    reference_kf_pose = map_lock.get_keyframe(&id).unwrap().pose;
                    if self.trajectory_poses.last().is_some() && reference_kf_pose.is_some() {
                        self.last_frame.as_mut().unwrap().pose = Some(*self.trajectory_poses.last().unwrap() * reference_kf_pose.unwrap());
                    } else {
                        warn!("Following should not be empty... last trajectory pose: {:?}, reference_kf_pose: {:?}", self.trajectory_poses.last(), reference_kf_pose);
                    }
                },
                None => { 
                    warn!("Last frame reference KF is none, should only be true when first creating map");
                }
            }
        }

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

    fn track_local_map(&mut self, current_frame: &mut Frame<InitialFrame>, current_state: &TrackingState) -> (bool, i32) {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2949
        // This is for visualization
        // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        self.update_local_keyframes(current_frame);
        self.update_local_points();
        self.search_local_points(current_frame, current_state);

        if !self.map.read().imu_initialized || (current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32)) {
            optimizer::optimize_pose(current_frame, &self.map)
                .map(|(_,pose)| current_frame.pose = Some(pose) );
        } else if !self.map_updated {
            let inliers = optimizer::pose_inertial_optimization_last_frame(current_frame, &self.map);
        } else {
            let inliers = optimizer::pose_inertial_optimization_last_keyframe(current_frame);
        }

        let mut matches_in_frame = 0;
        // Update MapPoints Statistics
        let mut increase_found = Vec::new();
        for (index, (mp_id, _)) in &current_frame.mappoint_matches {
            if !current_frame.is_mp_outlier(&index) {
                increase_found.push((*mp_id, 1));

                if self.localization_only_mode {
                    let map_read_lock = self.map.read();
                    let mappoint = map_read_lock.get_mappoint(&mp_id).unwrap();

                    if mappoint.get_observations().len() > 0 {
                        matches_in_frame+=1;
                    }
                } else {
                    matches_in_frame += 1;
                }

            } else if !self.sensor.is_mono() {
                todo!("Stereo");
                //mCurrentFrame.mappoint_matches[i] = static_cast<MapPoint*>(NULL);
                // Following probably needs to be inside map actor:
                // current_frame.as_mut().unwrap().mappoint_matches.remove(&index);
                // But why delete these points at all? and why here?
            }
        }

        debug!("TRACK LOCAL MAP matches {}", matches_in_frame);
        let map_msg = MapWriteMsg::increase_found(increase_found);
        self.actor_channels.find(MAP_ACTOR).unwrap().send(Box::new(map_msg)).unwrap();

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32) && matches_in_frame<50 {
            warn!("track_local_map unsuccessful; matches in frame < 50 : {}",matches_in_frame);
            return (false, matches_in_frame);
        }

        match current_state {
            TrackingState::RecentlyLost => { return (matches_in_frame > 10, matches_in_frame); },
            _ => {}
        }

        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => { 
                return (
                    !(matches_in_frame<15 && self.map.read().imu_initialized) && !(matches_in_frame<50 && !self.map.read().imu_initialized),
                    matches_in_frame
                )
            },
            Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => {
                return (matches_in_frame >= 15, matches_in_frame);
            },
            _ => { return (matches_in_frame > 30, matches_in_frame) }
        }
    }

    fn update_local_keyframes(&mut self, current_frame: &mut Frame<InitialFrame>) {
        // void Tracking::UpdateLocalKeyFrames()

        // Each map point votes for the keyframes in which it has been observed
        let mut kf_counter = HashMap::<Id, i32>::new();
        let frame = match !self.map.read().imu_initialized || current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
            true => &current_frame,
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
        let mut neighbors_to_add = HashSet::<Id>::new();
        for kf_id in new_local_keyframes.iter() {
            // Limit the number of keyframes
            if new_local_keyframes.len() + neighbors_to_add.len() > 80 { break; }

            let map_read_lock = self.map.read();
            let kf = map_read_lock.get_keyframe(&kf_id).unwrap();

            for kf_id in &kf.get_connections(10) {
                if self.local_keyframes.get(kf_id).is_none() { 
                    neighbors_to_add.insert(*kf_id);
                    break;
                }
            }
            for kf_id in &kf.full_kf_info.children {
                if self.local_keyframes.get(kf_id).is_none() { 
                    neighbors_to_add.insert(*kf_id);
                    break;
                }
            }

            kf.full_kf_info.parent.map(|parent_id| {
                if self.local_keyframes.get(&parent_id).is_none() {
                    neighbors_to_add.insert(parent_id);
                }
            });
        }
        new_local_keyframes.extend(neighbors_to_add); // TODO (MVP) I'm pretty sure I have to add these in the keyframe

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
            if let Some(kf) = map_read_lock.get_keyframe(kf_id) {
                let mp_ids = &kf.mappoint_matches;
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

    fn search_local_points(&mut self, current_frame: &mut Frame<InitialFrame>, current_state: &TrackingState) {
        //void Tracking::SearchLocalPoints()
        // Sofiya: not sure what the point of this function is?
        let mut mps_to_increase_visible = Vec::new();

        // Do not search map points already matched
        let mut seen_this_frame = HashSet::<Id>::new();
        for (_, (id, _)) in &current_frame.mappoint_matches {
            mps_to_increase_visible.push(*id);
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
            let (tracked_data_left, tracked_data_right) = current_frame.is_in_frustum(*mp_id, 0.5, &*map_read_lock);
            if tracked_data_left.is_some() || tracked_data_right.is_some() {
                mps_to_increase_visible.push(*mp_id);
                to_match += 1;
            }
            if let Some(d) = tracked_data_left {
                self.track_in_view.insert(*mp_id, d);
            }
            if let Some(d) = tracked_data_right {
                self.track_in_view_r.insert(*mp_id, d);
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
                th = 10;
            }

            // If the camera has been relocalised recently, perform a coarser search
            if current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
                th = 5;
            }
            match current_state {
                TrackingState::RecentlyLost | TrackingState::Lost => th = 15,
                _ => {}
            }

            self.actor_channels.find(MAP_ACTOR).unwrap().send(Box::new(
                MapWriteMsg::increase_visible(mps_to_increase_visible
            ))).unwrap();

            let matches = orbmatcher::search_by_projection(
                current_frame,
                &self.local_mappoints,
                th, 0.8,
                &self.track_in_view, &self.track_in_view_r,
                &self.map, self.sensor
            );
        }
    }

    fn create_new_keyframe(&mut self, current_frame: &Frame<InitialFrame>) {
        //CreateNewKeyFrame
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216

        let new_kf = Frame::<PrelimKeyFrame>::new(&current_frame);

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
        let map_actor = self.actor_channels.find(MAP_ACTOR).unwrap();
        map_actor.send(Box::new(MapWriteMsg::new_keyframe(new_kf, TRACKING_BACKEND))).unwrap();
    }

    fn need_new_keyframe(&self, current_frame: &Frame<InitialFrame>, matches_in_frame: i32, current_state: &TrackingState) -> bool {
        let kfs_in_map = self.kfs_in_map();
        let not_enough_frames_since_last_reloc = (current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32)) && (kfs_in_map > (self.max_frames as u32));
        let imu_not_initialized = self.sensor.is_imu() && !self.map.read().imu_initialized;

        let too_close_to_last_kf = match self.last_kf_timestamp {
            Some(timestamp) => current_frame.timestamp - timestamp < 0.25 as u64, // 250 milliseconds
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
        let (tracked_close, non_tracked_close) = current_frame.check_close_tracked_mappoints();
        let need_to_insert_close = (tracked_close<100) && (non_tracked_close>70);

        // Thresholds
        let th_ref_ratio = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::None) => 0.9,
            Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                // Points tracked from the local map
                if matches_in_frame > 350 { 0.75 } else { 0.90 }
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
        let c1c = sensor_is_right && ((matches_in_frame as f32) < tracked_mappoints * 0.25 || need_to_insert_close) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        let c2 = (((matches_in_frame as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && matches_in_frame > 15;

        // Temporal condition for Inertial cases
        let close_to_last_kf = match self.last_kf_timestamp {
            Some(timestamp) => current_frame.timestamp - timestamp < 0.5 as u64, // 500 milliseconds
            None => false
        };

        let c3 = self.sensor.is_imu() && close_to_last_kf;

        let recently_lost = match current_state {
            TrackingState::RecentlyLost => true,
            _ => false
        };
        let sensor_is_imumono = match self.sensor {
            Sensor(FrameSensor::Rgbd, ImuSensor::Some) => true,
            _ => false
        };
        let c4 = ((matches_in_frame < 75 && matches_in_frame > 15) || recently_lost) && sensor_is_imumono;

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
