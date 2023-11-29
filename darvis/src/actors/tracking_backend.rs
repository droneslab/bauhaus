use std::collections::{HashSet, HashMap};
use derivative::Derivative;
use log::{warn, info, debug, error, trace};
use dvcore::{maplock::ReadOnlyMap, config::*, sensor::{Sensor, FrameSensor, ImuSensor}, actor::{Actor, ActorMessage}, matrix::{DVVectorOfKeyPoint, DVMatrix}};
use crate::{
    actors::{messages::{ShutdownMsg}, shutdown::ShutdownActorMsg},
    actors::{map_actor::MapWriteMsg, visualizer::VisualizerMsg},
    registered_actors::{LOCAL_MAPPING, TRACKING_BACKEND, SHUTDOWN_ACTOR, TRACKING_FRONTEND, VISUALIZER, MAP_ACTOR},
    dvmap::{
        map::Map, map::Id,
        keyframe::{Frame, InitialFrame, PrelimKeyFrame}, pose::DVPose, mappoint::{MapPoint, FullMapPoint}, misc::Timestamp,
    },
    modules::{imu::ImuModule, optimizer::{self}, orbmatcher, map_initialization::Initialization, relocalization::Relocalization}, ActorChannels,
};

use super::{tracking_frontend::TrackingFrontendMsg, local_mapping::LocalMappingMsg};


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
    map: ReadOnlyMap<Map>,
    initialization: Initialization, // data sent to map actor to initialize new map

    // Frames
    last_frame_id: Id,
    current_frame: Frame<InitialFrame>,
    last_frame: Option<Frame<InitialFrame>>,

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

    // Idk where to put these
    map_updated : bool,  // TODO (design) I'm not sure we want to use this

    // Poses in trajectory
    trajectory_poses: Vec<DVPose>, //mlRelativeFramePoses

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
        'outer: loop {
            let message = self.actor_channels.receive().unwrap();

            if message.is::<TrackingBackendMsg>() {
                let msg = message.downcast::<TrackingBackendMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));
                match *msg {
                    TrackingBackendMsg::FeatureMsg { keypoints, descriptors, image_width, image_height, timestamp, frame_id } => {
                        // Regular tracking. Received from tracking frontend
                        let now = std::time::Instant::now();
                        debug!("Tracking working on frame {}", frame_id);
                        self.last_frame_id += 1;
                        self.current_frame = Frame::<InitialFrame>::new(
                            self.last_frame_id, 
                            keypoints,
                            descriptors,
                            image_width,
                            image_height,
                            timestamp
                        ).expect("Could not create frame!");
                        match self.track() {
                            Ok(()) => {
                                if self.current_frame.ref_kf_id.is_none() {
                                    self.current_frame.ref_kf_id = self.ref_kf_id;
                                }
                                self.last_frame = Some(self.current_frame.clone()); // TODO (clone)

                                match self.state {
                                    TrackingState::Ok | TrackingState::RecentlyLost => {
                                        self.update_trajectory_in_logs().expect("Could not save trajectory")
                                    },
                                    _ => {},
                                };
                            },
                            Err(e) => {
                                panic!("Error in Tracking Backend: {}", e);
                            }
                        };
                        trace!("Tracking took {} ms", now.elapsed().as_millis());
                    },
                    TrackingBackendMsg::MapInitializedMsg { curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, curr_kf_timestamp } => {
                        // Map needs to be initialized before tracking can begin. Received from map actor
                        self.frames_since_last_kf = 0;
                        self.local_keyframes.push(curr_kf_id);
                        self.local_keyframes.push(ini_kf_id);
                        self.local_mappoints = local_mappoints;
                        self.ref_kf_id = Some(curr_kf_id);
                        self.last_kf_timestamp = Some(curr_kf_timestamp);
                        {
                            let last_frame = self.last_frame.as_mut().unwrap();
                            last_frame.ref_kf_id = Some(curr_kf_id);
                            last_frame.pose = Some(curr_kf_pose);
                        }
                        self.state = TrackingState::Ok;

                        self.update_trajectory_in_logs().expect("Could not save trajectory");

                        // Let tracking frontend know the map is initialized
                        self.actor_channels.find(TRACKING_FRONTEND).send(Box::new(
                            TrackingFrontendMsg::TrackingStateMsg {
                                state: TrackingState::Ok,
                                init_id: ini_kf_id
                            }
                        )).expect("Could not send to tracking frontend");

                        // Send first two keyframes to local mapping
                        self.actor_channels.find(LOCAL_MAPPING).send(Box::new(
                            LocalMappingMsg::KeyFrameIdMsg { kf_id: ini_kf_id }
                        )).expect("Could not send to local mapping");
                        self.actor_channels.find(LOCAL_MAPPING).send(Box::new(
                            LocalMappingMsg::KeyFrameIdMsg { kf_id: curr_kf_id }
                        )).unwrap();
                    },
                    TrackingBackendMsg::KeyFrameIdMsg { kf_id } => {
                        // Received from the map actor after it inserts a keyframe
                        self.last_frame.as_mut().unwrap().ref_kf_id = Some(kf_id);
                    },
                    TrackingBackendMsg::LastKeyFrameUpdatedMsg {  } => {
                        // Received from local mapping after it culls and creates new MPs for the last inserted KF
                        self.state = TrackingState::Ok;
                    },
                }
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Tracking backend received unknown message type!");
            }
        }
    }
}

impl DarvisTrackingBack {
    pub fn new(map: ReadOnlyMap<Map>, actor_channels: ActorChannels) -> DarvisTrackingBack {
        // Note: This is suuuuuch a specific bug. Default::default() calls the default function 
        // for every single item in the struct, even if it is explicitly overriden (such as
        // map, camera, sensor, global_defaults, and optimizer in the below code). This means that
        // a second map is created, DISCARDED, and then DarvisTrackingBack.map is set to the first map
        // that we wanted in the first place. So it looks like it duplicates the map, but it doesn't actually
        // and you shouldn't worry if you see this.
        DarvisTrackingBack {
            actor_channels,
            map,
            sensor: SETTINGS.get::<Sensor>(SYSTEM, "sensor"),
            initialization: Initialization::new(),
            localization_only_mode: SETTINGS.get::<bool>(SYSTEM, "localization_only_mode"),
            frames_to_reset_imu: SETTINGS.get::<i32>(TRACKING_BACKEND, "frames_to_reset_IMU") as u32,
            insert_kfs_when_lost: SETTINGS.get::<bool>(TRACKING_BACKEND, "insert_KFs_when_lost"),
            max_frames: SETTINGS.get::<f64>(SYSTEM, "fps") as i64,
            min_frames: 0,
            last_frame_id: -1,
            ..Default::default()
        }
    }


    fn track(&mut self) -> Result<(), Box<dyn std::error::Error>>  {
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

        let now = std::time::Instant::now();

        // Initial estimation of camera pose and matching
        match (self.localization_only_mode, self.state) {
            (true, _) => {
                todo!("Localization only");
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1933
                // Look for "mbOnlyTracking" in Track() function
            },
            (false, TrackingState::WaitForMapResponse) => {
                self.state = TrackingState::WaitForMapResponse;
                return Ok(());
            },
            (false, TrackingState::NotInitialized) => {
                match self.initialization.try_initialize(&self.current_frame)? {
                    true => {
                        let msg = match self.sensor.frame() {
                            FrameSensor::Mono => MapWriteMsg::create_initial_map_monocular(self.initialization.clone()),
                            _ => MapWriteMsg::create_initial_map_stereo(self.initialization.clone(), TRACKING_BACKEND)
                        };
                        self.actor_channels.find(MAP_ACTOR).send(Box::new(msg))?;
                        self.state = TrackingState::WaitForMapResponse;
                        return Ok(());
                    },
                    false => {
                        self.state = TrackingState::NotInitialized;
                        return Ok(());
                    }
                }
            },
            (false, TrackingState::Ok) => {
                let no_motion_model = (self.imu.velocity.is_none() && !self.map.read().imu_initialized) || self.relocalization.frames_since_lost(&self.current_frame) < 2;
                // SOFIYA...CheckReplacedInLastFrame. this might be important?

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
                        match self.kfs_in_map() > 10 {
                            true =>{println!("State recently lost 271"); TrackingState::RecentlyLost},
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
                        self.relocalization.past_cutoff(&self.current_frame)
                    },
                    false => {
                        // TODO (relocalization): remove the call to shutdown actor and uncomment line of code below.
                        // This is just to gracefully shut down instead of panicking at the todo 
                        error!("Relocalization! Shutting down for now.");
                        self.actor_channels.find(SHUTDOWN_ACTOR).send(Box::new(ShutdownMsg{}))?;
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
                    false => {println!("State recently lost 299"); TrackingState::RecentlyLost}
                }
            },
            (false, TrackingState::Lost) => {
                match self.kfs_in_map() < 10 {
                    true => {
                        warn!("Reseting current map...");
                        error!("Resetting current map! Shutting down for now.");
                        self.actor_channels.find(SHUTDOWN_ACTOR).send(Box::new(ShutdownMsg{}))?;
                        self.state = TrackingState::Lost;
                        return Ok(());

                        // self.actor_channels.find(MAP_ACTOR).unwrap().send(Box::new(MapWriteMsg::reset_active_map()))?;
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


        trace!("...Initial tracking took {} ms", now.elapsed().as_millis());
        let now = std::time::Instant::now();

        // Only 3 valid states now
        // TrackingState::Ok | TrackingState::RecentlyLost | TrackingState::Lost

        // Track Local Map
        let (enough_matches, matches_in_frame) = self.track_local_map();
        if enough_matches {
            self.state = TrackingState::Ok;
        } else if matches!(self.state, TrackingState::Ok) {
            if self.sensor.is_imu() {
                warn!("Track lost for less than 1 second,");
                warn!("IMU, Reset... Reset map because local mapper set the bad imu flag");
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2149
            }
            self.relocalization.timestamp_lost = Some(self.current_frame.timestamp);
            self.state = TrackingState::RecentlyLost;
        }

        if self.sensor.is_imu() {
            todo!("IMU");
            // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
        }

        if enough_matches || matches!(self.state, TrackingState::RecentlyLost) {
            // Update motion model
            let last_frame = self.last_frame.as_ref();
            if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !self.current_frame.pose.is_none() {
                let last_pose = last_frame.expect("No last frame in tracking?").pose.as_ref().expect("Can't get last frame's pose?");
                let last_twc = last_pose.inverse();
                self.imu.velocity = Some(self.current_frame.pose.as_ref().expect("Can't get current frame?").clone() * last_twc);
            } else {
                self.imu.velocity = None;
            }

            // Clean VO matches
            self.current_frame.delete_mappoints_without_observations(&self.map.read());

            // Check if we need to insert a new keyframe
            let insert_if_lost_anyway = self.insert_kfs_when_lost && matches!(self.state, TrackingState::RecentlyLost) && self.sensor.is_imu();
            let need_new_kf = self.need_new_keyframe();
            println!("Need new keyframe: {} {:?}", need_new_kf, self.state);
            if need_new_kf && (matches!(self.state, TrackingState::Ok) || insert_if_lost_anyway) {
                self.create_new_keyframe();
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
                warn!("tracking_backend::handle_message;Track lost soon after initialization, resetting...",);
                let map_msg = MapWriteMsg::reset_active_map();
                self.actor_channels.find(MAP_ACTOR).send(Box::new(map_msg))?;
            } else {
                self.create_new_map();
            }
        }

        trace!("...Track local map took {} ms", now.elapsed().as_millis());

        Ok(())
    }

    fn update_trajectory_in_logs(
        &mut self, //current_frame: &Frame<InitialFrame>
        // &mut self, current_pose: DVPose, ref_kf_id: Id, mappoint_matches: HashMap<u32, (Id, bool)>, timestamp: Timestamp
    ) -> Result<(), Box<dyn std::error::Error>> {
        let frame = self.last_frame.as_ref().expect("No last frame in tracking?");
        let map = self.map.read();
        let pose_in_world = frame.get_pose_in_world_frame(&map);
        debug!("Current pose  {:?}", pose_in_world);
        debug!("Current pose in world frame {:?}", pose_in_world);
        self.trajectory_poses.push(pose_in_world);

        self.actor_channels.find(SHUTDOWN_ACTOR).send(Box::new(ShutdownActorMsg::TrajectoryMsg{
            pose: pose_in_world,
            ref_kf_id: frame.ref_kf_id.expect("Current frame needs ref kf"),
            timestamp: frame.timestamp
        }))?;

        if SETTINGS.get::<bool>(SYSTEM, "show_visualizer") {
            self.actor_channels.find(VISUALIZER).send(Box::new(VisualizerMsg::TrajectoryMsg{
                pose: pose_in_world,
                mappoint_matches: frame.mappoint_matches.clone(),
                timestamp: frame.timestamp
            }))?;
        }

        Ok(())
    }

    fn track_reference_keyframe(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        // Tracking::TrackReferenceKeyFrame()
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        let now = std::time::Instant::now();

        self.current_frame.compute_bow();
        let nmatches;
        let increase_found;
        {
            let map_read_lock = self.map.read();
            let ref_kf = map_read_lock.keyframes.get(&self.ref_kf_id.unwrap()).unwrap();

            (nmatches, increase_found) = orbmatcher::search_by_bow_f(ref_kf, &mut self.current_frame, true, 0.7)?;
        }

        println!("track reference keyframe matches {}", self.current_frame.mappoint_matches.len());

        let map_msg = MapWriteMsg::increase_found(increase_found);
        self.actor_channels.find(MAP_ACTOR).send(Box::new(map_msg))?;

        if nmatches < 15 {
            warn!("track_reference_keyframe has fewer than 15 matches = {}!!\n", nmatches);
            return Ok(false);
        }

        self.current_frame.pose = Some(self.last_frame.as_ref().unwrap().pose.unwrap());

        optimizer::optimize_pose(&mut self.current_frame, &self.map);

        // Discard outliers
        let nmatches_map = self.discard_outliers();
        println!("track reference keyframe matches after outliers {}", nmatches_map);

        trace!("Track reference keyframe: {} ms", now.elapsed().as_millis());

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

        let enough_frames_to_reset_imu = self.current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32);
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
            &mut self.current_frame,
            self.last_frame.as_ref().unwrap(),
            th,
            self.sensor.is_mono(),
            &self.map,
            self.sensor
        )?;

        println!("motion model matches {}", self.current_frame.mappoint_matches.len());

        // If few matches, uses a wider window search
        if matches < 20 {
            info!("tracking_backend::track_with_motion_model;not enough matches, wider window search");
            self.current_frame.clear_mappoints();
            matches = orbmatcher::search_by_projection_with_threshold(
                &mut self.current_frame,
                self.last_frame.as_ref().unwrap(),
                2 * th,
                self.sensor.is_mono(),
                &self.map,
                self.sensor
            )?;
        }

        println!("motion model matches with wider search {}", self.current_frame.mappoint_matches.len());


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

        println!("motion model matches after outliers {}", nmatches_map);
        match self.sensor.is_imu() {
            true => { return Ok(true); },
            false => { return Ok(nmatches_map >= 10); }
        };
    }

    fn update_last_frame(&mut self) {
        // Tracking::UpdateLastFrame()
        let now = std::time::Instant::now();

        // Update pose according to reference keyframe
        let ref_kf_id = self.last_frame.as_ref().expect("Should have last frame")
            .ref_kf_id.expect("Reference keyframe should have id");
        let map_lock = self.map.read();
        let ref_kf_pose = map_lock.keyframes.get(&ref_kf_id).expect("Reference kf should be in map")
            .pose.expect("Reference kf should have pose");
        self.last_frame.as_mut().unwrap().pose = Some(*self.trajectory_poses.last().unwrap() * ref_kf_pose);

        if self.sensor.is_mono() || self.frames_since_last_kf == 0 {
            return;
        }

        trace!("...Update last frame took {} ms", now.elapsed().as_millis());

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

    fn track_local_map(&mut self) -> (bool, i32) {
        // bool Tracking::TrackLocalMap()
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        self.update_local_keyframes();
        self.update_local_points();
        self.search_local_points();

        if !self.map.read().imu_initialized || (self.current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32)) {
            optimizer::optimize_pose(&mut self.current_frame, &self.map);
        } else if !self.map_updated {
            let inliers = optimizer::pose_inertial_optimization_last_frame(&mut self.current_frame, &self.map);
        } else {
            let inliers = optimizer::pose_inertial_optimization_last_keyframe(&mut self.current_frame);
        }

        self.matches_inliers = 0;
        // Update MapPoints Statistics
        let mut increase_found = Vec::new();
        for (index, (mp_id, _)) in &self.current_frame.mappoint_matches {
            if !self.current_frame.is_mp_outlier(&index) {
                increase_found.push((*mp_id, 1));

                if self.localization_only_mode {
                    let map_read_lock = self.map.read();
                    let mappoint = map_read_lock.mappoints.get(&mp_id).unwrap();

                    if mappoint.get_observations().len() > 0 {
                        self.matches_inliers+=1;
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

        // Sofiya matches
        //     mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
        // send this in local mapper message?

        println!("Matches in track local map {}", self.matches_inliers);
        let map_msg = MapWriteMsg::increase_found(increase_found);
        self.actor_channels.find(MAP_ACTOR).send(Box::new(map_msg)).unwrap();

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32) && self.matches_inliers<50 {
            warn!("track_local_map unsuccessful; matches in frame < 50 : {}",self.matches_inliers);
            return (false, self.matches_inliers);
        }

        match self.state {
            TrackingState::RecentlyLost => { return (self.matches_inliers > 10, self.matches_inliers); },
            _ => {}
        }

        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => { 
                return (
                    !(self.matches_inliers<15 && self.map.read().imu_initialized) && !(self.matches_inliers<50 && !self.map.read().imu_initialized),
                    self.matches_inliers
                )
            },
            Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => {
                return (self.matches_inliers >= 15, self.matches_inliers);
            },
            _ => { return (self.matches_inliers > 30, self.matches_inliers) }
        }
    }

    fn update_local_keyframes(&mut self) {
        // void Tracking::UpdateLocalKeyFrames()
        let now = std::time::Instant::now();

        // Each map point votes for the keyframes in which it has been observed
        let mut kf_counter = HashMap::<Id, i32>::new();
        let frame = match !self.map.read().imu_initialized || self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
            true => &self.current_frame,
            false => self.last_frame.as_ref().unwrap() // Using lastframe since current frame has no matches yet
        };
        for (_, (mp_id, _)) in &frame.mappoint_matches {
            let map_read_lock = self.map.read();
            let mp = map_read_lock.mappoints.get(&mp_id).unwrap();
            for kf_id in mp.get_observations().keys() {
                *kf_counter.entry(*kf_id).or_insert(0) += 1;
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
            self.kf_track_reference_for_frame.insert(kf_id, self.current_frame.frame_id);
        }

        // Also include some keyframes that are neighbors to already-included keyframes
        let mut i = 0;
        while self.local_keyframes.len() <= 80 && i < self.local_keyframes.len() { // Limit the number of keyframes
            let next_kf_id = self.local_keyframes[i];
            let map_read_lock = self.map.read();
            let kf = map_read_lock.keyframes.get(&next_kf_id).unwrap();

            for kf_id in &kf.get_covisibility_keyframes(10) {
                if self.kf_track_reference_for_frame.get(kf_id) != Some(&self.current_frame.frame_id) {
                    self.local_keyframes.push(*kf_id);
                    self.kf_track_reference_for_frame.insert(*kf_id, self.current_frame.frame_id);
                    break;
                }
            }
            for kf_id in &kf.full_kf_info.children {
                if self.kf_track_reference_for_frame.get(kf_id) != Some(&self.current_frame.frame_id) {
                    self.local_keyframes.push(*kf_id);
                    self.kf_track_reference_for_frame.insert(*kf_id, self.current_frame.frame_id);
                    break;
                }
            }
    
            if let Some(parent_id) = kf.full_kf_info.parent {
                if self.kf_track_reference_for_frame.get(&parent_id) != Some(&self.current_frame.frame_id) {
                    self.local_keyframes.push(parent_id);
                    self.kf_track_reference_for_frame.insert(parent_id, self.current_frame.frame_id);
                }
            };
            i += 1;
        }

        // Add 10 last temporal KFs (mainly for IMU)
        if self.sensor.is_imu() && self.local_keyframes.len() < 80 {
            todo!("IMU");
            // For the last 20 created keyframes, save into new_local_keyframes
            // until one of them was not in self.local_keyframes. Then, quit.
        }

        if max_kf_id != 0 {
            self.current_frame.ref_kf_id = Some(max_kf_id);
            self.ref_kf_id = Some(max_kf_id);
        }

        trace!("Update local keyframes: {} ms", now.elapsed().as_millis());
    }

    fn update_local_points(&mut self) {
        // void Tracking::UpdateLocalPoints()
        let now = std::time::Instant::now();
        self.local_mappoints.clear();
        for kf_id in &self.local_keyframes {
            let map_read_lock = self.map.read();
            let mp_ids = &map_read_lock.keyframes.get(kf_id).unwrap().mappoint_matches;
            for (_, (mp_id, _)) in mp_ids {
                if self.mp_track_reference_for_frame.get(mp_id) != Some(&self.current_frame.frame_id) {
                    self.local_mappoints.insert(*mp_id);
                    self.mp_track_reference_for_frame.insert(*mp_id, self.current_frame.frame_id);
                }
            }
        }
        trace!("Update local points: {} ms", now.elapsed().as_millis());
    }

    fn search_local_points(&mut self) {
        //void Tracking::SearchLocalPoints()
        let now = std::time::Instant::now();
        let mut mps_to_increase_visible = Vec::new();

        // Do not search map points already matched        
        for (_, (id, _)) in &self.current_frame.mappoint_matches {
            mps_to_increase_visible.push(*id);
            self.last_frame_seen.insert(*id, self.current_frame.frame_id);
            self.track_in_view.remove(id);
            self.track_in_view_r.remove(id);
        }

        // Project points in frame and check its visibility
        let mut to_match = 0;
        for mp_id in &self.local_mappoints {
            if self.last_frame_seen.get(mp_id) == Some(&self.current_frame.frame_id) {
                continue;
            }
            // Project (this fills MapPoint variables for matching)
            let map_read_lock = self.map.read();
            let (tracked_data_left, tracked_data_right) = self.current_frame.is_in_frustum(*mp_id, 0.5, &*map_read_lock);
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
            if self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
                th = 5;
            }
            match self.state {
                TrackingState::RecentlyLost | TrackingState::Lost => th = 15,
                _ => {}
            }

            self.actor_channels.find(MAP_ACTOR).send(Box::new(
                MapWriteMsg::increase_visible(mps_to_increase_visible
            ))).unwrap();

            let matches = orbmatcher::search_by_projection(
                &mut self.current_frame,
                &self.local_mappoints,
                th, 0.8,
                &self.track_in_view, &self.track_in_view_r,
                &self.map, self.sensor
            );
            println!("search local points matches {:?}", matches);
        }
        trace!("Search local points: {} ms", now.elapsed().as_millis());
    }

    fn create_new_keyframe(&mut self) {
        //CreateNewKeyFrame
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216
        let now = std::time::Instant::now();
        let new_kf = Frame::<PrelimKeyFrame>::new_clone(&self.current_frame);

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
        let map_actor = self.actor_channels.find(MAP_ACTOR);
        map_actor.send(Box::new(MapWriteMsg::new_keyframe(new_kf))).unwrap();

        trace!("Create new keyframe: {} ms", now.elapsed().as_millis());
    }

    fn need_new_keyframe(&self) -> bool {
        let now = std::time::Instant::now();

        let kfs_in_map = self.kfs_in_map();
        let not_enough_frames_since_last_reloc = (self.current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames as i32)) && (kfs_in_map > (self.max_frames as u32));
        let imu_not_initialized = self.sensor.is_imu() && !self.map.read().imu_initialized;

        let too_close_to_last_kf = match self.last_kf_timestamp {
            Some(timestamp) => self.current_frame.timestamp - timestamp < 0.25, // 250 milliseconds
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
        let tracked_mappoints = map_lock.keyframes.get(&self.ref_kf_id.unwrap())
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
                if self.matches_inliers > 350 { 0.75 } else { 0.90 }
            },
            Sensor(FrameSensor::Stereo, _) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => 0.75,
            Sensor(FrameSensor::Rgbd, ImuSensor::None) => { if kfs_in_map < 2 { 0.4 } else { 0.75 } }
        }; // TODO (mvp): is Rgbd right? See ORBSLAM3 code https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3142

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        let c1a = self.frames_since_last_kf >= (self.max_frames as i32);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // Note: removed localmappingidle from here
        let c1b = self.frames_since_last_kf >= (self.min_frames as i32);
        //Condition 1c: tracking is weak
        let sensor_is_right = match self.sensor {
            Sensor(FrameSensor::Mono, _) | Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => false,
            _ => true
        }; // TODO (mvp): why would they just be selecting for RGBD or Stereo without IMU??
        let c1c = sensor_is_right && ((self.matches_inliers as f32) < tracked_mappoints * 0.25 || need_to_insert_close) ;
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        let c2 = (((self.matches_inliers as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_inliers > 15;

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

        trace!("Need new keyframe: {} ms", now.elapsed().as_millis());
        // Note: removed code here about checking for idle local mapping and/or interrupting bundle adjustment
        return ((c1a||c1b||c1c) && c2)||c3 ||c4;
    }

    //* Helper functions */
    fn kfs_in_map(&self) -> u32 {
        let map_read_lock = self.map.read();
        map_read_lock.keyframes.len() as u32
    }

    fn discard_outliers(&mut self) -> i32 {
        let now = std::time::Instant::now();
        let discarded = self.current_frame.discard_outliers();
        for (_, (mp_id, _)) in discarded {
            self.track_in_view.remove(&mp_id);
            self.last_frame_seen.insert(mp_id, self.current_frame.frame_id);
            // TODO (Stereo) ... need to remove this from track_in_view_r if the mp is seen in the right camera
        }
        let result = self.current_frame.get_num_mappoints_with_observations(&*self.map.read());
        trace!("Discard outliers: {} ms", now.elapsed().as_millis());
        result
    }

    //* Next steps */
    fn create_new_map(&self) -> bool {
        todo!("Multimaps: Atlas::CreateNewMap");
    }
}

pub enum TrackingBackendMsg {
    FeatureMsg { keypoints: DVVectorOfKeyPoint, descriptors: DVMatrix, image_width: u32, image_height: u32, timestamp: Timestamp, frame_id: Id},
    MapInitializedMsg { curr_kf_pose: DVPose, curr_kf_id: Id, ini_kf_id: Id, local_mappoints: HashSet<Id>, curr_kf_timestamp: Timestamp },
    KeyFrameIdMsg { kf_id: Id },
    LastKeyFrameUpdatedMsg {},
}
impl ActorMessage for TrackingBackendMsg { }
