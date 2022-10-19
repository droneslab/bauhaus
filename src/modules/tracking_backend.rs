use std::{sync::Arc, collections::{HashSet, HashMap}};
use axiom::prelude::*;
use chrono::{prelude::*, Duration};
use opencv::core::Point2f;
use dvcore::{lockwrap::ReadOnlyWrapper, plugin_functions::Function, global_params::*, matrix::DVVectorOfPoint3f};
use crate::{
    modules::messages::{FeatureMsg, KeyFrameMsg},
    registered_modules::LOCAL_MAPPING,
    dvmap::{
        map::Map, map_actor::MapWriteMsg, map_actor::MAP_ACTOR, map::Id,
        keyframe::KeyFrame, frame::Frame, pose::Pose, mappoint::{MapPoint, FullMapPoint},
        keypoints::KeyPointsData, sensor::*
    },
    utils::{camera::Camera, camera::CameraType, orbmatcher::ORBmatcher, imu::IMUBias, optimizer::*},
};

#[derive(Debug, Clone)]
pub struct GlobalDefaults {
    // Defaults set from global variables
    // Never change, so just get once instead of having to look up each time
    recently_lost_cutoff: Duration,
    localization_only_mode: bool,
    frames_to_reset_imu: u32, //mnFramesToResetIMU
    insert_kfs_when_lost: bool,
    min_num_features: i32,
    max_frames : u32 , //mMaxFrames , set to fps , Max Frames to insert keyframes and to check relocalisation
    min_frames: u32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
}
impl GlobalDefaults {
    pub fn new() -> Self {
        Self {
            recently_lost_cutoff: Duration::seconds(GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "recently_lost_cutoff").into()),
            localization_only_mode: GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "localization_only_mode"),
            frames_to_reset_imu: GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "frames_to_reset_IMU") as u32,
            insert_kfs_when_lost: GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "insert_KFs_when_lost"),
            min_num_features: GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "min_num_features"),
            max_frames: 30, // fps set default as 30
            min_frames: 0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct DarvisTrackingBack<S: SensorType> {
    state: TrackingState,

    // Map
    map: ReadOnlyWrapper<Map<S>>,
    map_actor: Option<Aid>,
    initialization: Initialization<S>, // data sent to map actor to initialize new map

    // Feature matching
    orb_matcher: ORBmatcher,
    temporal_points: Vec<MapPoint<FullMapPoint>>,
    matches_in_frame : i32, // Current matches in frame
    prev_matched: Vec<Point2f>,// std::vector<cv::Point2f> mvbPrevMatched;

    // Frames
    last_frame_id: Id,
    current_frame: Frame<S>,
    last_frame: Option<Frame<S>>,
    reference_frame: Option<Frame<S>>, // TODO: Redundant hence will remove once frame database is established.

    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_ts: Option<DateTime<Utc>>,

    // Local map
    // TODO: Can we get rid of these and pipe this all through the map instead?
    local_keyframes: HashSet<Id>, //mvpLocalKeyFrames
    local_mappoints: HashSet<Id>, //mvpLocalMapPoints

    // IMU 
    velocity: Option<Pose>,
    last_bias: Option<IMUBias>,

    // Relocalization
    last_reloc_frame_id: Id, // last_reloc_frame_id
    timestamp_lost: Option<DateTime<Utc>>,

    // Idk where to put these
    camera: Camera,
    map_updated : bool,  // Sofiya: I'm not sure we want to use this

    // Utilities from darvis::utils
    optimizer: Optimizer,

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
    trajectory_times: Vec<DateTime<Utc>>, //mlFrameTimes
    trajectory_keyframes: Vec<Id>, //mlpReferences

    global_defaults: GlobalDefaults,
}

#[derive(Debug, Clone)]
enum TrackingState {
    NotInitialized,
    Lost,
    RecentlyLost,
    Ok
}

impl<S: SensorType + 'static> DarvisTrackingBack<S> {
    pub fn new(map: ReadOnlyWrapper<Map<S>>) -> DarvisTrackingBack<S> {
        let sensor = GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor");

        match sensor {
            Sensor::Mono => Self::hidden_new::<MonoSensor>(map),
            Sensor::ImuMono => Self::hidden_new::<ImuMonoSensor>(map),
            Sensor::Stereo => Self::hidden_new::<StereoSensor>(map),
            Sensor::ImuStereo => Self::hidden_new::<ImuStereoSensor>(map),
            Sensor::Rgbd => Self::hidden_new::<RgbdSensor>(map),
            Sensor::ImuRgbd => Self::hidden_new::<ImuRgbdSensor>(map),
        }
    }

    fn hidden_new<S2: SensorType> (map: ReadOnlyWrapper<Map<S>>) -> DarvisTrackingBack<S> {
        // Camera initialization
        let camera = Camera::new(CameraType::Pinhole);

        DarvisTrackingBack {
            last_frame_id: 0,
            last_frame: None,
            map,
            state: TrackingState::NotInitialized,
            temporal_points: Vec::new(),
            ref_kf_id: None,
            current_frame: Frame::fake(),
            reference_frame: None,
            frames_since_last_kf: -1,
            orb_matcher: ORBmatcher::new(0.9,true),
            prev_matched: Vec::new(),// std::vector<cv::Point2f> mvbPrevMatched;
            initialization: Initialization::new(),
            global_defaults: GlobalDefaults::new(),
            last_bias: None,
            velocity: None,
            timestamp_lost: None,
            last_reloc_frame_id: 0,
            camera,
            map_actor: None,
            map_updated: false,
            matches_in_frame: 0,
            optimizer: Optimizer::new(),
            last_kf_ts: None,
            local_keyframes: HashSet::new(),
            local_mappoints: HashSet::new(),
            trajectory_poses: Vec::new(),
            trajectory_times: Vec::new(),
            trajectory_keyframes: Vec::new()
        }
    }

    fn handle_message(&mut self, msg: Arc<FeatureMsg>) {
        let map_actor = msg.actor_ids.get(MAP_ACTOR).unwrap();
        self.map_actor = Some(map_actor.clone());

        self.last_frame_id += 1;
        self.current_frame = Frame::new(
            self.last_frame_id, 
            &msg.keypoints,
            &msg.descriptors,
            msg.image_width,
            msg.image_height,
            &self.camera
        );

        // TODO (reset): Reset map because local mapper set the bad imu flag
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1808

        // TODO (multimaps): Create new map if timestamp older than previous frame arrives
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1820

        // TODO (reset) TODO (multimaps): Timestamp jump detected, either reset active map or create new map
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1828
        // (entire block)

        // TODO: set bias of new frame = to bias of last
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1860

        match S::sensor_type() {
            Sensor::Mono | Sensor::ImuMono => {},
            _ => self.preintegrate_imu()
        }

        // TODO: update map change index. Not sure why this is useful
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1890

        // Initial estimation of camera pose and matching
        let mut success = match self.state {
            TrackingState::NotInitialized => {
                let should_send_to_map = self.initialization.initialize(&self.current_frame, &self.orb_matcher, &self.camera);
                self.map_actor.as_ref().unwrap().send_new(
                    MapWriteMsg::<S>::create_initial_map_monocular(self.initialization.clone())
                ).unwrap();

                self.last_frame = Some(self.current_frame.clone());
                return;
            },
            TrackingState::Ok => {
                let mut ok;

                let not_enough_frames_since_last_reloc = self.current_frame.id < self.last_reloc_frame_id + 2;
                let no_imu_data = self.velocity.is_none() && !self.is_imu_initialized();
                if no_imu_data || not_enough_frames_since_last_reloc {
                    ok = self.track_reference_keyframe();
                } else {
                    ok = self.track_with_motion_model();
                    if !ok {
                        ok = self.track_reference_keyframe();
                    }
                }

                if !ok {
                    let enough_frames_to_reset_imu = self.current_frame.id <= self.last_reloc_frame_id + (self.global_defaults.frames_to_reset_imu as i32);
                    if enough_frames_to_reset_imu && S::is_imu() {
                        self.state = TrackingState::Lost;
                    } else if self.kfs_in_map() > 10 {
                        self.state = TrackingState::RecentlyLost;
                        self.timestamp_lost = Some(self.current_frame.timestamp);
                    } else {
                        self.state = TrackingState::Lost;
                    }
                }
                ok
            },
            TrackingState::RecentlyLost => {
                println!("TRACK: State recently lost");
                let mut ok;
                let time_since_lost = self.current_frame.timestamp - self.timestamp_lost.unwrap();
                if S::is_imu() {
                    if self.is_imu_initialized() {
                        ok = self.predict_state_imu();
                    } else {
                        ok = false;
                    }

                    if time_since_lost > self.global_defaults.recently_lost_cutoff {
                        self.state = TrackingState::Lost;
                        println!("TRACK: Tracking lost...");
                        ok = false;
                    }
                } else {
                    ok = self.relocalization();
                    if time_since_lost > Duration::seconds(3) && !ok {
                        self.state = TrackingState::Lost;
                        println!("TRACK: Tracking lost...");
                        ok = false;
                    }
                }
                ok
            },
            TrackingState::Lost => {
                println!("TRACK: New map...");
                if self.kfs_in_map() < 10 {
                    println!("TRACK: Reseting current map...");
                    let map_msg: MapWriteMsg<S> = MapWriteMsg::<S>::reset_active_map();
                    map_actor.send_new(map_msg).unwrap();
                } else {
                    self.create_new_map();
                }

                self.last_frame = None;
                return;
            }
        };

        // TODO (localization-only): Above match should check for localization-only mode
        // and only run the above code if localization-only is NOT enabled. Need to implement
        // localization-only version.
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1933
        // Look for "mbOnlyTracking" in Track() function

        // Track Local Map
        if success {
            success = self.track_local_map();
        }
        if success {
            self.state = TrackingState::Ok;
        } else if matches!(self.state, TrackingState::Ok) {
            if S::is_imu() {
                println!("TRACK: Track lost for less than 1 second");

                // TODO (reset) TODO (IMU): Reset map because local mapper set the bad imu flag
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2149
            }
            self.state = TrackingState::RecentlyLost;
            self.timestamp_lost = Some(self.current_frame.timestamp);
        }

        // TODO (IMU)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
        // This code not done, so double check with C++ reference.
        // {
        //     // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        //     let enough_frames_to_reset_imu = self.current_frame.as_ref().unwrap().id <= self.last_reloc_frame_id + self.frames_to_reset_imu;
        //     if enough_frames_to_reset_imu && self.current_frame.as_ref().unwrap().id > self.frames_to_reset_imu && S::is_imu() && self.is_imu_initialized() {
        //         // Load preintegration
        //         // This code directly copied from C++, obviously needs to be changed
        //         // pF->mpImuPreintegratedFrame = new IMU::Preintegrated(self.current_frame.mpImuPreintegratedFrame);
        //     }
        //     if self.is_imu_initialized() && success {
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
                let _last_twc = last_pose.inverse();
                // TODO (IMU)
                self.velocity = Some(self.current_frame.pose.as_ref().unwrap().clone() * _last_twc);
            } else {
                self.velocity = None;
            }

            {
                // Sofiya: would like to move this into frame but don't want to pass a map read lock into it
                let map_lock = self.map.read();
                let mut to_remove = Vec::new();
                for (index, mp_id) in &self.current_frame.mappoint_matches {
                    if map_lock.get_mappoint(&mp_id).unwrap().observations().len() > 0 {
                        to_remove.push(*index);
                    }
                }
                for mp_id in to_remove {
                    self.current_frame.delete_mappoint_match(mp_id);
                }
            }

            self.temporal_points.clear();

            // Check if we need to insert a new keyframe
            let need_kf = self.need_new_keyframe();
            let insert_if_lost_anyway = self.global_defaults.insert_kfs_when_lost && matches!(self.state, TrackingState::RecentlyLost) && S::is_imu();
            if need_kf && (success || insert_if_lost_anyway) {
                self.create_new_keyframe(&msg.actor_ids);
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            let (_num_deleted, _all_left) = self.current_frame.discard_outliers();
        }

        // Reset if the camera get lost soon after initialization
        if matches!(self.state, TrackingState::Lost) {
            if self.kfs_in_map() <= 10 {
                let map_msg = MapWriteMsg::<S>::reset_active_map();
                map_actor.send_new(map_msg).unwrap();
                return;
            }
            if S::is_imu() && !self.is_imu_initialized() {
                println!("TRACK: Track lost before IMU initialisation, resetting...",);
                let map_msg = MapWriteMsg::<S>::reset_active_map();
                map_actor.send_new(map_msg).unwrap();
                return;
            }

            self.create_new_map();
            return;
        }

        if self.current_frame.ref_kf_id.is_none() {
            self.current_frame.ref_kf_id = self.ref_kf_id;
        }

        self.last_frame = Some(self.current_frame.clone());

        match self.state {
            TrackingState::Ok | TrackingState::RecentlyLost => self.store_pose_info_for_trajectory(),
            _ => {}
        }

        // self.align(_context, &msg);
    }

    //* MVP */
    fn track_reference_keyframe(&mut self) -> bool {
        // Tracking::TrackReferenceKeyFrame()
        // Compute Bag of Words vector
        let map_read_lock = self.map.read();
        self.current_frame.compute_bow(&map_read_lock.vocabulary);

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        let matcher = ORBmatcher::new(0.7, true);
        let mut vp_mappoint_matches = HashMap::<u32, Id>::new();

        let mut nmatches;
        if self.ref_kf_id.is_some() {
            let map_read_lock = self.map.read();
            let ref_kf = map_read_lock.get_keyframe(&self.ref_kf_id.unwrap());
            let cur_kf = map_read_lock.get_keyframe(&self.current_frame.id);
            if ref_kf.is_some() {
                nmatches = matcher.search_by_bow_f(ref_kf.unwrap(), &self.current_frame, &mut vp_mappoint_matches);
            } else {
                todo!("fix invalid ref KF assignment");
            }

            let map_read_lock = self.map.read();
            let _ref_kf = map_read_lock.get_keyframe(&self.ref_kf_id.unwrap());

            if self.reference_frame.is_some() {
                //TODO 10/17: uncomment when bindings are done
                //nmatches = matcher.search_by_bow(self.reference_frame.as_ref().unwrap(), cur_f, &mut vpMapPointMatches);

                nmatches = matcher.search_for_initialization(self.reference_frame.as_ref().unwrap(), &self.current_frame, &mut self.prev_matched, &mut vp_mappoint_matches, 100);
            } else {
                todo!("fix invalid ref KF assignment");
            }

        } else {
            todo!("fix ref KF assignment");
        }

        if nmatches < 15 {
            println!("TRACK_REF_KF: Less than 15 matches!!\n");
            return false;
        }

        self.current_frame.mappoint_matches = vp_mappoint_matches.clone();
        self.current_frame.set_pose(self.last_frame.as_ref().unwrap().get_pose());

        let (_inliers, pose) = self.optimizer.optimize_pose(&mut self.current_frame, &self.map);
        match pose {
            Some(pose) => { 
                let map_msg  = MapWriteMsg::<S>::set_pose(self.current_frame.id, pose);
                self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();
            },
            None => {}
        }

        // Discard outliers
        let (num_deleted, leftover_mps) = self.current_frame.discard_outliers();
        let mut nmatches_map = -num_deleted;
        {
            for mp_id in leftover_mps {
                let map_lock = self.map.read();
                if map_lock.get_mappoint(&mp_id).unwrap().observations().len() > 0 {
                    nmatches_map += 1;
                }
            }
        }

        match S::sensor_type().is_imu() {
            true => { return true; },
            false => { return nmatches_map >= 10; }
        };
    }

    fn track_with_motion_model(&mut self) -> bool {
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

        let enough_frames_to_reset_imu = self.current_frame.id <= self.last_reloc_frame_id + (self.global_defaults.frames_to_reset_imu as i32);
        if self.is_imu_initialized() && enough_frames_to_reset_imu {
            // Predict state with IMU if it is initialized and it doesnt need reset
            self.predict_state_imu();
            return true;
        } else {
            self.current_frame.set_pose(self.velocity.unwrap() * self.last_frame.as_ref().unwrap().get_pose());
        }

        self.current_frame.clear_mappoints();

        // Project points seen in previous frame
        let th = match S::sensor_type() {
            Sensor::Mono | Sensor::ImuMono => 15,
            _ => 7
        };

        let mut matches = self.orb_matcher.search_by_projection_with_threshold(
            &self.current_frame,
            self.last_frame.as_ref().unwrap(),
            th,
            S::is_mono(),
            &self.camera,
            &self.map
        ); 

        // If few matches, uses a wider window search
        if matches.len() < 20 {
            println!("TRACK: track_with_motion_model... not enough matches, wider window search");
            self.current_frame.clear_mappoints();
            matches = self.orb_matcher.search_by_projection_with_threshold(
                &self.current_frame,
                self.last_frame.as_ref().unwrap(),
                2 * th,
                S::is_mono(),
                &self.camera,
                &self.map
            ); 
        }

        if matches.len() < 20 {
            println!("TRACK: track_with_motion_model... not enough matches!!");
            return S::is_imu();
        }

        // Optimize frame pose with all matches
        let (_inliers, pose) = self.optimizer.optimize_pose(&mut self.current_frame, &self.map);
        match pose {
            Some(pose) => { 
                let map_msg  = MapWriteMsg::<S>::set_pose(self.current_frame.id, pose);
                self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();
            },
            None => {}
        }

        // Discard outliers
        let (num_deleted, leftover_mps) = self.current_frame.discard_outliers();
        let mut nmatches_map = -num_deleted;
        {
            for mp_id in leftover_mps {
                let map_lock = self.map.read();
                if map_lock.get_mappoint(&mp_id).unwrap().observations().len() > 0 {
                    nmatches_map += 1;
                }
            }
        }

        if self.global_defaults.localization_only_mode {
            // TODO (localization only)
            // mbVO = nmatchesMap<10;
            // return nmatches>20;
        }

        match S::sensor_type().is_imu() {
            true => { return true; },
            false => { return nmatches_map >= 10; }
        };
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
        self.last_frame.as_mut().unwrap().set_pose(reference_kf_pose);

        if S::is_mono() || self.frames_since_last_kf == 0 {
            return;
        }

        {
            // TODO (RGBD and Stereo)
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

    fn update_local_map(&mut self) {
        //void Tracking::UpdateLocalMap()

        // This is for visualization
        // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        self.update_local_keyframes();
        self.update_local_points();
    }

    fn update_local_keyframes(&mut self) {
        // void Tracking::UpdateLocalKeyFrames()

        // Each map point votes for the keyframes in which it has been observed
        let mut kf_counter = HashMap::<Id, i32>::new();
        let frame = match !self.is_imu_initialized() || self.current_frame.id < self.last_reloc_frame_id + 2 {
            true => &self.current_frame,
            false => self.last_frame.as_ref().unwrap() // Using lastframe since current frame has no matches yet
        };
        for (_index, mp_id) in &frame.mappoint_matches {
            let map_read_lock = self.map.read();
            let mp = map_read_lock.get_mappoint(&mp_id).unwrap();
            for (observing_kf, _index) in mp.observations() {
                if kf_counter.get(observing_kf).is_none() {
                    kf_counter.insert(*observing_kf, 1);
                } else {
                    *kf_counter.get_mut(observing_kf).unwrap() += 1;
                }
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
                get_first_new(&self.local_keyframes, &kf.get_neighbors(10))
            );
            neighbors_to_add.insert(
                get_first_new(&self.local_keyframes, &kf.children)
            );

            match kf.parent {
                Some(parent_id) => {
                    if self.local_keyframes.get(&parent_id).is_none() {
                        neighbors_to_add.insert(parent_id);
                    }
                },
                None => {}
            };
        }
        new_local_keyframes.extend(neighbors_to_add);

        // Add 10 last temporal KFs (mainly for IMU)
        if S::is_imu() && self.local_keyframes.len() < 80 {
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
            let mp_ids = &map_read_lock.get_keyframe(kf_id).unwrap().mappoint_matches;
            for (_index, mp_id) in mp_ids {
                if self.local_mappoints.get(mp_id).is_none() {
                    new_mappoints.insert(*mp_id);
                }
            }
        }

        self.local_mappoints.clear();
        self.local_mappoints = new_mappoints;
    }

    fn search_local_points(&mut self) {
        //void Tracking::SearchLocalPoints()

        todo!("TODO 10/17 fill");
        // // Do not search map points already matched
        // for(vector<MapPoint*>::iterator vit=self.current_frame.mappoint_matches.begin(), vend=self.current_frame.mappoint_matches.end(); vit!=vend; vit++)
        // {
        //     MapPoint* pMP = *vit;
        //     if(pMP)
        //     {
        //         if(pMP->isBad())
        //         {
        //             *vit = static_cast<MapPoint*>(NULL);
        //         }
        //         else
        //         {
        //             pMP->IncreaseVisible();
        //             pMP->last_frame_seen = self.current_frame.mnId;
        //             pMP->mbTrackInView = false;
        //             pMP->mbTrackInViewR = false;
        //         }
        //     }
        // }

        // int nToMatch=0;

        // // Project points in frame and check its visibility
        // for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
        // {
        //     MapPoint* pMP = *vit;

        //     if(pMP->last_frame_seen == self.current_frame.mnId)
        //         continue;
        //     if(pMP->isBad())
        //         continue;
        //     // Project (this fills MapPoint variables for matching)
        //     if(self.current_frame.isInFrustum(pMP,0.5))
        //     {
        //         pMP->IncreaseVisible();
        //         nToMatch++;
        //     }
        //     if(pMP->mbTrackInView)
        //     {
        //         self.current_frame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        //     }
        // }

        // if(nToMatch>0)
        // {
        //     ORBmatcher matcher(0.8);
        //     int th = 1;
        //     if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
        //         th=3;
        //     if(mpAtlas->isImuInitialized())
        //     {
        //         if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
        //             th=2;
        //         else
        //             th=6;
        //     }
        //     else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
        //     {
        //         th=10;
        //     }

        //     // If the camera has been relocalised recently, perform a coarser search
        //     if(self.current_frame.mnId<last_reloc_frame_id+2)
        //         th=5;

        //     if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
        //         th=15; // 15

        //     int matches = matcher.SearchByProjection(self.current_frame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
        // }

    }

    fn track_local_map(&mut self) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2949
        self.update_local_map();
        self.search_local_points();

        let mut inliers = 0;
        if !self.is_imu_initialized() || (self.current_frame.id <= self.last_reloc_frame_id + (self.global_defaults.frames_to_reset_imu as i32)) {
            let (_inliers, pose) = self.optimizer.optimize_pose(&mut self.current_frame, &self.map);
            match pose {
                Some(pose) => { 
                    let map_msg  = MapWriteMsg::<S>::set_pose(self.current_frame.id, pose);
                    self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();
                },
                None => {}
            }
        } else if !self.map_updated {
            let _inliers = self.optimizer.pose_inertial_optimization_last_frame(&mut self.current_frame);
        } else {
            let _inliers = self.optimizer.pose_inertial_optimization_last_keyframe(&mut self.current_frame);
        }

        self.matches_in_frame = 0;
        // Update MapPoints Statistics
        for (index, mp_id) in &self.current_frame.mappoint_matches {
            if !self.current_frame.mappoint_is_outlier(&index) {
                let map_msg = MapWriteMsg::<S>::increase_found(&mp_id, 1);
                self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();

                if self.global_defaults.localization_only_mode {
                    let map_read_lock = self.map.read();
                    let mappoint = map_read_lock.get_mappoint(&mp_id).unwrap();

                    if mappoint.observations().len() > 0 {
                        self.matches_in_frame+=1;
                    }
                } else {
                    self.matches_in_frame += 1;
                }

            } else if !S::is_mono() {
                //TODO: [Stereo] Handle Stereo Case
                //mCurrentFrame.mappoint_matches[i] = static_cast<MapPoint*>(NULL);
                // Following probably needs to be inside map actor:
                // self.current_frame.as_mut().unwrap().mappoint_matches.remove(&index);
                // But why delete these points at all? and why here?
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if self.current_frame.id < self.last_reloc_frame_id + (self.global_defaults.max_frames as i32) && self.matches_in_frame<50 {
            return false;
        }

        match self.state {
            TrackingState::RecentlyLost => { return self.matches_in_frame > 10; },
            _ => {}
        }

        match S::sensor_type() {
            Sensor::ImuMono => { 
                return !(self.matches_in_frame<15 && self.is_imu_initialized()) && !(self.matches_in_frame<50 && !self.is_imu_initialized())
            },
            Sensor::ImuStereo | Sensor::ImuRgbd => { return self.matches_in_frame >= 15; },
            _ => { return self.matches_in_frame > 30 }
        }
    }

    fn create_new_keyframe(&mut self, actor_ids: &std::collections::HashMap<String, axiom::actors::Aid>) {
        //CreateNewKeyFrame
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216

        let mut new_kf;
        {
            let map_read_lock = self.map.read();
            let map_id = map_read_lock.id;
            let vocabulary = &map_read_lock.vocabulary;

            new_kf = KeyFrame::new(&self.current_frame, map_id, &vocabulary);
        }

        self.current_frame.ref_kf_id = Some(new_kf.id);

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

        // KeyFrame created here and sent to LM, but not inserted into map yet
        let lm = actor_ids.get(LOCAL_MAPPING).unwrap();
        let actor_msg = GLOBAL_PARAMS.get::<String>(LOCAL_MAPPING, "actor_message");
        match actor_msg.as_ref() {
            "KeyFrameMsg" => {
                lm.send_new(KeyFrameMsg{
                    kf: new_kf,
                    actor_ids: actor_ids.clone()
                }).unwrap();
            },
            _ => {
                println!("Invalid Message type: selecting KeyFrameMsg");
                lm.send_new(KeyFrameMsg{
                    kf: new_kf,
                    actor_ids: actor_ids.clone()
                }).unwrap();
            },
        }
    }

    fn need_new_keyframe(&self) -> bool {
        //NeedNewKeyFrame

        let kfs_in_map = self.kfs_in_map();
        {
            let not_enough_frames_since_last_reloc = (self.current_frame.id < self.last_reloc_frame_id + (self.global_defaults.max_frames as i32)) && (kfs_in_map > (self.global_defaults.max_frames as u32));
            let imu_not_initialized = S::is_imu() && !self.is_imu_initialized();

            let map_read_lock = self.map.read();
            let too_close_to_last_kf = match self.last_kf_ts {
                Some(timestamp) => self.current_frame.timestamp - timestamp < Duration::milliseconds(250),
                None => false
            };

            if self.global_defaults.localization_only_mode || not_enough_frames_since_last_reloc || (imu_not_initialized && too_close_to_last_kf) {
                return false;
            } else if imu_not_initialized && !too_close_to_last_kf {
                return true;
            }
        }

        // Tracked MapPoints in the reference keyframe
        let mut min_observations = 3;
        if kfs_in_map <= 2 {
            min_observations = 2;
        }

        let tracked_mappoints;
        {
            let map_lock = self.map.read();
            tracked_mappoints = match self.ref_kf_id {
                Some(kf_id) => {
                    let kf = map_lock.get_keyframe(&kf_id).unwrap();
                    map_lock.tracked_mappoints_for_keyframe(kf, min_observations) as f32
                },
                None => 0.0
            };
        }

        // Check how many "close" points are being tracked and how many could be potentially created.
        let (tracked_close, non_tracked_close) = self.current_frame.check_close_tracked_mappoints();
        let need_to_insert_close = (tracked_close<100) && (non_tracked_close>70);

        // Thresholds
        let th_ref_ratio = match S::sensor_type() {
            Sensor::Mono => 0.9,
            Sensor::ImuMono => {
                // Points tracked from the local map
                if self.matches_in_frame > 350 { 0.75 } else { 0.90 }
            },
            Sensor::ImuStereo | Sensor::Stereo | Sensor::ImuRgbd => 0.75,
            Sensor::Rgbd => { if kfs_in_map < 2 { 0.4 } else { 0.75 } }
        }; // Sofiya: is Rgbd right? See ORBSLAM3 code https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3142

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        let c1a = self.frames_since_last_kf >= (self.global_defaults.max_frames as i32);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // Note: removed localmappingidle from here
        let c1b = self.frames_since_last_kf >= (self.global_defaults.min_frames as i32);
        //Condition 1c: tracking is weak
        let sensor_is_right = match S::sensor_type() {
            Sensor::Mono | Sensor::ImuMono | Sensor::ImuStereo | Sensor::ImuRgbd => false,
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

        let c3 = S::is_imu() && close_to_last_kf;

        let recently_lost = match self.state {
            TrackingState::RecentlyLost => true,
            _ => false
        };
        let sensor_is_imumono = match S::sensor_type() {
            Sensor::ImuMono => true,
            _ => false
        };
        let c4 = ((self.matches_in_frame < 75 && self.matches_in_frame > 15) || recently_lost) && sensor_is_imumono;

        // Note: removed code here about checking for idle local mapping and/or interrupting bundle adjustment
        return ((c1a||c1b||c1c) && c2)||c3 ||c4;
    }

    fn store_pose_info_for_trajectory(&mut self) {
        match self.current_frame.pose {
            Some(pose) => {
                let map_read_lock = self.map.read();
                let ref_kf_id = self.current_frame.ref_kf_id.unwrap();
                let ref_kf = map_read_lock.get_keyframe(&ref_kf_id).unwrap();
                let tcr = pose * ref_kf.pose.inverse();
                self.trajectory_poses.push(tcr);
                self.trajectory_times.push(self.current_frame.timestamp);
                self.trajectory_keyframes.push(ref_kf_id);
            },
            None => {
                // This can happen if tracking is lost
                // Duplicate last element of each vector
                if let Some(last) = self.trajectory_poses.last().cloned() { self.trajectory_poses.push(last); }
                if let Some(last) = self.trajectory_times.last().cloned() { self.trajectory_times.push(last); }
                if let Some(last) = self.trajectory_keyframes.last().cloned() { self.trajectory_keyframes.push(last); }
            }
        };

    }

    //* Helper functions */
    fn kfs_in_map(&self) -> u32 {
        let kfs_in_map;
        {
            let map_read_lock = self.map.read();
            kfs_in_map = map_read_lock.num_keyframes();
        }
        kfs_in_map as u32
    }

    //* Next steps */
    fn create_new_map(&self) -> bool {
        todo!("Multimaps: Atlas::CreateNewMap");
    }

    fn relocalization(&self) -> bool {
        todo!("Relocalization");
    }

    fn is_imu_initialized(&self) -> bool {
        let imu_initialized;
        {
            let map_read_lock = self.map.read();
            imu_initialized = map_read_lock.imu_initialized;
        }
        imu_initialized
    }

    fn predict_state_imu(&self) -> bool {
        todo!("IMU: PredictStateIMU");
    }

    fn preintegrate_imu(&self) {
        todo!("IMU: PreintegrateIMU");
    }

    fn reset_frame_imu(&self) -> bool {
        todo!("IMU: ResetFrameIMU");
    }
}
impl<S: SensorType + 'static> Function for DarvisTrackingBack<S> {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        if let Some(msg) = message.content_as::<FeatureMsg>() {
            self.handle_message(msg);
        }

        Ok(Status::done(()))
    }
}

#[derive(Debug, Clone)]
pub struct Initialization<S: SensorType> {
    // Initialization (Monocular)
    pub mp_matches: HashMap<u32, u32>,// ini_matches .. mvIniMatches;
    pub prev_matched: Vec<Point2f>,// std::vector<cv::Point2f> mvbPrevMatched;
    pub p3d: DVVectorOfPoint3f,// std::vector<cv::Point3f> mvIniP3D;
    pub ready_to_initializate: bool,
    pub initial_frame: Option<Frame<S>>,
    pub last_frame: Option<Frame<S>>,
    pub current_frame: Option<Frame<S>>
}

impl<S: SensorType> Initialization<S> {
    pub fn new() -> Self {
        Self {
            mp_matches: HashMap::new(),
            prev_matched: Vec::new(),
            p3d: DVVectorOfPoint3f::empty(),
            ready_to_initializate: false,
            initial_frame: None,
            last_frame: None,
            current_frame: None
        }
    }

    pub fn initialize(&mut self, current_frame: &Frame<S>, orb_matcher: &ORBmatcher, camera: &Camera) -> bool {
        // Only set once at beginning
        if self.initial_frame.is_none() {
            self.initial_frame = Some(current_frame.clone());
        }
        // If we never did initialization (ie, only 1 frame passed): initial, current, and last frame will all be set to the 1st frame.
        // If we've done the first step of initialization, update last frame and current frame.
        self.last_frame = match &self.current_frame {
            Some(frame) => Some(frame.clone()),
            None => Some(current_frame.clone())
        };
        self.current_frame = Some(current_frame.clone());

        match S::sensor_type() {
            Sensor::Mono | Sensor::ImuMono => self.monocular_initialization(orb_matcher, camera),
            _ => self.stereo_initialization()
        }
    }

    fn monocular_initialization(&mut self, orb_matcher: &ORBmatcher, camera: &Camera) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        if !self.ready_to_initializate && self.current_frame.as_ref().unwrap().keypoints_data.num_keypoints() > 100 {
            // Set Reference Frame
             self.prev_matched.resize(self.current_frame.as_ref().unwrap().keypoints_data.num_keypoints() as usize, Point2f::default());

            for i in 0..self.current_frame.as_ref().unwrap().keypoints_data.num_keypoints() as usize {
                self.prev_matched[i] = self.current_frame.as_ref().unwrap().keypoints_data.keypoints_un().get(i).unwrap().pt.clone();
            }

            match S::sensor_type() {
                Sensor::ImuMono => {
                    //TODO: (IMU) 
                    //Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L2467

                    // if(mpImuPreintegratedFromLastKF)
                    // {
                    //     delete mpImuPreintegratedFromLastKF;
                    // }
                    // mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                    // self.current_frame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
                },
                _ => {}
            }
            self.ready_to_initializate = true;
            return false;
        } else {
            if self.current_frame.as_ref().unwrap().keypoints_data.num_keypoints() <=100 || matches!(S::sensor_type(), Sensor::ImuMono) && self.last_frame.as_ref().unwrap().timestamp - self.initial_frame.as_ref().unwrap().timestamp > Duration::seconds(1) {
                self.ready_to_initializate = false;
                return false;
            }

            // Find correspondences
            // TODO 10/17: uncomment when bindings are done
            let mut nmatches =10; //orb_matcher.search_for_initialization(
            //     &self.initial_frame.unwrap(), 
            //     &self.current_frame.unwrap(), 
            //     &mut self.prev_matched,
            //     &mut self.mp_matches,
            //     100
            // );

            // Check if there are enough correspondences
            if nmatches < 100 {
                self.ready_to_initializate = false;
                return false;
            }

            let mut tcw = Pose::default();
            let mut vb_triangulated = Vec::<bool>::new();

            // TODO 10/17: uncomment when bindings are done
            let reconstruct_success = false;//camera.reconstruct_with_two_views(
            //     self.initial_frame.unwrap().keypoints_data.keypoints_un(),
            //     self.current_frame.unwrap().keypoints_data.keypoints_un(),
            //     &self.mp_matches,
            //     &mut tcw,
            //     &mut self.p3d,
            //     &mut vb_triangulated
            // );

            if reconstruct_success {
                let keys = self.mp_matches.keys().cloned().collect::<Vec<_>>();
                for index in keys {
                    if !vb_triangulated[index as usize] {
                        self.mp_matches.remove(&index);
                        nmatches-=1;
                    }
                }

                self.initial_frame.as_mut().unwrap().set_pose(Pose::default());
                self.current_frame.as_mut().unwrap().set_pose(tcw);

                return true;
            } else {
                return false;
            }
        }
    }

    fn stereo_initialization(&mut self) -> bool {
        todo!("Stereo: StereoInitialization");
    }
}
