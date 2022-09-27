use std::sync::Arc;
use std::collections::HashMap;
use axiom::prelude::*;
use chrono::{prelude::*, Duration};
use nalgebra::Matrix3;
use opencv::{prelude::*, core::Point2f, types::{VectorOfPoint3f},};
use darvis::{
    dvutils::*,
    map::{
        pose::Pose, map::Map, map_actor::MapWriteMsg, map_actor::MAP_ACTOR,
        keyframe::KeyFrame, frame::Frame, map::Id, misc::IMUBias, mappoint::MapPoint, orbmatcher::ORBmatcher, camera::DVCamera, localmapping::LocalMapping
    },
    utils::{camera::DVCamera, orbmatcher::ORBmatcher},
    lockwrap::ReadOnlyWrapper,
    plugin_functions::Function,
    global_params::*,
};
use crate::{
    modules::optimizer::optimize_pose,
    modules::messages::feature_msg::FeatureMsg
};

#[derive(Debug, Clone)]
pub struct DarvisTrackingBack {
    state: TrackingState,

    // Map
    map: ReadOnlyWrapper<Map>,
    map_actor: Option<Aid>,

    // Feature matching
    orb_matcher: ORBmatcher,
    temporal_points: Vec<MapPoint>,

    // Frames
    first_frame: bool,
    last_frame: Option<Frame>,
    last_frame_id: Id,
    current_frame: Option<Frame>,
    initial_frame: Option<Frame>,
    reference_frame: Option<Frame>, //Redundant hence will remove once frame database is established.

    // KeyFrames
    reference_keyframe_id: Id,
    reference_keyframe: Option<Id>,
    last_keyframe_id: Id,

    //Camera
    camera: DVCamera,

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    relative_frame_poses: Vec<Pose>,

    // Initialization (Monocular)
    // std::vector<int> mvIniLastMatches;
    ini_matches: HashMap<i32, Id>,// std::vector<int> mvIniMatches;
    prev_matched: Vec<Point2f>,// std::vector<cv::Point2f> mvbPrevMatched;
    ini_p3d: VectorOfPoint3f,// std::vector<cv::Point3f> mvIniP3D;
    ready_to_initializate: bool,

    // IMU 
    velocity: Option<Pose>,
    last_bias: Option<IMUBias>,

    // Relocalization
    last_reloc_frame_id: Id,
    timestamp_lost: Option<DateTime<Utc>>,

    // Defaults set from global variables
    // Never change, so just get once instead of having to look up each time
    recently_lost_cutoff: Duration,
    sensor: Sensor,
    localization_only_mode: bool,
    frames_to_reset_imu: i32,
    insert_kfs_when_lost: bool,
    min_num_features: i32,

    //Camera
    camera: Option<DVCamera>,

    K: Option<Mat>, // Might be redundant
    K_: Option<Matrix3<f32>>,

    // Reference Keyframe.
    mpReferenceKF: Option<Id>,//,*mut KeyFrame,//KeyFrame* mpReferenceKF;

    map_actor: Option<Aid>, //QUICK FIX to get going with the message communication,

    //mTrackedFr: i32, // frames with estimated pose
    //IMU related variables
    mnLastRelocFrameId : i32, 
    mnFramesToResetIMU : i32,
    mbMapUpdated : bool, 
    //Current matches in frame
    mnMatchesInliers : i32, 
    mMaxFrames : i32 , //fps
    mpLocalMapper : Option<LocalMapping>,
}

#[derive(Debug, Clone)]
enum TrackingState {
    NotInitialized,
    Lost,
    RecentlyLost,
    Ok
}

impl DarvisTrackingBack {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisTrackingBack {
        let recently_lost_cutoff: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "recently_lost_cutoff");
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");
        let localization_only_mode: bool = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "localization_only_mode");
        let frames_to_reset_imu: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "frames_to_reset_IMU");
        let insert_kfs_when_lost: bool = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "insert_KFs_when_lost");
        let min_num_features: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "min_num_features");

        // Camera initialization
        let fx: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_fx");
        let fy: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_fy");
        let cx: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_cx");
        let cy: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "camera_cy");
        let bf: f64 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "stereo_baseline_times_fx");
        let th_depth: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "thdepth");
        let mut camera = DVCamera::new(fx, fy, cx, cy, th_depth, bf);

        let mut tracking = DarvisTrackingBack {
            first_frame: true,
            last_frame: None,
            map: map,
            state: TrackingState::NotInitialized,
            last_frame_id: -1,
            temporal_points: Vec::new(),
            reference_keyframe_id: -1,
            current_frame: None,
            initial_frame: None,
            reference_frame: None,
            last_keyframe_id: -1,
            relative_frame_poses: Vec::new(),
            orb_matcher: ORBmatcher::new(0.9,true),
            ini_matches: HashMap::new(),
            prev_matched: Vec::new(),
            ini_p3d: VectorOfPoint3f::new(),
            ready_to_initializate: false,
            recently_lost_cutoff: Duration::seconds(recently_lost_cutoff.into()),
            sensor: sensor,
            localization_only_mode: localization_only_mode,
            frames_to_reset_imu: frames_to_reset_imu,
            insert_kfs_when_lost: insert_kfs_when_lost,
            min_num_features: min_num_features,
            last_bias: None,
            velocity: None,
            timestamp_lost: None,
            last_reloc_frame_id: 0,
            camera: camera,
            reference_keyframe: None,
            map_actor: None,
            //mTrackedFr: 0, // unused
            mnLastRelocFrameId: 0, 
            mnFramesToResetIMU: 0,
            mbMapUpdated: false,
            mnMatchesInliers: 0,
            mMaxFrames: 30, // fps set default as 30
            mpLocalMapper: None,
        };
        tracking
    }

    fn tracking_backend(&mut self, _context: Context, msg: Arc<FeatureMsg>) {
        let map_actor = msg.actor_ids.get(MAP_ACTOR).unwrap();
        self.map_actor = Some(map_actor.clone());

        self.last_frame_id += 1;
        self.current_frame = Some(Frame::new(
            self.last_frame_id, 
            Utc::now(), 
            msg.keypoints.cv_vector_of_keypoint(),
            msg.descriptors.grayscale_to_cv_mat(),
            msg.image_width,
            msg.image_height
        ));

        // TODO (reset): Reset map because local mapper set the bad imu flag
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1808

        // TODO (multimaps): Create new map if timestamp older than previous frame arrives
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1820

        // TODO (reset) TODO (multimaps): Timestamp jump detected, either reset active map or create new map
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1828
        // (entire block)

        // TODO: set bias of new frame = to bias of last
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1860

        if !self.sensor.is_mono() {
            self.preintegrate_IMU();
        }

        // TODO: update map change index. Not sure why this is useful
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1890

        // Initial estimation of camera pose and matching
        let mut success = match self.state {
            TrackingState::NotInitialized => {
                let ok = match self.sensor.is_mono() {
                    true => self.monocular_initialization(),
                    false => self.stereo_initialization()
                };
                if !ok {
                    self.last_frame = Some(self.current_frame.as_ref().unwrap().clone());
                    return;
                }
                ok
            },
            TrackingState::Ok => {
                let mut ok;

                let not_enough_frames_since_last_reloc = self.current_frame.as_ref().unwrap().id < self.last_reloc_frame_id + 2;
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
                    let enough_frames_to_reset_imu = self.current_frame.as_ref().unwrap().id <= self.last_reloc_frame_id + self.frames_to_reset_imu;
                    if enough_frames_to_reset_imu && self.sensor.is_imu() {
                        self.state = TrackingState::Lost;
                    } else if self.kfs_in_map() > 10 {
                        self.state = TrackingState::RecentlyLost;
                        self.timestamp_lost = Some(self.current_frame.as_ref().unwrap().timestamp);
                    } else {
                        self.state = TrackingState::Lost;
                    }
                }
                ok
            },
            TrackingState::RecentlyLost => {
                println!("TRACK: State recently lost");
                let mut ok;
                let time_since_lost = self.current_frame.as_ref().unwrap().timestamp - self.timestamp_lost.unwrap();
                if self.sensor.is_imu() {
                    if self.is_imu_initialized() {
                        ok = self.predict_state_IMU();
                    } else {
                        ok = false;
                    }

                    if time_since_lost > self.recently_lost_cutoff {
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
                    let map_msg = MapWriteMsg::reset_active_map();
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
            if self.sensor.is_imu() {
                println!("TRACK: Track lost for less than 1 second");

                // TODO (reset) TODO (IMU): Reset map because local mapper set the bad imu flag
                // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2149
            }
            self.state = TrackingState::RecentlyLost;
            self.timestamp_lost = Some(self.current_frame.as_ref().unwrap().timestamp);
        }

        // TODO (IMU)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
        // This code not done, so double check with C++ reference.
        // {
        //     // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        //     let enough_frames_to_reset_imu = self.current_frame.as_ref().unwrap().id <= self.last_reloc_frame_id + self.frames_to_reset_imu;
        //     if enough_frames_to_reset_imu && self.current_frame.as_ref().unwrap().id > self.frames_to_reset_imu && self.sensor.is_imu() && self.is_imu_initialized() {
        //         // Load preintegration
        //         // This code directly copied from C++, obviously needs to be changed
        //         // pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
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
            if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !self.current_frame.as_mut().unwrap().pose.is_none() {
                let last_pose = last_frame.unwrap().pose.as_ref().unwrap();
                let _last_Twc = last_pose.inverse();
                // TODO (IMU)
                self.velocity = Some(self.current_frame.as_ref().unwrap().pose.as_ref().unwrap().clone() * _last_Twc);
            } else {
                self.velocity = None;
            }

            // Clean VO matches
            self.current_frame.as_mut().unwrap().clean_VO_matches();
            // Delete temporal MapPoints
            self.temporal_points = Vec::new();

            // Check if we need to insert a new keyframe
            let need_kf = self.need_new_keyframe();
            let insert_if_lost_anyway = self.insert_kfs_when_lost && matches!(self.state, TrackingState::RecentlyLost) && self.sensor.is_imu();
            if need_kf && (success || insert_if_lost_anyway) {
                self.create_new_keyframe();
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            self.current_frame.as_mut().unwrap().delete_VO_matches_if_not_outliers();
        }

        // Reset if the camera get lost soon after initialization
        if matches!(self.state, TrackingState::Lost) {
            if self.kfs_in_map() <= 10 {
                let map_msg = MapWriteMsg::reset_active_map();
                map_actor.send_new(map_msg).unwrap();
                return;
            }
            if self.sensor.is_imu() && !self.is_imu_initialized() {
                println!("TRACK: Track lost before IMU initialisation, resetting...",);
                let map_msg = MapWriteMsg::reset_active_map();
                map_actor.send_new(map_msg).unwrap();
                return;
            }

            self.create_new_map();
            return;
        }

        if self.current_frame.as_ref().unwrap().reference_keyframe_id.is_none() {
            self.current_frame.as_mut().unwrap().reference_keyframe_id = Some(self.reference_keyframe_id);
        }

        self.last_frame = self.current_frame.clone();

        match self.state {
            TrackingState::Ok | TrackingState::RecentlyLost => self.store_pose_info_for_trajectory(),
            _ => {}
        }

        // self.align(_context, &msg);
    }

    //* MVP */
    fn monocular_initialization(&mut self) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        let current_frame = self.current_frame.as_ref().unwrap();
        if !self.ready_to_initializate {
            // Set Reference Frame
            if current_frame.keypoints.len() > 100 {
                self.initial_frame = self.current_frame.clone();
                self.last_frame = self.current_frame.clone();
                self.prev_matched.resize(current_frame.keypoints_un.len(), Point2f::default());

                for i in 0..current_frame.keypoints_un.len() {
                    self.prev_matched[i] = current_frame.keypoints_un.get(i).unwrap().pt.clone();
                }

                if self.sensor.is_mono() && self.sensor.is_imu() {
                    //TODO: (IMU) 
                    //Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L2467

                    // if(mpImuPreintegratedFromLastKF)
                    // {
                    //     delete mpImuPreintegratedFromLastKF;
                    // }
                    // mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                    // mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;    
                }

                self.ready_to_initializate = true;
                return true;
            }
        } else {
            let initial_frame = self.initial_frame.as_ref().unwrap();
            if (current_frame.keypoints.len() <=100)||((self.sensor.is_mono() && self.sensor.is_imu())&&(self.last_frame.as_ref().unwrap().timestamp - initial_frame.timestamp> Duration::seconds(1))) {
                self.ready_to_initializate  = false;
                return false;
            }

            // Find correspondences
            let mut nmatches = self.orb_matcher.search_for_initialization(
                &initial_frame, 
                &current_frame, 
                &mut self.prev_matched,
                &mut self.ini_matches,
                100
            );

            // Check if there are enough correspondences
            if nmatches < 100 {
                self.ready_to_initializate  = false;
                return false;
            }

            let mut Tcw = Pose::default();
            let mut vb_triangulated = Vec::<bool>::new();
            //vector<bool> vb_triangulated; // Triangulated Correspondences (mvIniMatches)

            let reconstruct_success = self.camera.reconstruct_with_two_views(
                &self.initial_frame.as_ref().unwrap().keypoints_un,
                &self.current_frame.as_ref().unwrap().keypoints_un,
                &self.ini_matches,
                &mut Tcw,
                &mut self.ini_p3d,
                &mut vb_triangulated
            );

            if reconstruct_success {
                let keys = self.ini_matches.keys().cloned().collect::<Vec<_>>();
                for index in keys {
                    if !vb_triangulated[index as usize] {
                        self.ini_matches.remove(&index);
                        nmatches-=1;
                    }
                }

                // Set Frame Poses
                let map_msg1 = MapWriteMsg::set_pose(initial_frame.id, Pose::default());
                let map_msg2 = MapWriteMsg::set_pose(current_frame.id, Tcw);
                self.map_actor.as_ref().unwrap().send_new(map_msg1).unwrap();
                self.map_actor.as_ref().unwrap().send_new(map_msg2).unwrap();

                self.create_initial_map_monocular();
            }
        }

        return false;
    }

    pub fn create_initial_map_monocular(&mut self) {
        todo!("Implement create_initial_map_monocular");
        // SOFIYA 9/26 TODO CreateInitialMapMonocular
    }


    fn track_reference_keyframe(&mut self) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2720

        // Compute Bag of Words vector
        self.current_frame.as_mut().unwrap().compute_BoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        let matcher = ORBmatcher::new(0.7, true);
        let mut vp_mappoint_matches = HashMap::<i32, Id>::new();

        let mut nmatches;
        if self.reference_keyframe.is_some() {
            //TODO: CHECK if we really need to use "KeyFrame" object for below code for BoW search,
            // For now just using current and ref "Frame" object.
            // let map_read_lock = self.map.read();
            // let ref_kf = map_read_lock.get_keyframe(&self.reference_keyframe.unwrap());
            // let cur_kf = map_read_lock.get_keyframe(&self.current_frame.as_ref().unwrap().id);
            // if ref_kf.is_some()
            // {
            //     nmatches = matcher.search_by_bow(ref_kf.unwrap(), cur_kf.unwrap(), &mut vpMapPointMatches);
            // }
            // else
            // {
            //     todo!("fix invalid ref KF assignment");
            // }

            let map_read_lock = self.map.read();
            let _ref_kf = map_read_lock.get_keyframe(&self.reference_keyframe.unwrap());

            let cur_f = self.current_frame.as_ref().unwrap();

            if self.reference_frame.is_some() {
                //TODO: Use BoW for searching with mappoints, right now not using any mappoint.
                //nmatches = matcher.search_by_bow(self.reference_frame.as_ref().unwrap(), cur_f, &mut vpMapPointMatches);

                nmatches = matcher.search_for_initialization(self.reference_frame.as_ref().unwrap(), cur_f, &mut self.prev_matched, &mut vp_mappoint_matches, 100);
            } else {
                todo!("fix invalid ref KF assignment");
            }

        } else {
            todo!("fix ref KF assignment");
        }

        if nmatches<15 {
            println!("TRACK_REF_KF: Less than 15 matches!!\n");
            return false;
        }

        self.current_frame.as_mut().unwrap().mappoint_matches = vp_mappoint_matches.clone();
        self.current_frame.as_mut().unwrap().set_pose(&self.last_frame.as_ref().unwrap().get_pose());

        //mCurrentFrame.PrintPointDistribution();
        // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;

        let (pose, inliers) = optimize_pose(&mut self.current_frame.as_mut().unwrap(), &self.map);
        if inliers != 0 {
            let map_msg = MapWriteMsg::set_pose(self.current_frame.as_ref().unwrap().id, pose.unwrap());
            self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();
        }

        // Discard outliers
        let nmatches_map = self.discard_outliers();

        match self.sensor.is_imu() {
            true => { return true; },
            false => { return nmatches_map >= 10; }
        };
    }

    fn track_with_motion_model(&mut self) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2854
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

        let enough_frames_to_reset_imu = self.current_frame.as_ref().unwrap().id <= self.last_reloc_frame_id + self.frames_to_reset_imu;
        if self.is_imu_initialized() && enough_frames_to_reset_imu {
            // Predict state with IMU if it is initialized and it doesnt need reset
            self.predict_state_IMU();
            return true;
        } else {
            self.current_frame.as_mut().unwrap().set_pose(&(
                self.velocity.unwrap() * self.last_frame.as_ref().unwrap().get_pose()
            ));
        }

        self.current_frame.as_mut().unwrap().clear_mappoints();

        // Project points seen in previous frame
        let th = match self.sensor.is_mono() {
            true => 15,
            false => 7
        };
        let mut matches = self.orb_matcher.search_by_projection_with_threshold(
            self.current_frame.as_ref().unwrap(),
            self.last_frame.as_ref().unwrap(),
            th,
            self.sensor.is_mono(),
            &self.camera,
            &self.map
        ); 

        // If few matches, uses a wider window search
        if matches.len() < 20 {
            println!("TRACK: track_with_motion_model... not enough matches, wider window search");
            self.current_frame.as_mut().unwrap().clear_mappoints();
            matches = self.orb_matcher.search_by_projection_with_threshold(
                self.current_frame.as_ref().unwrap(),
                self.last_frame.as_ref().unwrap(),
                2 * th,
                self.sensor.is_mono(),
                &self.camera,
                &self.map
            ); 
        }

        if matches.len() < 20 {
            println!("TRACK: track_with_motion_model... not enough matches!!");
            return self.sensor.is_imu();
        }

        // Optimize frame pose with all matches
        let (pose, inliers) = optimize_pose(&mut self.current_frame.as_mut().unwrap(), &self.map);
        if inliers != 0 {
            let map_msg = MapWriteMsg::set_pose(self.current_frame.as_ref().unwrap().id, pose.unwrap());
            self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();
        }

        // Discard outliers
        let nmatches_map = self.discard_outliers();

        // TODO (localization only)
        // if(mbOnlyTracking) {
        //     mbVO = nmatchesMap<10;
        //     return nmatches>20;
        // }

        match self.sensor.is_imu() {
            true => { return true; },
            false => { return nmatches_map >= 10; }
        };
    }

    fn update_last_frame(&self) {
        // Ref code:  https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2781

        // Update pose according to reference keyframe
        let Tlr = self.relative_frame_poses.last().unwrap();
        let mut reference_kf_pose;
        {
            let map_lock = self.map.read();
            let reference_kf = self.last_frame.as_ref().unwrap().reference_keyframe_id;
            reference_kf_pose = map_lock.get_keyframe(&reference_kf.unwrap()).unwrap().pose;
        }
        let map_msg = MapWriteMsg::set_pose(self.last_frame.as_ref().unwrap().id, *Tlr * reference_kf_pose);
        self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();

        if self.sensor.is_mono() || self.last_keyframe_id == self.last_frame_id {
            return;
        }

        {
            // TODO (RGBD and Stereo), don't need this for mono
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

    //void Tracking::UpdateLocalMap()
    fn update_local_map(&mut self)
    {
        todo!("update_local_map");
        // // This is for visualization
        // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // // Update
        // UpdateLocalKeyFrames();
        // UpdateLocalPoints();
    }

    //void Tracking::SearchLocalPoints()
    fn search_local_points(&mut self)
    {
        todo!("search_local_points");
        // // Do not search map points already matched
        // for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
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
        //             pMP->last_frame_seen = mCurrentFrame.mnId;
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

        //     if(pMP->last_frame_seen == mCurrentFrame.mnId)
        //         continue;
        //     if(pMP->isBad())
        //         continue;
        //     // Project (this fills MapPoint variables for matching)
        //     if(mCurrentFrame.isInFrustum(pMP,0.5))
        //     {
        //         pMP->IncreaseVisible();
        //         nToMatch++;
        //     }
        //     if(pMP->mbTrackInView)
        //     {
        //         mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
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
        //     if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
        //         th=5;

        //     if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
        //         th=15; // 15

        //     int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
        // }

    }

    fn track_local_map(&mut self) -> bool {
        // TODO
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2949
        //todo!("TRACK: local map");

        //self.mTrackedFr+=1; unused
        self.update_local_map();
        self.search_local_points();

        // TOO check outliers before PO
        let (mut aux1, mut aux2) = (0,0);


        for i in 0..self.current_frame.as_ref().unwrap().mvpMapPoints.len()
        {
            if self.current_frame.as_ref().unwrap().mvpMapPoints[i] !=-1
            {
                aux1+=1;
                if self.current_frame.as_ref().unwrap().mvbOutlier[i]
                {
                    aux2+=1;
                }

            }
        }

        let mut inliers = 0;
        if !self.is_imu_initialized()
        {
            //Optimizer::PoseOptimization(&mCurrentFrame);
            
            //todo!("Optimizer::PoseOptimization(&mCurrentFrame)");
        }
        else
        {
            if self.current_frame.as_ref().unwrap().id <= self.mnLastRelocFrameId+self.mnFramesToResetIMU
            {
                //Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
                //Optimizer::PoseOptimization(&mCurrentFrame);
                todo!("Optimizer::PoseOptimization(&mCurrentFrame)");
            }
            else
            {
                // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
                if !self.mbMapUpdated //  && (mnMatchesInliers>30))
                {
                    //Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                    //inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                    todo!("Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame)");
                }
                else
                {
                    //Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                    //inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());                    
                    todo!("Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame)");
                }
            }
        }

        aux1 = 0;
        aux2 = 0;

        for i in 0..self.current_frame.as_ref().unwrap().mvpMapPoints.len()
        {
            if self.current_frame.as_ref().unwrap().mvpMapPoints[i] !=-1
            {
                aux1+=1;
                if self.current_frame.as_ref().unwrap().mvbOutlier[i]
                {
                    aux2+=1;
                }

            }          
        }


        self.mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for i in 0..self.current_frame.as_ref().unwrap().mvpMapPoints.len()
        {
            if self.current_frame.as_ref().unwrap().mvpMapPoints[i] !=-1
            {
                aux1+=1;
                if self.current_frame.as_ref().unwrap().mvbOutlier[i]
                {
                    aux2+=1;
                }

            }          
        }

        for i in 0..self.current_frame.as_ref().unwrap().mvpMapPoints.len()
        {
            if self.current_frame.as_ref().unwrap().mvpMapPoints[i] !=-1
            {
                if !self.current_frame.as_ref().unwrap().mvbOutlier[i]
                {
                    let pMP = self.current_frame.as_ref().unwrap().mvpMapPoints[i];

                    //mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    let map_msg = MapWriteMsg::increase_found(&pMP, 1);
                    self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();

                    //TODO: need to implemented mbOnlyTracking mode, for now setting as false
                    if !false //(!mbOnlyTracking)
                    {
                        let map_read_lock = self.map.read();
                        let curr_map_pt = map_read_lock.get_mappoint(&self.current_frame.as_ref().unwrap().mvpMapPoints[i]);
    
                        if curr_map_pt.unwrap().observations()>0
                        {
                            self.mnMatchesInliers+=1;
                        }

                    }
                    else
                    {
                        self.mnMatchesInliers+=1;
                    }

                }
                else if  !self.sensor.is_mono()// (mSensor==System::STEREO)
                {
                    //TODO: [Stereo] Handle Stereo Case
                    //mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                    self.current_frame.as_mut().unwrap().mvpMapPoints[i] = -1;
                    todo!("Handle stereo case");

                }

            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        self.mpLocalMapper.unwrap().mnMatchesInliers = self.mnMatchesInliers;


        if self.current_frame.as_ref().unwrap().id < self.mnLastRelocFrameId +self.mMaxFrames && self.mnMatchesInliers<50
        {
            return false;
        }

        if self.mnMatchesInliers>10 
        {
            match self.state {
                TrackingState::RecentlyLost =>  return true,
                _ => {}
            }
        }


        if self.sensor.is_imu() && self.sensor.is_mono() //(mSensor == System::IMU_MONOCULAR)
        {
            if((self.mnMatchesInliers<15 && self.is_imu_initialized())||(self.mnMatchesInliers<50 && !self.is_imu_initialized()))
            {
                return false;
            }
            else
            {
                return true;
            }

        }
        else if self.sensor.is_imu() && !self.sensor.is_mono() //(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if(self.mnMatchesInliers<15)
            {
                return false;
            }
            else
            {
                return true;
            }

        }
        else
        {
            if self.mnMatchesInliers<30 // (mnMatchesInliers<30)
            {
                return false;
            }
            else
            {
                return true;
            }
        }        

    }

    fn create_new_keyframe(&self) {
        // SOFIYA 9/26 TODO
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216
        // let new_kf = KeyFrame {
        //     id: 1,
        //     // timestamp: 1,
        //     // map_points: Arc::new(),
        // };
        // let map_msg = MapWriteMsg::new_keyframe(1, new_kf);

        // let map_actor = msg.actor_ids.get(MAP_ACTOR).unwrap();
        // map_actor.send_new(map_msg).unwrap();
    }

    fn need_new_keyframe(&self) -> bool {
        // SOFIYA 9/26 TODO
        // let map_lock = self.map.read();
        // let num_kfs = map_lock.num_kfs;
        return true;
    }

    fn store_pose_info_for_trajectory(&self) {
        // TODO: Convert this to Rust obv
        // SOFIYA 9/26 TODO
        // if self.current_frame.is_set {

        // }
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // if(mCurrentFrame.isSet())
        // {
        //     Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
        //     relative_frame_poses.push_back(Tcr_);
        //     mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
        //     mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        //     mlbLost.push_back(mState==LOST);
        // }
        // else
        // {
        //     // This can happen if tracking is lost
        //     relative_frame_poses.push_back(relative_frame_poses.back());
        //     mlpReferences.push_back(mlpReferences.back());
        //     mlFrameTimes.push_back(mlFrameTimes.back());
        //     mlbLost.push_back(mState==LOST);
        // }
    }

    //* Helper functions */
    fn kfs_in_map(&self) -> u64 {
        let kfs_in_map;
        {
            let map_read_lock = self.map.read();
            kfs_in_map = map_read_lock.num_kfs;
        }
        kfs_in_map
    }

    fn discard_outliers(&self) -> i32 {
        let mut nmatches_map = 0;
        let matches = &self.current_frame.as_ref().unwrap().mappoint_matches;
        for (_index, mp_id) in matches {
            if self.current_frame.as_ref().unwrap().mappoint_outliers.contains_key(mp_id) {
                {
                    let map_msg = MapWriteMsg::delete_mappoint_match(self.current_frame.as_ref().unwrap().id, *mp_id, true);
                    self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();
                }

                //TODO:[Stereo] check for Stereo camera flow
                // pMP.mbTrackInView= false;      
                // pMP.last_frame_seen = self.current_frame.unwrap().id;
                // if(i < mCurrentFrame.Nleft){
                //     pMP->mbTrackInView = false;
                // }
                // else{
                //     pMP->mbTrackInViewR = false;
                // }
            } else {
                let map_lock = self.map.read();
                let mappoint_observations = map_lock.mappoints.get(mp_id).unwrap().num_observations;

                if mappoint_observations > 0 {
                    nmatches_map += 1;
                }
            }
        }
        return nmatches_map;
    }

    //* Next steps */
    fn create_new_map(&self) -> bool {
        // TODO (multimaps)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2662
        return false;
    }

    fn relocalization(&self) -> bool {
        todo!("TRACK: Relocalization");
        // TODO (relocalization)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3609
        // return false;
    }

    fn is_imu_initialized(&self) -> bool {
        let imu_initialized;
        {
            let map_read_lock = self.map.read();
            imu_initialized = map_read_lock.imu_initialized;
        }
        imu_initialized
    }

    fn stereo_initialization(&self) -> bool {
        // TODO (stereo)
        // Probably can skip stereo and RGBD modes until after first pass of system is done
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2335
        return false;
    }

    fn predict_state_IMU(&self) -> bool {
        // TODO (IMU)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1738
        return false;
    }

    fn preintegrate_IMU(&self) {
        // TODO (IMU): if using inertial, preintegrate sensor messages
        // Probably can skip all IMU code until after first pass of system is done
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1624
    }

    fn reset_frame_IMU(&self) -> bool {
        // TODO (IMU)
        // ResetFrameIMU in Tracking.cc
        return false;
    }
}

impl Function for DarvisTrackingBack {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        if let Some(msg) = message.content_as::<FeatureMsg>() {
            self.tracking_backend(context, msg);
        }

        Ok(Status::done(()))
    }
}