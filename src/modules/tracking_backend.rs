use std::sync::Arc;
use std::collections::HashMap;

use axiom::prelude::*;
use chrono::{prelude::*, Duration};
use nalgebra::Matrix3;
use opencv::{
    prelude::*,
    core::Point2f,
    types::{VectorOfPoint3f},
};
use darvis::{
    dvutils::*,
    map::{
        pose::Pose, map::Map, map_actor::MapWriteMsg, map_actor::MAP_ACTOR,
        // keyframe::KeyFrame, 
        frame::Frame, map::Id, misc::IMUBias, 
        mappoint::MapPoint
    },
    utils::{
        camera::DVCamera,
        orbmatcher::ORBmatcher
    },
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
    map_actor: Option<Aid>, //QUICK FIX to get going with the message communication 

    // Frames
    first_frame: bool,
    last_frame: Option<Frame>,
    last_frame_id: Id,
    current_frame: Option<Frame>,
    initial_frame: Option<Frame>,
    reference_frame: Option<Frame>, //Redundant hence will remove once frame database is established.

    // KeyFrames
    reference_keyframe_id: Id,
    last_keyframe_id: Id,

    temporal_points: Vec<MapPoint>,
    relative_frame_poses: Vec<Pose>, // Calculated poses

    orb_matcher: ORBmatcher,

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
    camera: DVCamera,
    camera_calibration_matrix: Matrix3<f32>,

    // Reference Keyframe.
    reference_keyframe: Option<Id>,
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
        let mut camera = DVCamera::new(&vec![fx as f32, fy as f32, cx as f32, cy as f32]);
        let mut camera_calibration_matrix = Matrix3::<f32>::zeros();
        camera_calibration_matrix.fill_with_identity();
        camera_calibration_matrix[(0,0)] = fx as f32;
        camera_calibration_matrix[(1,1)] = fy as f32;
        camera_calibration_matrix[(0,2)] = cx as f32;
        camera_calibration_matrix[(1,2)] = cy as f32;

        let mut tracker = DarvisTrackingBack {
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

            // Initialization Variables (Monocular)
            // std::vector<int> mvIniLastMatches;
            ini_matches: HashMap::new(),// std::vector<int> mvIniMatches;
            prev_matched: Vec::new(),// std::vector<cv::Point2f> mvbPrevMatched;
            ini_p3d: VectorOfPoint3f::new(),// std::vector<cv::Point3f> mvIniP3D;
            // Frame mInitialFrame;
            ready_to_initializate: false,

            // Defaults set from global variables
            recently_lost_cutoff: Duration::seconds(recently_lost_cutoff.into()),
            sensor: sensor,
            localization_only_mode: localization_only_mode,
            frames_to_reset_imu: frames_to_reset_imu,
            insert_kfs_when_lost: insert_kfs_when_lost,
            min_num_features: min_num_features,

            // IMU
            last_bias: None,
            velocity: None,

            // Relocalization
            timestamp_lost: None,
            last_reloc_frame_id: 0,

            //Camera
            camera: camera,
            camera_calibration_matrix: camera_calibration_matrix,

            reference_keyframe: None,
            map_actor: None,
        };

        tracker
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
                    ok = self.track_with_motion_model(map_actor);
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

        println!("after big block");
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
                // Ref code: This line exactly: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2211
                // Pose is Sophus::SE3 type, not sure what this multiplication is actually doing
                // self.velocity = current_frame.pose.unwrap() * last_twc; 
                self.velocity = Some(self.current_frame.as_ref().unwrap().pose.as_ref().unwrap().clone() * _last_Twc);
            } else {
                self.velocity = None;
            }

            // Clean VO matches
            self.current_frame.as_mut().unwrap().clean_VO_matches();

            // Delete temporal MapPoints
            // TODO:
            // mlpTemporalPoints is added to in UpdateLastFrame, which is called by TrackWithMotionModel
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

        if self.current_frame.as_mut().unwrap().reference_keyframe_id.is_none() {
            self.current_frame.as_mut().unwrap().reference_keyframe_id = Some(self.reference_keyframe_id);
        }

        self.last_frame = self.current_frame.clone();

        match self.state {
            TrackingState::Ok | TrackingState::RecentlyLost => self.store_pose_info(),
            _ => {}
        }

        // self.align(_context, &msg);

    }

    //* IMU stuff */
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

    //* MVP */
    fn kfs_in_map(&self) -> u64 {
        let kfs_in_map;
        {
            let map_read_lock = self.map.read();
            kfs_in_map = map_read_lock.num_kfs;
        }
        kfs_in_map
    }

    fn monocular_initialization(&mut self) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        if !self.ready_to_initializate {
            // Set Reference Frame
            if self.current_frame.as_ref().unwrap().key_points.len() > 100 {
                self.initial_frame = self.current_frame.clone();
                self.last_frame = self.current_frame.clone();
                self.prev_matched.resize(self.current_frame.as_ref().unwrap().key_points_un.len(), Point2f::default());

                for i in 0..self.current_frame.as_ref().unwrap().key_points_un.len() {
                    self.prev_matched[i] = self.current_frame.as_ref().unwrap().key_points_un.get(i).unwrap().pt.clone();
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
            if (self.current_frame.as_ref().unwrap().key_points.len() <=100)||((self.sensor.is_mono() && self.sensor.is_imu() )&&(self.last_frame.as_ref().unwrap().timestamp - self.initial_frame.as_ref().unwrap().timestamp> Duration::seconds(1))) {
                self.ready_to_initializate  = false;
                return false;
            }

            // Find correspondences
            let mut nmatches = self.orb_matcher.search_for_initialization(
                &self.initial_frame.as_ref().unwrap(), 
                &self.current_frame.as_ref().unwrap(), 
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
                &self.initial_frame.as_ref().unwrap().key_points_un,
                &self.current_frame.as_ref().unwrap().key_points_un,
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
                self.initial_frame.as_mut().unwrap().set_pose(&Pose::default());//.SetPose(Sophus::SE3f());
                self.current_frame.as_mut().unwrap().set_pose(&Tcw);

                self.create_initial_map_monocular();
            }
        }

        return false;
    }

    //void Tracking::CreateInitialMapMonocular()
    pub fn create_initial_map_monocular(&mut self) {
        todo!("Implement create_initial_map_monocular");
    }


    fn track_reference_keyframe(&mut self) -> bool {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2720

        // Compute Bag of Words vector
        self.current_frame.as_mut().unwrap().compute_BoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver

        let matcher = ORBmatcher::new(0.7, true);
        let mut vpMapPointMatches = HashMap::<i32, Id>::new();

        //ORBmatcher matcher(0.7,true);
        //vector<MapPoint*> vpMapPointMatches;

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
            
                nmatches = matcher.search_for_initialization(self.reference_frame.as_ref().unwrap(), cur_f, &mut self.prev_matched, &mut vpMapPointMatches, 100);
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

        self.current_frame.as_mut().unwrap().mappoint_matches = vpMapPointMatches.clone();
        self.current_frame.as_mut().unwrap().set_pose(&self.last_frame.as_ref().unwrap().get_pose());

        //mCurrentFrame.PrintPointDistribution();
        // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;

        //TODO: (Optimization) Implement pose optimization
        //Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        let mut nmatches_map = 0;
        let matches = &self.current_frame.as_ref().unwrap().mappoint_matches;
        for (index, mp_id) in matches {
            if self.current_frame.as_ref().unwrap().mappoint_outliers.contains_key(index) {
                let map_msg = MapWriteMsg::delete_mappoint_match(self.current_frame.as_ref().unwrap().id, *mp_id, true);
                self.map_actor.as_ref().unwrap().send_new(map_msg).unwrap();

                //TODO:[Stereo] check for Stereo camera flow
                // pMP.mbTrackInView= false;      
                // pMP.mnLastFrameSeen = self.current_frame.unwrap().id;                 

                // if(i < mCurrentFrame.Nleft){
                //     pMP->mbTrackInView = false;
                // }
                // else{
                //     pMP->mbTrackInViewR = false;
                // }
                //pMP->mbTrackInView = false;

                nmatches-=1;
            } else {
                let map_lock = self.map.read();
                let mappoint_observations = map_lock.mappoints.get(mp_id).unwrap().num_observations;

                if mappoint_observations > 0 {
                    nmatches_map+=1;
                }

            }
        }

        if self.sensor.is_imu() {
            return true;
        } else {
            return nmatches_map>=10;
        }

    }

    fn track_with_motion_model(&mut self, map_actor: &axiom::actors::Aid) -> bool {
        println!("TRACK: Track with motion model");
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
            // let pos = DVVector3::from_vec(vec![x,y,z]);

            // let mut rot = DVMatrix3::identity();
            // rot[(0,0)] = *r.at_2d::<f64>(0,0).unwrap();
            // rot[(0,1)] = *r.at_2d::<f64>(0,1).unwrap();
            // rot[(0,2)] = *r.at_2d::<f64>(0,2).unwrap();
            // rot[(1,0)] = *r.at_2d::<f64>(1,0).unwrap();
            // rot[(1,1)] = *r.at_2d::<f64>(1,1).unwrap();
            // rot[(1,2)] = *r.at_2d::<f64>(1,2).unwrap();
            // rot[(2,0)] = *r.at_2d::<f64>(2,0).unwrap();
            // rot[(2,1)] = *r.at_2d::<f64>(2,1).unwrap();
            // rot[(2,2)] = *r.at_2d::<f64>(2,2).unwrap();

            // let mut pose = Pose::new(&pos, &rot);
            // self.current_frame.SetPose(mVelocity * mLastFrame.GetPose());
        }

        self.current_frame.as_mut().unwrap().clear_mappoints();

        // Project points seen in previous frame
        let _th = match self.sensor.is_mono() {
            true => 15,
            false => 7
        };
        let mut matches = self.orb_matcher.search_by_projection(); // (mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

        // If few matches, uses a wider window search
        if matches.len() < 20 {
            println!("TRACK: track_with_motion_model... not enough matches, wider window search");
            self.current_frame.as_mut().unwrap().clear_mappoints();
            matches = self.orb_matcher.search_by_projection(); //(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        }

        if matches.len() < 20 {
            println!("TRACK: track_with_motion_model... not enough matches!!");
            return self.sensor.is_imu();
        }

        // Optimize frame pose with all matches
        let (pose, inliers) = optimize_pose(&mut self.current_frame.as_mut().unwrap(), &self.map);
        if inliers != 0 {
            let map_msg = MapWriteMsg::set_pose(self.current_frame.as_ref().unwrap().id, pose.unwrap());
            map_actor.send_new(map_msg).unwrap();
        }

        // Discard outliers
        let mut nmatches_map = 0;
        let matches = &self.current_frame.as_ref().unwrap().mappoint_matches; // Sofiya: honestly don't know why I need to borrow (&) here...
        for (_index, mp_id) in matches {
            if self.current_frame.as_ref().unwrap().mappoint_outliers.contains_key(mp_id) {
                {
                    let map_msg = MapWriteMsg::delete_mappoint_match(self.current_frame.as_ref().unwrap().id, *mp_id, true);
                    map_actor.send_new(map_msg).unwrap();
                }

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

        // TODO (localization only)
        // if(mbOnlyTracking)
        // {
        //     mbVO = nmatchesMap<10;
        //     return nmatches>20;
        // }

        match self.sensor.is_imu() {
            true => { return true; },
            false => { return nmatches_map >= 10; }
        };
    }

    fn update_last_frame(&self) {
        // TODO
        // Ref code:  https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2781

        // let last_frame = self.last_frame.unwrap();
        
        // // Update pose according to reference keyframe
        // let reference_kf = last_frame.reference_keyframe_id;
        // let Tlr = self.relative_frame_poses.last();
        // {
        //     let map_lock = self.map.read();
        //     let reference_kf_pose = map_lock.keyframes[reference_kf];
        //     last_frame.set_pose(Tlr * reference_kf_pose);
        // }

        // if self.sensor.is_mono() || self.last_keyframe_id.unwrap() == self.last_frame_id.unwrap() {
        //     return;
        // }

        
        // // Create "visual odometry" MapPoints
        // // We sort points according to their measured depth by the stereo/RGB-D sensor
        
        // let depth_idx = Vec::new
        
        // HashMap::<i32, f32>::new();
        // let num_features = match last_frame.Nleft {
        //     -1 => last_frame.N,
        //     _ => last_frame.Nleft
        // };

        // for i in 0..num_features {
        //     let z = last_frame.mv_depth[i];
        //     depth_idx
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

    fn track_local_map(&self) -> bool {
        // TODO
        // Ref code:
        todo!("TRACK: local map");
        // return false;
    }

    fn relocalization(&self) -> bool {
        todo!("TRACK: Relocalization");
        // TODO
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3609
        // return false;
    }

    fn create_new_keyframe(&self) {
        // TODO
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
        // TODO
        // let map_lock = self.map.read();
        // let num_kfs = map_lock.num_kfs;
        return true;
    }

    fn store_pose_info(&self) {
        // TODO: Convert this to Rust obv
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

    //* Multi-maps */
    fn create_new_map(&self) -> bool {
        // TODO (multimaps)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2662
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