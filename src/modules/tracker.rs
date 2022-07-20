use std::sync::Arc;
use axiom::prelude::*;
use chrono::{prelude::*, Duration};
use opencv::{
    prelude::*,
    core,
    features2d::{BFMatcher},
    types::{VectorOfKeyPoint, VectorOfDMatch, VectorOfPoint2f},
};
use darvis::{
    dvutils::*,
    map::{
        pose::Pose, map::Map, map_actor::MapWriteMsg, map_actor::MAP_ACTOR,
        keyframe::KeyFrame, frame::Frame, map::Id, misc::IMUBias, mappoint::MapPoint
    },
    lockwrap::ReadOnlyWrapper,
    plugin_functions::Function,
    global_params::*,
};
use crate::{
    registered_modules::VISUALIZER,
    modules::messages::{
        vis_msg::VisMsg,
        tracker_msg::TrackerMsg,
    },
};

#[derive(Debug, Clone)]
pub struct DarvisTracker {
    first_frame: bool,
    last_frame: Option<Frame>,
    map: ReadOnlyWrapper<Map>,
    state: TrackingState,
    last_frame_id: i32,
    temporal_points: Vec<MapPoint>,
    reference_keyframe_id: Option<Id>,

    // IMU 
    velocity: Option<Mat>, // Note: ORB_SLAM3 uses C++ package Sophus
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
}

#[derive(Debug, Clone)]
enum TrackingState {
    NotInitialized,
    NoImagesYet,
    Lost,
    RecentlyLost,
    Ok
}

impl DarvisTracker {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisTracker {
        let recently_lost_cutoff: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "recently_lost_cutoff");
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");
        let localization_only_mode: bool = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "localization_only_mode");
        let frames_to_reset_imu: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "frames_to_reset_IMU");
        let insert_kfs_when_lost: bool = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "insert_KFs_when_lost");
        let min_num_features: i32 = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "min_num_features");

        DarvisTracker {
            first_frame: true,
            last_frame: None,
            map: map,
            state: TrackingState::NoImagesYet,
            last_frame_id: -1,
            temporal_points: Vec::new(),
            reference_keyframe_id: None,

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
        }
    }

    fn track(&mut self, _context: Context, msg: Arc<TrackerMsg>) {
        let map_actor = msg.actor_ids.get(MAP_ACTOR).unwrap();

        self.last_frame_id += 1;
        let mut current_frame = Frame::new(
            self.last_frame_id, 
            Utc::now(), 
            msg.keypoints.cv_vector_of_keypoint(),
            msg.descriptors.grayscale_to_cv_mat()
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
                    self.last_frame = Some(current_frame);
                    return;
                }
                ok
            },
            TrackingState::Ok => {
                let mut ok;

                let enough_frames_since_last_reloc = current_frame.id < self.last_reloc_frame_id + 2;
                let no_imu_data = self.velocity.is_none() && !self.is_imu_initialized();
                if no_imu_data || enough_frames_since_last_reloc {
                    ok = self.track_reference_keyframe();
                } else {
                    ok = self.track_with_motion_model();
                    if !ok {
                        ok = self.track_reference_keyframe();
                    }
                }

                if !ok {
                    let enough_frames_to_reset_imu = current_frame.id <= self.last_reloc_frame_id + self.frames_to_reset_imu;
                    if enough_frames_to_reset_imu && self.sensor.is_imu() {
                        self.state = TrackingState::Lost;
                    } else if self.kfs_in_map() > 10 {
                        self.state = TrackingState::RecentlyLost;
                        self.timestamp_lost = Some(current_frame.timestamp);
                    } else {
                        self.state = TrackingState::Lost;
                    }
                }
                ok
            },
            TrackingState::RecentlyLost => {
                println!("TRACK: State recently lost");
                let mut ok;
                let time_since_lost = current_frame.timestamp - self.timestamp_lost.unwrap();
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
            },
            TrackingState::NoImagesYet => {
                panic!("Should not be possible to get here!");
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
            self.timestamp_lost = Some(current_frame.timestamp);
        }

        // TODO (IMU)
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
        // This code not done, so double check with C++ reference.
        {
            // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
            let enough_frames_to_reset_imu = current_frame.id <= self.last_reloc_frame_id + self.frames_to_reset_imu;
            if enough_frames_to_reset_imu && current_frame.id > self.frames_to_reset_imu && self.sensor.is_imu() && self.is_imu_initialized() {
                // Load preintegration
                // This code directly copied from C++, obviously needs to be changed
                // pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
            }
            if self.is_imu_initialized() && success {
                if current_frame.id == self.last_reloc_frame_id + self.frames_to_reset_imu {
                    println!("TRACK: RESETING FRAME!!!");
                    self.reset_frame_IMU();
                }
                else if current_frame.id > self.last_reloc_frame_id + 30 {
                    self.last_bias = current_frame.imu_bias;
                }
            }
        }

        if success || matches!(self.state, TrackingState::RecentlyLost) {
            // Update motion model
            let last_frame = self.last_frame.as_ref();
            if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !current_frame.pose.is_none() {
                let last_pose = last_frame.unwrap().pose.as_ref().unwrap();
                let _last_twc = last_pose.inverse();
                // TODO (IMU)
                // Ref code: This line exactly: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2211
                // Pose is Sophus::SE3 type, not sure what this multiplication is actually doing
                // self.velocity = current_frame.pose.unwrap() * last_twc; 
            } else {
                self.velocity = None;
            }

            // Clean VO matches
            current_frame.clean_VO_matches();

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
            current_frame.delete_VO_matches_if_not_outliers();
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

        if current_frame.reference_keyframe_id.is_none() {
            current_frame.reference_keyframe_id = self.reference_keyframe_id;
        }

        self.last_frame = Some(current_frame);

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

    fn monocular_initialization(&self) -> bool {
        // TODO
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        return false;
    }

    fn track_reference_keyframe(&self) -> bool {
        println!("TRACK: Track with respect to the reference KF");
        // TODO
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2720
        return false;
    }

    fn track_with_motion_model(&self) -> bool {
        println!("TRACK: Track with motion model");
        // TODO
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2854
        // If tracking was successful for last frame, we use a constant
        // velocity motion model to predict the camera pose and perform
        // a guided search of the map points observed in the last frame. If
        // not enough matches were found (i.e. motion model is clearly
        // violated), we use a wider search of the map points around
        // their position in the last frame. The pose is then optimized
        // with the found correspondences.
        return false;
    }

    fn track_local_map(&self) -> bool {
        // TODO
        // Ref code:
        return false;
    }

    fn relocalization(&self) -> bool {
        println!("TRACK: Relocalization");
        // TODO
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3609
        return false;
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
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // if(mCurrentFrame.isSet())
        // {
        //     Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
        //     mlRelativeFramePoses.push_back(Tcr_);
        //     mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
        //     mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        //     mlbLost.push_back(mState==LOST);
        // }
        // else
        // {
        //     // This can happen if tracking is lost
        //     mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
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

impl Function for DarvisTracker {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        if let Some(msg) = message.content_as::<TrackerMsg>() {
            self.track(context, msg);
        }

        Ok(Status::done(()))
    }
}