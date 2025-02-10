use std::collections::{BTreeSet, HashMap, HashSet};

use opencv::prelude::Mat;
use core::{
    matrix::{ DVMatrix, DVVectorOfKeyPoint}, system::{ActorMessage, Timestamp},
};
use crate::{
    actors::tracking_backend::TrackingState, map::{frame::Frame, map::Id, pose::Pose}, modules::imu::{ImuBias, ImuMeasurements}
};

pub struct Reset {
    pub map_version: u64
}
impl ActorMessage for Reset {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

// * TRACKING FRONTEND **//
pub struct ImagePathMsg{ 
    pub image_path: String,
    pub imu_measurements: ImuMeasurements,
    pub timestamp: Timestamp,
    pub frame_id: u32,
    pub map_version: u64
}
impl ActorMessage for ImagePathMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}
pub struct ImageMsg{ 
    pub image: Mat, 
    pub imu_measurements: ImuMeasurements,
    pub timestamp: Timestamp, 
    pub frame_id: u32,
}
impl ActorMessage for ImageMsg {
    fn get_map_version(&self) -> u64 {
        0
    }
}
pub struct FeatureTracksAndIMUMsg {
    pub frame: Frame,
    pub imu_measurements: ImuMeasurements,
    pub mappoint_ids: Vec<i32>,
}
impl ActorMessage for FeatureTracksAndIMUMsg {
    fn get_map_version(&self) -> u64 {
        0
    }
}

pub struct TrackingStateMsg{
    pub state: TrackingState, 
    pub init_id: Id,
    pub map_version: u64
}
impl ActorMessage for TrackingStateMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

// * TRACKING BACKEND **//
pub struct FeatureMsg { 
    pub keypoints: DVVectorOfKeyPoint, 
    pub descriptors: DVMatrix, 
    pub image_width: u32, 
    pub image_height: u32, 
    pub imu_measurements: ImuMeasurements,
    pub timestamp: Timestamp, 
    pub frame_id: Id,
}
impl ActorMessage for FeatureMsg {
    fn get_map_version(&self) -> u64 {
        0
    }
}
pub struct MapInitializedMsg { 
    pub curr_kf_pose: Pose, 
    pub curr_kf_id: Id, 
    pub ini_kf_id: Id, 
    pub local_mappoints: HashSet<Id>, 
    pub curr_kf_timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for MapInitializedMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

pub struct InitKeyFrameMsg { 
    pub kf_id: Id,
    pub map_version: u64
    // pub mappoint_matches: 
}
impl ActorMessage for InitKeyFrameMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

pub struct LastKeyFrameUpdatedMsg {
    pub map_version: u64
}
impl ActorMessage for LastKeyFrameUpdatedMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

// * LOCAL MAPPING */
pub struct UpdateFrameIMUMsg {
    pub scale: f64,
    pub imu_bias: ImuBias,
    pub current_kf_id: Id,
    pub imu_initialized: bool,
    pub map_version: u64
}
impl ActorMessage for UpdateFrameIMUMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

pub struct NewKeyFrameMsg {
    pub keyframe: Frame,
    pub tracking_state: TrackingState,
    pub matches_in_tracking: i32,
    pub tracked_mappoint_depths: HashMap<Id, f64>,
    pub map_version: u64
}
impl ActorMessage for NewKeyFrameMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

// * LOOP CLOSING */
pub struct KeyFrameIdMsg { 
    pub kf_id: Id,
    pub map_version: u64
}
impl ActorMessage for KeyFrameIdMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

pub struct LoopClosureMapPointFusionMsg {
    pub mappoint_matches: Vec<Id>,
    pub loop_mappoints: Vec<Id>,
    pub keyframes_affected: Vec<Id>,
    pub timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for LoopClosureMapPointFusionMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

pub struct LoopClosureEssentialGraphMsg {
    pub relevant_keyframes: HashSet<Id>,
    pub timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for LoopClosureEssentialGraphMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

pub struct LoopClosureGBAMsg {
    pub kf_id: Id,
    pub timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for LoopClosureGBAMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

//* VISUALIZER */
pub struct VisFeaturesMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub image: Mat,
    pub timestamp: Timestamp,
}
impl ActorMessage for VisFeaturesMsg {
    fn get_map_version(&self) -> u64 {
        0
    }
}
pub struct VisFeatureMatchMsg {
    pub matches: opencv::core::Vector<opencv::core::DMatch>,
    pub timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for VisFeatureMatchMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}
pub struct VisTrajectoryMsg { 
    pub pose: Pose, 
    pub mappoint_matches: Vec<Option<(i32, bool)>>,
    pub nontracked_mappoints: HashMap<Id, i32>,
    pub mappoints_in_tracking: BTreeSet<Id>,
    pub timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for VisTrajectoryMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}
pub struct VisUpdateMsg {
    pub pose: Pose,
    pub timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for VisUpdateMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

//* Shutdown Actor *//
pub struct TrajectoryMsg{ 
    pub pose: Pose, 
    pub ref_kf_id: Id, 
    pub timestamp: Timestamp,
    pub map_version: u64
}
impl ActorMessage for TrajectoryMsg {
    fn get_map_version(&self) -> u64 {
        self.map_version
    }
}

#[derive(Debug)]
pub struct ShutdownMsg { }
impl ActorMessage for ShutdownMsg {
    fn get_map_version(&self) -> u64 { 0 }
}


