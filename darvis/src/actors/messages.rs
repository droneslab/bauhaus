use std::collections::{HashSet, HashMap};

use opencv::prelude::Mat;
use dvcore::{
    matrix::{ DVVectorOfKeyPoint, DVMatrix}, actor::ActorMessage,
};
use crate::{
    dvmap::{pose::DVPose, map::Id, misc::Timestamp},
    actors::tracking_backend::TrackingState
};

// * TRACKING FRONTEND **//
pub struct ImagePathMsg{ 
    pub image_path: String,
    pub timestamp: Timestamp,
    pub frame_id: u32
}
impl ActorMessage for ImagePathMsg {}
pub struct ImageMsg{ 
    pub image: Mat, 
    pub timestamp: Timestamp, 
    pub frame_id: u32
}
impl ActorMessage for ImageMsg {}
pub struct TrackingStateMsg{
    pub state: TrackingState, 
    pub init_id: Id
}
impl ActorMessage for TrackingStateMsg {}

// * TRACKING BACKEND **//
pub struct FeatureMsg { 
    pub keypoints: DVVectorOfKeyPoint, 
    pub descriptors: DVMatrix, 
    pub image_width: u32, 
    pub image_height: u32, 
    pub timestamp: Timestamp, 
    pub frame_id: Id
}
impl ActorMessage for FeatureMsg {}
pub struct MapInitializedMsg { 
    pub curr_kf_pose: DVPose, 
    pub curr_kf_id: Id, 
    pub ini_kf_id: Id, 
    pub local_mappoints: HashSet<Id>, 
    pub curr_kf_timestamp: Timestamp
}
impl ActorMessage for MapInitializedMsg {}

pub struct KeyFrameIdMsg { 
    pub kf_id: Id 
}
impl ActorMessage for KeyFrameIdMsg {}

pub struct LastKeyFrameUpdatedMsg {}
impl ActorMessage for LastKeyFrameUpdatedMsg {}

pub struct Reset {}
impl ActorMessage for Reset {}

// * LOCAL MAPPING */
pub struct IMUInitializedMsg {
    imu_initialized: bool,
    imu_ba2: bool
}
impl ActorMessage for IMUInitializedMsg {}

//* VISUALIZER */
pub struct VisFeaturesMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub image: Mat,
    pub timestamp: Timestamp,
}
impl ActorMessage for VisFeaturesMsg {}
pub struct VisFeatureMatchMsg {
    pub matches: opencv::core::Vector<opencv::core::DMatch>,
    pub timestamp: Timestamp
}
impl ActorMessage for VisFeatureMatchMsg {}
pub struct VisTrajectoryMsg { 
    pub pose: DVPose, 
    pub mappoint_matches: HashMap<u32, (i32, bool)>, 
    pub timestamp: Timestamp
}
impl ActorMessage for VisTrajectoryMsg {}
pub struct VisUpdateMsg {
    pub pose: DVPose,
    pub timestamp: Timestamp,
}
impl ActorMessage for VisUpdateMsg {}

//* Shutdown Actor *//
pub struct TrajectoryMsg{ 
    pub pose: DVPose, 
    pub ref_kf_id: Id, 
    pub timestamp: Timestamp
}
impl ActorMessage for TrajectoryMsg {}

#[derive(Debug)]
pub struct ShutdownMsg {}
impl ActorMessage for ShutdownMsg {}


