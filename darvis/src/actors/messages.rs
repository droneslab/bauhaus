use std::collections::HashSet;

use opencv::prelude::Mat;
use dvcore::{
    matrix::{ DVVectorOfKeyPoint, DVMatrix}, base::ActorMessage,
};
use crate::{
    dvmap::{keyframe::{Frame, FrameState}, pose::Pose, map::Id},
    actors::tracking_backend::TrackingState
};

// Note: Can avoid having to serialize/deserialize every message 
// by removing Serialize,Deserialize derives and doing 
// "impl ActorMessage for FeatureMsg" instead of just "impl FeatureMsg".
// Switch this in the future if we want to do anything over the network.
// https://github.com/rsimmonsjr/axiom/issues/99

//* VISUALIZER */
pub struct VisFeaturesMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub image: Mat,
    pub timestamp: u64,
    pub image_filename: String,
}
impl ActorMessage for VisFeaturesMsg {}

pub struct VisKeyFrameMsg {
    pub pose: Pose,
}
impl ActorMessage for VisKeyFrameMsg {}

pub struct VisMapPointsMsg {}
impl ActorMessage for VisMapPointsMsg {}


//* Shutdown Actor *//
pub struct ShutdownMessage {}
impl ActorMessage for ShutdownMessage {}

#[derive(Clone)]
pub struct TrajectoryMessage {
    pub pose: Pose,
    pub ref_kf_id: Id,
    pub timestamp: u64,
}
impl ActorMessage for TrajectoryMessage {}
impl TrajectoryMessage {
    pub fn new(pose: Pose, ref_kf_id: Id, timestamp: u64) -> Self {
        TrajectoryMessage{
            pose,
            ref_kf_id,
            timestamp
        }
    }
}

#[derive(Debug)]
pub struct TrackingStateMsg {
    pub state: TrackingState,
    pub init_id: Id
}
impl ActorMessage for TrackingStateMsg {}


pub struct Reset {}
impl ActorMessage for Reset {}


#[derive(Debug)]
pub struct FeatureMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub descriptors: DVMatrix,
    pub image_width: i32,
    pub image_height: i32,
    pub timestamp: u64,
}
impl ActorMessage for FeatureMsg {}


#[derive(Debug)]
pub struct ImagePathMsg {
    // Uncomment this to pass the image directly instead of the path
    // pub frame: DVMatrixGrayscale,
    pub image_path: String,
    pub timestamp: u64,
}
impl ActorMessage for ImagePathMsg {}

pub struct ImageMsg {
    pub image: Mat,
    pub timestamp: u64,
    pub image_path: String,
}
impl ActorMessage for ImageMsg{}

#[derive(Debug)]
pub struct ImagesMsg {
    // Vector of image paths to read in/extract
    pub img_paths: Vec<String>,
}
impl ActorMessage for ImagesMsg {}

// Message that map sends to tracking letting it know that map has been initialized
#[derive(Debug)]
pub struct MapInitializedMsg {
    pub curr_kf_pose: Pose,
    pub curr_kf_id: Id,
    pub ini_kf_id: Id,
    pub local_mappoints: HashSet<Id>,
}
impl ActorMessage for MapInitializedMsg { }

#[derive(Debug)]
pub struct KeyFrameMsg<S: FrameState> {
    // Note: if using serde to serialize/deserialize, need to
    // uncomment the following line of code. 
    // See https://github.com/serde-rs/serde/issues/1296
    // #[serde(bound = "")]
    pub keyframe: Frame<S>,
}

impl<S: FrameState + 'static> ActorMessage for KeyFrameMsg<S> {}

#[derive(Debug)]
pub struct KeyFrameIdMsg {
    pub keyframe_id: Id
}
impl ActorMessage for KeyFrameIdMsg {}

#[derive(Debug)]
pub struct LastKeyFrameUpdatedMsg {}
impl ActorMessage for LastKeyFrameUpdatedMsg {}



