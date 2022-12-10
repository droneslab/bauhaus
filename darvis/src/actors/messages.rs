use std::collections::HashSet;

use axiom::message::ActorMessage;
use chrono::{DateTime, Utc};
use dvcore::{
    matrix::{ DVVectorOfKeyPoint, DVMatrix, DVMatrixGrayscale},
};
use serde::{Serialize, Deserialize};
use crate::dvmap::{keyframe::{KeyFrame, KeyFrameState}, pose::Pose, map::Id};

use super::tracking_backend::TrackingState;

// Note: Can avoid having to serialize/deserialize every message 
// by removing Serialize,Deserialize derives and doing 
// "impl ActorMessage for FeatureMsg" instead of just "impl FeatureMsg".
// Switch this in the future if we want to do anything over the network.
// https://github.com/rsimmonsjr/axiom/issues/99

// For shutdown actor
pub struct ShutdownMessage {}
impl ActorMessage for ShutdownMessage {}
pub struct TrajectoryMessage {
    pub pose: Option<Pose>,
    pub ref_kf_id: Option<Id>,
    pub timestamp: Option<DateTime<Utc>>,
}
impl ActorMessage for TrajectoryMessage {}
impl TrajectoryMessage {
    pub fn empty() -> Self {
        TrajectoryMessage{
            pose: None,
            ref_kf_id: None,
            timestamp: None,
        }
    }
    pub fn new(pose: Pose, ref_kf_id: Id, timestamp: DateTime<Utc>) -> Self {
        TrajectoryMessage{
            pose: Some(pose),
            ref_kf_id: Some(ref_kf_id),
            timestamp: Some(timestamp),
        }
    }
}

#[derive(Debug)]
pub struct TrackingStateMsg {
    pub state: TrackingState
}
impl ActorMessage for TrackingStateMsg {}

#[derive(Debug, Serialize, Deserialize)]
pub struct VisPathMsg {
    pub last_img_path: String // last processed image path.
}

impl VisPathMsg {
    pub fn new(last_img_path: String) -> Self {
        Self { last_img_path }
    }
}

pub struct Reset {}
impl ActorMessage for Reset {}

pub struct VisMsg {
    pub new_pose: Pose,
}
impl ActorMessage for VisMsg {}

#[derive(Debug)]
pub struct FeatureMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub descriptors: DVMatrix,
    pub image_width: i32,
    pub image_height: i32,
}
impl ActorMessage for FeatureMsg {}


#[derive(Debug)]
pub struct ImageMsg {
    pub frame: DVMatrixGrayscale,
}
impl ActorMessage for ImageMsg {}


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
pub struct KeyFrameMsg<S: KeyFrameState> {
    // Note: if using serde to serialize/deserialize, need to
    // uncomment the following line of code. 
    // See https://github.com/serde-rs/serde/issues/1296
    // #[serde(bound = "")]
    pub keyframe: KeyFrame<S>,
}

impl<S: KeyFrameState + 'static> ActorMessage for KeyFrameMsg<S> {}

#[derive(Debug)]
pub struct KeyFrameIdMsg {
    pub keyframe_id: Id
}
impl ActorMessage for KeyFrameIdMsg {}

// Sofiya: When KF has been added to the map already, so instead of sending the keyframe data,
// send the keyframe Id. This is only for initialization, so maybe we can figure out a way to
// combine this logic into one keyframe message?
#[derive(Debug)]
pub struct InitialMapMsg {
    // Note: if using serde to serialize/deserialize, need to
    // uncomment the following line of code. 
    // See https://github.com/serde-rs/serde/issues/1296
    // #[serde(bound = "")]
    pub kf: Id,
}

impl ActorMessage for InitialMapMsg {}
