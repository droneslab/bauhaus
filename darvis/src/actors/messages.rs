use std::collections::{HashSet, HashMap};

use opencv::prelude::Mat;
use dvcore::{
    matrix::{ DVVectorOfKeyPoint, DVMatrix}, actor::ActorMessage,
};
use crate::{
    dvmap::{keyframe::{Frame, FrameState}, pose::DVPose, map::Id, misc::Timestamp},
    actors::tracking_backend::TrackingState
};

//* VISUALIZER */
pub struct VisFeaturesMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub image: Mat,
    pub timestamp: Timestamp,
    pub image_filename: String,
}
impl ActorMessage for VisFeaturesMsg {}
pub struct VisFeatureMatchMsg {
    pub matches: opencv::core::Vector<opencv::core::DMatch>,
    pub timestamp: Timestamp
}
impl ActorMessage for VisFeatureMatchMsg {}

pub struct VisUpdateMsg {
    pub pose: DVPose,
    pub timestamp: Timestamp,
}
impl ActorMessage for VisUpdateMsg {}

//* Shutdown Actor *//
#[derive(Debug)]
pub struct ShutdownMsg {}
impl ActorMessage for ShutdownMsg {}


