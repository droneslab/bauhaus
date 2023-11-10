use std::collections::{HashSet, HashMap};

use opencv::prelude::Mat;
use dvcore::{
    matrix::{ DVVectorOfKeyPoint, DVMatrix}, base::ActorMessage,
};
use crate::{
    dvmap::{keyframe::{Frame, FrameState}, pose::DVPose, map::Id},
    actors::tracking_backend::TrackingState
};

//* VISUALIZER */
pub struct VisFeaturesMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub image: Mat,
    pub timestamp: u64,
    pub image_filename: String,
}
impl ActorMessage for VisFeaturesMsg {}
pub struct VisFeatureMatchMsg {
    pub matches: opencv::core::Vector<opencv::core::DMatch>,
    pub timestamp: u64
}
impl ActorMessage for VisFeatureMatchMsg {}

pub struct VisUpdateMsg {
    pub pose: DVPose,
    pub timestamp: u64,
}
impl ActorMessage for VisUpdateMsg {}

//* Shutdown Actor *//
pub struct ShutdownMsg {}
impl ActorMessage for ShutdownMsg {}

#[derive(Clone)]
pub struct TrajectoryMsg {
    pub pose: DVPose,
    pub ref_kf_id: Id,
    pub mappoint_matches: HashMap<u32, (i32, bool)>,
    pub timestamp: u64,
}
impl ActorMessage for TrajectoryMsg {}

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
    pub curr_kf_pose: DVPose,
    pub curr_kf_id: Id,
    pub ini_kf_id: Id,
    pub local_mappoints: HashSet<Id>,
}
impl ActorMessage for MapInitializedMsg { }

pub struct VisInitialMapMsg {
    pub kf1_id: Id,
    pub kf2_id: Id,
}
impl ActorMessage for VisInitialMapMsg { }

#[derive(Debug)]
pub struct KeyFrameMsg<S: FrameState> {
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



