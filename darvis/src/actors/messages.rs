use axiom::message::ActorMessage;
use dvcore::{
    matrix::{ DVVectorOfKeyPoint, DVMatrix, DVMatrixGrayscale},
};
use crate::dvmap::{keyframe::{KeyFrame, PrelimKeyFrame, KeyFrameState}, pose::Pose, sensor::SensorType, map::Id};

// Note: Can avoid having to serialize/deserialize every message 
// by removing Serialize,Deserialize derives and doing 
// "impl ActorMessage for FeatureMsg" instead of just "impl FeatureMsg".
// Switch this in the future if we want to do anything over the network.
// https://github.com/rsimmonsjr/axiom/issues/99

pub type ActorIds = std::collections::HashMap<String, axiom::actors::Aid>;

pub struct Reset {}
impl ActorMessage for Reset {}

pub struct VisMsg {
    pub new_pose: Pose,
    pub actor_ids: ActorIds
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


#[derive(Debug)]
pub struct KeyFrameMsg<S: SensorType> {
    // Note: if using serde to serialize/deserialize, need to
    // uncomment the following line of code. 
    // See https://github.com/serde-rs/serde/issues/1296
    // #[serde(bound = "")]
    pub kf: KeyFrame<PrelimKeyFrame, S>,
}

impl<S: SensorType + 'static> ActorMessage for KeyFrameMsg<S> {}

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
