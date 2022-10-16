use axiom::message::ActorMessage;
use dvcore::{
    matrix::{ DVVectorOfKeyPoint, DVMatrix, DVMatrixGrayscale},
};
use crate::dvmap::{keyframe::KeyFrame, pose::Pose, sensor::SensorType};

// Note: Can avoid having to serialize/deserialize every message 
// by removing Serialize,Deserialize derives and doing 
// "impl ActorMessage for FeatureMsg" instead of just "impl FeatureMsg".
// Switch this in the future if we want to do anything over the network.
// https://github.com/rsimmonsjr/axiom/issues/99

pub struct VisMsg {
    pub new_pose: Pose,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
}
impl ActorMessage for VisMsg {}

#[derive(Debug)]
pub struct FeatureMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub descriptors: DVMatrix,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
    pub image_width: i32,
    pub image_height: i32,
}
impl ActorMessage for FeatureMsg {}


#[derive(Debug)]
pub struct ImageMsg {
    pub frame: DVMatrixGrayscale,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}
impl ActorMessage for ImageMsg {}


#[derive(Debug)]
pub struct ImagesMsg {
    // Vector of image paths to read in/extract
    pub img_paths: Vec<String>,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}
impl ActorMessage for ImagesMsg {}


#[derive(Debug)]
pub struct KeyFrameMsg<S: SensorType> {
    // Note: if using serde to serialize/deserialize, need to
    // uncomment the following line of code. 
    // See https://github.com/serde-rs/serde/issues/1296
    // #[serde(bound = "")]
    pub kf: KeyFrame<S>,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl<S: SensorType + 'static> ActorMessage for KeyFrameMsg<S> {}