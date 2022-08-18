use serde::{Deserialize, Serialize};
use darvis::dvutils::{
    DVVectorOfKeyPoint,
    DVMatrixGrayscale
};
use darvis::map::frame::Frame;

// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct FeatureMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub descriptors: DVMatrixGrayscale,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
    pub im_width: i32,
    pub im_height: i32,
}
impl FeatureMsg {
    pub fn new(keypoints: DVVectorOfKeyPoint, descriptors: DVMatrixGrayscale, im_width: i32, im_height: i32, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            keypoints: keypoints,
            descriptors: descriptors,
            actor_ids: ids,
            im_width: im_width,
            im_height: im_height,
        }
    }
}
