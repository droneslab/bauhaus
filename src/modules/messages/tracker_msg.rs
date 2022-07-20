use serde::{Deserialize, Serialize};
use darvis::dvutils::{
    DVVectorOfKeyPoint,
    DVMatrixGrayscale
};
use darvis::map::frame::Frame;

// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct TrackerMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub descriptors: DVMatrixGrayscale,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}
impl TrackerMsg {
    pub fn new(keypoints: DVVectorOfKeyPoint, descriptors: DVMatrixGrayscale, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            keypoints: keypoints,
            descriptors: descriptors,
            actor_ids: ids,
        }
    }
}
