use serde::{Deserialize, Serialize};
use darvis::dvutils::{
    DVVectorOfKeyPoint,
    DVMatrix
};

#[derive(Debug, Serialize, Deserialize)]
pub struct FeatureMsg {
    pub keypoints: DVVectorOfKeyPoint,
    pub descriptors: DVMatrix,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
    pub image_width: i32,
    pub image_height: i32,
}
impl FeatureMsg {
    pub fn new(
        keypoints: DVVectorOfKeyPoint,
        descriptors: DVMatrix,
        image_width: i32,
        image_height: i32,
        actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
    ) -> Self {
        Self {
            keypoints: keypoints,
            descriptors: descriptors,
            actor_ids: actor_ids,
            image_width: image_width,
            image_height: image_height,
        }
    }
}
