use serde::{Deserialize, Serialize};
use darvis::dvutils::{
    DVVectorOfKeyPoint,
    DVMatrixGrayscale
};

// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct TrackerMsg {
    pub img1_kps: DVVectorOfKeyPoint,
    pub img1_des: DVMatrixGrayscale,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl TrackerMsg {
    pub fn new(kps1: DVVectorOfKeyPoint, des1: DVMatrixGrayscale, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            img1_kps: kps1,
            img1_des: des1,
            actor_ids: ids,
        }
    }
}
