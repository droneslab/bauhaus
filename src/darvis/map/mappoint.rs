use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use crate::{
    map::map::Id,
    dvutils::DVVector3
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapPoint {
    id: Id,
    first_keyframe_id: Id,
    // 3-Vector
    pub position: DVVector3, //na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
    ref_keyframe: u64,
    seen_by_keyframes: Vec<Id>,
    depth_threshold: f64,
    last_frame_seen: Id,

    pub num_observations: i32
}
