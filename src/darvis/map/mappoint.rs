use std::sync::Arc;
use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use crate::map::keyframe::KeyFrame;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapPoint {
    id: u64,
    first_keyframe_id: u64,
    // 3-Vector
    position: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
    keyframe_ref: Arc<KeyFrame>,
    keyframe_list: Arc<Vec<KeyFrame>>,
    depth_threshold: f64,
}
