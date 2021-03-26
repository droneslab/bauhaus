extern crate nalgebra as na;

use std::sync::Arc;

#[allow(dead_code)] // TODO: This is temporary
pub struct Pose {
    // 3-Vector
    pos: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
    // 3x3 Matrix
    rot: na::Matrix<f64, na::U3, na::U3, na::base::storage::Owned<f64, na::U3, na::U3>>,
}

#[allow(dead_code)] // TODO: This is temporary
pub struct Frame {
    id: u64,
    timestamp: u64,
    key_points: Arc<opencv::types::VectorOfKeyPoint>,
    descriptors: opencv::core::Mat,
    pose: Pose,
    depth_threshold: u64,
    cam_params: opencv::core::Mat,
}

#[allow(dead_code)] // TODO: This is temporary
pub struct KeyFrame {
    id: u64,
    timestamp: u64,
    map_points: Arc<Vec<MapPoint>>,
    bow: abow::BoW,
    pose: Pose,
    map: Arc<Map>,
    bow_db: Arc<BowDB>,
    scale: f64,
    depth_threshold: f64,
}

#[allow(dead_code)] // TODO: This is temporary
pub struct MapPoint {
    id: u64,
    first_keyframe_id: u64,
    // 3-Vector
    position: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
    keyframe_ref: Arc<KeyFrame>,
    keyframe_list: Arc<Vec<KeyFrame>>,
    map: Arc<Map>,
    depth_threshold: f64,
}

#[allow(dead_code)] // TODO: This is temporary
pub struct Map {

}

#[allow(dead_code)] // TODO: This is temporary
pub struct BowDB {

}
