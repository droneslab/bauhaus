extern crate nalgebra as na;

use std::rc::Rc;

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
    key_points: Rc<opencv::types::VectorOfKeyPoint>,
    descriptors: opencv::core::Mat,
    pose: Pose,
    depth_threshold: u64,
    cam_params: opencv::core::Mat,
}

#[allow(dead_code)] // TODO: This is temporary
pub struct KeyFrame {
    id: u64,
    timestamp: u64,
    map_points: Rc<Vec<MapPoint>>,
    bow: abow::BoW,
    pose: Pose,
    map: Rc<Map>,
    bow_db: Rc<BowDB>,
    scale: f64,
    depth_threshold: f64,
}

#[allow(dead_code)] // TODO: This is temporary
pub struct MapPoint {
    id: u64,
    first_keyframe_id: u64,
    // 3-Vector
    position: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
    keyframe_ref: Rc<KeyFrame>,
    keyframe_list: Rc<Vec<KeyFrame>>,
    map: Rc<Map>,
    depth_threshold: f64,
}

#[allow(dead_code)] // TODO: This is temporary
pub struct Map {

}

#[allow(dead_code)] // TODO: This is temporary
pub struct BowDB {

}
