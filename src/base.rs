extern crate nalgebra as na;
use na::*;
use std::sync::Arc;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct Pose {
    // Position 3-Vector
    pub pos: Vector3<f64>,
    // Rotation 3x3 Matrix
    pub rot: Matrix3<f64>,
}

impl Pose {
    pub fn default_ones() -> Pose {
        Pose {
            pos: Vector3::new(1.0,1.0,1.0),
            rot: Matrix3::new(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0),
        }
    }
}

pub struct Frame {
    id: u64,
    timestamp: u64,
    key_points: Arc<opencv::types::VectorOfKeyPoint>,
    descriptors: opencv::core::Mat,
    pose: Pose,
    depth_threshold: u64,
    cam_params: opencv::core::Mat,
}

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

pub struct Map {

}

pub struct BowDB {

}

#[derive(Debug, Default, Clone)]
// Struct holding configuration paramters for a given actor
pub struct ActorConf{
    pub name: String,
    pub file: String,
    pub actor_message: String,
    pub actor_function: String,
    pub ip_address: String,
    pub port: String,
    pub multithreaded: bool,
    pub threads: i64,
    pub possible_paths: HashMap<String, String>
}
