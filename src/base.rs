pub mod base {

    extern crate nalgebra as na;

    pub struct Pose {
        // 3-Vector
        pos: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
        // 3x3 Matrix
        rot: na::Matrix<f64, na::U3, na::U3, na::base::storage::Owned<f64, na::U3, na::U3>>,
    }

    pub struct Frame {
        id: u64,
        timestamp: u64,
        keyPoints: opencv::types::VectorOfKeyPoint, // TODO: Pointers?
        descriptors: opencv::core::Mat,
        pose: Pose,
        depthThreshold: u64,
        camParams: opencv::core::Mat,
    }

    pub struct KeyFrame {
        id: u64,
        timestamp: u64,
        mapPoints: Vec<MapPoint>, // TODO: Pointers?
        bow: abow::BoW,
        pose: Pose,
        map: Map, // TODO: Pointers?
        bow_db: BowDB, // TODO: Pointers?
        scale: f64,
        depthThreshold: f64,
    }

    pub struct MapPoint {
        id: u64,
        firstKeyframeId: u64,
        // 3-Vector
        position: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
        keyFrameRef: KeyFrame, // TODO: Pointers?
        keyFrameList: Vec<KeyFrame>, // TODO: Pointers?
        map: Map, // TODO: Pointers?
        depthThreshold: f64,
    }

    pub struct Map {

    }

    pub struct BowDB {

    }

}