use std::sync::Arc;

pub struct Frame {
    id: u64,
    timestamp: u64,
    key_points: Arc<opencv::types::VectorOfKeyPoint>,
    descriptors: opencv::core::Mat,
    pose: Pose,
    depth_threshold: u64,
    cam_params: opencv::core::Mat,
}