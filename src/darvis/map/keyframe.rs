use std::sync::Arc;
use crate::map::mappoint::MapPoint;
use crate::map::pose::Pose;
use crate::map::misc::BowDB;

#[derive(Debug)]
pub struct KeyFrame {
    id: u64,
    timestamp: u64,
    map_points: Arc<Vec<MapPoint>>,
    bow: abow::BoW,
    pose: Pose,
    bow_db: Arc<BowDB>,
    scale: f64,
    depth_threshold: f64,
}
