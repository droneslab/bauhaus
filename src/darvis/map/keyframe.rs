use std::sync::Arc;
use serde::{Deserialize, Serialize};
use crate::map::mappoint::MapPoint;
use crate::map::pose::Pose;
use crate::map::misc::BowDB;

#[derive(Debug, Clone, Serialize, Deserialize, Copy)]
pub struct KeyFrame {
    pub id: u64,
    // timestamp: u64,
    // map_points: Arc<Vec<MapPoint>>,
    // bow: abow::BoW,
    // pose: Pose,
    // bow_db: Arc<BowDB>,
    // scale: f64,
    // depth_threshold: f64,
}
// impl KeyFrame {
//     pub fn new(id, ) -> Self {
//         Self {

//         }
//     }
// }