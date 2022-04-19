use serde::{Deserialize, Serialize};
use crate::dvutils::*;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    // Position 3-Vector
    pub pos: DVVector3,
    // Rotation 3x3 Matrix
    pub rot: DVMatrix3,
}

impl Pose {
    pub fn default_ones() -> Pose {
        Pose {
            pos: DVVector3::new(1.0,1.0,1.0),
            rot: DVMatrix3::new(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0),
        }
    }
}
