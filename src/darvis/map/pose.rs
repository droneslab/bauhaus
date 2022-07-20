use std::ops::Mul;
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

    pub fn inverse(&self) -> Pose{
        // TODO: what is inverse?
        Pose::default_ones()
    }
}
impl Mul for Pose {
    type Output = Pose;

    fn mul(self, _other: Pose) -> Pose {
        // TODO
        // implement this, used by tracker
        // (look for current_frame.pose.unwrap() * last_twc)
        Pose::default_ones()
    }
}
