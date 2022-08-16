use std::ops::Mul;
use nalgebra::{Isometry3, Rotation3};
use serde::{Deserialize, Serialize};
use crate::dvutils::*;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    // Position 3-Vector
    //pub pos: DVVector3,
    // Rotation 3x3 Matrix
    //pub rot: DVMatrix3,
    _pose: Isometry3<f64>
}

impl Pose {
    pub fn default() -> Pose {
        Pose {
            //pos: DVVector3::new(0.0,0.0,0.0),
            //rot: DVMatrix3::identity(),
            _pose : Isometry3::identity() 
            //::new(DVVector3::new(0.0,0.0,0.0), Rotation3::<f64>::from_matrix(&DVMatrix3::identity()).scaled_axis())
        }
    }

    pub fn new(pos : &DVVector3, rot: &DVMatrix3) -> Pose {
        Pose {
            //pos: pos.clone(),
            //rot: rot.clone(),
            _pose: Isometry3::new(pos.clone(), Rotation3::<f64>::from_matrix(&rot).scaled_axis())
        }
    }

    pub fn pos(&self) -> DVVector3
    {
        self._pose.translation.vector.clone()
    }

    pub fn rot(&self) -> DVMatrix3
    {
        self._pose.rotation.to_rotation_matrix().matrix().clone()
    }

    pub fn set_pos(&mut self, x: f64, y: f64, z: f64)
    {
        self._pose.translation.x = x;
        self._pose.translation.x = y;
        self._pose.translation.x = z;

    }

    pub fn set_rot(&mut self, rot : &DVMatrix3)
    {
        self._pose.rotation = nalgebra::UnitQuaternion::<f64>::from_rotation_matrix(&Rotation3::<f64>::from_matrix(rot));        
    }


    pub fn inverse(&self) -> Pose{
        // TODO: what is inverse?
        //let mut pose = Isometry3::new(self.pos, Rotation3::<f64>::from_matrix(&self.rot).scaled_axis());
        //pose = pose.inverse();
        //Pose{pos : pose.translation.vector, rot: pose.rotation.to_rotation_matrix().matrix().clone(), _pose: pose.clone()}

        Pose{_pose: self._pose.inverse()}
    }
}
impl Mul for Pose {
    type Output = Pose;

    fn mul(self, _other: Pose) -> Pose {
        // TODO
        // implement this, used by tracker
        // (look for current_frame.pose.unwrap() * last_twc)
        Pose{_pose: self._pose* _other._pose}

        //Pose::default()
    }
}
