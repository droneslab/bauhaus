use std::ops::Mul;
use nalgebra::{
    Isometry3, Rotation3, Quaternion, Translation3,
    geometry::UnitQuaternion
};
use serde::{Deserialize, Serialize};
use crate::dvutils::*;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    // Position 3-Vector
    //pub pos: DVVector3,
    // Rotation 3x3 Matrix
    //pub rot: DVMatrix3,
    pose: Isometry3<f64>
}

impl Pose {
    pub fn default() -> Pose {
        Pose {
            //pos: DVVector3::new(0.0,0.0,0.0),
            //rot: DVMatrix3::identity(),
            pose : Isometry3::identity() 
            //::new(DVVector3::new(0.0,0.0,0.0), Rotation3::<f64>::from_matrix(&DVMatrix3::identity()).scaled_axis())
        }
    }

    // Sofiya note: Should pos be renamed to translation?
    pub fn new(pos : &DVVector3, rot: &DVMatrix3) -> Pose {
        Pose {
            //pos: pos.clone(),
            //rot: rot.clone(),
            pose: Isometry3::new(pos.clone(), Rotation3::<f64>::from_matrix(&rot).scaled_axis())
        }
    }

    pub fn new_from_bridgepose(pose: g2orust::ffi::Pose) -> Pose {
        let translation = Translation3::new(
            pose.translation[0],
            pose.translation[1],
            pose.translation[2]
        );
        let quaternion = Quaternion::<f64>::new(
            pose.rotation[0],
            pose.rotation[1],
            pose.rotation[2],
            pose.rotation[3],
        );
        let rotation = UnitQuaternion::<f64>::from_quaternion(quaternion);//.to_rotation_matrix();
        Pose {
            pose: Isometry3::from_parts(translation,rotation)
        }
    }

    pub fn t_to_array(&self) -> [f64; 3] {
        [self.pose.translation.x, self.pose.translation.y, self.pose.translation.z]
    }

    pub fn r_quaternion_to_array(&self) -> [f64; 4] {
        [self.pose.rotation.w, self.pose.rotation.i, self.pose.rotation.j, self.pose.rotation.k]
    }

    pub fn pos(&self) -> DVVector3
    {
        self.pose.translation.vector.clone()
    }

    pub fn rot(&self) -> DVMatrix3
    {
        self.pose.rotation.to_rotation_matrix().matrix().clone()
    }

    pub fn set_pos(&mut self, x: f64, y: f64, z: f64)
    {
        self.pose.translation.x = x;
        self.pose.translation.x = y;
        self.pose.translation.x = z;

    }

    pub fn set_rot(&mut self, rot : &DVMatrix3)
    {
        self.pose.rotation = nalgebra::UnitQuaternion::<f64>::from_rotation_matrix(&Rotation3::<f64>::from_matrix(rot));        
    }


    pub fn inverse(&self) -> Pose{
        // TODO: what is inverse?
        //let mut pose = Isometry3::new(self.pos, Rotation3::<f64>::from_matrix(&self.rot).scaled_axis());
        //pose = pose.inverse();
        //Pose{pos : pose.translation.vector, rot: pose.rotation.to_rotation_matrix().matrix().clone(), pose: pose.clone()}

        Pose{pose: self.pose.inverse()}
    }
}
impl Mul for Pose {
    type Output = Pose;

    fn mul(self, _other: Pose) -> Pose {
        // TODO
        // implement this, used by tracker
        // (look for current_frame.pose.unwrap() * last_twc)
        Pose{pose: self.pose* _other.pose}

        //Pose::default()
    }
}
