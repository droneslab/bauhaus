use std::ops::Mul;
use nalgebra::{
    Isometry3, Rotation3, Quaternion, Translation3,
    geometry::UnitQuaternion,
    Vector3, Matrix3
};
use serde::{Deserialize, Serialize};
use crate::dvutils::*;

#[derive(Debug, Clone, Serialize, Deserialize, Copy)]
pub struct Pose {
    pose: Isometry3<f64>
}

impl Pose {
    pub fn default() -> Pose {
        Pose { pose : Isometry3::identity() }
    }

    // sofiya...need to get rid of this
    pub fn new_from_opencv_vector(translation : &Vector3<f64>, rotation: &Matrix3<f64>) -> Pose {
        let pose = Isometry3::new( // why not use from_rotation_matrix() ?
            translation.clone(),
            Rotation3::<f64>::from_matrix(&rotation).scaled_axis()
        );
        Pose { pose: pose }
    }

    pub fn new_from_vector(translation : &DVVector3<f64>, rotation: &DVMatrix3<f64>) -> Pose {
        let pose = Isometry3::new( // why not use from_rotation_matrix() ?
            translation.into(),
            Rotation3::<f64>::from_matrix(&rotation.into()).scaled_axis()
        );
        Pose { pose: pose }
    }

    pub fn new_from_bridgepose(pose: g2orust::ffi::Pose) -> Pose {
        // g2orust::ffi:Pose needed to send pose to C++ bindings to g2o
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
        let rotation = UnitQuaternion::<f64>::from_quaternion(quaternion);
        Pose { pose: Isometry3::from_parts(translation,rotation) }
    }

    pub fn convert_to_bridgepose(&self) -> g2orust::ffi::Pose {
        // g2orust::ffi:Pose needed to send pose to C++ bindings to g2os
        // Note: can't implement From trait because 
        // g2orust::ffi::Pose isn't defined in this crate
        g2orust::ffi::Pose {
            translation: self.translation_to_array(),
            rotation: self.rotation_quaternion_to_array()
        }
    }

    pub fn translation_to_array(&self) -> [f64; 3] {
        [self.pose.translation.x, self.pose.translation.y, self.pose.translation.z]
    }

    pub fn rotation_quaternion_to_array(&self) -> [f64; 4] {
        [self.pose.rotation.w, self.pose.rotation.i, self.pose.rotation.j, self.pose.rotation.k]
    }

    pub fn get_translation(&self) -> DVVector3<f64> {
        DVVector3::new(self.pose.translation.vector)
    }

    pub fn get_rotation(&self) -> DVMatrix3<f64> {
        DVMatrix3::new(*self.pose.rotation.to_rotation_matrix().matrix())
    }

    pub fn set_translation(&mut self, x: f64, y: f64, z: f64) {
        self.pose.translation.x = x;
        self.pose.translation.x = y;
        self.pose.translation.x = z;
    }

    pub fn set_rotation(&mut self, rot : &DVMatrix3<f64>) {
        // self.pose.rotation = nalgebra::UnitQuaternion::<f64>::from_rotation_matrix(&Rotation3::<f64>::from_matrix(rot.into()));
        self.pose.rotation = nalgebra::UnitQuaternion::<f64>::from_matrix(&rot.into());
    }

    pub fn inverse(&self) -> Pose{
        Pose{pose: self.pose.inverse()}
    }
}

impl Mul for Pose {
    type Output = Pose;

    fn mul(self, _other: Pose) -> Pose {
        Pose{pose: self.pose * _other.pose}
    }
}

impl From<g2orust::ffi::Pose> for Pose {
    fn from(pose: g2orust::ffi::Pose) -> Self { 
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
        let rotation = UnitQuaternion::<f64>::from_quaternion(quaternion);
        Pose { pose: Isometry3::from_parts(translation,rotation) }
    }
}
// impl Into<Pose> for g2orust::ffi::Pose {
//     fn into(&self) -> g2orust::ffi::Pose { 
//         g2orust::ffi::Pose {
//             translation: self.translation_to_array(),
//             rotation: self.rotation_quaternion_to_array()
//         }
//     }
// }
