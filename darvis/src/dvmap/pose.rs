use std::ops::Mul;
use dvcore::matrix::{DVVector3, DVMatrix3};
use log::debug;
use nalgebra::{Isometry3, Rotation3, Quaternion, Translation3,geometry::UnitQuaternion, Vector3, Matrix3};
use serde::{Deserialize, Serialize};

pub type Translation = DVVector3<f64>; // TODO: Should we use nalgebra::Translation3 instead of our own matrices?
pub type Rotation = DVMatrix3<f64>;

#[derive(Debug, Clone, Serialize, Deserialize, Copy, Default)]
// Note: I'm not sure that Isometry3 is thread safe, could be the same problem
// as with opencv matrices pointing to the same underlying memory even though
// it looks like different objects in Rust. I think we need to be careful here.
pub struct Pose ( Isometry3<f64> );

impl Pose {
    pub fn new(translation : &Vector3<f64>, rotation: &Matrix3<f64>) -> Pose {
        let pose = Isometry3::new( // why not use from_rotation_matrix() ?
            translation.clone(),
            Rotation3::<f64>::from_matrix(&rotation).scaled_axis()
        );
        Pose (pose)
    }

    // TODO (design, runtime optimization): I think we should rethink how pose, translation, and rotation is structured
    // Which variables should we be returning and when?
    // This forces a copy of the matrix/vector each time, which might not be ideal for just reads.
    pub fn get_translation(&self) -> Translation { DVVector3::new(self.0.translation.vector) }
    pub fn get_rotation(&self) -> Rotation { DVMatrix3::new(*self.0.rotation.to_rotation_matrix().matrix()) }

    pub fn set_translation(&mut self, x: f64, y: f64, z: f64) {
        self.0.translation.x = x;
        self.0.translation.x = y;
        self.0.translation.x = z;
    }

    pub fn set_rotation(&mut self, rot: &Rotation) {
        self.0.rotation = nalgebra::UnitQuaternion::<f64>::from_matrix(&rot.into());
    }

    pub fn inverse(&self) -> Pose{
        Pose(self.0.inverse())
    }
}
impl From<Pose> for [f64; 4] {
    fn from(pose: Pose) -> [f64; 4] { [pose.0.rotation.w, pose.0.rotation.i, pose.0.rotation.j, pose.0.rotation.k] }
}
impl From<Pose> for [f64; 3] {
    fn from(pose: Pose) -> [f64; 3]  { [pose.0.translation.x, pose.0.translation.y, pose.0.translation.z] }
}

impl Mul for Pose {
    type Output = Pose;

    fn mul(self, _other: Pose) -> Pose {
        Pose(self.0 * _other.0)
    }
}

impl From<g2o::ffi::Pose> for Pose {
    fn from(pose: g2o::ffi::Pose) -> Self {
        let translation = Translation3::new(
            pose.translation[0],
            pose.translation[1],
            pose.translation[2]
        );
        let rotation = UnitQuaternion::<f64>::from_quaternion(
            Quaternion::<f64>::new(
                pose.rotation[0],
                pose.rotation[1],
                pose.rotation[2],
                pose.rotation[3],
            )
        );
        Pose ( Isometry3::from_parts(translation,rotation) )
    }
}
impl From<Pose> for g2o::ffi::Pose {
    fn from(pose: Pose) -> Self { 
        g2o::ffi::Pose {
            translation: pose.into(),
            rotation: pose.into()
        }
    }
}

impl From<dvos3binding::ffi::Pose> for Pose {
    fn from(pose: dvos3binding::ffi::Pose) -> Self {
        let translation = Translation3::new(
            pose.translation[0] as f64,
            pose.translation[1] as f64,
            pose.translation[2] as f64
        );
        let rotation = UnitQuaternion::<f64>::from_quaternion(
            Quaternion::<f64>::new(
                pose.rotation[0] as f64,
                pose.rotation[1] as f64,
                pose.rotation[2] as f64,
                pose.rotation[3] as f64,
            )
        );
        Pose ( Isometry3::from_parts(translation,rotation) )
    }
}