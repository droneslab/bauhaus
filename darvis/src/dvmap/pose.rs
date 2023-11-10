use std::ops::Mul;
use dvcore::matrix::{DVVector3, DVMatrix3};
use nalgebra::{IsometryMatrix3, Rotation3, Quaternion, Translation3,geometry::UnitQuaternion, Vector3, Matrix3, Matrix3x4};
use serde::{Deserialize, Serialize};

// TODO: Should we use nalgebra::Translation3 instead of our own matrices? Seems like there's a lot of converting happening right now
pub type DVTranslation = DVVector3<f64>;
pub type DVRotation = DVMatrix3<f64>;

#[derive(Debug, Clone, Serialize, Deserialize, Copy, Default)]
// Note: I'm not sure that Isometry3 is thread safe, could be the same problem
// as with opencv matrices pointing to the same underlying memory even though
// it looks like different objects in Rust. I think we need to be careful here.
pub struct DVPose ( IsometryMatrix3<f64> );

impl DVPose {
    pub fn new(translation : &Vector3<f64>, rotation: &Matrix3<f64>) -> DVPose {
        let trans = Translation3::from(translation.clone());
        let rot = Rotation3::from_matrix(rotation);
        let pose = IsometryMatrix3::from_parts(trans, rot);
        DVPose (pose)
    }

    pub fn new_with_default_rot(translation: DVTranslation) -> DVPose {
        let translation = Translation3::new(
            translation[0] as f64,
            translation[1] as f64,
            translation[2] as f64
        );
        let rotation3 = Rotation3::identity();
        DVPose ( IsometryMatrix3::from_parts(translation,rotation3) )
    }

    // TODO (memory): his forces a copy of the matrix/vector each time, which might not be ideal for just reads.
    pub fn get_translation(&self) -> DVTranslation { DVTranslation::new(self.0.translation.vector) }
    pub fn get_rotation(&self) -> DVRotation { DVRotation::new(*self.0.rotation.matrix()) }
    pub fn get_quaternion(&self) -> UnitQuaternion<f64> { UnitQuaternion::from_rotation_matrix(&self.0.rotation) }

    pub fn set_translation(&mut self, x: f64, y: f64, z: f64) {
        self.0.translation.x = x;
        self.0.translation.y = y;
        self.0.translation.z = z;
    }

    pub fn set_rotation(&mut self, rot: &DVRotation) {
        self.0.rotation = Rotation3::from_matrix(rot);
    }

    pub fn inverse(&self) -> DVPose{
        DVPose(self.0.inverse())
    }
}

impl Mul for DVPose {
    type Output = DVPose;

    fn mul(self, _other: DVPose) -> DVPose {
        DVPose(self.0 * _other.0)
    }
}

//* Into 3x4 matrix */
impl From<DVPose> for Matrix3x4<f64> {
    fn from(pose: DVPose) -> Matrix3x4<f64> {
        let binding = pose.0.rotation;
        let r = binding.matrix();
        let t = *pose.get_translation();
        let matrix = nalgebra::Matrix3x4::new(
            r[(0,0)], r[(0,1)], r[(0,2)], t[0],
            r[(1,0)], r[(1,1)], r[(1,2)], t[1],
            r[(2,0)], r[(2,1)], r[(2,2)], t[2]
        );

        matrix
    }
}

//* bindings with g2o */
impl From<DVPose> for g2o::ffi::Pose {
    fn from(pose: DVPose) -> Self { 
        let quat: UnitQuaternion<f64> = pose.get_quaternion();
        // Note: quaternion in nalgebra (here) is [w,i,j,k]
        // but in eigen (in C++ bindings) is [i,j,k,w] 
        let rotation = [quat.i, quat.j, quat.k, quat.w];
        let translation = [pose.0.translation.x, pose.0.translation.y, pose.0.translation.z];

        g2o::ffi::Pose { translation, rotation }
    }
}
impl From<g2o::ffi::Pose> for DVPose {
    fn from(pose: g2o::ffi::Pose) -> Self {
        let translation = Translation3::new(
            pose.translation[0],
            pose.translation[1],
            pose.translation[2]
        );
        // Note: quaternion in nalgebra (here) is [w,i,j,k]
        // but in eigen (in C++ bindings) is [i,j,k,w] 
        let rotation = UnitQuaternion::<f64>::from_quaternion(
            Quaternion::<f64>::new(
                pose.rotation[3],
                pose.rotation[0],
                pose.rotation[1],
                pose.rotation[2],
            )
        ).to_rotation_matrix();
        DVPose ( IsometryMatrix3::from_parts(translation, rotation) )
    }
}

//* bindings with orbslam */
impl From<dvos3binding::ffi::Pose> for DVPose {
    fn from(pose: dvos3binding::ffi::Pose) -> Self {
        let translation = Translation3::new(
            pose.translation[0] as f64,
            pose.translation[1] as f64,
            pose.translation[2] as f64
        );

        // Surely this can't be the best way to do this?
        let matrix3 = Matrix3::<f64>::new(
            pose.rotation[0][0].into(), pose.rotation[1][0].into(), pose.rotation[2][0].into(),
            pose.rotation[0][1].into(), pose.rotation[1][1].into(), pose.rotation[2][1].into(),
            pose.rotation[0][2].into(), pose.rotation[1][2].into(), pose.rotation[2][2].into()
        );
        let rotation3 = Rotation3::from_matrix_unchecked(matrix3);
        DVPose ( IsometryMatrix3::from_parts(translation, rotation3) )
    }
}
