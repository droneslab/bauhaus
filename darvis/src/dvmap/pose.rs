use std::ops::{Mul, Deref};
use dvcore::matrix::{DVVector3, DVMatrix3};
use serde::{Deserialize, Serialize};

pub type DVTranslation = DVVector3<f64>;
pub type DVRotation = DVMatrix3<f64>;

#[derive(Clone, Serialize, Deserialize, Copy, Default)]
// Note: I'm not sure that Isometry3 is thread safe, could be the same problem
// as with opencv matrices pointing to the same underlying memory even though
// it looks like different objects in Rust. I think we need to be careful here.
pub struct DVPose ( nalgebra::IsometryMatrix3<f64> );

impl DVPose {
    pub fn new(translation : nalgebra::Vector3<f64>, rotation: nalgebra::Matrix3<f64>) -> DVPose {
        let trans = nalgebra::Translation3::from(translation);
        let rot = nalgebra::Rotation3::from_matrix(&rotation);
        let pose = nalgebra::IsometryMatrix3::from_parts(trans, rot);
        DVPose (pose)
    }

    pub fn new_with_default_rot(translation: DVTranslation) -> DVPose {
        let translation = nalgebra::Translation3::new(
            translation[0] as f64,
            translation[1] as f64,
            translation[2] as f64
        );
        let rotation3 = nalgebra::Rotation3::identity();
        DVPose ( nalgebra::IsometryMatrix3::from_parts(translation,rotation3) )
    }

    // TODO (timing): his forces a copy of the matrix/vector each time, which might not be ideal for just reads.
    pub fn get_translation(&self) -> DVTranslation { DVTranslation::new(self.0.translation.vector) }
    pub fn get_rotation(&self) -> DVRotation { DVRotation::new(*self.0.rotation.matrix()) }
    pub fn get_quaternion(&self) -> nalgebra::geometry::UnitQuaternion<f64> { nalgebra::geometry::UnitQuaternion::from_rotation_matrix(&self.0.rotation) }

    pub fn set_translation(&mut self, trans: nalgebra::Vector3<f64>) {
        let t = nalgebra::Translation3::from(trans);
        self.0.translation = t;
    }

    pub fn set_rotation(&mut self, rot: &DVRotation) {
        self.0.rotation = nalgebra::Rotation3::from_matrix(rot);
    }

    pub fn inverse(&self) -> DVPose {
        DVPose(self.0.inverse())
    }

    pub fn component_mul(&self, other: &DVPose) -> (DVTranslation, DVRotation) {
        let matrix_self = self.0.to_matrix();
        let matrix_other = (*other).to_matrix();
        let pose12 = matrix_self.component_mul(&matrix_other);
        let rot_mat = pose12.fixed_view::<3,3>(0, 0);
        let trans_mat = pose12.fixed_view::<1,3>(2, 0);
        let trans = DVTranslation::new_with(trans_mat[0], trans_mat[1], trans_mat[2]);
        let rot = DVRotation::new(rot_mat.into());
        (trans, rot)
    }

}
impl Deref for DVPose {
    type Target = nalgebra::IsometryMatrix3<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Mul for DVPose {
    type Output = DVPose;

    fn mul(self, _other: DVPose) -> DVPose {
        DVPose(self.0 * _other.0)
    }
}

//* Into 3x4 matrix */
impl From<DVPose> for nalgebra::Matrix3x4<f64> {
    fn from(pose: DVPose) -> nalgebra::Matrix3x4<f64> {
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
        let quat: nalgebra::geometry::UnitQuaternion<f64> = pose.get_quaternion();
        // Note: quaternion in nalgebra (here) is [w,i,j,k]
        // but in eigen (in C++ bindings) is [i,j,k,w] 
        // but eigen constructor takes [w,i,j,k]
        let rotation = [quat.w, quat.i, quat.j, quat.k];
        let translation = [pose.0.translation.x, pose.0.translation.y, pose.0.translation.z];
        g2o::ffi::Pose { translation, rotation }
    }
}
impl From<g2o::ffi::Pose> for DVPose {
    fn from(pose: g2o::ffi::Pose) -> Self {
        let translation = nalgebra::Translation3::new(
            pose.translation[0],
            pose.translation[1],
            pose.translation[2]
        );
        // Note: quaternion in nalgebra (here) is [w,i,j,k]
        // orbslam bindings return wijk
        let rotation = nalgebra::geometry::UnitQuaternion::<f64>::from_quaternion(
            nalgebra::Quaternion::<f64>::new(
                pose.rotation[0],
                pose.rotation[1],
                pose.rotation[2],
                pose.rotation[3],
            )
        ).to_rotation_matrix();
        DVPose ( nalgebra::IsometryMatrix3::from_parts(translation, rotation) )
    }
}

//* bindings with orbslam */
impl From<dvos3binding::ffi::Pose> for DVPose {
    fn from(pose: dvos3binding::ffi::Pose) -> Self {
        let translation = nalgebra::Translation3::new(
            pose.translation[0] as f64,
            pose.translation[1] as f64,
            pose.translation[2] as f64
        );

        // Surely this can't be the best way to do this?
        let matrix3 = nalgebra::Matrix3::<f64>::new(
            pose.rotation[0][0].into(), pose.rotation[1][0].into(), pose.rotation[2][0].into(),
            pose.rotation[0][1].into(), pose.rotation[1][1].into(), pose.rotation[2][1].into(),
            pose.rotation[0][2].into(), pose.rotation[1][2].into(), pose.rotation[2][2].into()
        );
        let rotation3 = nalgebra::Rotation3::from_matrix_unchecked(matrix3);
        DVPose ( nalgebra::IsometryMatrix3::from_parts(translation, rotation3) )
    }
}


/* Pretty print for testing */
impl std::fmt::Debug for DVPose {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let rot = self.get_quaternion();
        let trans = self.get_translation();

        write!(
            f,
            "translation = [{:.4},{:.4},{:.4}] / rotation = [{:.4},{:.4},{:.4},{:.4}]",
            trans[0], trans[1], trans[2],
            rot[0], rot[1], rot[2], rot[3],
        )
    }
}
