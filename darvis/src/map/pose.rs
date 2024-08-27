use std::ops::{Mul, Deref};
use core::matrix::{DVMatrix3, DVVector3};
use nalgebra::{Quaternion, UnitQuaternion};
use num_traits::abs;
use serde::{Deserialize, Serialize};

pub type DVTranslation = DVVector3<f64>;
pub type DVRotation = DVMatrix3<f64>;

#[derive(Clone, Serialize, Deserialize, Copy, Default)]
// Note: I'm not sure that Isometry3 is thread safe, could be the same problem
// as with opencv matrices pointing to the same underlying memory even though
// it looks like different objects in Rust. I think we need to be careful here.
pub struct Pose ( nalgebra::IsometryMatrix3<f64> );

impl Pose {
    pub fn new(translation : nalgebra::Vector3<f64>, rotation: nalgebra::Matrix3<f64>) -> Pose {
        let trans = nalgebra::Translation3::from(translation);
        let rot = nalgebra::Rotation3::from_matrix(&rotation);
        let pose = nalgebra::IsometryMatrix3::from_parts(trans, rot);
        Pose (pose)
    }

    pub fn new_with_default_rot(translation: DVTranslation) -> Pose {
        let translation = nalgebra::Translation3::new(
            translation[0] as f64,
            translation[1] as f64,
            translation[2] as f64
        );
        let rotation3 = nalgebra::Rotation3::identity();
        Pose ( nalgebra::IsometryMatrix3::from_parts(translation,rotation3) )
    }

    pub fn new_with_default_trans(rotation: nalgebra::Matrix3<f64>) -> Pose {
        let translation = nalgebra::Translation3::new(0.0, 0.0, 0.0);
        let rot = nalgebra::Rotation3::from_matrix(&rotation);
        Pose ( nalgebra::IsometryMatrix3::from_parts(translation,rot) )
    }


    pub fn identity() -> Pose {
        let translation = nalgebra::Translation3::new(0.0, 0.0, 0.0);
        let rotation3 = nalgebra::Rotation3::identity();
        Pose ( nalgebra::IsometryMatrix3::from_parts(translation,rotation3) )
    }

    // TODO (timing): his forces a copy of the matrix/vector each time, which might not be ideal for just reads.
    pub fn get_translation(&self) -> DVTranslation { DVTranslation::new(self.0.translation.vector) }
    pub fn get_rotation(&self) -> DVRotation { DVRotation::new(*self.0.rotation.matrix()) }
    pub fn get_quaternion(&self) -> nalgebra::geometry::UnitQuaternion<f64> { nalgebra::geometry::UnitQuaternion::from_rotation_matrix(&self.0.rotation) }
    pub fn get_3x4(&self) -> nalgebra::Matrix3x4<f64> { 
        let rot = self.rotation.matrix();
        let trans = self.translation.vector;
        nalgebra::Matrix3x4::<f64>::new(
            rot[0], rot[1], rot[2], trans[0],
            rot[3], rot[4], rot[5], trans[1],
            rot[6], rot[7], rot[8], trans[2],
            // 0.0, 0.0, 0.0, 1.0
        )
    }

    pub fn set_translation(&mut self, trans: nalgebra::Vector3<f64>) {
        let t = nalgebra::Translation3::from(trans);
        self.0.translation = t;
    }

    pub fn set_rotation(&mut self, rot: &DVRotation) {
        self.0.rotation = nalgebra::Rotation3::from_matrix(rot);
    }

    pub fn inverse(&self) -> Pose {
        Pose(self.0.inverse())
    }

    pub fn component_mul(&self, other: &Pose) -> (DVTranslation, DVRotation) {
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
impl Deref for Pose {
    type Target = nalgebra::IsometryMatrix3<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Mul for Pose {
    type Output = Pose;

    fn mul(self, _other: Pose) -> Pose {
        Pose(self.0 * _other.0)
    }
}

impl PartialEq for Pose {
    fn eq(&self, other: &Self) -> bool {
        let t1 = self.get_translation();
        let r1 = self.get_rotation();
        let t2 = other.get_translation();
        let r2 = other.get_rotation();
        abs(t1[0] - t2[0]) < 0.0001 &&
        abs(t1[1] - t2[1]) < 0.0001 &&
        abs(t1[2] - t2[2]) < 0.0001 &&
        abs(r1[0] - r2[0]) < 0.0001 &&
        abs(r1[1] - r2[1]) < 0.0001 &&
        abs(r1[2] - r2[2]) < 0.0001 &&
        abs(r1[3] - r2[3]) < 0.0001
    }
}

//* Into 3x4 matrix */
impl From<Pose> for nalgebra::Matrix3x4<f64> {
    fn from(pose: Pose) -> nalgebra::Matrix3x4<f64> {
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
impl Into<Sim3> for Pose {
    fn into(self) -> Sim3 {
        Sim3 {
            pose: self,
            scale: 1.0
        }
    }
}
//* bindings with g2o */
impl From<Pose> for g2o::ffi::Pose {
    fn from(pose: Pose) -> Self { 
        let quat: nalgebra::geometry::UnitQuaternion<f64> = pose.get_quaternion();
        // Note: quaternion in nalgebra (here) is [w,i,j,k]
        // but in eigen (in C++ bindings) is [i,j,k,w] 
        // but eigen constructor takes [w,i,j,k]
        let rotation = [quat.w, quat.i, quat.j, quat.k];
        let translation = [pose.0.translation.x, pose.0.translation.y, pose.0.translation.z];
        g2o::ffi::Pose { translation, rotation }
    }
}
impl From<g2o::ffi::Pose> for Pose {
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
        Pose ( nalgebra::IsometryMatrix3::from_parts(translation, rotation) )
    }
}
impl From<g2o::ffi::RustSim3> for Sim3 {
    fn from(sim3: g2o::ffi::RustSim3) -> Self {
        let translation = nalgebra::Translation3::new(
            sim3.translation[0],
            sim3.translation[1],
            sim3.translation[2]
        );
        // Note: quaternion in nalgebra (here) is [w,i,j,k]
        // orbslam bindings return wijk
        let rotation = nalgebra::geometry::UnitQuaternion::<f64>::from_quaternion(
            nalgebra::Quaternion::<f64>::new(
                sim3.rotation[0],
                sim3.rotation[1],
                sim3.rotation[2],
                sim3.rotation[3],
            )
        ).to_rotation_matrix();
        Sim3 {
            pose: Pose (nalgebra::IsometryMatrix3::from_parts(translation, rotation)),
            scale: sim3.scale
        }

    }
}

//* bindings with orbslam */
impl From<dvos3binding::ffi::Pose> for Pose {
    fn from(pose: dvos3binding::ffi::Pose) -> Self {
        let translation = nalgebra::Translation3::new(
            pose.translation[0] as f64,
            pose.translation[1] as f64,
            pose.translation[2] as f64
        );

        // Surely this can't be the best way to do this?
        // let matrix3 = nalgebra::Matrix3::<f64>::new(
        //     pose.rotation[0][0].into(), pose.rotation[1][0].into(), pose.rotation[2][0].into(),
        //     pose.rotation[0][1].into(), pose.rotation[1][1].into(), pose.rotation[2][1].into(),
        //     pose.rotation[0][2].into(), pose.rotation[1][2].into(), pose.rotation[2][2].into()
        // );

        let matrix3 = nalgebra::Matrix3::<f64>::new(
            pose.rotation[0][0].into(), pose.rotation[0][1].into(), pose.rotation[0][2].into(),
            pose.rotation[1][0].into(), pose.rotation[1][1].into(), pose.rotation[1][2].into(),
            pose.rotation[2][0].into(), pose.rotation[2][1].into(), pose.rotation[2][2].into()
        );
        let rotation3 = nalgebra::Rotation3::from_matrix_unchecked(matrix3);
        Pose ( nalgebra::IsometryMatrix3::from_parts(translation, rotation3) )
    }
}


/* Pretty print for testing */
impl std::fmt::Debug for Pose {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let rot = self.get_quaternion();
        let trans = self.get_translation();

        write!(
            f,
            "t[{:.5},{:.5},{:.5}] r[{:.4},{:.4},{:.4},{:.4}]",
            trans[0], trans[1], trans[2],
            rot[0], rot[1], rot[2], rot[3],
        )
    }
}


#[derive(Copy)]
pub struct Sim3 {
    pub pose: Pose,
    pub scale: f64,
}
impl Sim3 {
    pub fn new(trans: DVTranslation, rot: DVRotation, scale: f64) -> Sim3 {
        Sim3 {
            pose: Pose::new(*trans,* rot),
            scale
        }
    }
    pub fn new_from_pose(pose: Pose, scale: f64) -> Sim3 {
        Sim3 {
            pose,
            scale
        }
    }
    pub fn new_scaled_rotation(&self) -> Self {
        // In ORB_SLAM3, see Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) in Converter.cc
        // Sophus::RxSO3d scales the rotation quaternion by the square root of the scale...
        // From sophus rxso3.hpp:
            // Constructor from scale factor and rotation matrix ``R``.
            //
            // Precondition: Rotation matrix ``R`` must to be orthogonal with determinant
            //               of 1 and ``scale`` must not be close to zero.
            //
            // SOPHUS_FUNC RxSO3(Scalar const& scale, Transformation const& R)
            //     : quaternion_(R) {
            // SOPHUS_ENSURE(scale >= Constants<Scalar>::epsilon(),
            //                 "Scale factor must be greater-equal epsilon.");
            // using std::sqrt;
            // quaternion_.coeffs() *= sqrt(scale);
            // }
        let new_rot = *self.pose.get_rotation() * f64::sqrt(self.scale);
        let new_pose = Pose::new(
            *self.pose.get_translation(), new_rot
        );

        Sim3{
            pose: new_pose,
            scale: self.scale
        }
    }

    pub fn identity() -> Sim3 {
        Sim3 {
            pose: Pose::identity(),
            scale: 1.0
        }
    }
    pub fn inverse(&self) -> Sim3 {
        // From g2o, see:
            // Sim3 inverse() const
            // {
            //   return Sim3(r.conjugate(), r.conjugate()*((-1./s)*t), 1./s);
            // }
        let rot_conjugated: UnitQuaternion<f64> = self.pose.get_quaternion().conjugate();
        let trans = rot_conjugated * ((-1.0 / self.scale) * *self.pose.get_translation());

        let pose = nalgebra::IsometryMatrix3::from_parts(
            nalgebra::Translation3::from(trans),
            rot_conjugated.to_rotation_matrix()
        );

        Sim3 {
            pose: Pose(pose),
            scale: 1.0 / self.scale
        }
    }
    pub fn map(&self, other: &DVVector3<f64>) -> DVVector3<f64> {
        DVVector3::new((*self.pose.get_rotation() * **other).mul(self.scale) + *self.pose.get_translation())
    }
}
impl Clone for Sim3 {
    fn clone(&self) -> Self {
        Sim3 {
            pose: self.pose.clone(),
            scale: self.scale
        }
    }
}
impl Mul for Sim3 {
    type Output = Sim3;

    fn mul(self, other: Sim3) -> Sim3 {
        let r = *self.pose.get_rotation();
        let t = *self.pose.get_translation();
        let s = self.scale;
        let o_r = *other.pose.get_rotation();
        let o_t = *other.pose.get_translation();
        let o_s = other.scale;
        Sim3 {
            pose: Pose::new(
                s * (r * o_t) + t,
                r * o_r,
            ),
            scale: s * o_s
        }
    }
}
impl Into<Pose> for Sim3 {
    fn into(self) -> Pose {
        Pose::new(
            *self.pose.get_translation() / self.scale,
            *self.pose.get_rotation()
        )
    }
}

/* Pretty print for testing */
impl std::fmt::Debug for Sim3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let rot = self.pose.get_quaternion();
        let trans = self.pose.get_translation();

        write!(
            f,
            "t[{:.5},{:.5},{:.5}] r[{:.4},{:.4},{:.4},{:.4}] s {:.3}",
            trans[0], trans[1], trans[2],
            rot[0], rot[1], rot[2], rot[3],
            self.scale
        )
    }
}

impl Into<g2o::ffi::RustSim3> for Sim3 {
    fn into(self) -> g2o::ffi::RustSim3 {
        let pose: g2o::ffi::Pose = self.pose.into();
        g2o::ffi::RustSim3 {
            translation: pose.translation,
            rotation: pose.rotation,
            scale: self.scale
        }
    }
}