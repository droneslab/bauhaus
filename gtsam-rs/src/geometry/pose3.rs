use cxx::UniquePtr;
use nalgebra::{Isometry3, IsometryMatrix3, UnitQuaternion, Vector3};
use std::fmt::{Debug, Formatter, Result};

use super::{
    point3::{Point3, Point3Ref},
    rot3::{Rot3, Rot3Ref},
};

pub struct Pose3 {
    pub(crate) inner: UniquePtr<::sys::Pose3>,
}

impl Default for Pose3 {
    fn default() -> Self {
        Self {
            inner: ::sys::default_pose3(),
        }
    }
}

impl From<Isometry3<f64>> for Pose3 {
    fn from(value: Isometry3<f64>) -> Self {
        Self::from_parts(value.translation.into(), value.rotation.into())
    }
}
impl From<&Pose3> for Isometry3<f64> {
    fn from(value: &Pose3) -> Self {
        Isometry3::from_parts(value.translation().into(), value.rotation().into())
    }
}

impl Pose3 {
    pub fn new(translation: Vector3<f64>, axisangle: Vector3<f64>) -> Self {
        Self::from_parts(
            translation.into(),
            UnitQuaternion::from_scaled_axis(axisangle).into(),
        )
    }

    pub fn from_parts(point: Point3, rotation: Rot3) -> Self {
        Self {
            inner: ::sys::new_pose3(&rotation.inner, &point.inner),
        }
    }

    pub fn translation(&'_ self) -> Point3Ref<'_> {
        Point3Ref {
            inner: ::sys::pose3_translation(self.inner.as_ref().unwrap()),
        }
    }
    pub fn rotation(&'_ self) -> Rot3Ref<'_> {
        Rot3Ref {
            inner: ::sys::pose3_rotation(self.inner.as_ref().unwrap()),
        }
    }
}
impl Debug for Pose3 {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let pose_nalgebra: Isometry3<f64> = self.into();
        write!(f, "Pose3: {:?}", pose_nalgebra)
    }
}

pub struct Pose3Ref<'a> {
    pub(crate) inner: &'a ::sys::Pose3,
}

impl<'a> From<Pose3Ref<'a>> for IsometryMatrix3<f64> {
    fn from(value: Pose3Ref<'a>) -> Self {

        let trans = {
            let temp: nalgebra::Vector3<f64> = value.translation().into();
            nalgebra::Translation3::from(temp)
        };
        let rot = nalgebra::Rotation3::from_matrix_unchecked(value.rotation().into());
        // println!("Rotation in isometrymatrix: {:?}", rot);
        // println!("...Quaternion is: {:?}", UnitQuaternion::from_rotation_matrix(&rot));
        nalgebra::IsometryMatrix3::from_parts(trans, rot)
    }
}

impl<'a> Pose3Ref<'a> {
    pub fn rotation(&'_ self) -> Rot3Ref<'_> {
        Rot3Ref {
            inner: ::sys::pose3_rotation(self.inner),
        }
    }

    pub fn translation<'b>(&'_ self) -> Point3Ref<'_> {
        Point3Ref {
            inner: ::sys::pose3_translation(self.inner),
        }
    }
}

impl<'a> From<Pose3Ref<'a>> for Pose3 {
    fn from(value: Pose3Ref<'a>) -> Self {
        let trans = Point3Ref {
            inner: ::sys::pose3_translation(value.inner),
        };

        let rot = Rot3Ref {
            inner: ::sys::pose3_rotation(value.inner),
        };
        Self::from_parts(trans.into(), rot.into())
    }
}
