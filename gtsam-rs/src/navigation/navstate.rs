use cxx::{UniquePtr, SharedPtr};
use crate::{base::vector::{Vector3, Vector3Ref}, geometry::pose3::{Pose3, Pose3Ref}};

pub struct NavState {
    pub inner: UniquePtr<::sys::NavState>,
}

impl NavState {
    pub fn new(
        pose: &Pose3, velocity: &Vector3
    ) -> Self {
        Self {
            inner: ::sys::new_navstate(&pose.inner, &velocity.inner),
        }
    }

    pub fn get_pose(&self) -> Pose3Ref {
        Pose3Ref {
            inner: ::sys::pose(&self.inner)
        }
    }

    pub fn get_velocity(&self) -> Vector3Ref {
        Vector3Ref {
            inner: ::sys::velocity(&self.inner)
        }
    }
}