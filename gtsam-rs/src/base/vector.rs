use cxx::UniquePtr;
use std::fmt::Debug;

pub struct Vector3 {
    pub(crate) inner: UniquePtr<::sys::Vector3>,
}

impl Default for Vector3 {
    fn default() -> Self {
        Self {
            inner: ::sys::default_vector3(),
        }
    }
}

impl Vector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: ::sys::new_vector3(x, y, z),
        }
    }
    pub fn get_raw(&self) -> [f64; 3] {
        let mut dst = [0.0; 3];
        ::sys::vector3_to_raw(self.inner.as_ref().unwrap(), &mut dst);
        dst
    }
}

impl Debug for Vector3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut dst = [0.0; 3];
        ::sys::vector3_to_raw(self.inner.as_ref().unwrap(), &mut dst);
        write!(f, "Vector3: {:?}", dst)
    }
}

impl From<::nalgebra::Vector3<f64>> for Vector3 {
    fn from(value: ::nalgebra::Vector3<f64>) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}


pub struct Vector3Ref<'a> {
    pub(crate) inner: &'a ::sys::Vector3,
}
impl Vector3Ref<'_> {
    pub fn get_raw(&self) -> [f64; 3] {
        let mut dst = [0.0; 3];
        ::sys::vector3_to_raw(self.inner, &mut dst);
        dst
    }
}

impl<'a> From<Vector3Ref<'a>> for Vector3 {
    fn from(value: Vector3Ref<'a>) -> Self {
        let mut dst = [0.0; 3];
        ::sys::vector3_to_raw(value.inner, &mut dst);
        Vector3::new(dst[0], dst[1], dst[2])
    }
}
impl Debug for Vector3Ref<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut dst = [0.0; 3];
        ::sys::vector3_to_raw(self.inner, &mut dst);
        write!(f, "Vector3Ref: {:?}", dst)
    }
}