use cxx::UniquePtr;

pub struct Point2 {
    pub(crate) inner: UniquePtr<::sys::Point2>,
}

impl Default for Point2 {
    fn default() -> Self {
        Self {
            inner: ::sys::default_point2(),
        }
    }
}

impl From<::nalgebra::Vector2<f64>> for Point2 {
    fn from(value: ::nalgebra::Vector2<f64>) -> Self {
        Self::new(value.x, value.y)
    }
}

impl Point2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            inner: ::sys::new_point2(x, y),
        }
    }
}



pub struct StereoPoint2 {
    pub(crate) inner: UniquePtr<::sys::StereoPoint2>,
}

impl Default for StereoPoint2 {
    fn default() -> Self {
        Self {
            inner: ::sys::default_stereopoint2(),
        }
    }
}

impl From<::nalgebra::Vector3<f64>> for StereoPoint2 {
    fn from(value: ::nalgebra::Vector3<f64>) -> Self {
        Self::new(value.x, Some(value.y), value.z)
    }
}

impl StereoPoint2 {
    pub fn new(u_l: f64, u_r: Option<f64>, v: f64) -> Self {
        if let Some(u_r) = u_r {
            Self {
                inner: ::sys::new_stereopoint2(u_l, u_r, v),
            }
        } else {
            Self {
                inner: ::sys::new_stereopoint2_nour(u_l, v)
            }
        }
    }
}
