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
