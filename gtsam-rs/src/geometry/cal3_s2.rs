pub struct Cal3S2 {
    pub(crate) inner: cxx::SharedPtr<sys::Cal3_S2>,
}

impl Cal3S2 {
    pub fn default() -> Self {
        Self {
            inner: sys::default_cal3_s2(),
        }
    }

    pub fn new(fx: f64, fy: f64, s: f64, u0: f64, v0: f64) -> Self {
        Self {
            inner: sys::new_cal3_s2(fx, fy, s, u0, v0),
        }
    }
}