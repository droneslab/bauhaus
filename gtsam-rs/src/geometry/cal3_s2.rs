pub struct Cal3S2 {
    pub(crate) inner: cxx::SharedPtr<sys::Cal3_S2>,
}

impl Cal3S2 {
    pub fn default() -> Self {
        Self {
            inner: sys::default_cal3_s2(),
        }
    }
}