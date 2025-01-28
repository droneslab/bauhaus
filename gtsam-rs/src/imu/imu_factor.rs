use cxx::UniquePtr;

pub struct ImuFactor {
    pub(crate) inner: UniquePtr<::sys::ImuFactor>,
}

impl Default for ImuFactor {
    fn default() -> Self {
        Self {
            inner: ::sys::default_imufactor(),
        }
    }
}

pub struct PreintegratedImuMeasurements {
    pub(crate) inner: UniquePtr<::sys::PreintegratedImuMeasurements>,
}
impl Default for PreintegratedImuMeasurements {
    fn default() -> Self {
        Self {
            inner: ::sys::default_preintegratedimumeasurements(),
        }
    }
}