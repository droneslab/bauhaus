use cxx::UniquePtr;

pub struct ImuFactor {
    pub(crate) _inner: UniquePtr<::sys::ImuFactor>,
}

impl Default for ImuFactor {
    fn default() -> Self {
        Self {
            _inner: ::sys::default_imufactor(),
        }
    }
}

pub struct PreintegratedImuMeasurements {
    pub(crate) _inner: UniquePtr<::sys::PreintegratedImuMeasurements>,
}
impl Default for PreintegratedImuMeasurements {
    fn default() -> Self {
        Self {
            _inner: ::sys::default_preintegratedimumeasurements(),
        }
    }
}