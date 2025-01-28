use cxx::UniquePtr;

pub struct LevenbergMarquardtParams {
    pub(super) inner: UniquePtr<::sys::LevenbergMarquardtParams>,
}

impl Default for LevenbergMarquardtParams {
    fn default() -> Self {
        Self {
            inner: ::sys::default_levenberg_marquardt_params(),
        }
    }
}

impl LevenbergMarquardtParams {
    pub fn set_max_iterations(&mut self, value: u32) {
        ::sys::levenberg_marquardt_params_set_max_iterations(self.inner.pin_mut(), value)
    }
    pub fn set_lambda_upper_bound(&mut self, value: f64) {
        ::sys::levenberg_marquardt_params_set_lambda_upper_bound(self.inner.pin_mut(), value)
    }
    pub fn set_lambda_lower_bound(&mut self, value: f64) {
        ::sys::levenberg_marquardt_params_set_lambda_lower_bound(self.inner.pin_mut(), value)
    }
    pub fn set_diagonal_damping(&mut self, flag: bool) {
        ::sys::levenberg_marquardt_params_set_diagonal_damping(self.inner.pin_mut(), flag)
    }
    pub fn set_relative_error_to_l(&mut self, value: f64) {
        ::sys::levenberg_marquardt_params_set_relative_error_to_l(self.inner.pin_mut(), value)
    }
    pub fn set_absolute_error_to_l(&mut self, value: f64) {
        ::sys::levenberg_marquardt_params_set_absolute_error_to_l(self.inner.pin_mut(), value)
    }

}
