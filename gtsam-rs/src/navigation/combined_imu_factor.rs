use cxx::{UniquePtr, SharedPtr};
use crate::{base::vector::Vector3, imu::imu_bias::ConstantBias, inference::key::IntoKey};

use super::navstate::NavState;

pub struct CombinedImuFactor {
    pub inner: SharedPtr<::sys::CombinedImuFactor>,
}

impl Default for CombinedImuFactor {
    fn default() -> Self {
        Self {
            inner: ::sys::default_combined_imu_factor(),
        }
    }
}
impl CombinedImuFactor {
    pub fn new(
        pose_i: impl IntoKey, pose_j: impl IntoKey, vel_i: impl IntoKey, vel_j: impl IntoKey, 
        bias_i: impl IntoKey, bias_j: impl IntoKey, measurements: &PreintegratedCombinedMeasurements
    ) -> Self {
        Self {
            inner: ::sys::new_combined_imu_factor(
                pose_i.into_key(),
                pose_j.into_key(),
                vel_i.into_key(),
                vel_j.into_key(),
                bias_i.into_key(),
                bias_j.into_key(),
                & measurements.inner,
            ),
        }
    }
}

pub struct PreintegratedCombinedMeasurements {
    pub(super) inner: UniquePtr<::sys::PreintegratedCombinedMeasurements>,
}

impl Default for PreintegratedCombinedMeasurements {
    fn default() -> Self {
        Self {
            inner: ::sys::default_preintegrated_combined_measurements(),
        }
    }
}

impl PreintegratedCombinedMeasurements {
    pub fn new(params: PreintegrationCombinedParams, bias: &ConstantBias) -> Self {
        Self {
            inner: ::sys::new_preintegrated_combined_measurements(params.inner, &bias.inner),
        }
    }

    pub fn integrate_measurement(
        &mut self,
        measured_acc: &Vector3,
        measured_omega: &Vector3,
        dt: f64,
    ) {
        ::sys::integrateMeasurement(
            self.inner.pin_mut(),
            &measured_acc.inner,
            &measured_omega.inner,
            dt
        )
    }

    pub fn predict(
        &self,
        navstate: &NavState,
        bias: &ConstantBias,
    ) -> NavState {
        NavState {
            inner: ::sys::predict(&self.inner, &navstate.inner, &bias.inner),
        }
    }

    pub fn reset_integration_and_set_bias(
        &mut self,
        bias: &ConstantBias,
    ) {
        ::sys::reset_integration_and_set_bias(self.inner.pin_mut(), &bias.inner)
    }
}

pub struct PreintegrationCombinedParams {
    pub(super) inner: SharedPtr<::sys::PreintegrationCombinedParams>,
}

impl PreintegrationCombinedParams {
    pub fn makesharedu() -> Self {
        Self {
            inner: ::sys::new_preintegrated_combined_params_makesharedu(),
        }
    }
    pub fn set_params(&mut self, sigma_a_sq: f64, sigma_g_sq: f64, sigma_wa_sq: f64, sigma_wg_sq: f64, int_covar_val: f64, bias_acc_omega_int_val2: f64) {
        ::sys::set_accelerometer_covariance(&mut self.inner, sigma_a_sq);
        ::sys::set_gyroscope_covariance(&mut self.inner, sigma_g_sq);
        ::sys::bias_acc_covariance(&mut self.inner, sigma_wa_sq);
        ::sys::bias_omega_covariance(&mut self.inner, sigma_wg_sq);
        ::sys::set_integration_covariance(&mut self.inner, int_covar_val);
        ::sys::bias_acc_omega_int(&mut self.inner, bias_acc_omega_int_val2);
    }
}