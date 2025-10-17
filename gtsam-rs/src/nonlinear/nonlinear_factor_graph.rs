use cxx::UniquePtr;

use crate::{
    base::vector::Vector3, geometry::pose3::Pose3, imu::imu_bias::ConstantBias, inference::key::IntoKey, linear::noise_model::{DiagonalNoiseModel, GaussianNoiseModel, IsotropicNoiseModel}, navigation::combined_imu_factor::CombinedImuFactor, slam::projection_factor::SmartProjectionPoseFactorCal3S2,
    slam::projection_factor::GenericProjectionFactorPose3Point3Cal3S2,
};

pub struct NonlinearFactorGraph {
    pub(super) inner: UniquePtr<::sys::NonlinearFactorGraph>,
}

impl Default for NonlinearFactorGraph {
    fn default() -> Self {
        Self {
            inner: ::sys::default_nonlinear_factor_graph(),
        }
    }
}

impl NonlinearFactorGraph {
    pub fn resize(&mut self, size: usize) {
        ::sys::nonlinear_factor_graph_resize(self.inner.pin_mut(), size)
    }

    pub fn add_between_factor_pose3(
        &mut self,
        from: impl IntoKey,
        to: impl IntoKey,
        measured: &Pose3,
        model: &DiagonalNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_between_factor_pose3(
            self.inner.pin_mut(),
            from.into_key(),
            to.into_key(),
            &measured.inner,
            &model.to_base_model().inner,
        )
    }

    pub fn add_prior_factor_pose3(
        &mut self,
        symbol: impl IntoKey,
        prior: &Pose3,
        model: &GaussianNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_pose3(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }

    pub fn add_prior_factor_constant_bias_isotropic(
        &mut self,
        symbol: impl IntoKey,
        prior: &ConstantBias,
        model: &IsotropicNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_constant_bias(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }

    pub fn add_prior_factor_constant_bias_diagonal(
        &mut self,
        symbol: impl IntoKey,
        prior: &ConstantBias,
        model: &DiagonalNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_constant_bias(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }

    // TODO these functions can probably be combined into one by taking either type of noise model,
    // but would probably have to make noisemodel an enum or something
    pub fn add_prior_factor_vector3_isotropicnoisemodel(
        &mut self,
        symbol: impl IntoKey,
        prior: &Vector3,
        model: &IsotropicNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_vector3(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }
    pub fn add_prior_factor_vector3_diagonalnoisemodel(
        &mut self,
        symbol: impl IntoKey,
        prior: &Vector3,
        model: &DiagonalNoiseModel,
    ) {
        ::sys::nonlinear_factor_graph_add_prior_factor_vector3(
            self.inner.pin_mut(),
            symbol.into_key(),
            &prior.inner,
            &model.to_base_model().inner,
        )
    }


    pub fn add_combined_imu_factor(
        &mut self,
        factor: &CombinedImuFactor,
    ) {
        ::sys::nonlinear_factor_graph_add_combined_imu_factor(
            self.inner.pin_mut(),
            &factor.inner,
        )
    }

    pub fn add_smartfactor(&mut self, factor: &SmartProjectionPoseFactorCal3S2) {
        ::sys::nonlinear_factor_graph_add_smart_projection_pose_factor(self.inner.pin_mut(), &factor.inner)
    }

    pub fn add_generic_factor(&mut self, factor: &GenericProjectionFactorPose3Point3Cal3S2) {
        ::sys::nonlinear_factor_graph_add_generic_projection_pose_factor(self.inner.pin_mut(), &factor.inner)
    }
}
