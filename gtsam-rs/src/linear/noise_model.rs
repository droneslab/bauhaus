use cxx::SharedPtr;
use nalgebra::SVector;

pub struct BaseNoiseModel {
    pub(crate) inner: SharedPtr<::sys::BaseNoiseModel>,
}

pub struct DiagonalNoiseModel {
    pub(crate) inner: SharedPtr<::sys::DiagonalNoiseModel>,
}

impl DiagonalNoiseModel {
    pub fn from_sigmas<const D: usize>(mut sigmas: SVector<f64, D>) -> Self {
        Self {
            inner: ::sys::from_diagonal_noise_model_sigmas(sigmas.data.as_mut_slice()),
        }
    }

    pub(crate) fn to_base_model(&self) -> BaseNoiseModel {
        BaseNoiseModel {
            inner: ::sys::cast_diagonal_noise_model_to_base_noise_model(&self.inner),
        }
    }
}

pub struct IsotropicNoiseModel {
    pub(crate) inner: SharedPtr<::sys::IsotropicNoiseModel>,
}

impl IsotropicNoiseModel {
    pub fn from_dim_and_sigma(dim: usize, sigma: f64) -> Self {
        Self {
            inner: ::sys::from_isotropic_noise_model_dim_and_sigma(dim, sigma),
        }
    }

    pub(crate) fn to_base_model(&self) -> BaseNoiseModel {
        BaseNoiseModel {
            inner: ::sys::cast_isotropic_noise_model_to_base_noise_model(&self.inner),
        }
    }
}
