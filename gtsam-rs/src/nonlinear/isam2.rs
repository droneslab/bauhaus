use cxx::UniquePtr;
use sys::ISAM2ResultRust;

use crate::{inference::key::IntoKey, nonlinear::{nonlinear_factor_graph::NonlinearFactorGraph, values::Values}};
use crate::sys::DoubleVec;

pub struct ISAM2 {
    pub(super) inner: UniquePtr<::sys::ISAM2>,
}

impl Default for ISAM2 {
    fn default() -> Self {
        Self {
            inner: ::sys::default_isam2(
                0.1, 10, true, false
            ),
        }
    }
}

impl ISAM2 {
    pub fn new(
        relinearize_threshold: f64, relinearize_skip: i32, cache_linearized_factors: bool, enable_detailed_results: bool
    ) -> Self {
        Self {
            inner: ::sys::default_isam2(
                relinearize_threshold, relinearize_skip, cache_linearized_factors, enable_detailed_results
            ),
        }
    }

    pub fn update(
        &mut self,
        graph: &NonlinearFactorGraph,
        values: &Values,
        new_affected_keys: & Vec<DoubleVec>,
        keys_to_remove: & Vec<u64>,
    ) -> ISAM2ResultRust {
        ::sys::update(self.inner.pin_mut(), &graph.inner, &values.inner, & new_affected_keys, & keys_to_remove)
    }

    pub fn calculate_estimate(
        &self,
    ) -> Values {
        Values {
            inner: ::sys::calculate_estimate(& self.inner),
        }
    }

    pub fn get_marginal_covariance(
        &self,
        key: impl IntoKey,
    ) -> Vec<DoubleVec> {
        ::sys::get_marginal_covariance(& self.inner, key.into_key())
    }
}
