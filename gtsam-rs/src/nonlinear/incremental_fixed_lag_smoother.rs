use cxx::UniquePtr;

use crate::{inference::key::IntoKey, nonlinear::{nonlinear_factor_graph::NonlinearFactorGraph, values::Values}};
use crate::sys::DoubleVec;
use crate::sys::ISAM2ResultRust;

pub struct IncrementalFixedLagSmoother {
    pub(super) inner: UniquePtr<::sys::IncrementalFixedLagSmoother>,
}

impl Default for IncrementalFixedLagSmoother {
    fn default() -> Self {
        Self {
            inner: ::sys::new_incremental_fixed_lag_smoother(
                0.0, 0.1, 10, true, false
            ),
        }
    }
}


impl IncrementalFixedLagSmoother {
    pub fn new(
        smoother_lag: f64,
        relinearize_threshold: f64,
        relinearize_skip: i32,
        cache_linearized_factors: bool,
        enable_detailed_results: bool,
    ) -> Self {
        Self {
            inner: ::sys::new_incremental_fixed_lag_smoother(
                smoother_lag,
                relinearize_threshold,
                relinearize_skip,
                cache_linearized_factors,
                enable_detailed_results,
            ),
        }
    }

    pub fn update(
        &mut self,
        graph: &NonlinearFactorGraph,
        values: &Values,
        // new_affected_keys: & Vec<DoubleVec>,
        // keys_to_remove: & Vec<u64>,
    ) -> ISAM2ResultRust {
        ::sys::update_smoother(self.inner.pin_mut(), &graph.inner, &values.inner)
    }

//     // pub fn calculate_estimate(
//     //     &self,
//     // ) -> Values {
//     //     Values {
//     //         inner: ::sys::calculate_estimate(& self.inner),
//     //     }
//     // }

//     // pub fn get_marginal_covariance(
//     //     &self,
//     //     key: impl IntoKey,
//     // ) -> Vec<DoubleVec> {
//     //     ::sys::get_marginal_covariance(& self.inner, key.into_key())
//     // }
}
