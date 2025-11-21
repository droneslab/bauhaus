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
                0.1, 1, true, false, 0.001, true, 25
            ),
        }
    }
}


impl IncrementalFixedLagSmoother {
    pub fn new(
        relinearize_threshold: f64,
        relinearize_skip: i32,
        cache_linearized_factors: bool,
        enable_detailed_results: bool,
        wildfire_threshold: f32, find_unused_factor_slots: bool,
        nr_states: i32
    ) -> Self {
        Self {
            inner: ::sys::new_incremental_fixed_lag_smoother(
                relinearize_threshold,
                relinearize_skip,
                cache_linearized_factors,
                enable_detailed_results,
                wildfire_threshold,
                find_unused_factor_slots,
                nr_states
            ),
        }
    }

    pub fn update(
        &mut self,
        new_factors: &NonlinearFactorGraph,
        new_values: &Values,
        timestamps: & Vec<DoubleVec>,
        delete_slots: &Vec<u64>
        // new_affected_keys: & Vec<DoubleVec>,
        // keys_to_remove: & Vec<u64>,
    ) -> ISAM2ResultRust {
        ::sys::update_smoother(
            self.inner.pin_mut(),
            &new_factors.inner,
            &new_values.inner,
            &timestamps,
            &delete_slots
        )
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
