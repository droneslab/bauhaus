// use cxx::UniquePtr;

// use crate::{inference::key::IntoKey, nonlinear::{nonlinear_factor_graph::NonlinearFactorGraph, values::Values}};
// use crate::sys::DoubleVec;

// pub struct IncrementalFixedLagSmoother {
//     pub(super) inner: UniquePtr<::sys::IncrementalFixedLagSmoother>,
// }

// impl Default for IncrementalFixedLagSmoother {
//     fn default(smoother_lag: f64) -> Self {
//         Self {
//             inner: ::sys::default_incremental_fixed_lag_smoother(smoother_lag),
//         }
//     }
// }

// impl IncrementalFixedLagSmoother {
//     // pub fn update(
//     //     &mut self,
//     //     graph: &NonlinearFactorGraph,
//     //     values: &Values,
//     //     new_affected_keys: & Vec<DoubleVec>,
//     //     keys_to_remove: & Vec<u64>,
//     // ) -> ISAM2ResultRust {
//     //     ::sys::update(self.inner.pin_mut(), &graph.inner, &values.inner, & new_affected_keys, & keys_to_remove)
//     // }

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
// }
