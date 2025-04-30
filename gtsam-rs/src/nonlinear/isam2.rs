use cxx::UniquePtr;

use crate::{inference::key::IntoKey, nonlinear::{nonlinear_factor_graph::NonlinearFactorGraph, values::{Values, ValuesRef}}};
use crate::sys::DoubleVec;

pub struct ISAM2 {
    pub(super) inner: UniquePtr<::sys::ISAM2>,
}

impl Default for ISAM2 {
    fn default() -> Self {
        Self {
            inner: ::sys::default_isam2(),
        }
    }
}

impl ISAM2 {
    pub fn update_noresults(
        &mut self,
        graph: &NonlinearFactorGraph,
        values: &Values,
    ) {
        ::sys::update_noresults(self.inner.pin_mut(), &graph.inner, &values.inner);
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
