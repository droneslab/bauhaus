use cxx::UniquePtr;

use crate::nonlinear::{values::{Values, ValuesRef}, nonlinear_factor_graph::NonlinearFactorGraph};

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
}
