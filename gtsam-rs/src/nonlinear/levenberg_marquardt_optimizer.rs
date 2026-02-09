use cxx::UniquePtr;

use super::{
    levenberg_marquardt_params::LevenbergMarquardtParams,
    nonlinear_factor_graph::NonlinearFactorGraph,
    values::Values,
};

pub struct LevenbergMarquardtOptimizer {
    inner: UniquePtr<::sys::LevenbergMarquardtOptimizer>,
}

impl LevenbergMarquardtOptimizer {
    pub fn new(
        graph: &NonlinearFactorGraph,
        initial_values: &Values,
        params: &LevenbergMarquardtParams,
    ) -> Self {
        Self {
            inner: ::sys::new_levenberg_marquardt_optimizer(
                &graph.inner,
                &initial_values.inner,
                &params.inner,
            ),
        }
    }

    pub fn optimize_safely(&mut self) -> Values {
        Values {
            inner: ::sys::optimize_safely(self.inner.pin_mut()),
        }
    }
}
