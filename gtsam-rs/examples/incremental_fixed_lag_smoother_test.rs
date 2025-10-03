use std::f64;

use gtsam::{
    inference::symbol::Symbol,
    linear::noise_model::DiagonalNoiseModel,
    nonlinear::{
        levenberg_marquardt_optimizer::LevenbergMarquardtOptimizer,
        levenberg_marquardt_params::LevenbergMarquardtParams,
        nonlinear_factor_graph::NonlinearFactorGraph, values::Values,
    },
};
use nalgebra::{Isometry3, Vector3, Vector6};
use gtsam::nonlinear::incremental_fixed_lag_smoother::IncrementalFixedLagSmoother;
use gtsam_sys::get_foobar;

fn main() {
    let smoother = IncrementalFixedLagSmoother::new(1.0, 1.0, 1, true, true);

    get_foobar();

}
