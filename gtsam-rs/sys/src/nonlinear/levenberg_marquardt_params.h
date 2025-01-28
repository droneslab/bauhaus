#pragma once

#include "rust/cxx.h"
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <memory>

namespace gtsam {

std::unique_ptr<LevenbergMarquardtParams> default_levenberg_marquardt_params();

void levenberg_marquardt_params_set_max_iterations(
    LevenbergMarquardtParams &params, uint32_t value);

void levenberg_marquardt_params_set_lambda_upper_bound(
    LevenbergMarquardtParams &params, double value);

void levenberg_marquardt_params_set_lambda_lower_bound(
    LevenbergMarquardtParams &params, double value);

void levenberg_marquardt_params_set_diagonal_damping(
    LevenbergMarquardtParams &params, bool flag);

void levenberg_marquardt_params_set_relative_error_to_l(
    LevenbergMarquardtParams &params, double value);

void levenberg_marquardt_params_set_absolute_error_to_l(
    LevenbergMarquardtParams &params, double value);
} // namespace gtsam
