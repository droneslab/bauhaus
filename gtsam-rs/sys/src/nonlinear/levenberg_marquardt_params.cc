#include "levenberg_marquardt_params.h"

namespace gtsam {

std::unique_ptr<LevenbergMarquardtParams> default_levenberg_marquardt_params() {
  return std::make_unique<LevenbergMarquardtParams>();
}

void levenberg_marquardt_params_set_max_iterations(
    LevenbergMarquardtParams &params, uint32_t value) {
  return params.setMaxIterations(value);
}

void levenberg_marquardt_params_set_lambda_upper_bound(
    LevenbergMarquardtParams &params, double value)
{
    return params.setlambdaUpperBound(value);
}

void levenberg_marquardt_params_set_lambda_lower_bound(
    LevenbergMarquardtParams &params, double value)
{
    return params.setlambdaLowerBound(value);
}

void levenberg_marquardt_params_set_diagonal_damping(
    LevenbergMarquardtParams &params, bool flag)
{
    return params.setDiagonalDamping(flag);
}

void levenberg_marquardt_params_set_relative_error_to_l(
    LevenbergMarquardtParams &params, double value)
{
    return params.setRelativeErrorTol(value);
}

void levenberg_marquardt_params_set_absolute_error_to_l(
    LevenbergMarquardtParams &params, double value)
{
    return params.setAbsoluteErrorTol(value);
}

} // namespace gtsam
