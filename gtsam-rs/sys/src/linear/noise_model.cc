#include "noise_model.h"
#include "../base/rust.hpp"

namespace gtsam {

std::shared_ptr<DiagonalNoiseModel> from_diagonal_noise_model_sigmas(const rust::Slice<double> sigmas) {
  Vector eigenSigmas = Eigen::Map<Vector>(sigmas.data(), sigmas.size(), 1);
  constexpr bool smart = true;

  return to_std_ptr(DiagonalNoiseModel::Sigmas(eigenSigmas, smart));
}

std::shared_ptr<BaseNoiseModel> cast_diagonal_noise_model_to_base_noise_model(
    const std::shared_ptr<DiagonalNoiseModel> &a) {
  return downcast<DiagonalNoiseModel, BaseNoiseModel>(a);
}

std::shared_ptr<IsotropicNoiseModel> from_isotropic_noise_model_dim_and_sigma(const size_t dim, const double sigma) {
    constexpr bool smart = true;

    return to_std_ptr(IsotropicNoiseModel::Sigma(dim, sigma, smart));
}

std::shared_ptr<BaseNoiseModel> cast_isotropic_noise_model_to_base_noise_model(
    const std::shared_ptr<IsotropicNoiseModel> &a) {
    return downcast<IsotropicNoiseModel, BaseNoiseModel>(a);
}

} // namespace gtsam
