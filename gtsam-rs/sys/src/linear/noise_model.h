#pragma once

#include "rust/cxx.h"
#include <gtsam/linear/NoiseModel.h>
#include <memory>

namespace gtsam {

typedef noiseModel::Base BaseNoiseModel;
typedef noiseModel::Diagonal DiagonalNoiseModel;
typedef noiseModel::Isotropic IsotropicNoiseModel;

std::shared_ptr<DiagonalNoiseModel> from_diagonal_noise_model_sigmas(const rust::Slice<double> sigmas);
std::shared_ptr<BaseNoiseModel> cast_diagonal_noise_model_to_base_noise_model(
    const std::shared_ptr<DiagonalNoiseModel> &a);

std::shared_ptr<IsotropicNoiseModel> from_isotropic_noise_model_dim_and_sigma(const size_t dim, const double sigma);
std::shared_ptr<BaseNoiseModel> cast_isotropic_noise_model_to_base_noise_model(
    const std::shared_ptr<IsotropicNoiseModel> &a);

} // namespace gtsam
