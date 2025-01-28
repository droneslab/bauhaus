#include "nonlinear_factor_graph.h"
#include "../base/rust.hpp"

namespace gtsam {

std::unique_ptr<NonlinearFactorGraph> default_nonlinear_factor_graph() {
  return std::make_unique<NonlinearFactorGraph>();
}

void nonlinear_factor_graph_resize(NonlinearFactorGraph &graph, size_t size)
{
    return graph.resize(size);
}

void nonlinear_factor_graph_add_between_factor_pose3(
    NonlinearFactorGraph &graph, Key key1, Key key2, const Pose3 &measured,
    const std::shared_ptr<noiseModel::Base> &model) {
  return graph.add(
      BetweenFactorPose3(key1, key2, measured, to_boost_ptr(model)));
}

void nonlinear_factor_graph_add_prior_factor_pose3(
    NonlinearFactorGraph &graph, Key key, const Pose3 &prior,
    const std::shared_ptr<noiseModel::Base> &model) {
  return graph.addPrior(key, prior, to_boost_ptr(model));
}

void nonlinear_factor_graph_add_prior_factor_constant_bias(
    NonlinearFactorGraph &graph, Key key, const imuBias::ConstantBias &prior,
    const std::shared_ptr<noiseModel::Base> &model)
{
    return graph.addPrior(key, prior, to_boost_ptr(model));
}

void nonlinear_factor_graph_add_prior_factor_vector3(
    NonlinearFactorGraph &graph, Key key, const Vector3 &prior,
    const std::shared_ptr<noiseModel::Base> &model)
{
    return graph.addPrior(key, prior, to_boost_ptr(model));
}

void nonlinear_factor_graph_add_combined_imu_factor(
    NonlinearFactorGraph &graph, const std::shared_ptr<CombinedImuFactor> &factor) {
    return graph.add(to_boost_ptr(factor));
}

void nonlinear_factor_graph_add_smart_projection_pose_factor(
    NonlinearFactorGraph &graph,
    const gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> &factor)
{
    return graph.add(factor);
}

} // namespace gtsam
