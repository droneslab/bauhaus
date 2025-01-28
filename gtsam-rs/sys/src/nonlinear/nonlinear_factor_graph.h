#pragma once

#include "rust/cxx.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <memory>

namespace gtsam {

    typedef BetweenFactor<Pose3> BetweenFactorPose3;

    std::unique_ptr<NonlinearFactorGraph> default_nonlinear_factor_graph();

    void nonlinear_factor_graph_resize(NonlinearFactorGraph &graph, size_t size);

    void nonlinear_factor_graph_add_between_factor_pose3(
        NonlinearFactorGraph &graph, Key from, Key to, const Pose3 &prior,
        const std::shared_ptr<noiseModel::Base> &model);

    void nonlinear_factor_graph_add_prior_factor_pose3(
        NonlinearFactorGraph &graph, Key key, const Pose3 &prior,
        const std::shared_ptr<noiseModel::Base> &model);

    void nonlinear_factor_graph_add_prior_factor_constant_bias(
        NonlinearFactorGraph &graph, Key key, const imuBias::ConstantBias &prior,
        const std::shared_ptr<noiseModel::Base> &model);

    void nonlinear_factor_graph_add_prior_factor_vector3(
        NonlinearFactorGraph &graph, Key key, const Vector3 &prior,
        const std::shared_ptr<noiseModel::Base> &model);

    // void nonlinear_factor_graph_add_imu_factor(
    //     NonlinearFactorGraph &graph, Key pose_i, Key vel_i,
    //     Key pose_j, Key vel_j, Key bias,
    //     const std::shared_ptr<PreintegratedImuMeasurements> &preintegratedMeasurements);

    void nonlinear_factor_graph_add_combined_imu_factor(
        NonlinearFactorGraph &graph, const std::shared_ptr<CombinedImuFactor> &factor);

    void nonlinear_factor_graph_add_smart_projection_pose_factor(
        NonlinearFactorGraph &graph,
        const gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> &factor);
}
// namespace gtsam
