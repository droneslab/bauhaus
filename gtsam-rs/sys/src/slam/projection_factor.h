#pragma once

#include "rust/cxx.h"
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>

namespace gtsam
{
    typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartProjectionPoseFactorCal3_S2;

    std::unique_ptr<SmartProjectionPoseFactorCal3_S2> new_smart_projection_pose_factor(
        const std::shared_ptr<noiseModel::Isotropic> &measurement_noise,
        const std::shared_ptr<Cal3_S2> &K,
        const Pose3 &sensor_P_body);

    void add(SmartProjectionPoseFactorCal3_S2 &factor, const Point2 & point, Key key);

} // namespace gtsam
