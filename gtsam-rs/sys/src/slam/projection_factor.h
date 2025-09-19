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
    typedef gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2> GenericProjectionFactorPose3Point3Cal3_S2;

    std::shared_ptr<SmartProjectionPoseFactorCal3_S2> new_smart_projection_pose_factor(
        const std::shared_ptr<noiseModel::Isotropic> &measurement_noise,
        const std::shared_ptr<Cal3_S2> &K,
        const Pose3 &sensor_P_body);

    void add(std::shared_ptr<SmartProjectionPoseFactorCal3_S2> &factor, const Point2 &point, Key key);

    std::shared_ptr<GenericProjectionFactorPose3Point3Cal3_S2> new_generic_projection_factor(
        const Point2 & measured,
        const std::shared_ptr<noiseModel::Isotropic> &measurement_noise,
        Key pose_key,
        Key point_key,
        const std::shared_ptr<Cal3_S2> &K,
        const Pose3 &sensor_P_body);

} // namespace gtsam
