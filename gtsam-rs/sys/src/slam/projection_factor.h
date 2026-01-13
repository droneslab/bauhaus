#pragma once

#include "rust/cxx.h"
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/StereoPoint2.h>

#include <memory>

namespace gtsam
{
    typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartProjectionPoseFactorCal3_S2;
    typedef gtsam::SmartProjectionFactor<gtsam::Cal3_S2> SmartProjectionFactorCal3_S2;
    typedef gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2> GenericProjectionFactorPose3Point3Cal3_S2;


    std::shared_ptr<SmartProjectionPoseFactorCal3_S2> new_smart_projection_pose_factor(
        const std::shared_ptr<noiseModel::Isotropic> &measurement_noise,
        const std::shared_ptr<Cal3_S2> &K,
        const Pose3 &sensor_P_body);
    std::shared_ptr<SmartProjectionPoseFactorCal3_S2> clone_smart_projection_pose_factor(
        const std::shared_ptr<SmartProjectionPoseFactorCal3_S2> & old_factor);
    void add_smart(
        std::shared_ptr<SmartProjectionPoseFactorCal3_S2> &factor,
        const Point2 &point,
        Key key
    );


    std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> new_smart_stereo_projection_pose_factor(
        const std::shared_ptr<noiseModel::Base> &measurement_noise,
        const Pose3 &sensor_P_body);
    std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> clone_smart_stereo_projection_pose_factor(
        const std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> & old_factor);
    void add_smartstereo(
        std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> &factor,
        const StereoPoint2 &point,
        Key key,
        const std::shared_ptr<Cal3_S2Stereo> &K
    );


    std::shared_ptr<GenericProjectionFactorPose3Point3Cal3_S2> new_generic_projection_factor(
        const Point2 & measured,
        const std::shared_ptr<noiseModel::Isotropic> &measurement_noise,
        Key pose_key,
        Key point_key,
        const std::shared_ptr<Cal3_S2> &K,
        const Pose3 &sensor_P_body);

} // namespace gtsam
