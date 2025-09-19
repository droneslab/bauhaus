#include "projection_factor.h"
#include "../base/rust.hpp"

namespace gtsam
{
    std::shared_ptr<SmartProjectionPoseFactor<gtsam::Cal3_S2>> new_smart_projection_pose_factor(
        const std::shared_ptr<noiseModel::Isotropic> & measurement_noise,
        const std::shared_ptr<Cal3_S2> & K,
        const Pose3 & sensor_P_body)
    {
        SmartProjectionParams params;
        params.linearizationMode = LinearizationMode::JACOBIAN_SVD;
        auto factor = SmartProjectionPoseFactor<gtsam::Cal3_S2>(
            to_boost_ptr(measurement_noise),
            to_boost_ptr(K),
            sensor_P_body,
            params
        );

        return std::unique_ptr<SmartProjectionPoseFactor<gtsam::Cal3_S2>>(new SmartProjectionPoseFactor<gtsam::Cal3_S2>(factor));
    }


    std::shared_ptr<gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>> new_generic_projection_factor(
        const Point2 & measured,
        const std::shared_ptr<noiseModel::Isotropic> & measurement_noise,
        Key pose_key,
        Key point_key,
        const std::shared_ptr<Cal3_S2> & K,
        const Pose3 & sensor_P_body)
    {
        auto factor = gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
            measured,
            to_boost_ptr(measurement_noise),
            pose_key,
            point_key,
            to_boost_ptr(K),
            sensor_P_body);

        return std::unique_ptr<gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(new gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>(factor));
    }

    void add(std::shared_ptr<SmartProjectionPoseFactorCal3_S2> &factor, const Point2 & point, Key key) {
        factor->add(point, key);
    }
} // namespace gtsam
