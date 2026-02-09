#include "projection_factor.h"
#include "../base/rust.hpp"

namespace gtsam
{
    std::shared_ptr<SmartProjectionPoseFactorCal3_S2> new_smart_projection_pose_factor(
        const std::shared_ptr<noiseModel::Isotropic> & measurement_noise,
        const std::shared_ptr<Cal3_S2> & K,
        const Pose3 & sensor_P_body)
    {
        SmartProjectionParams params;
        params.setRankTolerance(1);
        params.setLandmarkDistanceThreshold(10);
        params.setRetriangulationThreshold(0.001);
        params.setDynamicOutlierRejectionThreshold(3);
        //! EPI: If set to true, will refine triangulation using LM.
        params.setEnableEPI(false);
        params.setLinearizationMode(gtsam::HESSIAN);
        params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
        params.throwCheirality = false;
        params.verboseCheirality = false;

        auto factor = SmartProjectionPoseFactor<gtsam::Cal3_S2>(
            to_boost_ptr(measurement_noise),
            to_boost_ptr(K),
            sensor_P_body,
            params
        );

        return std::shared_ptr<SmartProjectionPoseFactor<gtsam::Cal3_S2>>(new SmartProjectionPoseFactor<gtsam::Cal3_S2>(factor));
    }

    void add_smart(
        std::shared_ptr<SmartProjectionPoseFactorCal3_S2> &factor,
        const Point2 & point,
        Key key
    ) {
        factor->add(point, key);
    }

    std::shared_ptr<SmartProjectionPoseFactorCal3_S2> clone_smart_projection_pose_factor(
        const std::shared_ptr<SmartProjectionPoseFactorCal3_S2> & old_factor
    ) {
        return std::shared_ptr<SmartProjectionPoseFactorCal3_S2>(new SmartProjectionPoseFactorCal3_S2(*old_factor));
    }

    std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> new_smart_stereo_projection_pose_factor(
        const std::shared_ptr<noiseModel::Base> &measurement_noise,
        const Pose3 & sensor_P_body)
    {
        SmartStereoProjectionParams params;
        params.setRankTolerance(1);
        params.setLandmarkDistanceThreshold(10);
        params.setRetriangulationThreshold(0.001);
        params.setDynamicOutlierRejectionThreshold(3);
        //! EPI: If set to true, will refine triangulation using LM.
        params.setEnableEPI(false);
        params.setLinearizationMode(gtsam::HESSIAN);
        params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
        params.throwCheirality = false;
        params.verboseCheirality = false;

        // smart_noise_, smart_factors_params_, B_Pose_leftCamRect_

        return std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor>(
            new gtsam::SmartStereoProjectionPoseFactor(
                to_boost_ptr(measurement_noise),
                params,
                sensor_P_body)
        );
    }

    std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> clone_smart_stereo_projection_pose_factor(
        const std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> & old_factor
    ) {
        return std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor>(new gtsam::SmartStereoProjectionPoseFactor(*old_factor));
    }

    void add_smartstereo(
        std::shared_ptr<gtsam::SmartStereoProjectionPoseFactor> &factor,
        const StereoPoint2 &point,
        Key key,
        const std::shared_ptr<Cal3_S2Stereo> &K
    ) {
        factor->add(point, key, to_boost_ptr(K));
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

        return std::shared_ptr<gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(new gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>(factor));
    }

} // namespace gtsam
