#include "projection_factor.h"
#include "../base/rust.hpp"

namespace gtsam
{
    std::unique_ptr<SmartProjectionPoseFactor<gtsam::Cal3_S2>> new_smart_projection_pose_factor(
        const std::shared_ptr<noiseModel::Isotropic> & measurement_noise,
        const std::shared_ptr<Cal3_S2> & K,
        const Pose3 & sensor_P_body)
    {
        auto factor = SmartProjectionPoseFactor<gtsam::Cal3_S2>(
            to_boost_ptr(measurement_noise),
            to_boost_ptr(K),
            sensor_P_body);

        // std::cout << "FACTOR: ";
        // factor.print();
        // std::cout << std::endl;

        return std::unique_ptr<SmartProjectionPoseFactor<gtsam::Cal3_S2>>(new SmartProjectionPoseFactor<gtsam::Cal3_S2>(factor));
    }

    void add(SmartProjectionPoseFactorCal3_S2 &factor, const Point2 & point, Key key) {
        factor.add(point, key);

        // std::cout << "FACTOR AFTER ADD: ";
        // factor.print();
        // std::cout << std::endl;
    }
} // namespace gtsam
