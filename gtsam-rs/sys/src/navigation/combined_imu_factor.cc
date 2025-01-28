#include "combined_imu_factor.h"
#include "../base/rust.hpp"

namespace gtsam
{

    std::shared_ptr<CombinedImuFactor> default_combined_imu_factor()
    {
        return std::make_shared<CombinedImuFactor>();
    }

    std::shared_ptr<CombinedImuFactor> new_combined_imu_factor(
        Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i, Key bias_j,
        const PreintegratedCombinedMeasurements & preintegratedMeasurements)
    {
        return std::make_shared<CombinedImuFactor>(
            pose_i, vel_i, pose_j, vel_j, bias_i, bias_j, preintegratedMeasurements);
    }

    std::shared_ptr<PreintegrationCombinedParams> new_preintegrated_combined_params_makesharedu()
    {
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> params;
        params = PreintegrationCombinedParams::MakeSharedU();

        return to_std_ptr(params);
    }

    void set_accelerometer_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_a_sq)
    {
        params->setAccelerometerCovariance(gtsam::I_3x3 * sigma_a_sq);
    }

    void set_gyroscope_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_g_sq)
    {
        params->setGyroscopeCovariance(gtsam::I_3x3 * sigma_g_sq);
    }

    void bias_acc_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_wa_sq)
    {
        params->biasAccCovariance = sigma_wa_sq * gtsam::Matrix33::Identity(3, 3);
    }

    void bias_omega_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_wg_sq)
    {
        params->biasOmegaCovariance = sigma_wg_sq * gtsam::Matrix33::Identity(3, 3);
    }

    void set_integration_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double val)
    {
        params->setIntegrationCovariance(gtsam::I_3x3 * val);
    }

    void bias_acc_omega_int(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double val)
    {
        params->biasAccOmegaInt = val * gtsam::Matrix66::Identity(6, 6);
    }

    std::unique_ptr<PreintegratedCombinedMeasurements> default_preintegrated_combined_measurements()
    {
        return std::make_unique<PreintegratedCombinedMeasurements>();
    }

    std::unique_ptr<PreintegratedCombinedMeasurements> new_preintegrated_combined_measurements(
        const std::shared_ptr<PreintegrationCombinedParams> params,
        const imuBias::ConstantBias &bias)
    {
        // PreintegratedCombinedMeasurements * msmts = new PreintegratedCombinedMeasurements(to_boost_ptr(params));
        return std::make_unique<PreintegratedCombinedMeasurements>(to_boost_ptr(params), bias);
    }

    void integrateMeasurement(
        PreintegratedCombinedMeasurements &preintegrated_measurements,
        const Vector3 &measuredAcc,
        const Vector3 &measuredOmega,
        const double dt)
    {
        preintegrated_measurements.integrateMeasurement(measuredAcc, measuredOmega, dt);
    }

    std::unique_ptr<NavState> predict(
        const PreintegratedCombinedMeasurements &preintegrated_measurements,
        const NavState &state_i, const imuBias::ConstantBias & bias_i)
    {
        NavState state = preintegrated_measurements.predict(state_i, bias_i);
        return std::make_unique<NavState>(state);
    }

    void reset_integration_and_set_bias(
        PreintegratedCombinedMeasurements &preintegrated_measurements,
        const imuBias::ConstantBias &bias)
    {
            preintegrated_measurements.resetIntegrationAndSetBias(bias);
    }
}