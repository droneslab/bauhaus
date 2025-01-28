#include "rust/cxx.h"
#include <memory>
#include <gtsam/navigation/CombinedImuFactor.h>

namespace gtsam
{
    std::shared_ptr<CombinedImuFactor> default_combined_imu_factor();
    std::shared_ptr<CombinedImuFactor> new_combined_imu_factor(
        Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias_i, Key bias_j,
        const PreintegratedCombinedMeasurements & preintegratedMeasurements);


    std::shared_ptr<PreintegrationCombinedParams> new_preintegrated_combined_params_makesharedu();
    void set_accelerometer_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_a_sq);
    void set_gyroscope_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_g_sq);
    void bias_acc_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_wa_sq);
    void bias_omega_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double sigma_wg_sq);
    void set_integration_covariance(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double val);
    void bias_acc_omega_int(
        std::shared_ptr<PreintegrationCombinedParams> &params,
        double val);

    std::unique_ptr<PreintegratedCombinedMeasurements> default_preintegrated_combined_measurements();
    std::unique_ptr<PreintegratedCombinedMeasurements> new_preintegrated_combined_measurements(
        const std::shared_ptr<PreintegrationCombinedParams> params,
        const imuBias::ConstantBias &bias);

    void integrateMeasurement(
        PreintegratedCombinedMeasurements &preintegrated_measurements,
        const Vector3 &measuredAcc, const Vector3 &measuredOmega, const double dt);

    std::unique_ptr<NavState> predict(
        const PreintegratedCombinedMeasurements &preintegrated_measurements,
        const NavState &state_i, const imuBias::ConstantBias &bias_i);

    void reset_integration_and_set_bias(
        PreintegratedCombinedMeasurements &preintegrated_measurements,
        const imuBias::ConstantBias &bias);
}