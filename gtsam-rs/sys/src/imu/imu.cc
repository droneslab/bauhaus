#include "imu.h"

namespace gtsam
{
namespace imuBias {
    std::unique_ptr<ConstantBias> default_constantbias() { return std::make_unique<ConstantBias>(); }

    std::unique_ptr<ConstantBias> new_constantbias(const Vector3 &biasAcc, const Vector3 &biasGyro)
    {
        return std::make_unique<ConstantBias>(biasAcc, biasGyro);
    }

    const Vector3 &accel_bias(const ConstantBias &bias) { return bias.accelerometer(); }
    const Vector3 &gyro_bias(const ConstantBias &bias) { return bias.gyroscope(); }
}

std::unique_ptr<ImuFactor> default_imufactor() { return std::make_unique<ImuFactor>(); }

std::unique_ptr<ImuFactor> new_imufactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias,
                                         const PreintegratedImuMeasurements &preintegratedMeasurements)
{
    return std::make_unique<ImuFactor>(pose_i, vel_i, pose_j, vel_j, bias, preintegratedMeasurements);
}

std::unique_ptr<PreintegratedImuMeasurements> default_preintegratedimumeasurements() { return std::make_unique<PreintegratedImuMeasurements>(); }

std::unique_ptr<PreintegratedImuMeasurements> new_preintegratedimumeasurements(const boost::shared_ptr<PreintegrationParams> &p)
{
    return std::make_unique<PreintegratedImuMeasurements>(p);
}

} // namespace gtsam
