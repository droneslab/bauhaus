use dvcore::{lockwrap::ReadOnlyWrapper, config::{Sensor, FrameSensor, ImuSensor}};
use serde::{Deserialize, Serialize};

use crate::dvmap::{pose::Pose, map::Map};

#[derive(Debug, Clone, Default)]
pub struct ImuModule {
    pub velocity: Option<Pose>,
    _last_bias: Option<IMUBias>,
    sensor: Sensor
}

impl ImuModule {
    pub fn ready(&self, map: &ReadOnlyWrapper<Map>) -> bool {
        self.sensor.is_imu() && !self.velocity.is_none() && map.read().imu_initialized
    }

    pub fn predict_state(&self) -> bool {
        self.check_imu_sensor();
        todo!("IMU: PredictStateIMU");
    }

    pub fn preintegrate(&self) {
        // void Tracking::PreintegrateIMU()
        self.check_imu_sensor();
        todo!("IMU: PreintegrateIMU");
    }

    pub fn initialize(&self) {
        self.check_imu_sensor();
        //void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
        let (_prior_g, _prior_a, _fiba) = match self.sensor.frame() {
            FrameSensor::Mono => (1e2, 1e10, true),
            FrameSensor::Stereo | FrameSensor::Rgbd => (1e2, 1e5, true),
        };
        todo!("IMU: initialize");
    }

    fn check_imu_sensor(&self) {
        if matches!(self.sensor.imu(), ImuSensor::None) {
            panic!("{}", "Should not use IMU module if IMU sensor is not set".to_string());
        }
    }
}


#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct IMUPreIntegrated {
// TODO (IMU): fill this in, ImuTypes.cc in orbslam3
}

#[derive(Clone, Debug, Copy, Serialize, Deserialize)]
pub struct IMUBias {
    // TODO (IMU)
}
