use core::sensor::{FrameSensor, ImuSensor, Sensor};
use serde::{Deserialize, Serialize};

use crate::map::pose::Pose;

use super::module::ImuModule;

#[derive(Debug, Clone)]
pub struct DVImu {
    pub velocity: Option<Pose>,
    pub is_initialized: bool, // isImuInitialized(), set true by local mapper
    pub imu_ba2: bool, // mbIMU_BA2
    _last_bias: Option<IMUBias>,
    sensor: Sensor
}
impl ImuModule for DVImu {
    fn ready(&self) -> bool {
        self.sensor.is_imu() && !self.velocity.is_none() && self.is_initialized
    }

    fn predict_state(&self) -> bool {
        todo!("IMU: PredictStateIMU");
    }

    fn preintegrate(&self) {
        // void Tracking::PreintegrateIMU()
        todo!("IMU: PreintegrateIMU");
    }

    fn initialize(&self) {
        //void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
        let (_prior_g, _prior_a, _fiba) = match self.sensor.frame() {
            FrameSensor::Mono => (1e2, 1e10, true),
            FrameSensor::Stereo | FrameSensor::Rgbd => (1e2, 1e5, true),
        };
        todo!("IMU: initialize");
    }
}

impl DVImu {
    pub fn new(velocity: Option<Pose>, last_bias: Option<IMUBias>, sensor: Sensor, is_initialized: bool, imu_ba2: bool) -> Self {
        Self {
            velocity,
            _last_bias: last_bias,
            sensor,
            is_initialized,
            imu_ba2
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
