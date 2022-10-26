use std::marker::PhantomData;

use dvcore::{lockwrap::ReadOnlyWrapper, global_params::Sensor};
use serde::{Deserialize, Serialize};

use crate::dvmap::{pose::Pose, sensor::{SensorType, IMUSensorType}, map::Map};

#[derive(Debug, Clone, Default)]
pub struct ImuModule<S: SensorType> {
    pub velocity: Option<Pose>,
    last_bias: Option<IMUBias>,
    _sensor: PhantomData<S>
}

impl<S: SensorType> ImuModule<S> {
    pub fn ready(&self, map: &ReadOnlyWrapper<Map<S>>) -> bool {
        S::is_imu() && !self.velocity.is_none() && map.read().imu_initialized
    }

    pub fn predict_state(&self) -> bool {
        todo!("IMU: PredictStateIMU");
    }

    pub fn preintegrate(&self) {
        todo!("IMU: PreintegrateIMU");
    }

    pub fn initialize(&self) {
        match S::sensor_type() {
            Sensor::ImuMono => self.hidden_initialize(1e2, 1e10, true),
            Sensor::ImuStereo | Sensor::ImuRgbd => self.hidden_initialize(1e2, 1e5, true),
            _ => {}
        }
    }

    fn hidden_initialize(&self, prior_g: f64, prior_a: f64, fiba: bool) {
        //void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
        todo!("IMU: initialize");
    }

    pub fn reset_frame_imu(&self) -> bool {
        todo!("IMU: ResetFrameIMU");
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
