use serde::{de::DeserializeOwned, Deserialize, Serialize};
use dvcore::global_params::Sensor;

use super::{
    keypoints::*
};

// All sensors implement this trait.
pub trait SensorType: Copy + Serialize + DeserializeOwned + Send + Sync + Default {
    type KeyPointsData: KeyPointsData;
    fn sensor_type() -> Sensor;
    fn is_imu() -> bool;
    fn is_mono() -> bool;
}
// Implement this for a sensor if it uses IMU info.
// This lets you use the code in ImuModule to handle IMU data.
pub trait IMUSensorType {}

#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct MonoSensor {}
impl SensorType for MonoSensor {
    type KeyPointsData = KeyPointsMono;
    fn sensor_type() -> Sensor { Sensor::Mono } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { true }
}

#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct ImuMonoSensor {}
impl SensorType for ImuMonoSensor {
    type KeyPointsData = KeyPointsMono;
    fn sensor_type() -> Sensor { Sensor::ImuMono } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { true }
}
impl IMUSensorType for ImuMonoSensor {}

#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct StereoSensor {}
impl SensorType for StereoSensor {
    type KeyPointsData = KeyPointsStereo;
    fn sensor_type() -> Sensor { Sensor::Stereo } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { false }
}

#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct ImuStereoSensor {}
impl SensorType for ImuStereoSensor {
    type KeyPointsData = KeyPointsStereo;
    fn sensor_type() -> Sensor { Sensor::ImuStereo } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { false }
}
impl IMUSensorType for ImuStereoSensor {}

#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct RgbdSensor {}
impl SensorType for RgbdSensor {
    type KeyPointsData = KeyPointsRgbd;
    fn sensor_type() -> Sensor { Sensor::Rgbd } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { false }
}

#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct ImuRgbdSensor {}
impl SensorType for ImuRgbdSensor {
    type KeyPointsData = KeyPointsRgbd;
    fn sensor_type() -> Sensor { Sensor::ImuRgbd } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { false }
}
impl IMUSensorType for ImuRgbdSensor {}
