use serde::{de::DeserializeOwned, Deserialize, Serialize};
use dvcore::global_params::Sensor;
use crate::dvmap::keypoints::*;

pub trait SensorType: Copy + Serialize + DeserializeOwned + Send + Sync {
    type KeyPointsData: KeyPointsData;
    fn sensor_type() -> Sensor;
    fn is_imu() -> bool;
    fn is_mono() -> bool;
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct MonoSensor {}
impl SensorType for MonoSensor {
    type KeyPointsData = KeyPointsMono;
    fn sensor_type() -> Sensor { Sensor::Mono } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { true }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuMonoSensor {}
impl SensorType for ImuMonoSensor {
    type KeyPointsData = KeyPointsMono;
    fn sensor_type() -> Sensor { Sensor::ImuMono } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { true }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct StereoSensor {}
impl SensorType for StereoSensor {
    type KeyPointsData = KeyPointsStereo;
    fn sensor_type() -> Sensor { Sensor::Stereo } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { false }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuStereoSensor {}
impl SensorType for ImuStereoSensor {
    type KeyPointsData = KeyPointsStereo;
    fn sensor_type() -> Sensor { Sensor::ImuStereo } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { false }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct RgbdSensor {}
impl SensorType for RgbdSensor {
    type KeyPointsData = KeyPointsRgbd;
    fn sensor_type() -> Sensor { Sensor::Rgbd } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { false }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuRgbdSensor {}
impl SensorType for ImuRgbdSensor {
    type KeyPointsData = KeyPointsRgbd;
    fn sensor_type() -> Sensor { Sensor::ImuRgbd } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { false }
}
