use serde::{de::DeserializeOwned, Deserialize, Serialize};
use crate::map::keypoints::{
    KeyPointsMono, KeyPointsStereo, KeyPointsRgbd, KeyPointsData
};

#[derive(Clone, Copy, Debug)]
pub enum Sensor {
    Mono,
    ImuMono,
    Stereo,
    ImuStereo,
    Rgbd,
    ImuRgbd
}

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
// impl Serialize for MonoSensor {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
//         let mut state = serializer.serialize_struct("MonoSensor", 0)?;
//         state.end()
//     }
// }
// impl<'de> Deserialize<'de> for MonoSensor {
//     fn deserialize<D>(deserializer: D) -> Result<MonoSensor, D::Error> where D: Deserializer<'de> {
//         Ok(MonoSensor { })
//     }
// }

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuMonoSensor {}
impl SensorType for ImuMonoSensor {
    type KeyPointsData = KeyPointsMono;
    fn sensor_type() -> Sensor { Sensor::ImuMono } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { true }
}
// impl Serialize for ImuMonoSensor {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
//         let mut state = serializer.serialize_struct("MonoSensor", 0)?;
//         state.end()
//     }
// }
// impl<'de> Deserialize<'de> for ImuMonoSensor {
//     fn deserialize<D>(deserializer: D) -> Result<ImuMonoSensor, D::Error> where D: Deserializer<'de> {
//         Ok(ImuMonoSensor { })
//     }
// }

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct StereoSensor {}
impl SensorType for StereoSensor {
    type KeyPointsData = KeyPointsStereo;
    fn sensor_type() -> Sensor { Sensor::Stereo } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { false }
}
// impl Serialize for StereoSensor {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
//         let mut state = serializer.serialize_struct("MonoSensor", 0)?;
//         state.end()
//     }
// }
// impl<'de> Deserialize<'de> for StereoSensor {
//     fn deserialize<D>(deserializer: D) -> Result<StereoSensor, D::Error> where D: Deserializer<'de> {
//         Ok(StereoSensor { })
//     }
// }

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuStereoSensor {}
impl SensorType for ImuStereoSensor {
    type KeyPointsData = KeyPointsStereo;
    fn sensor_type() -> Sensor { Sensor::ImuStereo } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { false }
}
// impl Serialize for ImuStereoSensor {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
//         let mut state = serializer.serialize_struct("MonoSensor", 0)?;
//         state.end()
//     }
// }
// impl<'de> Deserialize<'de> for ImuStereoSensor {
//     fn deserialize<D>(deserializer: D) -> Result<ImuStereoSensor, D::Error> where D: Deserializer<'de> {
//         Ok(ImuStereoSensor { })
//     }
// }

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct RgbdSensor {}
impl SensorType for RgbdSensor {
    type KeyPointsData = KeyPointsRgbd;
    fn sensor_type() -> Sensor { Sensor::Rgbd } 
    fn is_imu() -> bool { false }
    fn is_mono() -> bool { false }
}
// impl Serialize for RgbdSensor {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
//         let mut state = serializer.serialize_struct("MonoSensor", 0)?;
//         state.end()
//     }
// }
// impl<'de> Deserialize<'de> for RgbdSensor {
//     fn deserialize<D>(deserializer: D) -> Result<RgbdSensor, D::Error> where D: Deserializer<'de> {
//         Ok(RgbdSensor { })
//     }
// }

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuRgbdSensor {}
impl SensorType for ImuRgbdSensor {
    type KeyPointsData = KeyPointsRgbd;
    fn sensor_type() -> Sensor { Sensor::ImuRgbd } 
    fn is_imu() -> bool { true }
    fn is_mono() -> bool { false }
}
// impl Serialize for ImuRgbdSensor {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
//         let mut state = serializer.serialize_struct("MonoSensor", 0)?;
//         state.end()
//     }
// }
// impl<'de> Deserialize<'de> for ImuRgbdSensor {
//     fn deserialize<D>(deserializer: D) -> Result<ImuRgbdSensor, D::Error> where D: Deserializer<'de> {
//         Ok(ImuRgbdSensor { })
//     }
// }


impl Sensor {
    pub fn is_imu(&self) -> bool {
        match *self {
            Sensor::ImuMono | Sensor::ImuStereo | Sensor::ImuRgbd => true,
            _ => false
        }
    }

    pub fn is_mono(&self) -> bool {
        match *self {
            Sensor::ImuMono | Sensor::Mono => true,
            _ => false
        }
    }
}
