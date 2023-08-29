use std::fmt;


#[derive(Clone, Copy, Debug, Default)]
pub enum FrameSensor {
    #[default] Mono,
    Stereo,
    Rgbd,
}

#[derive(Clone, Copy, Debug, Default)]
pub enum ImuSensor {
    #[default] None,
    Some
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Sensor (pub FrameSensor, pub ImuSensor);

impl Sensor {
    pub fn is_mono(&self) -> bool {
        matches!(self.0, FrameSensor::Mono)
    }
    pub fn is_imu(&self) -> bool {
        matches!(self.1, ImuSensor::Some)
    }
    pub fn frame(&self) -> FrameSensor {
        self.0
    }
    pub fn imu(&self) -> ImuSensor {
        self.1
    }
}

impl fmt::Display for Sensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let framesensor_str = match self.0 {
            FrameSensor::Mono => "Mono",
            FrameSensor::Stereo => "Stereo",
            FrameSensor::Rgbd => "Rgbd",
        };
        let imusensor_str = match self.1 {
            ImuSensor::None => "No",
            ImuSensor::Some => "Yes",
        };

        write!(f, "(Frame: {}, Imu: {})", framesensor_str, imusensor_str)
    }
}
