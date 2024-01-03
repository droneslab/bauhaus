use core::config::{SETTINGS, SYSTEM};
use core::matrix::DVVectorOfPoint2f;
use core::sensor::{Sensor, FrameSensor, ImuSensor};
use log::debug;
use core::matrix::DVVectorOfPoint3f;
use opencv::prelude::KeyPointTraitConst;
use crate::map::{frame::Frame, pose::Pose};
use crate::modules::camera::CAMERA_MODULE;

use super::orbmatcher;


#[derive(Debug, Clone, Default)]
pub struct Initialization {
    // Monocular
    pub mp_matches: Vec<i32>,// ini_matches .. mvIniMatches;
    pub prev_matched: DVVectorOfPoint2f,// std::vector<cv::Point2f> mvbPrevMatched;
    pub p3d: DVVectorOfPoint3f,// std::vector<cv::Point3f> mvIniP3D;
    pub ready_to_initialize: bool,
    pub initial_frame: Option<Frame>,
    pub last_frame: Option<Frame>,
    pub current_frame: Option<Frame>,
    sensor: Sensor,
}

impl Initialization {
    pub fn new() -> Self {
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");

        Self {
            mp_matches: Vec::new(),
            prev_matched: DVVectorOfPoint2f::empty(),
            p3d: DVVectorOfPoint3f::empty(),
            ready_to_initialize: false,
            initial_frame: None,
            last_frame: None,
            current_frame: None,
            sensor
        }
    }

    pub fn try_initialize(&mut self, current_frame: &Frame) -> Result<bool, Box<dyn std::error::Error>> {
        // Only set once at beginning
        if self.initial_frame.is_none() {
            self.initial_frame = Some(current_frame.clone());
        }
        // If we never did initialization (ie, only 1 frame passed): initial, current, and last frame will all be set to the 1st frame.
        // If we've done the first step of initialization, update last frame and current frame.
        self.last_frame = match &self.current_frame {
            Some(frame) => Some(frame.clone()),
            None => Some(current_frame.clone())
        };
        self.current_frame = Some(current_frame.clone());

        match self.sensor.frame() {
            FrameSensor::Mono => self.monocular_initialization(),
            _ => self.stereo_initialization()
        }
    }

    fn monocular_initialization(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        let current_frame = self.current_frame.as_ref().unwrap();
        let initial_frame = self.initial_frame.as_ref().unwrap();
        let last_frame = self.last_frame.as_ref().unwrap();

        if !self.ready_to_initialize && current_frame.features.num_keypoints > 100 {
            // Set Reference Frame
            for i in 0..current_frame.features.num_keypoints as usize {
                self.prev_matched.push(current_frame.features.get_keypoint(i).0.pt().clone());
            }

            match self.sensor.imu() {
                ImuSensor::Some => {
                    todo!("IMU");
                    //Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L2467

                    // if(mpImuPreintegratedFromLastKF)
                    // {
                    //     delete mpImuPreintegratedFromLastKF;
                    // }
                    // mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                    // self.current_frame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
                },
                _ => {}
            }
            self.ready_to_initialize = true;
            return Ok(false);
        } else {
            if current_frame.features.num_keypoints <=100 || matches!(self.sensor.imu(), ImuSensor::Some) && last_frame.timestamp - initial_frame.timestamp > 1.0 {
                self.ready_to_initialize = false;
                return Ok(false);
            }

            // Find correspondences
            let (mut num_matches, mp_matches) = orbmatcher::search_for_initialization(
                &initial_frame, 
                &current_frame,
                &mut self.prev_matched,
                100
            );
            self.mp_matches = mp_matches;

            // Check if there are enough correspondences
            if num_matches < 100 {
                self.ready_to_initialize = false;
                return Ok(false);
            };

            if let Some((tcw, v_p3d, vb_triangulated)) = CAMERA_MODULE.reconstruct_with_two_views(
                initial_frame.features.get_all_keypoints(),
                current_frame.features.get_all_keypoints(),
                & self.mp_matches,
            ) {
                self.p3d = v_p3d;

                for index in 0..self.mp_matches.len() {
                    if self.mp_matches[index] >= 0 && !vb_triangulated[index as usize] {
                        self.mp_matches[index] = -1;
                        num_matches -= 1;
                    }
                }

                self.initial_frame.as_mut().unwrap().pose = Some(Pose::default());
                self.current_frame.as_mut().unwrap().pose = Some(tcw);

                return Ok(true);
            } else {
                debug!("MonocularInitialization, ReconstructWithTwoViews... failure");
                return Ok(false);
            }
        }
    }

    fn stereo_initialization(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        todo!("Stereo: StereoInitialization");
    }
}
