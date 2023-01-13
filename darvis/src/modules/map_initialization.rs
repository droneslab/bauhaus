use chrono::Duration;
use dvcore::config::{GLOBAL_PARAMS, SYSTEM_SETTINGS, FrameSensor, ImuSensor};
use dvcore::matrix::DVVectorOfPoint2f;
use log::debug;
use dvcore::{matrix::DVVectorOfPoint3f, config::Sensor};
use crate::dvmap::keyframe::Frame;
use crate::dvmap::{keyframe::InitialFrame, pose::Pose};
use crate::modules::camera::{CAMERA_MODULE};

use super::orbmatcher;


#[derive(Debug, Clone, Default)]
pub struct Initialization {
    // Initialization (Monocular)
    pub mp_matches: Vec<i32>,// ini_matches .. mvIniMatches;
    pub prev_matched: DVVectorOfPoint2f,// std::vector<cv::Point2f> mvbPrevMatched;
    pub p3d: DVVectorOfPoint3f,// std::vector<cv::Point3f> mvIniP3D;
    pub ready_to_initializate: bool,
    pub initial_frame: Option<Frame<InitialFrame>>,
    pub last_frame: Option<Frame<InitialFrame>>,
    pub current_frame: Option<Frame<InitialFrame>>,
    sensor: Sensor,
}

impl Initialization {
    pub fn new() -> Self {
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

        Self {
            mp_matches: Vec::new(),
            prev_matched: DVVectorOfPoint2f::empty(),
            p3d: DVVectorOfPoint3f::empty(),
            ready_to_initializate: false,
            initial_frame: None,
            last_frame: None,
            current_frame: None,
            sensor
        }
    }

    pub fn try_initialize(&mut self, current_frame: &Frame<InitialFrame>) -> Result<bool, Box<dyn std::error::Error>> {
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

        if !self.ready_to_initializate && current_frame.features.num_keypoints > 100 {
            // Set Reference Frame
            for i in 0..current_frame.features.num_keypoints as usize {
                self.prev_matched.push(current_frame.features.get_keypoint(i).0.pt.clone()); // TODO (clone)
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
            self.ready_to_initializate = true;
            return Ok(false);
        } else {
            debug!("Current frame ID {}", current_frame.frame_id);

            if current_frame.features.num_keypoints <=100 || matches!(self.sensor.imu(), ImuSensor::Some) && last_frame.timestamp - initial_frame.timestamp > Duration::seconds(1) {
                self.ready_to_initializate = false;
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
            debug!("MonocularInitialization, search for initialization.. {} matches", num_matches);

            // Check if there are enough correspondences
            if num_matches < 100 {
                self.ready_to_initializate = false;
                return Ok(false);
            };

            let (reconstruct_success, tcw, v_p3d, vb_triangulated) = CAMERA_MODULE.reconstruct_with_two_views(
                initial_frame.features.get_all_keypoints(),
                current_frame.features.get_all_keypoints(),
                & self.mp_matches,
            );
            self.p3d = v_p3d;

            if reconstruct_success {
                for index in 0..self.mp_matches.len() {
                    if !vb_triangulated[index as usize] {
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
