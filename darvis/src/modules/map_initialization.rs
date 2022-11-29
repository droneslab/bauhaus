use std::collections::HashMap;
use chrono::Duration;
use dvcore::config::{GLOBAL_PARAMS, SYSTEM_SETTINGS, FrameSensor, ImuSensor};
use log::debug;
use opencv::core::Point2f;
use dvcore::{matrix::DVVectorOfPoint3f, config::Sensor};
use crate::dvmap::{frame::Frame, pose::Pose};

use crate::modules::camera::Camera;

use super::orbmatcher;


#[derive(Debug, Clone, Default)]
pub struct Initialization {
    // Initialization (Monocular)
    pub mp_matches: HashMap<u32, u32>,// ini_matches .. mvIniMatches;
    pub prev_matched: Vec<Point2f>,// std::vector<cv::Point2f> mvbPrevMatched;
    pub p3d: DVVectorOfPoint3f,// std::vector<cv::Point3f> mvIniP3D;
    pub ready_to_initializate: bool,
    pub initial_frame: Option<Frame>,
    pub last_frame: Option<Frame>,
    pub current_frame: Option<Frame>,
    sensor: Sensor,
}

impl Initialization {
    pub fn new() -> Self {
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

        Self {
            mp_matches: HashMap::new(),
            prev_matched: Vec::new(),
            p3d: DVVectorOfPoint3f::empty(),
            ready_to_initializate: false,
            initial_frame: None,
            last_frame: None,
            current_frame: None,
            sensor
        }
    }

    pub fn try_initialize(&mut self, current_frame: &Frame, camera: &mut Camera) -> Result<bool, Box<dyn std::error::Error>> {
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
            FrameSensor::Mono => self.monocular_initialization(camera),
            _ => self.stereo_initialization()
        }
    }

    fn monocular_initialization(&mut self, camera: &mut Camera) -> Result<bool, Box<dyn std::error::Error>> {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        let current_frame = self.current_frame.as_ref().unwrap();
        let initial_frame = self.initial_frame.as_ref().unwrap();
        let last_frame = self.last_frame.as_ref().unwrap();

        if !self.ready_to_initializate && current_frame.features.num_keypoints > 100 {
            // Set Reference Frame
             self.prev_matched.resize(current_frame.features.num_keypoints as usize, Point2f::default());

            for i in 0..current_frame.features.num_keypoints as usize {
                self.prev_matched[i] = current_frame.features.get_keypoint(i).pt.clone();
            }

            match self.sensor.imu() {
                ImuSensor::Some => {
                    //TODO: (IMU) 
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
            debug!("MonocularInitialization, completed step 1");
            return Ok(false);
        } else {
            if current_frame.features.num_keypoints <=100 || matches!(self.sensor.imu(), ImuSensor::Some) && last_frame.timestamp - initial_frame.timestamp > Duration::seconds(1) {
                self.ready_to_initializate = false;
                return Ok(false);
            }

            // Find correspondences
            let mut nmatches = orbmatcher::search_for_initialization(
                &initial_frame, 
                &current_frame, 
                &mut self.prev_matched,
                &mut self.mp_matches,
                100
            );
            debug!("MonocularInitialization, search for initialization.. {} matches", nmatches);

            // Check if there are enough correspondences
            if nmatches < 100 {
                self.ready_to_initializate = false;
                return Ok(false);
            };

            let mut tcw = Pose::default();
            let mut vb_triangulated = Vec::<bool>::new();

            let reconstruct_success = camera.reconstruct_with_two_views(
                initial_frame.features.get_all_keypoints(),
                current_frame.features.get_all_keypoints(),
                &self.mp_matches,
                &mut tcw,
                &mut self.p3d,
                &mut vb_triangulated
            );

            if reconstruct_success {
                let keys = self.mp_matches.keys().cloned().collect::<Vec<_>>();
                for index in keys {
                    if !vb_triangulated[index as usize] {
                        self.mp_matches.remove(&index);
                        nmatches-=1;
                    }
                }

                self.initial_frame.as_mut().unwrap().pose = Some(Pose::default());
                self.current_frame.as_mut().unwrap().pose = Some(tcw);

                debug!("MonocularInitialization, ReconstructWithTwoViews... success ... can initialize now");
                return Ok(true);
            } else {
                // TODO (tracking bugs): Reconstruction always fails at the first attempt, is this normal?
                debug!("MonocularInitialization, ReconstructWithTwoViews... failure");
                return Ok(false);
            }
        }
    }

    fn stereo_initialization(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        todo!("Stereo: StereoInitialization");
    }
}
