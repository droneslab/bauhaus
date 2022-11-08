use std::collections::HashMap;
use chrono::Duration;
use dvcore::global_params::{GLOBAL_PARAMS, SYSTEM_SETTINGS, FrameSensor, ImuSensor};
use opencv::core::Point2f;
use dvcore::{matrix::DVVectorOfPoint3f, global_params::Sensor};
use crate::dvmap::{frame::Frame, pose::Pose, features::Features};

use crate::modules::camera::Camera;


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

    pub fn try_initialize(&mut self, current_frame: &Frame, camera: &Camera) -> Result<(), String> {
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");
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

    fn monocular_initialization(&mut self, camera: &Camera) -> Result<(), String> {
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2448
        if !self.ready_to_initializate && self.current_frame.as_ref().unwrap().features.num_keypoints > 100 {
            // Set Reference Frame
             self.prev_matched.resize(self.current_frame.as_ref().unwrap().features.num_keypoints as usize, Point2f::default());

            for i in 0..self.current_frame.as_ref().unwrap().features.num_keypoints as usize {
                self.prev_matched[i] = self.current_frame.as_ref().unwrap().features.keypoints_get(i).pt.clone();
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
            return Err("Not ready to initialize yet 1.".to_string());
        } else {
            if self.current_frame.as_ref().unwrap().features.num_keypoints <=100 || matches!(self.sensor.imu(), ImuSensor::Some) && self.last_frame.as_ref().unwrap().timestamp - self.initial_frame.as_ref().unwrap().timestamp > Duration::seconds(1) {
                self.ready_to_initializate = false;
                return Err("Not ready to initialize yet 2.".to_string());
            }

            // Find correspondences
            // TODO 10/17: BINDINGS
            let mut nmatches =10; //orb_matcher.search_for_initialization(
            //     &self.initial_frame.unwrap(), 
            //     &self.current_frame.unwrap(), 
            //     &mut self.prev_matched,
            //     &mut self.mp_matches,
            //     100
            // );

            // Check if there are enough correspondences
            if nmatches < 100 {
                self.ready_to_initializate = false;
                return Err("Not ready to initialize yet 3.".to_string());
            }

            let mut tcw = Pose::default();
            let mut vb_triangulated = Vec::<bool>::new();

            // TODO 10/17: BINDINGS
            let reconstruct_success = false;//camera.reconstruct_with_two_views(
            //     self.initial_frame.unwrap().keypoints_data.keypoints_un(),
            //     self.current_frame.unwrap().keypoints_data.keypoints_un(),
            //     &self.mp_matches,
            //     &mut tcw,
            //     &mut self.p3d,
            //     &mut vb_triangulated
            // );

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

                return Ok(());
            } else {
                return Err("Not ready to initialize yet 4.".to_string());
            }
        }
    }

    fn stereo_initialization(&mut self) -> Result<(), String> {
        todo!("Stereo: StereoInitialization");
    }
}
