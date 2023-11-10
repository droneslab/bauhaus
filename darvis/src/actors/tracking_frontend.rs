extern crate g2o;
use cxx::UniquePtr;
use log::warn;
use std::{fmt, fmt::Debug};
use opencv::{prelude::*, types::VectorOfKeyPoint,};

use dvcore::{
    base::Actor,
    sensor::{Sensor, FrameSensor},
    matrix::*,
    config::*
};
use crate::{
    registered_actors::{FEATURE_DETECTION, CAMERA, VISUALIZER},
    actors::{
        messages::{ImageMsg, FeatureMsg,TrackingStateMsg, ShutdownMsg, ImagePathMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    },
    modules::image,
    ActorChannels,
    dvmap::map::Id
};


pub struct DVORBextractor {
    pub extractor: UniquePtr<dvos3binding::ffi::ORBextractor>,
    pub max_features: i32
}
impl DVORBextractor {
    pub fn new(max_features: i32) -> Self {
        DVORBextractor{
            max_features,
            extractor: dvos3binding::ffi::new_orb_extractor(
                max_features,
                GLOBAL_PARAMS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
                GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "min_th_fast"),
                GLOBAL_PARAMS.get::<i32>(CAMERA, "stereo_overlapping_begin"),
                GLOBAL_PARAMS.get::<i32>(CAMERA, "stereo_overlapping_end")
            )
        }
    }
}
impl Clone for DVORBextractor {
    fn clone(&self) -> Self {
        DVORBextractor::new(self.max_features)
    }
}
impl Debug for DVORBextractor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DVORBextractor")
         .finish()
    }
}

#[derive(Debug)]
pub struct DarvisTrackingFront {
    actor_system: ActorChannels,
    orb_extractor_left: DVORBextractor,
    orb_extractor_right: Option<DVORBextractor>,
    orb_extractor_ini: Option<DVORBextractor>,
    map_initialized: bool,
    last_id: Id,
    init_id: Id,
    max_frames: i32,
    sensor: Sensor,
}

impl Actor for DarvisTrackingFront {
    fn run(&mut self) {
        loop {
            // Note: Run-time errors ... downcasting to process message is run-time error

            let message = self.actor_system.receive().unwrap();

            if let Some(msg) = message.downcast_ref::<ImagePathMsg>() {
                let image = image::read_image_file(&msg.image_path);
                let (keypoints, descriptors) = self.extract_features(&image);
                self.send_to_backend(keypoints.clone(), descriptors, &image, msg.timestamp);
                self.send_to_visualizer(keypoints, image, msg.timestamp, &msg.image_path);
                self.last_id += 1;
            } else if message.is::<ImageMsg>() {
                if let Ok(msg) = message.downcast::<ImageMsg>() {
                    let (keypoints, descriptors) = self.extract_features(&msg.image);
                    self.send_to_backend(keypoints, descriptors, &msg.image, msg.timestamp);
                } else {
                    panic!("Failed to downcast ImageMsg");
                }
                self.last_id += 1;
            } else if let Some(msg) = message.downcast_ref::<TrackingStateMsg>() {
                match msg.state {
                    TrackingState::Ok => { 
                        self.map_initialized = true;
                        self.init_id = msg.init_id
                    },
                    _ => {}
                };
            } else if let Some(_) = message.downcast_ref::<ShutdownMsg>() {
                break;
            } else {
                warn!("Tracking frontend received unknown message type!");
            }
        }
    }
}

impl DarvisTrackingFront {
    pub fn new(actor_system: ActorChannels) -> DarvisTrackingFront {
        let max_features = GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "max_features");
        let sensor = GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor");
        let orb_extractor_right = match sensor.frame() {
            FrameSensor::Stereo => Some(DVORBextractor::new(max_features)),
            FrameSensor::Mono | FrameSensor::Rgbd => None,
        };
        let orb_extractor_ini = match sensor.is_mono() {
            true => Some(DVORBextractor::new(max_features*5)),
            false => None
        };
        DarvisTrackingFront {
            actor_system,
            orb_extractor_left: DVORBextractor::new(max_features),
            orb_extractor_right,
            orb_extractor_ini,
            map_initialized: false,
            init_id: 0,
            last_id: 0,
            max_frames: GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "fps") as i32,
            sensor,
        }
    }

    fn extract_features(&mut self, image: &opencv::core::Mat) -> (VectorOfKeyPoint, Mat) {
        let image_dv: dvos3binding::ffi::WrapBindCVMat = DVMatrix::new(image.clone()).into();

        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = DVMatrix::default().into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();

        if self.map_initialized && (self.last_id - self.init_id < self.max_frames) {
            self.orb_extractor_left.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
        } else if self.sensor.is_mono() {
            self.orb_extractor_ini.as_mut().unwrap().extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
        } else {
            self.orb_extractor_left.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
        }
        match self.sensor.frame() {
            FrameSensor::Stereo => todo!("Stereo"), //Also call extractor_right, see Tracking::GrabImageStereo,
            _ => {}
        }

        (keypoints.kp_ptr.kp_ptr, descriptors.mat_ptr.mat_ptr)
    }

    fn send_to_backend(&self, keypoints: VectorOfKeyPoint, descriptors: Mat, image: &Mat, timestamp: u64) {
        // Send features to backend
        // Note: Run-time errors ... actor lookup is runtime error
        let backend = self.actor_system.find("TRACKING_BACKEND").unwrap();
        backend.send(Box::new(
            FeatureMsg {
                keypoints: DVVectorOfKeyPoint::new(keypoints.clone()),
                descriptors: DVMatrix::new(descriptors),
                image_width: image.cols(),
                image_height: image.rows(),
                timestamp
            }
        )).unwrap();
    }

    fn send_to_visualizer(&mut self, keypoints: VectorOfKeyPoint, image: Mat, timestamp: u64, image_path: &String) {
        // Send image and features to visualizer
        if let Some(vis_actor) = self.actor_system.find(VISUALIZER) {
            vis_actor.send(Box::new(VisFeaturesMsg {
                keypoints: DVVectorOfKeyPoint::new(keypoints),
                image,
                timestamp,
                image_filename: image_path.split("/").last().unwrap().split(".").nth(0).unwrap().to_string(),
            })).unwrap();
        }
    }
}

