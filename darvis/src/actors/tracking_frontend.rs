extern crate g2o;
use cxx::{UniquePtr};
use dvcore::base::{Actor, Base};
use dvcore::sensor::{Sensor, FrameSensor};
use log::{ warn, info };
use opencv::imgcodecs;
use std::any::Any;
use std::sync::mpsc::Receiver;
use std::{sync::Arc, fmt};
use std::fmt::Debug;
use opencv::{prelude::*,types::{VectorOfKeyPoint},};
use dvcore::{matrix::*,config::*,};
use crate::{ActorChannels};
use crate::dvmap::map::Id;
use crate::registered_actors::VISUALIZER;
use crate::{
    registered_actors::{TRACKING_BACKEND, FEATURE_DETECTION, CAMERA},
    actors::{
        messages::{ImageMsg, FeatureMsg,TrackingStateMsg}, tracking_backend::TrackingState,
        visualizer::VisFeaturesMsg
    },
};

use super::messages::ShutdownMessage;

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
    actor_channels: ActorChannels,
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
            let message = self.actor_channels.receive().unwrap();

            if let Some(msg) = message.downcast_ref::<ImageMsg>() {
                self.tracking_frontend(msg);
            } else if let Some(msg) = message.downcast_ref::<TrackingStateMsg>() {
                match msg.state {
                    TrackingState::Ok => { 
                        self.map_initialized = true;
                        self.init_id = msg.init_id
                    },
                    _ => {}
                };
            } else if let Some(_) = message.downcast_ref::<ShutdownMessage>() {
                break;
            } else {
                warn!("Tracking frontend received unknown message type!");
            }
        }
    }
}

impl DarvisTrackingFront {
    pub fn new(actor_channels: ActorChannels) -> DarvisTrackingFront {
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
            actor_channels,
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

    fn tracking_frontend(&mut self, message: &ImageMsg) {
        // TODO (vis): If visualizer is running, this will cause the image to be read twice! Can we figure out a way to convert the Mat into something rerun can use?
        let image = imgcodecs::imread(&message.image_path, imgcodecs::IMREAD_GRAYSCALE).expect("Could not read image.");

        // If passing image directly, uncomment this. Taking ownership of message should not fail
        // let image: Mat = (&message.frame).into();
        let (keypoints, descriptors) = self.extract_features(&image);
        self.pass_to_backend(&image, &keypoints, descriptors);
        if GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_visualizer") {
            self.send_keypoints_to_visualizer(keypoints);
        }
        self.last_id += 1;
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

    fn pass_to_backend(
        &mut self,
        image: &Mat, keypoints: &VectorOfKeyPoint, descriptors: Mat
    ) {
        let backend = self.actor_channels.find("TRACKING_BACKEND").unwrap();
        let new_message = GLOBAL_PARAMS.get::<String>(TRACKING_BACKEND, "actor_message");
        match new_message.as_ref() {
            "FeatureMsg" => {
                backend.send(Box::new(
                    FeatureMsg {
                        keypoints: DVVectorOfKeyPoint::new(keypoints.clone()),
                        descriptors: DVMatrix::new(descriptors),
                        image_width: image.cols(),
                        image_height: image.rows(),
                    }
                )).unwrap();
            },
            _ => {
                warn!("Invalid Message type: selecting FeatureMsg");
                backend.send(Box::new(
                    FeatureMsg {
                        keypoints: DVVectorOfKeyPoint::new(keypoints.clone()),
                        descriptors: DVMatrix::new(descriptors),
                        image_width: image.cols(),
                        image_height: image.rows(),
                    }
                )).unwrap();
            },
        }
    }

    fn send_keypoints_to_visualizer(&mut self, keypoints: VectorOfKeyPoint) {
        let vis_actor = self.actor_channels.find(VISUALIZER).unwrap();
        vis_actor.send(Box::new(VisFeaturesMsg {
            keypoints: DVVectorOfKeyPoint::new(keypoints)
        })).unwrap();
    }
}

