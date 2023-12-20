extern crate g2o;
use cxx::UniquePtr;
use log::{warn, debug, info};
use logging_timer::{time, timer};
use std::{fmt, fmt::Debug};
use opencv::{prelude::*, types::VectorOfKeyPoint,};

use dvcore::{
    actor::Actor,
    sensor::{Sensor, FrameSensor},
    matrix::*,
    config::*
};
use crate::{
    registered_actors::{FEATURE_DETECTION, CAMERA, VISUALIZER},
    actors::{
        messages::{ShutdownMsg, ImagePathMsg, ImageMsg, TrackingStateMsg, FeatureMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    },
    modules::image,
    ActorChannels,
    dvmap::{map::Id, misc::Timestamp}
};


#[derive(Debug)]
pub struct TrackingFrontEnd {
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

impl Actor for TrackingFrontEnd {
    type MapRef = ();

    fn new_actorstate(actor_channels: ActorChannels, _map: Self::MapRef) -> TrackingFrontEnd {
        let max_features = SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features");
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let orb_extractor_right = match sensor.frame() {
            FrameSensor::Stereo => Some(DVORBextractor::new(max_features)),
            FrameSensor::Mono | FrameSensor::Rgbd => None,
        };
        let orb_extractor_ini = match sensor.is_mono() {
            true => Some(DVORBextractor::new(max_features*5)),
            false => None
        };
        TrackingFrontEnd {
            actor_channels,
            orb_extractor_left: DVORBextractor::new(max_features),
            orb_extractor_right,
            orb_extractor_ini,
            map_initialized: false,
            init_id: 0,
            last_id: 0,
            max_frames: SETTINGS.get::<f64>(SYSTEM, "fps") as i32,
            sensor,
        }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = TrackingFrontEnd::new_actorstate(actor_channels, map);
        let max_queue_size = actor.actor_channels.receiver_bound.unwrap_or(100);

        tracy_client::set_thread_name!("tracking frontend");
        
        'outer: loop {
            let message = actor.actor_channels.receive().unwrap();

            if message.is::<ImagePathMsg>() {
                if actor.actor_channels.queue_len() > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Tracking frontend dropped 1 frame");
                    continue;
                }

                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

                let image = image::read_image_file(&msg.image_path);
                let image_cols = image.cols() as u32;
                let image_rows = image.rows() as u32;

                // TODO (timing) ... cloned if visualizer running. maybe make global shared object?
                let (keypoints, descriptors) = match SETTINGS.get::<bool>(SYSTEM, "show_visualizer") {
                    true => {
                        let (keypoints, descriptors) = actor.extract_features(image.clone());
                        actor.send_to_visualizer(keypoints.clone(), image, msg.timestamp);
                        (keypoints, descriptors)
                    },
                    false => {
                        let (keypoints, descriptors) = actor.extract_features(image);
                        (keypoints, descriptors)
                    }
                };
                actor.send_to_backend(keypoints, descriptors, image_cols, image_rows, msg.timestamp, msg.frame_id);

                actor.last_id += 1;
            } else if message.is::<ImageMsg>() {
                if actor.actor_channels.queue_len() > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Tracking frontend dropped 1 frame");
                    continue;
                }

                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

                let image_cols = msg.image.cols() as u32;
                let image_rows = msg.image.rows() as u32;

                // TODO (timing) ... cloned if visualizer running. maybe make global shared object?
                let (keypoints, descriptors) = match SETTINGS.get::<bool>(SYSTEM, "show_visualizer") {
                    true => {
                        let (keypoints, descriptors) = actor.extract_features(msg.image.clone());
                        actor.send_to_visualizer(keypoints.clone(), msg.image, msg.timestamp);
                        (keypoints, descriptors)
                    },
                    false => {
                        let (keypoints, descriptors) = actor.extract_features(msg.image);
                        (keypoints, descriptors)
                    }
                };


                actor.send_to_backend(keypoints, descriptors, image_cols, image_rows, msg.timestamp, msg.frame_id);
                actor.last_id += 1;
            } else if message.is::<TrackingStateMsg>() {
                let msg = message.downcast::<TrackingStateMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

                match msg.state {
                    TrackingState::Ok => { 
                        actor.map_initialized = true;
                        actor.init_id = msg.init_id
                    },
                    _ => {}
                };
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Tracking frontend received unknown message type!");
            }
        }
    }
}

impl TrackingFrontEnd {
    fn extract_features(&mut self, image: opencv::core::Mat) -> (VectorOfKeyPoint, Mat) {
        let _span = tracy_client::span!("extract features");

        let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image)).into();
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();

        // TODO (C++ and Rust optimizations) ... this takes ~70 ms which is way high compared to ORB-SLAM3. I think this is because the rust and C++ bindings are not getting optimized together.
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

    fn send_to_backend(&self, keypoints: VectorOfKeyPoint, descriptors: Mat, image_width: u32, image_height: u32, timestamp: Timestamp, frame_id: u32) {
        // Send features to backend
        // Note: Run-time errors ... actor lookup is runtime error
        // Note: not currently sending image to backend
        let backend = self.actor_channels.find("TRACKING_BACKEND");

        backend.send(Box::new(FeatureMsg{
            keypoints: DVVectorOfKeyPoint::new(keypoints),
            descriptors: DVMatrix::new(descriptors),
            image_width,
            image_height,
            timestamp,
            frame_id: frame_id as i32
        })).unwrap();
    }

    fn send_to_visualizer(&mut self, keypoints: VectorOfKeyPoint, image: Mat, timestamp: Timestamp) {
        // Send image and features to visualizer
        self.actor_channels.find(VISUALIZER).send(Box::new(VisFeaturesMsg {
            keypoints: DVVectorOfKeyPoint::new(keypoints),
            image,
            timestamp,
        })).unwrap();
    }
}


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
                SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor") as f32,
                SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "ini_th_fast"),
                SETTINGS.get::<i32>(FEATURE_DETECTION, "min_th_fast"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_begin"),
                SETTINGS.get::<i32>(CAMERA, "stereo_overlapping_end")
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