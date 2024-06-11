extern crate g2o;
use log::{warn, info};
use opencv::{prelude::*, types::VectorOfKeyPoint,};

use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, Timestamp}
};
use crate::{
    actors::{
        messages::{FeatureMsg, ImageMsg, ImagePathMsg, ShutdownMsg, TrackingStateMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    }, map::map::Id, modules::{image, orbextractor::DVORBextractor}, registered_actors::{new_feature_extraction_module, FEATURE_DETECTION, VISUALIZER}, System
};
use crate::modules::module::FeatureExtractionModule;



pub struct TrackingFrontEnd {
    system: System,
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    _orb_extractor_right: Option<Box<dyn FeatureExtractionModule>>,
    orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    map_initialized: bool,
    last_id: Id,
    init_id: Id,
    max_frames: i32,
    sensor: Sensor,
}

impl Actor for TrackingFrontEnd {
    type MapRef = ();

    fn new_actorstate(system: System, _map: Self::MapRef) -> TrackingFrontEnd {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");

        let _orb_extractor_right = match sensor.frame() {
            FrameSensor::Stereo => Some(new_feature_extraction_module(false)),
            FrameSensor::Mono | FrameSensor::Rgbd => None,
        };
        let orb_extractor_ini = match sensor.is_mono() {
            true => Some(new_feature_extraction_module(true)),
            false => None
        };

        TrackingFrontEnd {
            system,
            orb_extractor_left: new_feature_extraction_module(false),
            _orb_extractor_right,
            orb_extractor_ini,
            map_initialized: false,
            init_id: 0,
            last_id: 0,
            max_frames: SETTINGS.get::<f64>(SYSTEM, "fps") as i32,
            sensor,
        }
    }

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = TrackingFrontEnd::new_actorstate(system, map);
        let max_queue_size = actor.system.receiver_bound.unwrap_or(100);

        tracy_client::set_thread_name!("tracking frontend");

        'outer: loop {
            let message = actor.system.receive().unwrap();

            if message.is::<ImagePathMsg>() {
                if actor.system.queue_len() > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Tracking frontend dropped 1 frame");
                    continue;
                }

                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

                let image = image::read_image_file(&msg.image_path);
                let image_cols = image.cols() as u32;
                let image_rows = image.rows() as u32;

                // TODO (timing) ... cloned if visualizer running. maybe make global shared object?
                let (keypoints, descriptors) = match actor.system.actors.get(VISUALIZER).is_some() {
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
                if actor.system.queue_len() > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Tracking frontend dropped 1 frame");
                    continue;
                }

                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

                let image_cols = msg.image.cols() as u32;
                let image_rows = msg.image.rows() as u32;

                // TODO (timing) ... cloned if visualizer running. maybe make global shared object?
                let (keypoints, descriptors) = match actor.system.actors.get(VISUALIZER).is_some() {
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
        let _span = tracy_client::span!("extract_features");

        let (keypoints, descriptors) = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::None) => {
                if !self.map_initialized || (self.last_id - self.init_id < self.max_frames) {
                    self.orb_extractor_ini.as_mut().unwrap().extract(image).unwrap()
                } else {
                    self.orb_extractor_left.extract(image).unwrap()
                }
            },
            _ => { 
                // See GrabImageMonocular, GrabImageStereo, GrabImageRGBD in Tracking.cc
                todo!("IMU, Stereo, RGBD")
            }
        };

        match self.sensor.frame() {
            FrameSensor::Stereo => todo!("Stereo"), //Also call extractor_right, see Tracking::GrabImageStereo,
            _ => {}
        }

        (keypoints, descriptors)
    }

    fn send_to_backend(&self, keypoints: VectorOfKeyPoint, descriptors: Mat, image_width: u32, image_height: u32, timestamp: Timestamp, frame_id: u32) {
        // Send features to backend
        // Note: Run-time errors ... actor lookup is runtime error
        // Note: not currently sending image to backend
        let backend = self.system.find_actor("TRACKING_BACKEND");

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
        self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
            keypoints: DVVectorOfKeyPoint::new(keypoints),
            image,
            timestamp,
        }));
    }
}
