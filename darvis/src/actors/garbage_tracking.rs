extern crate g2o;
use ahash::HashMap;
use log::{warn, info, debug};
use std::{collections::{BTreeSet}};
use std::{sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{core::{Point, Point2f, Scalar, CV_8U}, imgproc::circle, prelude::*, types::{VectorOfPoint2f, VectorOfu8}};
use core::{
    config::*, matrix::*, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{local_mapping::LOCAL_MAPPING_IDLE, messages::{FeatureTracksAndIMUMsg, ImagePathMsg, VisTrajectoryMsg, TrajectoryMsg, ImageMsg, InitKeyFrameMsg, ShutdownMsg, VisFeaturesMsg}}, map::{frame::Frame, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image, imu::{ImuMeasurements, IMU}, map_initialization::MapInitialization, module_definitions::{MapInitializationModule, FeatureExtractionModule}}, registered_actors::{new_feature_extraction_module, CAMERA_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}
};


pub struct GarbageTracking {
    system: System,

    /// References to map
    map: ReadWriteMap,
}

impl Actor for GarbageTracking {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = GarbageTracking {
            system,
            map,

        };
        tracy_client::set_thread_name!("tracking frontend gtsam");

        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message) {
                break;
            }
            actor.map.match_map_version();
        }
    }

}

impl GarbageTracking {
    fn handle_message(&mut self, message: MessageBox) -> bool {
        if message.is::<ImagePathMsg>() || message.is::<ImageMsg>() {
            let (image, timestamp, mut imu_measurements, imu_initialization) = if message.is::<ImagePathMsg>() {
                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (image::read_image_file(&msg.image_path), msg.timestamp, msg.imu_measurements, msg.imu_initialization)
            } else {
                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (msg.image, msg.timestamp, msg.imu_measurements, msg.imu_initialization)
            };

            let mut frame = Frame::new_no_features(
                1,
                None,
                timestamp,
                None,
            ).unwrap();
            frame.pose = {
                let imu_init = imu_initialization.unwrap();
                let pose_original = Pose::new_with_quaternion_convert(
                    *imu_init.translation,
                    imu_init.rotation,
                );
                let transform = Pose::new_with_quaternion(
                    nalgebra::Vector3::<f64>::new(0.0, 0.0, 0.0),
                    nalgebra::geometry::UnitQuaternion::<f64>::from_quaternion(
                    nalgebra::Quaternion::<f64>::new(
                        // 0.0, 0.7071, -0.0, -0.7071,
                        1.0, 0.0, 0.0, 0.0,
                    )
                    )
                );

                let pose_convert = transform * pose_original;

                println!("Timestamp: {:?}", timestamp);
                println!("Ground truth pose: {:?}", pose_original);
                println!("Converted pose: {:?}", pose_convert);
                println!("Imu measurements: {:?}", imu_measurements);
                println!("Bias: {:?}", crate::modules::imu::ImuBias::new_with(imu_init.gyro_bias, imu_init.acc_bias));

                Some(pose_convert)
            };
            let new_kf_id = self.map.write().unwrap().insert_keyframe_to_map(frame, false);


            self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
                keypoints: DVVectorOfKeyPoint::empty(),
                image,
                timestamp,
            }));

            self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
                pose: self.map.read().unwrap().get_keyframe(new_kf_id).get_pose().clone(),
                mappoint_matches: vec![],
                mappoints_in_tracking: BTreeSet::new(),
                timestamp: timestamp,
                map_version: 1
            }));

        } else if message.is::<ShutdownMsg>() {
            // Sleep a little to allow other threads to finish
            sleep(Duration::from_millis(100));
            return true;
        } else {
            warn!("Tracking frontend GTSAM received unknown message type!");
        }
        return false;
    }

}
