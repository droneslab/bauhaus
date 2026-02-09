extern crate g2o;
use crate::{
    actors::messages::{ImageMsg, ImagePathMsg, ShutdownMsg, VisFeaturesMsg, VisTrajectoryMsg},
    map::{frame::Frame, pose::Pose, read_only_lock::ReadWriteMap},
    modules::image,
    registered_actors::VISUALIZER,
};
use core::{
    matrix::*,
    system::{Actor, MessageBox, System},
};
use log::warn;
use opencv::imgcodecs;
use std::collections::BTreeSet;
use std::{thread::sleep, time::Duration};

pub struct GarbageTracking {
    system: System,

    /// References to map
    map: ReadWriteMap,
}

impl Actor for GarbageTracking {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = GarbageTracking { system, map };
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
            let (image, timestamp, mut imu_measurements, imu_initialization) =
                if message.is::<ImagePathMsg>() {
                    let msg = message
                        .downcast::<ImagePathMsg>()
                        .unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                    (
                        image::read_image_file(&msg.image_path, imgcodecs::IMREAD_GRAYSCALE),
                        msg.timestamp,
                        msg.imu_measurements,
                        msg.imu_initialization,
                    )
                } else {
                    let msg = message
                        .downcast::<ImageMsg>()
                        .unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                    (
                        msg.image,
                        msg.timestamp,
                        msg.imu_measurements,
                        msg.imu_initialization,
                    )
                };

            let mut frame = Frame::new_no_features(1, None, timestamp, None).unwrap();
            frame.pose = {
                let imu_init = imu_initialization.unwrap();
                let pose_original = imu_init.pose.clone();

                let transform1 = Pose::new_with_quaternion(
                    nalgebra::Vector3::<f64>::new(0.0, 0.0, 0.0),
                    nalgebra::geometry::UnitQuaternion::<f64>::from_quaternion(
                        nalgebra::Quaternion::<f64>::new
                        // (0.0, 7.07106781e-01, 0.0, 7.07106781e-01) // gt to imu frame
                        (-0.7071, 0.0, 0.7071, 0.7071),
                        // (1.0, 0.0, 0.0, 0.0) // Identity

                        // (0.7071, 0.7071, 0.0, 0.0),  // +X	90° around +X
                        // (0.7071, -0.7071, 0.0, 0.0),  // -X	90° around -X
                        // (0.7071, 0.0, 0.7071, 0.0),  // +Y	90° around +Y
                        // (0.7071, 0.0, -0.7071, 0.0),  // -Y	90° around -Y
                        // (0.7071, 0.0, 0.0, 0.7071),  // +Z	90° around +Z
                        // (0.7071, 0.0, 0.0, -0.7071), // -Z	90° around -Z
                    ),
                );
                // let transform2 = Pose::new_with_quaternion(
                //     nalgebra::Vector3::<f64>::new(0.0, 0.0, 0.0),
                //     nalgebra::geometry::UnitQuaternion::<f64>::from_quaternion(
                //     nalgebra::Quaternion::<f64>::new
                //         // 0.0, 7.07106781e-01, 0.0, 7.07106781e-01 // gt to imu frame
                //         // (1.0, 0.0, 0.0, 0.0) // Identity

                //         (0.7071, 0.7071, 0.0, 0.0),  // +X	90° around +X
                //         // (0.7071, -0.7071, 0.0, 0.0),  // -X	90° around -X
                //         // (0.7071, 0.0, 0.7071, 0.0),  // +Y	90° around +Y
                //         // (0.7071, 0.0, -0.7071, 0.0),  // -Y	90° around -Y
                //         // (0.7071, 0.0, 0.0, 0.7071),  // +Z	90° around +Z
                //         // (0.7071, 0.0, 0.0, -0.7071), // -Z	90° around -Z

                //     )
                // );
                let transform = transform1;
                // let pose_convert = ImuCalib::new().tcb * transform1 *  pose_original; //tcb * tbg * tgw
                let pose_convert = transform1 * pose_original;
                println!("Transform is: {:?}", transform.get_quaternion());
                println!("Transform is: {:?}", transform.get_rotation());

                println!("Timestamp: {:?}", timestamp);
                println!("Ground truth pose: {:?}", pose_original);
                println!("Converted pose: {:?}", pose_convert);

                Some(pose_convert)
                // Some(pose_original)
            }; // **** should be set to TCW
            let new_kf_id = self
                .map
                .write()
                .unwrap()
                .insert_keyframe_to_map(frame, false);

            self.system.send(
                VISUALIZER,
                Box::new(VisFeaturesMsg {
                    keypoints: DVVectorOfKeyPoint::empty(),
                    image,
                    timestamp,
                }),
            );
            println!("Measurements: {:?}", imu_measurements);

            self.system.try_send(
                VISUALIZER,
                Box::new(VisTrajectoryMsg {
                    pose: self
                        .map
                        .read()
                        .unwrap()
                        .get_keyframe(new_kf_id)
                        .get_pose()
                        .clone(),
                    mappoint_matches: vec![],
                    mappoints_in_tracking: BTreeSet::new(),
                    timestamp: timestamp,
                    map_version: 1,
                }),
            );
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
