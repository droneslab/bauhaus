use std::{fs::File, path::Path, io::{Write, BufWriter}};
use log::warn;

use crate::map::{map::Id, pose::Pose, read_only_lock::ReadWriteMap};
use core::{config::{SETTINGS, SYSTEM}, sensor::Sensor, system::{Actor, MessageBox, System, Timestamp}};
use super::{messages::{ShutdownMsg, TrajectoryMsg, TrackingStateMsg}, tracking_backend::TrackingState};


pub struct ShutdownActor {
    system: System,
    map: ReadWriteMap,
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
    trajectory_times: Vec<Timestamp>, //mlFrameTimes
    trajectory_keyframes: Vec<Id>, //mlpReferences

    results_folder: String,
    camera_trajectory_filename: String,
    keyframe_trajectory_filename: String,

    sensor: Sensor,

}

impl Actor for ShutdownActor {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = ShutdownActor{
            system,
            map,
            sensor: SETTINGS.get::<Sensor>(SYSTEM, "sensor"),
            trajectory_poses : Vec::new (),
            trajectory_times : Vec::new (),
            trajectory_keyframes : Vec::new (),
            results_folder : SETTINGS.get::<String>(SYSTEM, "results_folder"),
            camera_trajectory_filename : SETTINGS.get::<String>(SYSTEM, "camera_trajectory_file_name"),
            keyframe_trajectory_filename : SETTINGS.get::<String>(SYSTEM, "keyframe_trajectory_file_name")
        };
        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message) {
                break;
            }
        }
    }

}

impl ShutdownActor {
    fn handle_message(&mut self, message: MessageBox) -> bool {
        if message.is::<TrajectoryMsg>() {
            let msg = message.downcast::<TrajectoryMsg>().unwrap_or_else(|_| panic!("Could not downcast shutdown actor message!"));
            self.trajectory_poses.push(msg.pose);
            self.trajectory_times.push(msg.timestamp);
            self.trajectory_keyframes.push(msg.ref_kf_id);
            return false;
        } else if message.is::<TrackingStateMsg>() {
            let msg = message.downcast::<TrackingStateMsg>().unwrap_or_else(|_| panic!("Could not downcast shutdown actor message!"));

            match msg.state {
                TrackingState::Lost => {
                    // This can happen if tracking is lost. Duplicate last element of each vector
                    self.trajectory_poses.push(self.trajectory_poses.last().unwrap().clone());
                    self.trajectory_times.push(self.trajectory_times.last().unwrap().clone());
                    self.trajectory_keyframes.push(self.trajectory_keyframes.last().unwrap().clone());
                },
                _ => {}
            }
            return false;
        } else if message.is::<ShutdownMsg>() {
            warn!("Triggered shutdown, saving trajectory info");
            let file_camera = File::create(
                Path::new(&self.results_folder)
                .join(&self.camera_trajectory_filename)
            );
            let file_keyframe = File::create(
                Path::new(&self.results_folder)
                .join(&self.keyframe_trajectory_filename)
            );

            let map = self.map.read().unwrap();

            match file_camera {
                Ok(file) => {
                    let mut f = BufWriter::new(file);

                    for i in 0..self.trajectory_poses.len() {
                        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
                        // We need to get first the keyframe pose and then concatenate the relative transformation.
                        // Frames not localized (tracking failure) are not saved.

                        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
                        // which is true when tracking failed (lbL).

                        let pose = self.trajectory_poses[i];
                        let trans = pose.get_translation();
                        let rot = pose.get_quaternion();
                        let string = format!(
                            "{} {:.6} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}\n", 
                            self.trajectory_times[i], 
                            trans[0], trans[1], trans[2],
                            rot[0], rot[1], rot[2], rot[3]
                        );
                        write!(f, "{}", string).expect("unable to write");
                    }
                },
                Err(_) => {
                    warn!("Could not create trajectory file {:?}", Path::new(&self.results_folder).join(&self.camera_trajectory_filename));
                    println!("Here is the trajectory: ");
                    for i in 0..self.trajectory_poses.len() {
                        let pose = self.trajectory_poses[i];
                        let trans = pose.get_translation();
                        let rot = pose.get_quaternion();
                        println!(
                            "{} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}\n", 
                            self.trajectory_times[i], 
                            trans[0], trans[1], trans[2],
                            rot[0], rot[1], rot[2], rot[3]
                        );
                    }
                }
            };

            match file_keyframe {
                Ok(file) =>{
                    let mut f = BufWriter::new (file);

                    for id in 0..map.num_keyframes() {
                        if map.has_keyframe(id as i32) {
                            let kf = map.get_keyframe(id as i32);
                            if self.sensor.is_imu() {
                                todo!("IMU");
                                // Sophus::SE3f Twb = pKF->GetImuPose();
                                // Eigen::Quaternionf q = Twb.unit_quaternion();
                                // Eigen::Vector3f twb = Twb.translation();
                                // f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

                            } else {
                                let pose = kf.get_pose().group_inverse();
                                let trans = pose.get_translation();
                                let rot = pose.get_quaternion();

                                println!("Keyframe {} pose: {:?}", id, pose);

                                let string = format!(
                                    "{} {:.6} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}\n", 
                                    kf.timestamp * 1e18,
                                    trans[0], trans[1], trans[2],
                                    rot[0], rot[1], rot[2], rot[3]
                                );
                                write!(f, "{}", string).expect("unable to write");
                            }
                        } else {
                            println!("SKIP KEYFRAME {}", id);
                        }
                    }
                },
                Err(_) => {
                    warn!("Could not create trajectory file {:?}", Path::new(&self.results_folder).join(&self.camera_trajectory_filename));
                }
            };

            for (_, actor_tx) in &self.system.actors {
                actor_tx.send(Box::new(ShutdownMsg{})).unwrap();
            }

            return false;
        } else {
            return false;
        }
    }
}