use std::{fs::File, path::Path, io::{Write, BufWriter}};
use log::{debug, warn};

use crate::{map::{map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::imu::ImuCalib};
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

            let imu_calib = match self.sensor.is_imu() {
                true => Some(ImuCalib::new()),
                false => None
            };

            match file_camera {
                Ok(file) => {
                    // Nitin ... printing trajectory
                    // Look at void System::SaveTrajectoryEuRoC(const string &filename) in System.cc
                    let mut f = BufWriter::new(file);
                    let lock = self.map.read().unwrap();

                    // Transform all keyframes so that the first keyframe is at the origin.
                    // After a loop closure the first keyframe might not be at the origin.
                    let first_keyframe = lock.get_first_keyframe();
                    let twb;  // Can be word to cam0 or world to b depending on IMU or not.
                    if self.sensor.is_imu() {
                        twb = first_keyframe.get_imu_pose();
                    } else {
                        twb = first_keyframe.get_pose().inverse();
                    }

                    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
                    // We need to get first the keyframe pose and then concatenate the relative transformation.
                    // Frames not localized (tracking failure) are not saved.

                    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
                    // which is true when tracking failed (lbL).
                    // list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
                    // list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
                    // list<bool>::iterator lbL = mpTracker->mlbLost.begin();


                    for i in 0..self.trajectory_poses.len() {
                        let mut ref_kf = self.trajectory_keyframes[i];
                        let mut trw = Pose::default();

                        debug!("Trajectory pose {} with ref kf {}!!", i, ref_kf);

                        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
                        while !lock.has_keyframe(ref_kf) {
                            let (parent_id, pose_relative_to_parent) = lock.get_deleted_keyframe_info(ref_kf);
                            trw = trw * pose_relative_to_parent;
			
                            ref_kf = parent_id;
                            debug!("..don't have ref kf, looking at parent {}", ref_kf);
                        }

                        trw = trw * lock.get_keyframe(ref_kf).get_pose() * twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference
                        // debug! ("Debugging value: {:?}",trw);
			
                        if self.sensor.is_imu() {

                            let twc = (imu_calib.as_ref().unwrap().tbc * self.trajectory_poses[i] * trw).inverse();
                            let rot = twc.get_quaternion();
                            let trans = twc.get_translation();

                            let string = format!(
                                "{} {:.6} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}\n", 
                                self.trajectory_times[i] * 1e9, 
                                trans[0], trans[1], trans[2],
                                rot[0], rot[1], rot[2], rot[3]
                            );
                            write!(f, "{}", string).expect("unable to write");
                        } else {
			    debug!("ELSE BRANCH IS TAKEN WITH THE IMU");
                            let twc = (self.trajectory_poses[i] * trw).inverse();
			    //debug!("Twc value - {:?}!!",twc);			    
                            let rot = twc.get_quaternion();
			    debug!("Quaternion value - {:?}!!",rot);
                            let trans = twc.get_translation();
                            debug!("Translation value - {:?}!!",trans);			    			    
                            let string = format!(
                                "{} {:.6} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}\n", 
                                self.trajectory_times[i] * 1e9, 
                                trans[0], trans[1], trans[2],
                                rot[0], rot[1], rot[2], rot[3]
                            );
                            write!(f, "{}", string).expect("unable to write");

                        }

                    }



                //     for i in 0..self.trajectory_poses.len() {

                //         let trw = Pose::default();
                //         let ref_kf_id = self.trajectory_keyframes[i];
                //         while !lock.has_keyframe(ref_kf_id) {

                //         }

                //         if !lock.has_keyframe(self.trajectory_keyframes[i] as i32) {
                //             trw = trw * pkf.mtcp;
                //             pkf = pkf.get_parent();
                //         }

                //         trw = trw * pkf.get_pose() * twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference



                //         let pose = self.trajectory_poses[i];
                //         let trans = pose.get_translation();
                //         let rot = pose.get_quaternion();
                //         let string = format!(
                //             "{} {:.6} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}\n", 
                //             self.trajectory_times[i] * 1e9, 
                //             trans[0], trans[1], trans[2],
                //             rot[0], rot[1], rot[2], rot[3]
                //         );
                //         write!(f, "{}", string).expect("unable to write");
                //     }
                },
                Err(_) => {
                    warn!("Could not create trajectory file {:?}", Path::new(&self.results_folder).join(&self.camera_trajectory_filename));
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
                                let pose = kf.get_pose().inverse();
                                let trans = pose.get_translation();
                                let rot = pose.get_quaternion();

                                debug!("Keyframe {} pose: {:?}", id, pose);

                                let string = format!(
                                    "{} {:.6} {:.7} {:.7} {:.7} {:.7} {:.7} {:.7}\n", 
                                    kf.timestamp * 1e9,
                                    trans[0], trans[1], trans[2],
                                    rot[0], rot[1], rot[2], rot[3]
                                );
                                write!(f, "{}", string).expect("unable to write");
                            }
                        } else {
                            debug!("SKIP KEYFRAME {}", id);
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
