use std::{fs::File, path::Path, io::{Write, BufWriter}};
use log::warn;

use crate::map::{pose::Pose, map::Id};
use core::{config::{SETTINGS, SYSTEM}, system::{Actor, MessageBox, System, Timestamp}};
use super::{messages::{ShutdownMsg, TrajectoryMsg, TrackingStateMsg}, tracking_backend::TrackingState};


pub struct ShutdownActor {
    system: System,
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
    trajectory_times: Vec<Timestamp>, //mlFrameTimes
    trajectory_keyframes: Vec<Id>, //mlpReferences

    results_folder: String,
    trajectory_filename: String
}

impl Actor for ShutdownActor {
    type MapRef = ();

    fn spawn(system: System, _map: Self::MapRef) {
        let mut actor = ShutdownActor{
            system,
            trajectory_poses: Vec::new(),
            trajectory_times: Vec::new(),
            trajectory_keyframes: Vec::new(),
            results_folder: SETTINGS.get::<String>(SYSTEM, "results_folder"),
            trajectory_filename: SETTINGS.get::<String>(SYSTEM, "trajectory_file_name")
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
            let file = File::create(
                Path::new(&self.results_folder)
                .join(&self.trajectory_filename)
            );
            match file {
                Ok(file) => {
                    let mut f = BufWriter::new(file);

                    for i in 0..self.trajectory_poses.len() {
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
                    warn!("Could not create trajectory file {:?}", Path::new(&self.results_folder).join(&self.trajectory_filename));
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

            for (_, actor_tx) in &self.system.actors {
                actor_tx.send(Box::new(ShutdownMsg{})).unwrap();
            }
            return false;
        } else {
            return false;
        }
    }
}