use std::{fs::File, path::Path, io::{Write, BufWriter}};
use log::warn;

use crate::map::{pose::Pose, map::Id};
use core::{config::{SETTINGS, SYSTEM}, system::{Actor, System, Timestamp}};
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

    fn new_actorstate(system: System, _map: Self::MapRef) -> ShutdownActor {
        ShutdownActor{
            system,
            trajectory_poses: Vec::new(),
            trajectory_times: Vec::new(),
            trajectory_keyframes: Vec::new(),
            results_folder: SETTINGS.get::<String>(SYSTEM, "results_folder"),
            trajectory_filename: SETTINGS.get::<String>(SYSTEM, "trajectory_file_name")
        }
    }

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = ShutdownActor::new_actorstate(system, map);
        loop {
            let message = actor.system.receive().unwrap();
            if message.is::<TrajectoryMsg>() {
                let msg = message.downcast::<TrajectoryMsg>().unwrap_or_else(|_| panic!("Could not downcast shutdown actor message!"));
                actor.trajectory_poses.push(msg.pose);
                actor.trajectory_times.push(msg.timestamp);
                actor.trajectory_keyframes.push(msg.ref_kf_id);
            } else if message.is::<TrackingStateMsg>() {
                let msg = message.downcast::<TrackingStateMsg>().unwrap_or_else(|_| panic!("Could not downcast shutdown actor message!"));

                match msg.state {
                    TrackingState::Lost => {
                        // This can happen if tracking is lost. Duplicate last element of each vector
                        actor.trajectory_poses.push(actor.trajectory_poses.last().unwrap().clone());
                        actor.trajectory_times.push(actor.trajectory_times.last().unwrap().clone());
                        actor.trajectory_keyframes.push(actor.trajectory_keyframes.last().unwrap().clone());
                    },
                    _ => {}
                }
            } else if message.is::<ShutdownMsg>() {
                warn!("Triggered shutdown, saving trajectory info");
                let file = File::create(
                    Path::new(&actor.results_folder)
                    .join(&actor.trajectory_filename)
                );
                match file {
                    Ok(file) => {
                        let mut f = BufWriter::new(file);

                        // TODO (MVP) ... look at SaveTrajectoryEuRoC in orbslam, I think we need to apply some transform to these poses
                        // I think they are currently saved as the relative pose between frames but we need world coords
                        for i in 0..actor.trajectory_poses.len() {
                            let pose = actor.trajectory_poses[i];
                            let trans = pose.get_translation();
                            let rot = pose.get_quaternion();
                            let string = format!(
                                "{} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}\n", 
                                actor.trajectory_times[i], 
                                trans[0], trans[1], trans[2],
                                rot[0], rot[1], rot[2], rot[3]
                            );
                            write!(f, "{}", string).expect("unable to write");
                        }
                    },
                    Err(_) => {
                        warn!("Could not create trajectory file {:?}", Path::new(&actor.results_folder).join(&actor.trajectory_filename));
                        println!("Here is the trajectory: ");
                        for i in 0..actor.trajectory_poses.len() {
                            let pose = actor.trajectory_poses[i];
                            let trans = pose.get_translation();
                            let rot = pose.get_quaternion();
                            println!(
                                "{} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}\n", 
                                actor.trajectory_times[i], 
                                trans[0], trans[1], trans[2],
                                rot[0], rot[1], rot[2], rot[3]
                            );
                        }
                    }
                };

                for (_, actor_tx) in &actor.system.actors {
                    actor_tx.send(Box::new(ShutdownMsg{})).unwrap();
                }
                return;
            }
        }
    }
}
