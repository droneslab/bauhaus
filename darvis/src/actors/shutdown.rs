use std::{fs::File, path::Path, io::Write, collections::HashMap};
use log::warn;

use crate::dvmap::{pose::DVPose, map::Id, misc::Timestamp};
use dvcore::{actor::{ActorChannels, Actor, ActorMessage}, config::{SYSTEM, SETTINGS}};
use super::{messages::{ShutdownMsg}, tracking_backend::TrackingState};


pub struct ShutdownActor {
    actor_channels: ActorChannels,
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    trajectory_poses: Vec<DVPose>, //mlRelativeFramePoses
    trajectory_times: Vec<Timestamp>, //mlFrameTimes
    trajectory_keyframes: Vec<Id>, //mlpReferences

    results_folder: String,
    trajectory_filename: String
}

impl ShutdownActor {
    pub fn new(actor_channels: ActorChannels) -> ShutdownActor {
        ShutdownActor{
            actor_channels,
            trajectory_poses: Vec::new(),
            trajectory_times: Vec::new(),
            trajectory_keyframes: Vec::new(),
            results_folder: SETTINGS.get::<String>(SYSTEM, "results_folder"),
            trajectory_filename: SETTINGS.get::<String>(SYSTEM, "trajectory_file_name")
        }
    }
}
impl Actor for ShutdownActor {
    fn run(&mut self) {
        loop {
            let message = self.actor_channels.receive().unwrap();
            if message.is::<ShutdownActorMsg>() {
                let msg = message.downcast::<ShutdownActorMsg>().unwrap_or_else(|_| panic!("Could not downcast shutdown actor message!"));

                match *msg {
                    ShutdownActorMsg::TrajectoryMsg{ pose, ref_kf_id, timestamp } => {
                        self.trajectory_poses.push(pose);
                        self.trajectory_times.push(timestamp);
                        self.trajectory_keyframes.push(ref_kf_id);
                    },
                    ShutdownActorMsg::TrackingStateMsg{ state } => {
                        match state {
                            TrackingState::Lost => {
                                // This can happen if tracking is lost. Duplicate last element of each vector
                                self.trajectory_poses.push(self.trajectory_poses.last().unwrap().clone());
                                self.trajectory_times.push(self.trajectory_times.last().unwrap().clone());
                                self.trajectory_keyframes.push(self.trajectory_keyframes.last().unwrap().clone());
                            },
                            _ => {}
                        }
                    },
                };
            } else if message.is::<ShutdownMsg>() {
                warn!("Triggered shutdown, saving trajectory info");
                let mut file = File::create(
                    Path::new(&self.results_folder)
                    .join(&self.trajectory_filename)
                ).unwrap();
                for i in 0..self.trajectory_poses.len() {
                    let string = format!("{:?} {:?}", self.trajectory_times[i], self.trajectory_poses[i]);
                    file.write_all(string.as_bytes()).unwrap();
                }

                if SETTINGS.get::<bool>(SYSTEM, "create_flamegraph") {
                    flame::dump_html(File::create(
                        Path::new(&self.results_folder).join("flamegraph.html")
                    ).unwrap()).unwrap();
                }

                for (_, actor_tx) in &self.actor_channels.actors {
                    actor_tx.send(Box::new(ShutdownMsg{})).unwrap();
                }
            }
        }
    }
}

pub enum ShutdownActorMsg {
    TrajectoryMsg{ pose: DVPose, ref_kf_id: Id, timestamp: Timestamp,},
    TrackingStateMsg{ state: TrackingState },
}
impl ActorMessage for ShutdownActorMsg {}