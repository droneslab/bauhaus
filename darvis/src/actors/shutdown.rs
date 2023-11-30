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

impl Actor for ShutdownActor {
    type MapRef = ();

    fn new_actorstate(actor_channels: ActorChannels, _map: Self::MapRef) -> ShutdownActor {
        ShutdownActor{
            actor_channels,
            trajectory_poses: Vec::new(),
            trajectory_times: Vec::new(),
            trajectory_keyframes: Vec::new(),
            results_folder: SETTINGS.get::<String>(SYSTEM, "results_folder"),
            trajectory_filename: SETTINGS.get::<String>(SYSTEM, "trajectory_file_name")
        }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = ShutdownActor::new_actorstate(actor_channels, map);
        loop {
            let message = actor.actor_channels.receive().unwrap();
            if message.is::<ShutdownActorMsg>() {
                let msg = message.downcast::<ShutdownActorMsg>().unwrap_or_else(|_| panic!("Could not downcast shutdown actor message!"));

                match *msg {
                    ShutdownActorMsg::TrajectoryMsg{ pose, ref_kf_id, timestamp } => {
                        actor.trajectory_poses.push(pose);
                        actor.trajectory_times.push(timestamp);
                        actor.trajectory_keyframes.push(ref_kf_id);
                    },
                    ShutdownActorMsg::TrackingStateMsg{ state } => {
                        match state {
                            TrackingState::Lost => {
                                // This can happen if tracking is lost. Duplicate last element of each vector
                                actor.trajectory_poses.push(actor.trajectory_poses.last().unwrap().clone());
                                actor.trajectory_times.push(actor.trajectory_times.last().unwrap().clone());
                                actor.trajectory_keyframes.push(actor.trajectory_keyframes.last().unwrap().clone());
                            },
                            _ => {}
                        }
                    },
                };
            } else if message.is::<ShutdownMsg>() {
                warn!("Triggered shutdown, saving trajectory info");
                let mut file = File::create(
                    Path::new(&actor.results_folder)
                    .join(&actor.trajectory_filename)
                ).unwrap();
                for i in 0..actor.trajectory_poses.len() {
                    let string = format!("{:?} {:?}", actor.trajectory_times[i], actor.trajectory_poses[i]);
                    file.write_all(string.as_bytes()).unwrap();
                }

                for (_, actor_tx) in &actor.actor_channels.actors {
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