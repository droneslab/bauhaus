use std::{any::Any, fs::File, path::Path, io::Write};

use chrono::{DateTime, Utc};
use dvcore::{base::{ActorChannels, Actor}, config::{SYSTEM_SETTINGS, GLOBAL_PARAMS}};
use log::warn;

use crate::{dvmap::{pose::Pose, map::Id}, RESULTS_FOLDER};

use super::messages::{ShutdownMessage, TrajectoryMessage};


pub struct ShutdownActor {
    actor_channels: ActorChannels,
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
    trajectory_times: Vec<DateTime<Utc>>, //mlFrameTimes
    trajectory_keyframes: Vec<Id>, //mlpReferences
}

impl ShutdownActor {
    pub fn new(actor_channels: ActorChannels) -> ShutdownActor {
        ShutdownActor{
            actor_channels,
            trajectory_poses: Vec::new(),
            trajectory_times: Vec::new(),
            trajectory_keyframes: Vec::new()
        }
    }
}
impl Actor for ShutdownActor {
    fn run(&mut self) {
        loop {
            let message = self.actor_channels.receive().unwrap();
            if let Some(_) = message.downcast_ref::<ShutdownMessage>() {
                warn!("Triggered shutdown, saving trajectory info");
                let mut file = File::create(
                    Path::new(RESULTS_FOLDER)
                    .join(GLOBAL_PARAMS.get::<String>(SYSTEM_SETTINGS, "trajectory_file_name"))
                ).unwrap();
                for i in 0..self.trajectory_poses.len() {
                    let string = format!("{:?} {:?}", self.trajectory_times[i], self.trajectory_poses[i]);
                    file.write_all(string.as_bytes()).unwrap();
                }

                if GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "should_profile") {
                    flame::dump_html(File::create(
                        Path::new(RESULTS_FOLDER).join("flamegraph.html")
                    ).unwrap()).unwrap();
                }

                for (_, actor_tx) in &self.actor_channels.actors {
                    actor_tx.send(Box::new(ShutdownMessage{})).unwrap();
                }
            } else if let Some(msg) = message.downcast_ref::<TrajectoryMessage>() {
                match msg.pose {
                    Some(pose) => {
                        self.trajectory_poses.push(pose);
                        self.trajectory_times.push(msg.timestamp.unwrap());
                        self.trajectory_keyframes.push(msg.ref_kf_id.unwrap());
                    },
                    None => {
                        // This can happen if tracking is lost. Duplicate last element of each vector
                        if let Some(last) = self.trajectory_poses.last().cloned() { self.trajectory_poses.push(last); }
                        if let Some(last) = self.trajectory_times.last().cloned() { self.trajectory_times.push(last); }
                        if let Some(last) = self.trajectory_keyframes.last().cloned() { self.trajectory_keyframes.push(last); }
                    }
                };
            } else {
                warn!("Shutdown actor received unknown message type!");
            }
        }
    }
}