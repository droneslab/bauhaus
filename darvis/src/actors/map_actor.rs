use dvcore::{base::{ActorChannels, ActorMessage, Actor}};
use log::{info, warn};

use crate::{
    lockwrap::ReadWriteWrapper,
    dvmap::{keyframe::*, map::*, mappoint::{MapPoint, PrelimMapPoint}, pose::DVPose},
    modules::map_initialization::Initialization, actors::messages::{MapInitializedMsg, KeyFrameIdMsg, LastKeyFrameUpdatedMsg},
};

use super::messages::ShutdownMsg;

pub struct MapActor {
    actor_system: ActorChannels,
    map: ReadWriteWrapper<Map>,
}
impl Actor for MapActor {
    fn run(&mut self) {
        loop {
            let message = self.actor_system.receive().unwrap();
            if let Some(msg) = message.downcast_ref::<MapWriteMsg>() {
                let msg = &*msg;
                let mut write_lock = self.map.write();

                match &msg.target {
                    MapWriteTarget::CreateInitialMapMonocular { initialization_data, callback_actor } => {
                        match write_lock.create_initial_map_monocular(&initialization_data) {
                                Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints)) => {
                                    self.actor_system.find(callback_actor).unwrap().send(Box::new(
                                        MapInitializedMsg {curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints}
                                    )).unwrap();
                                    info!("successfully created initial monocular map");
                                },
                                None => { 
                                    warn!("Could not create initial map");
                                }
                        };
                    },
                    MapWriteTarget::MapPoint__NewMany { mps, callback_actor } => {
                        for (mp, observations) in mps {
                            let _ = write_lock.insert_mappoint_to_map(mp, observations);
                        }
                        // Inform tracking that we are done with creating new mappoints. Needed because tracking fails if new points are not created
                        // before track_with_reference_keyframe, so to avoid the race condition we have it wait until the new points are ready.
                        self.actor_system.find(callback_actor).unwrap().send(Box::new(LastKeyFrameUpdatedMsg{})).unwrap();
                    },
                    MapWriteTarget::MapPoint__DiscardMany { ids } => {
                        for id in ids {
                            write_lock.discard_mappoint(&id);
                        }
                    },

                    MapWriteTarget::MapPoint__IncreaseFound { mp_ids_and_nums } => {
                        for (mp, n) in mp_ids_and_nums {
                            write_lock.mappoints.get_mut(mp).unwrap().increase_found(n);
                        }
                    },
                    MapWriteTarget::MapPoint__IncreaseVisible {mp_ids} => {
                        for mp_id in mp_ids {
                            write_lock.mappoints.get_mut(mp_id).unwrap().increase_visible();
                        }
                    },
                    MapWriteTarget::KeyFrame__New { kf, callback_actor } => {
                        let new_kf_id = write_lock.insert_keyframe_to_map(kf, false);
                        // Send the new keyframe ID directly back to the sender so they can use the ID 
                        self.actor_system.find(callback_actor).unwrap().send(Box::new(KeyFrameIdMsg{keyframe_id: new_kf_id})).unwrap();
                    },

                    MapWriteTarget::KeyFrame__Delete { id } => {
                        let _ = write_lock.discard_keyframe(id);
                    },
                    MapWriteTarget::MapPoint__AddObservation {mp_id, kf_id, index} => {
                        let num_keypoints = write_lock.get_keyframe(kf_id).unwrap().features.num_keypoints;
                        write_lock.mappoints.get_mut(mp_id).unwrap().add_observation(&kf_id, num_keypoints, *index as u32);
                        write_lock.keyframes.get_mut(kf_id).unwrap().add_mappoint(*index as u32, *mp_id, false);
                    },
                    MapWriteTarget::MapPoint__Replace {mp_to_replace, mp} => {
                        write_lock.replace_mappoint(mp_to_replace, mp);
                    },
                    MapWriteTarget::MapPoint__Pose{mp_id, pose} => {
                        write_lock.mappoints.get_mut(mp_id).unwrap().position = pose.get_translation();
                    }
                    _ => {
                        warn!("Map Actor received unknown target! Contents: {:?}", msg.target);
                    },
                }
            } else if let Some(_) = message.downcast_ref::<ShutdownMsg>() {
                break;
            } else {
                warn!("Map Actor received unknown message type!");
            }
        }
    }
}

impl MapActor {//+ std::marker::Send + std::marker::Sync
    pub fn new (actor_system: ActorChannels, map: ReadWriteWrapper<Map>) -> Self {
        MapActor {
            actor_system,
            map
        }
    }
}

/// *** Message to send map actor an edit request *** ///
#[allow(non_camel_case_types)]
#[derive(Debug)]
enum MapWriteTarget {
    //Note: Tried this out as a way for actors to send a bunch of messages at once without 
    // having to send at a ridiculously high rate, but this doesn't work because (I think)
    // each message needs a write lock and the actor is handling them asynchronously
    // BulkMsg{msgs: Vec<MapWriteMsg>}

    CreateInitialMapMonocular{initialization_data: Initialization, callback_actor: String},
    CreateInitialMapStereo{initialization_data: Initialization, callback_actor: String},
    KeyFrame__New{kf: Frame<PrelimKeyFrame>, callback_actor: String},
    KeyFrame__Delete{id: Id},
    KeyFrame__Pose{kf_id: Id, pose: DVPose},
    Map__ResetActive{},
    MapPoint__NewMany{mps: Vec<(MapPoint<PrelimMapPoint>, Vec<(Id, u32, usize)>)>, callback_actor: String},
    MapPoint__DiscardMany{ids: Vec<Id>},
    MapPoint__Replace{mp_to_replace: Id, mp: Id},
    MapPoint__IncreaseFound{mp_ids_and_nums: Vec::<(Id, i32)>},
    MapPoint__IncreaseVisible{mp_ids: Vec<Id>},
    MapPoint__AddObservation{mp_id: Id, kf_id: Id, index: usize},
    MapPoint__Pose{mp_id: Id, pose: DVPose},
}
#[derive(Debug)]
pub struct MapWriteMsg {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    target: MapWriteTarget,
}
impl ActorMessage for MapWriteMsg { }

impl MapWriteMsg {
    // pub fn create_bulk_message(msgs: Vec<MapWriteMsg>) -> Self {
    //     Self {
    //         target: MapWriteTarget::BulkMsg{msgs}
    //     }
    // }
    pub fn create_initial_map_monocular(
        initialization_data: Initialization,
        callback_actor: &str
    ) -> Self {
        Self {
            target: MapWriteTarget::CreateInitialMapMonocular{initialization_data, callback_actor: callback_actor.to_string() },
        }
    }
    pub fn create_initial_map_stereo(
        initialization_data: Initialization,
        callback_actor: &str
    ) -> Self {
        Self {
            target: MapWriteTarget::CreateInitialMapStereo{initialization_data, callback_actor: callback_actor.to_string()},
        }
    }
    pub fn new_keyframe(kf: Frame<PrelimKeyFrame>, callback_actor: &str) -> Self {
        Self {
            target: MapWriteTarget::KeyFrame__New {kf, callback_actor: callback_actor.to_string()},
        }
    }
    pub fn delete_keyframe(id: Id) -> Self {
        Self {
            target: MapWriteTarget::KeyFrame__Delete { id }
        }
    }
    pub fn edit_keyframe_pose(kf_id: Id, pose: DVPose) -> Self {
        Self {
            target: MapWriteTarget::KeyFrame__Pose {kf_id, pose}
        }
    }
    pub fn reset_active_map() -> Self {
        Self {
            target: MapWriteTarget::Map__ResetActive{},
        }
    }
    pub fn create_many_mappoints(mps: Vec<(MapPoint<PrelimMapPoint>, Vec<(Id, u32, usize)>)>, callback_actor: &str) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__NewMany {mps, callback_actor: callback_actor.to_string()},
        }
    }
    pub fn discard_many_mappoints(ids: &Vec<Id>) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__DiscardMany {ids : ids.clone()},
        }
    }
    pub fn increase_found(mp_ids_and_nums: Vec<(Id, i32)>) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__IncreaseFound { mp_ids_and_nums },
        }
    }
    pub fn increase_visible(mp_ids: Vec<Id>) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__IncreaseVisible { mp_ids },
        }
    }
    pub fn replace_mappoint(mp_to_replace: Id, mp: Id) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__Replace{mp_to_replace, mp},
        }
    }
    pub fn add_observation(mp_id: Id, kf_id: Id, index: usize) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__AddObservation{mp_id, kf_id, index}
        }
    }
    pub fn edit_mappoint_pose(mp_id: Id, pose: DVPose) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__Pose {mp_id, pose}
        }
    }
}
