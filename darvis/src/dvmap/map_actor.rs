use axiom::{prelude::*, message::ActorMessage};
use log::{info, warn, debug};
use nalgebra::Vector3;
use crate::{
    lockwrap::ReadWriteWrapper,
    dvmap::{keyframe::*, map::*},
    modules::map_initialization::Initialization, actors::messages::{MapInitializedMsg, KeyFrameIdMsg}, registered_modules::{LOCAL_MAPPING, TRACKING_BACKEND},
};

use super::mappoint::{MapPoint, PrelimMapPoint};

pub static MAP_ACTOR: &str = "MAP_ACTOR"; 

pub struct MapActor {
    map: ReadWriteWrapper<Map>,
}
impl MapActor {//+ std::marker::Send + std::marker::Sync
    pub fn spawn (actor_system: &ActorSystem, map: ReadWriteWrapper<Map>) -> Aid {
        let mapactor = MapActor { map };
        let aid = actor_system.spawn().name(MAP_ACTOR).with(mapactor, MapActor::handle).unwrap();
        aid
    }

    async fn handle(self, context: Context, message: Message) -> ActorResult<Self> {
        if let Some(msg) = message.content_as::<MapWriteMsg>() {
            let mut write_lock = self.map.write();
            let msg = &*msg;

            match &msg.target {
                MapEditTarget::CreateInitialMapMonocular { initialization_data, callback_actor } => {
                    match write_lock.create_initial_map_monocular(&initialization_data) {
                            Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints)) => {
                                callback_actor.send_new(
                                    MapInitializedMsg {curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints}
                                ).unwrap();
                                info!("successfully created initial monocular map");
                            },
                            None => { }
                    };
                },

                MapEditTarget::MapPoint__New { mp ,observations_to_add } => {
                    let new_mp_id = write_lock.insert_mappoint_to_map(mp, observations_to_add);

                },

                MapEditTarget::MapPoint__Discard { id } => {
                    write_lock.discard_mappoint(&id);
                },

                MapEditTarget::MapPoint__IncreaseFound { mp_ids_and_nums } => {
                    for (mp, n) in mp_ids_and_nums {
                        write_lock.mappoints.get_mut(mp).unwrap().increase_found(n);
                    }
                },

                MapEditTarget::KeyFrame__New { kf, callback_actor } => {
                    let new_kf_id = write_lock.insert_keyframe_to_map(kf, false);
                    // Send the new keyframe ID directly back to the sender so they can use the ID 
                    callback_actor.send_new(KeyFrameIdMsg{keyframe_id: new_kf_id}).unwrap();
                },

                MapEditTarget::KeyFrame__Delete { id } => {
                    let new_kf_id = write_lock.discard_keyframe(id);
                },
                MapEditTarget::MapPoint__AddObservation {mp_id, kf_id, index} => {
                    let num_keypoints = write_lock.get_keyframe(kf_id).unwrap().features.num_keypoints;
                    write_lock.mappoints.get_mut(mp_id).unwrap().add_observation(&kf_id, num_keypoints, *index as u32);
                    write_lock.keyframes.get_mut(kf_id).unwrap().add_mappoint(*mp_id, *index as u32, false);
                },
                MapEditTarget::MapPoint__Replace {mp_to_replace, mp} => {
                    write_lock.replace_mappoint(mp_to_replace, mp);
                },
                _ => {
                    warn!("invalid message type to map actor");
                },
            }
        }

        Ok(Status::done(self))
    }
}

/// *** Message to send map actor an edit request *** ///
#[allow(non_camel_case_types)]
enum MapEditTarget {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    CreateInitialMapMonocular{initialization_data: Initialization, callback_actor: axiom::actors::Aid},
    CreateInitialMapStereo{initialization_data: Initialization, callback_actor: axiom::actors::Aid},
    KeyFrame__New{kf: KeyFrame<PrelimKeyFrame>, callback_actor: axiom::actors::Aid},
    KeyFrame__Delete{id: Id},
    Map__ResetActive(),
    MapPoint__New{mp: MapPoint<PrelimMapPoint>, observations_to_add: Vec<(Id, u32, usize)>},
    MapPoint__Position{ id: u64, pos: Vector3<f32> },
    MapPoint__Discard{id: Id},
    MapPoint__Replace{mp_to_replace: Id, mp: Id},
    MapPoint__IncreaseFound{mp_ids_and_nums: Vec::<(Id, i32)>},
    MapPoint__AddObservation{mp_id: Id, kf_id: Id, index: usize}
}
pub struct MapWriteMsg {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    target: MapEditTarget,
}
impl ActorMessage for MapWriteMsg { }

impl MapWriteMsg {
    pub fn create_initial_map_monocular(
        initialization_data: Initialization,
        callback_actor: axiom::actors::Aid
    ) -> MapWriteMsg {
        Self {
            target: MapEditTarget::CreateInitialMapMonocular{initialization_data, callback_actor },
        }
    }
    pub fn create_initial_map_stereo(
        initialization_data: Initialization,
        callback_actor: axiom::actors::Aid
    ) -> MapWriteMsg {
        Self {
            target: MapEditTarget::CreateInitialMapStereo{initialization_data, callback_actor},
        }
    }
    pub fn new_keyframe(kf: KeyFrame<PrelimKeyFrame>, callback_actor: axiom::actors::Aid) -> MapWriteMsg {
        Self {
            target: MapEditTarget::KeyFrame__New {kf, callback_actor},
        }
    }
    pub fn delete_keyframe(id: Id) -> MapWriteMsg {
        Self {
            target: MapEditTarget::KeyFrame__Delete { id }
        }
    }
    pub fn reset_active_map() -> MapWriteMsg {
        Self {
            target: MapEditTarget::Map__ResetActive(),
        }
    }
    pub fn create_new_mappoint(mp: MapPoint<PrelimMapPoint>, observations_to_add: Vec<(Id, u32, usize)>) -> MapWriteMsg {
        Self {
            target: MapEditTarget::MapPoint__New {mp, observations_to_add},
        }
    }
    pub fn update_mappoint_position(kf_id: u64, pos : &Vector3<f32>) -> MapWriteMsg {
        Self {
            target: MapEditTarget::MapPoint__Position {id :kf_id, pos: pos.clone()},
        }
    }
    pub fn discard_mappoint(mp_id: &Id) -> MapWriteMsg {
        Self {
            target: MapEditTarget::MapPoint__Discard {id : mp_id.clone()},
        }
    }
    pub fn increase_found(mp_ids_and_nums: Vec<(Id, i32)>) -> MapWriteMsg {
        Self {
            target: MapEditTarget::MapPoint__IncreaseFound { mp_ids_and_nums },
        }
    }
    pub fn replace_mappoint(mp_to_replace: Id, mp: Id) -> MapWriteMsg {
        Self {
            target: MapEditTarget::MapPoint__Replace{mp_to_replace, mp},
        }
    }
    pub fn add_observation(mp_id: Id, kf_id: Id, index: usize) -> MapWriteMsg {
        Self {
            target: MapEditTarget::MapPoint__AddObservation{mp_id, kf_id, index}
        }
    }
}
