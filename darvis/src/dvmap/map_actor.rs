use axiom::{prelude::*, message::ActorMessage};
use log::{info, warn, debug};
use nalgebra::Vector3;
use crate::{
    lockwrap::ReadWriteWrapper,
    dvmap::{keyframe::*, map::*},
    modules::map_initialization::Initialization, actors::messages::MapInitializedMsg,
};

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

    async fn handle(self, _context: Context, message: Message) -> ActorResult<Self> {
        if let Some(msg) = message.content_as::<MapWriteMsg>() {
            debug!("map_actor::handle;received map edit msg");
            let mut write_lock = self.map.write();
            let msg = &*msg;

            match &msg.target {
                MapEditTarget::CreateInitialMapMonocular { initialization_data, tracking_actor } => {
                    match write_lock.create_initial_map_monocular(&initialization_data) {
                            Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints)) => {
                                let tracking_msg = MapInitializedMsg {curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints};
                                tracking_actor.send_new(tracking_msg).unwrap();
                                info!("successfully created initial monocular map");
                            },
                            None => { }
                    };
                },

                MapEditTarget::MapPoint__Discard { id } => {
                    write_lock.discard_mappoint(&id);
                },

                MapEditTarget::MapPoint_IncreaseFound { mp_ids_and_nums } => {
                    for (mp, n) in mp_ids_and_nums {
                        write_lock.increase_mappoint_found(&mp, &n);
                    }
                },

                MapEditTarget::KeyFrame__New { kf } => {
                    // Note: called by local mapping
                    write_lock.insert_keyframe_to_map(kf);
                },

                _ => {
                    warn!("invalid message type to map actor");
                },
            }
        }
        else if let Some(msg) = message.content_as::<KeyframeDatabaseWriteMsg>() {
            debug!("map_actor::handle;received map edit msg");
            let mut write_lock = self.map.write();
            let msg = &*msg;
            match &msg.target {


                KeyframeDatabaseEditTarget::KeyFrame__add { kf_id } => {
                    // Note: called by loop closing
                    write_lock.add_keyframe_to_database(kf_id);
                },
                KeyframeDatabaseEditTarget::KeyFrame__erase { kf_id } => {
                    write_lock.erase_keyframe_from_database(kf_id);
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
    CreateInitialMapMonocular{initialization_data: Initialization, tracking_actor: axiom::actors::Aid},
    CreateInitialMapStereo{initialization_data: Initialization, tracking_actor: axiom::actors::Aid},
    KeyFrame__New{kf: KeyFrame<PrelimKeyFrame>},
    Map__ResetActive(),
    MapPoint__Position{ id: u64, pos: Vector3<f32> },
    MapPoint__Discard{id: Id},
    MapPoint_IncreaseFound{mp_ids_and_nums: Vec::<(Id, i32)>},
}
pub struct MapWriteMsg {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    target: MapEditTarget,
}
impl ActorMessage for MapWriteMsg { }

impl MapWriteMsg {
    pub fn create_initial_map_monocular(
        initialization_data: Initialization,
        tracking_actor: axiom::actors::Aid
    ) -> MapWriteMsg {
        Self {
            target: MapEditTarget::CreateInitialMapMonocular{initialization_data, tracking_actor},
        }
    }
    pub fn create_initial_map_stereo(
        initialization_data: Initialization,
        tracking_actor: axiom::actors::Aid
    ) -> MapWriteMsg {
        Self {
            target: MapEditTarget::CreateInitialMapStereo{initialization_data, tracking_actor},
        }
    }
    pub fn new_keyframe(kf: KeyFrame<PrelimKeyFrame>) -> MapWriteMsg {
        Self {
            target: MapEditTarget::KeyFrame__New {kf: kf},
        }
    }
    pub fn reset_active_map() -> MapWriteMsg {
        Self {
            target: MapEditTarget::Map__ResetActive(),
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
            target: MapEditTarget::MapPoint_IncreaseFound { mp_ids_and_nums },
        }
    }
}


enum KeyframeDatabaseEditTarget {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    KeyFrame__add{kf_id: Id},
    KeyFrame__erase{kf_id: Id},
}

pub struct KeyframeDatabaseWriteMsg {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    target: KeyframeDatabaseEditTarget,
}
impl ActorMessage for KeyframeDatabaseWriteMsg { }

impl KeyframeDatabaseWriteMsg {

    pub fn add(kf_id: Id) -> KeyframeDatabaseWriteMsg {
        Self {
            target: KeyframeDatabaseEditTarget::KeyFrame__add {kf_id: kf_id},
        }
    }

    pub fn erase(kf_id: Id) -> KeyframeDatabaseWriteMsg {
        Self {
            target: KeyframeDatabaseEditTarget::KeyFrame__erase {kf_id: kf_id},
        }
    }
}
