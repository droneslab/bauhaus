use std::{marker::PhantomData, collections::HashSet};
use axiom::{prelude::*, message::ActorMessage};
use log::{info, warn};
use nalgebra::Vector3;
use crate::{
    lockwrap::ReadWriteWrapper,
    dvmap::{keyframe::*, map::*, sensor::SensorType},
    modules::map_initialization::Initialization,
};

use super::pose::Pose;

pub static MAP_ACTOR: &str = "MAP_ACTOR"; 

pub struct MapActor<S: SensorType> {
    map: ReadWriteWrapper<Map<S>>,
}
impl<S: SensorType + 'static> MapActor<S> {//+ std::marker::Send + std::marker::Sync
    pub fn spawn<S2: SensorType + 'static> (map: ReadWriteWrapper<Map<S2>>) -> Aid {
        let mapactor = MapActor { map: map };
        let system = ActorSystem::create(ActorSystemConfig::default());  
        let aid = system.spawn().name("MAPACTOR").with(mapactor, MapActor::<S2>::handle).unwrap();
        aid.clone()
    }

    async fn handle(self, _context: Context, message: Message) -> ActorResult<Self> {
        if let Some(_msg) = message.content_as::<MapWriteMsg<S>>() {
            info!("map_actor::handle;received map edit msg");
            let mut write_lock = self.map.write();

            match &_msg.target{

                MapEditTarget::CreateInitialMapMonocular { initialization_data, tracking_actor } => {
                    match write_lock.create_initial_map_monocular(initialization_data) {
                            Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints)) => {
                                let tracking_msg = MapInitializedMsg {curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints};
                                tracking_actor.send_new(tracking_msg).unwrap();
                                info!("map::create_initial_map_monocular;success");
                            },
                            None => { }
                    };
                },

                MapEditTarget::MapPoint__Discard { id } => {
                    write_lock.discard_mappoint(&id);
                },
                MapEditTarget::MapPoint_IncreaseFound { mp_ids_and_nums } => {
                    let mut write_lock = self.map.write();
                    for (mp, n) in mp_ids_and_nums {
                        write_lock.increase_mappoint_found(mp, n);
                    }
                },
                _ => {
                    warn!("map_actor::handle;invalid message type to map actor");
                },
                MapEditTarget::KeyFrame__New { kf } => {
                    // Note: called by local mapping
                    // let mut write_lock = self.map.write();
                    // write_lock.new_keyframe(*kf);
                },
            }
        }

        Ok(Status::done(self))
    }
}

/// *** Message to send map actor an edit request *** ///
#[allow(non_camel_case_types)]
enum MapEditTarget<S: SensorType> {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    CreateInitialMapMonocular{initialization_data: Initialization<S>, tracking_actor: axiom::actors::Aid},
    KeyFrame__New{kf: KeyFrame<PrelimKeyFrame, S>},
    Map__ResetActive(),
    MapPoint__Position{ id: u64, pos: Vector3<f32> },
    MapPoint__Discard{id: Id},
    MapPoint_IncreaseFound{mp_ids_and_nums: Vec::<(Id, i32)>},
}
pub struct MapWriteMsg<S: SensorType> {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    target: MapEditTarget<S>,
    _pd: PhantomData<S> // Note: Don't remove, needed so MapWriteMsg can be generic on SensorType
}
impl<S: SensorType + std::marker::Send + 'static> ActorMessage for MapWriteMsg<S> { }

impl<S: SensorType + std::marker::Send + 'static> MapWriteMsg<S> {
    pub fn create_initial_map_monocular(
        initialization_data: Initialization<S>,
        tracking_actor: axiom::actors::Aid
    ) -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::CreateInitialMapMonocular{initialization_data, tracking_actor},
            _pd: PhantomData,
        }
    }
    pub fn new_keyframe(kf: KeyFrame<PrelimKeyFrame, S>) -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::KeyFrame__New {kf: kf},
            _pd: PhantomData,
        }
    }
    pub fn reset_active_map() -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::Map__ResetActive(),
            _pd: PhantomData,
        }
    }
    pub fn update_mappoint_position(kf_id: u64, pos : &Vector3<f32>) -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::MapPoint__Position {id :kf_id, pos: pos.clone()},
            _pd: PhantomData,
        }
    }
    pub fn discard_mappoint(mp_id: &Id) -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::MapPoint__Discard {id : mp_id.clone()},
            _pd: PhantomData,
        }
    }
    pub fn increase_found(mp_ids_and_nums: Vec<(Id, i32)>) -> MapWriteMsg<S>
    {
        Self {
            target: MapEditTarget::MapPoint_IncreaseFound { mp_ids_and_nums },
            _pd: PhantomData,
        }
    }
}

// Message that map sends to tracking letting it know that map has been initialized
#[derive(Debug)]
pub struct MapInitializedMsg {
    pub curr_kf_pose: Pose,
    pub curr_kf_id: Id,
    pub ini_kf_id: Id,
    pub local_mappoints: HashSet<Id>,
}
impl ActorMessage for MapInitializedMsg { }
