use std::{marker::PhantomData, collections::HashMap};
use axiom::{prelude::*, message::ActorMessage};
use nalgebra::Vector3;
use serde::{Serialize, Deserialize};
use crate::{
    lockwrap::ReadWriteWrapper,
    dvmap::{keyframe::*, map::*, pose::*, sensor::SensorType}, modules::tracking_backend::Initialization,
};

use super::frame::Frame;

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
            println!("received map edit msg");
            let mut write_lock = self.map.write();

            match &_msg.target{
                MapEditTarget::CreateInitialMapMonocular { initialization_data } => {
                    write_lock.create_initial_map_monocular(initialization_data);
                },

                MapEditTarget::MapPoint__Discard { id } => {
                    write_lock.discard_mappoint(&id);
                },
                MapEditTarget::MapPoint_IncreaseFound { id, n } => {
                    let mut write_lock = self.map.write();
                    write_lock.increase_found(&id, *n);
                },
                _ => {
                    println!("Invalid Message type:");
                },
                MapEditTarget::KeyFrame__New { kf } => {
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
    CreateInitialMapMonocular{initialization_data: Initialization<S>},
    KeyFrame__New{kf: KeyFrame<S>},
    Map__ResetActive(),
    Frame__Pose{frame_id: Id, pose: Pose},
    MapPoint__Position{ id: u64, pos: Vector3<f32> },
    MapPoint__Discard{id: Id},
    MapPoint_IncreaseFound{id : Id, n : i32},
}
pub struct MapWriteMsg<S: SensorType> {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    target: MapEditTarget<S>,
    _pd: PhantomData<S> // Note: Don't remove, needed so MapWriteMsg can be generic on SensorType
}
impl<S: SensorType + std::marker::Send + 'static> ActorMessage for MapWriteMsg<S> { }

impl<S: SensorType + std::marker::Send + 'static> MapWriteMsg<S> {
    pub fn create_initial_map_monocular(initialization_data: Initialization<S>) -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::CreateInitialMapMonocular{initialization_data},
            _pd: PhantomData,
        }
    }
    pub fn new_keyframe(kf: KeyFrame<S>) -> MapWriteMsg<S> {
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
    pub fn set_pose(frame_id: Id, pose: Pose) -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::Frame__Pose{frame_id: frame_id, pose: pose},
            _pd: PhantomData,
        }
    }
    pub fn increase_found(mp_id: &Id, n : i32) -> MapWriteMsg<S>
    {
        Self {
            target: MapEditTarget::MapPoint_IncreaseFound {id : mp_id.clone(), n : n},
            _pd: PhantomData,
        }
    }
    // pub fn delete_keyframe(kf_id: u64) -> Self {
    //     Self {
    //         target: MapEditTarget::KeyFrame_Delete(),
    //         id: kf_id
    //     }
    // }

    // pub fn update_keyframe_pose(kf_id: u64, pose: Pose) -> Self {
    //     Self {
    //         target: MapEditTarget::KeyFrame_Pose(pose),
    //         id: kf_id
    //     }
    // }
}

pub struct MapInitializationResultMsg {
    success: bool,
}
impl ActorMessage for MapInitializationResultMsg { }