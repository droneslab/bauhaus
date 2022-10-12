use std::marker::PhantomData;
use axiom::prelude::*;
use nalgebra::Vector3;
use serde::{Serialize, Deserialize};
use crate::{
    lockwrap::ReadWriteWrapper,
    map::{
        keyframe::KeyFrame, map::Map, map::Id, pose::Pose
    },
    utils::sensor::SensorType,
};

pub static MAP_ACTOR: &str = "MAP_ACTOR"; 

pub struct MapActor<S: SensorType> {
    map: ReadWriteWrapper<Map<S>>
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

            match _msg.target{
                MapEditTarget::MapPoint__Discard { id } => {
                    write_lock.discard_mappoint(&id);
                },
                MapEditTarget::Frame__DeleteMapPointMatch { frame_id, mp_id, is_outlier } => {
                    todo!("sofiya...move out of actor, probably safe to do in tracking");
                    // delete mappoint in frame's mappoint list
                    // if is_outlier is true, should also delete the mappoint in mappoint_outliers
                    // set mappoint's last_frame_seen to the frame ID
                },
                MapEditTarget::MapPoint_IncreaseFound { id, n } => {
                    let mut write_lock = self.map.write();
                    write_lock.increase_found(&id, n);
                },
                _ => {
                    println!("Invalid Message type:");
                },
            }

        }

        Ok(Status::done(self))
    }
}

/// *** Message to send map actor an edit request *** ///
#[derive(Serialize, Deserialize)]
#[allow(non_camel_case_types)]
enum MapEditTarget<S: SensorType> {
    #[serde(bound = "")]
    KeyFrame__New{kf: KeyFrame<S>},
    Map__ResetActive(),
    Frame__Pose{frame_id: Id, pose: Pose},
    Frame__DeleteMapPointMatch{frame_id: Id, mp_id: Id, is_outlier: bool},
    MapPoint__Position{ id: u64, pos: Vector3<f32> },
    MapPoint__Discard{id: Id},
    MapPoint_IncreaseFound{id : Id, n : i32},
    // MapPoint__KeyFrameRef(Vec<KeyFrame>),
    // MapPoint__KeyFrameList(Vec<KeyFrame>),
    // test(i32) 
    // State(State)
}
#[derive(Serialize, Deserialize)]
pub struct MapWriteMsg<S: SensorType> {
    #[serde(bound = "")]
    target: MapEditTarget<S>,
    _pd: PhantomData<S> // Don't use don't remove
}
impl<S: SensorType + std::marker::Send> MapWriteMsg<S> {
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

    pub fn delete_mappoint_match(frame_id: Id, mappoint_id: Id, is_outlier: bool) -> MapWriteMsg<S> {
        Self {
            target: MapEditTarget::Frame__DeleteMapPointMatch{
                frame_id: frame_id, mp_id: mappoint_id, is_outlier: is_outlier
            },
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
