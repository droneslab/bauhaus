use axiom::prelude::*;
use nalgebra::Vector3;
use serde::{Serialize, Deserialize};
use crate::{
    lockwrap::ReadWriteWrapper,
    map::{map::Map, keyframe::KeyFrame}
};

use super::map::Id;

pub static MAP_ACTOR: &str = "MAP_ACTOR"; 

pub struct MapActor {
    map: ReadWriteWrapper<Map>
}
impl MapActor {
    pub fn spawn(map: ReadWriteWrapper<Map>) -> Aid {
        let mapactor = MapActor { map: map };
        let system = ActorSystem::create(ActorSystemConfig::default());  
        let aid = system.spawn().name("MAPACTOR").with(mapactor, MapActor::handle).unwrap();
        aid.clone()
    }

    async fn handle(self, _context: Context, message: Message) -> ActorResult<Self> {
        if let Some(_msg) = message.content_as::<MapWriteMsg>() {
            println!("received map edit msg");


            // if let MapEditTarget::KeyFrame_New(target) = msg.target {
            //     let mut write_lock = self.map.write();
            //     write_lock.insert_kf(target, msg.id);
            // }

            // write_lock.insert_kf(msg.)
            match _msg.target{
                MapEditTarget::MapPoint__Discard { id } => {
                    let mut write_lock = self.map.write();
                    write_lock.discard_mappoint(&id);
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
#[derive(Debug, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
enum MapEditTarget {
    KeyFrame__New(),
    Map__ResetActive(),
    // KeyFrame__Delete(),
    // KeyFrame__Pose(Pose),
    // KeyFrame__MapPoints(Vec<MapPoint>),
    // KeyFrame__BoW(abow::BoW),
    MapPoint__Position{ id: u64, pos: Vector3<f32> }, //na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>),
    MapPoint__Discard{id: Id},
    // MapPoint__KeyFrameRef(Vec<KeyFrame>),
    // MapPoint__KeyFrameList(Vec<KeyFrame>),
    // test(i32)
    // State(State)
}
#[derive(Debug, Serialize, Deserialize)]
pub struct MapWriteMsg {
    target: MapEditTarget,
}
// TODO: Finalize format for map write messages
impl MapWriteMsg {
    pub fn new_keyframe(_kf_id: u64, _kf: Box<KeyFrame>) -> Self {
        Self {
            target: MapEditTarget::KeyFrame__New(),
        }
    }

    pub fn reset_active_map() -> Self {
        Self {
            target: MapEditTarget::Map__ResetActive(),
        }
    }

    pub fn update_mappoint_position(kf_id: u64, pos : &Vector3<f32>) -> Self {
        Self {
            target: MapEditTarget::MapPoint__Position {id :kf_id, pos: pos.clone()},
        }
    }

    pub fn discard_mappoint(mp_id: &Id) -> Self {
        Self {
            target: MapEditTarget::MapPoint__Discard {id : mp_id.clone()},
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
