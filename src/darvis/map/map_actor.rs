use axiom::prelude::*;
use serde::{Serialize, Deserialize};
use crate::{
    lockwrap::ReadWriteWrapper,
    map::{
        keyframe::KeyFrame, map::Map, map::Id, pose::Pose
    }
};

pub static MAP_ACTOR: &str = "MAP_ACTOR"; 

pub struct MapActor {
    _map: ReadWriteWrapper<Map>
}
impl MapActor {
    pub fn spawn(map: ReadWriteWrapper<Map>) -> Aid {
        let mapactor = MapActor { _map: map };
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
    Frame__Pose(Id, Pose),
    Frame__MapPoint(Id, Id, bool),
    // KeyFrame__MapPoints(Vec<MapPoint>),
    // KeyFrame__BoW(abow::BoW),
    // MapPoint__Position(na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>),
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

    pub fn set_pose(frame_id: Id, pose: Pose) -> Self {
        Self {
            target: MapEditTarget::Frame__Pose(frame_id, pose)
        }
    }
    
    pub fn delete_mappoint_match(frame_id: Id, mappoint_id: Id, is_outlier: bool) -> Self {
        Self {
            target: MapEditTarget::Frame__MapPoint(frame_id, mappoint_id, is_outlier)
        }
        // Sofiya: 
        // delete mappoint in frame's mappoint list
        // if is_outlier is true, should also delete the mappoint in mappoint_outliers
        // set mappoint's last_frame_seen to the frame ID
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
