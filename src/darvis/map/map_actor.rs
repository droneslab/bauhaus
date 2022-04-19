use axiom::prelude::*;
use serde::{Serialize, Deserialize};
use crate::{
    lockwrap::ReadWriteWrapper,
    map::{map::Map, pose::Pose, keyframe::KeyFrame}
};

pub static MAP_ACTOR: &str = "MAP_ACTOR"; 

pub struct MapActor {
    map: ReadWriteWrapper<Map>
}
impl MapActor {
    pub fn spawn(map: ReadWriteWrapper<Map>) -> Aid {
        let mapactor = MapActor { map };
        let system = ActorSystem::create(ActorSystemConfig::default());  
        let aid = system.spawn().name("MAPACTOR").with(mapactor, MapActor::handle).unwrap();
        aid.clone()
    }

    async fn handle(self, _context: Context, message: Message) -> ActorResult<Self> {
        if let Some(msg) = message.content_as::<MapWriteMsg>() {
            println!("received map edit msg");


            if let MapEditTarget::KeyFrame_New(target) = msg.target {
                let mut write_lock = self.map.write();
                write_lock.insert_kf(target, msg.id);
            }

            // write_lock.insert_kf(msg.)
        }

        Ok(Status::done(self))
    }
}

/// *** Message to send map actor an edit request *** ///
#[derive(Debug, Serialize, Deserialize, Clone)]
#[allow(non_camel_case_types)]
enum MapEditTarget {
    KeyFrame_New(KeyFrame),
    KeyFrame_Delete(),
    KeyFrame_Pose(Pose),
    // KeyFrame_MapPoints(Vec<MapPoint>),
    // KeyFrame_BoW(abow::BoW),
    // MapPoint_Position(na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>),
    // MapPoint_KeyFrameRef(Vec<KeyFrame>),
    // MapPoint_KeyFrameList(Vec<KeyFrame>),
    // test(i32)
}
#[derive(Debug, Serialize, Deserialize)]
pub struct MapWriteMsg {
    target: MapEditTarget,
    id: u64,
}
impl MapWriteMsg {
    pub fn new_keyframe(kf_id: u64, kf: KeyFrame) -> Self {
        Self {
            target: MapEditTarget::KeyFrame_New(kf),
            id: kf_id
        }
    }

    pub fn delete_keyframe(kf_id: u64) -> Self {
        Self {
            target: MapEditTarget::KeyFrame_Delete(),
            id: kf_id
        }
    }

    pub fn update_keyframe_pose(kf_id: u64, pose: Pose) -> Self {
        Self {
            target: MapEditTarget::KeyFrame_Pose(pose),
            id: kf_id
        }
    }
}
