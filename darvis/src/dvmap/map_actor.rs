use axiom::{prelude::*, message::ActorMessage};
use dvcore::matrix::DVVector3;
use log::{info, warn, debug};

use crate::{
    lockwrap::ReadWriteWrapper,
    dvmap::{keyframe::*, map::*},
    modules::map_initialization::Initialization, actors::messages::{MapInitializedMsg, KeyFrameIdMsg, LastKeyFrameUpdatedMsg},
};
use super::{mappoint::{MapPoint, PrelimMapPoint}, pose::Pose};

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
            let msg = &*msg;
            let mut write_lock = self.map.write();

            match &msg.target {
                MapWriteTarget::CreateInitialMapMonocular { initialization_data, callback_actor } => {
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

                MapWriteTarget::MapPoint__New { mp ,observations_to_add } => {
                    let _ = write_lock.insert_mappoint_to_map(mp, observations_to_add);
                },
                MapWriteTarget::MapPoint__NewMany { mps, callback_actor } => {
                    for (mp, observations) in mps {
                        let _ = write_lock.insert_mappoint_to_map(mp, observations);
                    }
                    // Inform tracking that we are done with creating new mappoints. Needed because tracking fails if new points are not created
                    // before track_with_reference_keyframe, so to avoid the race condition we have it wait until the new points are ready.
                    callback_actor.send_new(LastKeyFrameUpdatedMsg{}).unwrap();
                    debug!("Local mapping finished creating {} new mappoints", mps.len());
                },
                MapWriteTarget::MapPoint__Discard { id } => {
                    write_lock.discard_mappoint(&id);
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
                    callback_actor.send_new(KeyFrameIdMsg{keyframe_id: new_kf_id}).unwrap();
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
                    warn!("invalid message type to map actor");
                },
            }
        }

        Ok(Status::done(self))
    }
}

/// *** Message to send map actor an edit request *** ///
#[allow(non_camel_case_types)]
enum MapWriteTarget {
    // #[serde(bound = "")] If using serialize/deserialize, uncomment this
    // BulkMsg{msgs: Vec<MapWriteMsg>}, Note: Tried this out as a way for actors to send a bunch of messages at once without having to send at a ridiculously high rate, but this doesn't work because (I think) each message needs a write lock and the actor is handling them asynchronously
    CreateInitialMapMonocular{initialization_data: Initialization, callback_actor: axiom::actors::Aid},
    CreateInitialMapStereo{initialization_data: Initialization, callback_actor: axiom::actors::Aid},
    KeyFrame__New{kf: Frame<PrelimKeyFrame>, callback_actor: axiom::actors::Aid},
    KeyFrame__Delete{id: Id},
    KeyFrame__Pose{kf_id: Id, pose: Pose},
    Map__ResetActive{},
    MapPoint__New{mp: MapPoint<PrelimMapPoint>, observations_to_add: Vec<(Id, u32, usize)>},
    MapPoint__NewMany{mps: Vec<(MapPoint<PrelimMapPoint>, Vec<(Id, u32, usize)>)>, callback_actor: axiom::actors::Aid},
    MapPoint__Discard{id: Id},
    MapPoint__Replace{mp_to_replace: Id, mp: Id},
    MapPoint__IncreaseFound{mp_ids_and_nums: Vec::<(Id, i32)>},
    MapPoint__IncreaseVisible{mp_ids: Vec<Id>},
    MapPoint__AddObservation{mp_id: Id, kf_id: Id, index: usize},
    MapPoint__Pose{mp_id: Id, pose: Pose},
}
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
        callback_actor: axiom::actors::Aid
    ) -> Self {
        Self {
            target: MapWriteTarget::CreateInitialMapMonocular{initialization_data, callback_actor },
        }
    }
    pub fn create_initial_map_stereo(
        initialization_data: Initialization,
        callback_actor: axiom::actors::Aid
    ) -> Self {
        Self {
            target: MapWriteTarget::CreateInitialMapStereo{initialization_data, callback_actor},
        }
    }
    pub fn new_keyframe(kf: Frame<PrelimKeyFrame>, callback_actor: axiom::actors::Aid) -> Self {
        Self {
            target: MapWriteTarget::KeyFrame__New {kf, callback_actor},
        }
    }
    pub fn delete_keyframe(id: Id) -> Self {
        Self {
            target: MapWriteTarget::KeyFrame__Delete { id }
        }
    }
    pub fn edit_keyframe_pose(kf_id: Id, pose: Pose) -> Self {
        Self {
            target: MapWriteTarget::KeyFrame__Pose {kf_id, pose}
        }
    }
    pub fn reset_active_map() -> Self {
        Self {
            target: MapWriteTarget::Map__ResetActive{},
        }
    }
    pub fn create_new_mappoint(mp: MapPoint<PrelimMapPoint>, observations_to_add: Vec<(Id, u32, usize)>) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__New {mp, observations_to_add},
        }
    }
    pub fn create_many_mappoints(mps: Vec<(MapPoint<PrelimMapPoint>, Vec<(Id, u32, usize)>)>, callback_actor: axiom::actors::Aid) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__NewMany {mps, callback_actor},
        }
    }
    pub fn discard_mappoint(mp_id: &Id) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__Discard {id : mp_id.clone()},
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
    pub fn edit_mappoint_pose(mp_id: Id, pose: Pose) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__Pose {mp_id, pose}
        }
    }
}
