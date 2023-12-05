use dvcore::{actor::{ActorChannels, ActorMessage, Actor}, matrix::DVVectorOfPoint3f};
use log::{info, warn};
use logging_timer::{timer, finish};

use crate::{
    maplock::ReadWriteMap,
    dvmap::{keyframe::*, map::*, mappoint::{MapPoint, PrelimMapPoint}, pose::DVPose},
    modules::{map_initialization::Initialization, optimizer::BundleAdjustmentResult}, actors::messages::{ShutdownMsg, KeyFrameIdMsg}, registered_actors::{TRACKING_BACKEND, LOCAL_MAPPING},
};

use super::messages::{MapInitializedMsg, LastKeyFrameUpdatedMsg};

pub struct MapActor {
    actor_system: ActorChannels,
    map: ReadWriteMap<Map>,
}

impl Actor for MapActor {
    type MapRef = ReadWriteMap<Map>;

    fn new_actorstate (actor_system: ActorChannels, map: Self::MapRef) -> Self {
        MapActor { actor_system, map }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let actor = MapActor::new_actorstate(actor_channels, map);

        'outer: loop {
            let message = actor.actor_system.receive().unwrap();
            if message.is::<MapWriteMsg>() {
                let msg = message.downcast::<MapWriteMsg>().unwrap_or_else(|_| panic!("Could not downcast map actor message!"));

                match msg.target {
                    MapWriteTarget::CreateInitialMap { mp_matches, p3d, initial_frame, current_frame } => {
                        // TODO (STEREO) - Add option to make the map monocular or stereo

                        let mut write_lock = actor.map.write();
                        match write_lock.create_initial_map_monocular(mp_matches, p3d, initial_frame, current_frame) {
                                Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, curr_kf_timestamp)) => {
                                    actor.actor_system.send(TRACKING_BACKEND, Box::new(
                                        MapInitializedMsg {
                                            curr_kf_pose, curr_kf_id, ini_kf_id, 
                                            local_mappoints, curr_kf_timestamp
                                        }
                                    ));
                                },
                                None => { 
                                    warn!("Could not create initial map");
                                }
                        };
                    },
                    MapWriteTarget::Map__Optimization { poses } => {
                        // TODO (timing) ... 1.50 += 0.83
                        let mut lock = actor.map.write();
                        lock.update_after_ba(poses, 0);
                    },
                    MapWriteTarget::KeyFrame__New { kf } => {
                        let _timer = timer!("MapActor::Total-insert_keyframe_to_map");
                        let kf_id = actor.map.write().insert_keyframe_to_map(kf, false);
                        info!("Created keyframe {}", kf_id);
                        // Tell local mapping to process keyframe
                        actor.actor_system.find(LOCAL_MAPPING).send(Box::new(KeyFrameIdMsg{kf_id})).unwrap();
                        // Send the new keyframe ID directly back to the sender so they can use the ID 
                        actor.actor_system.find(TRACKING_BACKEND).send(Box::new(KeyFrameIdMsg{kf_id})).unwrap();
                    },
                    MapWriteTarget::KeyFrame__Delete { id } => {
                        let _timer = timer!("MapActor::Total-discard_keyframe");
                        actor.map.write().discard_keyframe(id);
                    },
                    MapWriteTarget::MapPoint__New { mp , observations} => {
                        // TODO (timing) ... 0.43 += 18.21
                        let id = actor.map.write().insert_mappoint_to_map(mp, observations);
                    },
                    MapWriteTarget::MapPoint__NewMany { mps, callback_actor } => {
                        let _timer = timer!("MapActor::Total-Many-insert_mappoint_to_map");
                        let mut map = actor.map.write();
                        for (mp, observations) in mps {
                            let _ = map.insert_mappoint_to_map(mp, observations);
                        }
                        // Inform tracking that we are done with creating new mappoints. Needed because tracking fails if new points are not created
                        // before track_with_reference_keyframe, so to avoid the race condition we have it wait until the new points are ready.
                        actor.actor_system.send(&callback_actor, Box::new(LastKeyFrameUpdatedMsg{}));
                    },
                    MapWriteTarget::MapPoint__DiscardMany { ids } => {
                        let _timer = timer!("MapActor::Total-Many-discard_mappoint");
                        for id in ids {
                            actor.map.write().discard_mappoint(&id);
                        }
                    },
                    MapWriteTarget::MapPoint__Discard { id } => {
                        // TODO (timing) ... 5.20 += 96.51
                        actor.map.write().discard_mappoint(&id);
                    },
                    MapWriteTarget::MapPoint__IncreaseFound { id, amount } => {
                        // TODO (timing) ... 0.00 += 0.02
                        if let Some(mp) = actor.map.write().mappoints.get_mut(&id) {
                            mp.increase_found(amount);
                        }
                    },
                    MapWriteTarget::MapPoint__IncreaseVisible {id} => {
                        // TODO (timing) ... 0.08 += 7.61
                        if let Some(mp) = actor.map.write().mappoints.get_mut(&id) {
                            mp.increase_visible();
                        }
                    },
                    MapWriteTarget::MapPoint__AddObservation {mp_id, kf_id, index} => {
                        let _timer = timer!("MapActor::Total-add_observation");
                        let num_keypoints = actor.map.write().keyframes.get(&kf_id).expect(&format!("Could not get kf {}", kf_id)).features.num_keypoints;
                        let mut write = actor.map.write();
                        write.mappoints.get_mut(&mp_id).unwrap().add_observation(&kf_id, num_keypoints, index as u32);
                        write.keyframes.get_mut(&kf_id).unwrap().add_mappoint(index as u32, mp_id, false);
                    },
                    MapWriteTarget::MapPoint__Replace {mp_to_replace, mp} => {
                        let _timer = timer!("MapActor::Total-replace_mappoint");
                        actor.map.write().replace_mappoint(mp_to_replace, mp);
                    },
                    MapWriteTarget::MapPoint__Pose{mp_id, pose} => {
                        let _timer = timer!("MapActor::Total-UpdateMapPointPose");
                        actor.map.write().mappoints.get_mut(&mp_id).unwrap().position = pose.get_translation();
                    }
                    _ => {
                        warn!("Unimplemented target: {:?}", msg.target);
                    },
                }
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Map Actor received unknown message type!");
            }
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

    CreateInitialMap{
        mp_matches: Vec<i32>, p3d: DVVectorOfPoint3f, initial_frame: Frame<InitialFrame>, 
        current_frame: Frame<InitialFrame>
    },
    KeyFrame__New{kf: Frame<PrelimKeyFrame>},
    KeyFrame__Delete{id: Id},
    KeyFrame__Pose{kf_id: Id, pose: DVPose},
    Map__ResetActive{},
    Map__Optimization{poses: BundleAdjustmentResult},
    MapPoint__New{mp: MapPoint<PrelimMapPoint>, observations: Vec<(Id, u32, usize)>},
    MapPoint__NewMany{mps: Vec<(MapPoint<PrelimMapPoint>, Vec<(Id, u32, usize)>)>, callback_actor: String},
    MapPoint__DiscardMany{ids: Vec<Id>},
    MapPoint__Discard{id: Id},
    MapPoint__Replace{mp_to_replace: Id, mp: Id},
    MapPoint__IncreaseFound{id: Id, amount: u32},
    MapPoint__IncreaseVisible{id: Id}, //mp_ids: Vec<Id>},
    MapPoint__AddObservation{mp_id: Id, kf_id: Id, index: usize},
    MapPoint__Pose{mp_id: Id, pose: DVPose},
    Shutdown{msg: ShutdownMsg}
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
    pub fn create_initial_map(ini_data: Initialization) -> Self {
        Self {
            target: MapWriteTarget::CreateInitialMap{
                mp_matches: ini_data.mp_matches,
                p3d: ini_data.p3d,
                initial_frame: ini_data.initial_frame.unwrap(), 
                current_frame: ini_data.current_frame.unwrap(),
            },
        }
    }
    pub fn new_keyframe(kf: Frame<PrelimKeyFrame>) -> Self {
        Self {
            target: MapWriteTarget::KeyFrame__New {kf},
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
    pub fn local_map_optimization(poses: BundleAdjustmentResult) -> Self {
        Self {
            target: MapWriteTarget::Map__Optimization{poses},
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
    pub fn create_one_mappoint(mp: MapPoint<PrelimMapPoint>, observations: Vec<(Id, u32, usize)>) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__New {mp, observations}
        }
    }
    pub fn discard_many_mappoints(ids: Vec<Id>) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__DiscardMany {ids},
        }
    }
    pub fn discard_one_mappoint(id: Id) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__Discard { id }
        }
    }
    pub fn increase_found(id: Id, amount: u32) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__IncreaseFound { id, amount },
        }
    }
    pub fn increase_visible(id: Id) -> Self { //mp_ids: Vec<Id>) -> Self {
        Self {
            target: MapWriteTarget::MapPoint__IncreaseVisible { id } //mp_ids },
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
    pub fn shutdown() -> Self {
        Self {
            target: MapWriteTarget::Shutdown{msg: ShutdownMsg{}}
        }
    }
}
