use core::system::Actor;
use core::config::{SETTINGS, SYSTEM};
use core::sensor::{FrameSensor, ImuSensor, Sensor};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;
use log::{debug, error, info, trace, warn};
use opencv::core::KeyPointTraitConst;
use crate::actors::local_mapping::LOCAL_MAPPING_IDLE;
use crate::actors::messages::{KeyFrameIdMsg, LoopClosureGBAMsg};
use crate::map::pose::{DVTranslation, Pose, Sim3};
use crate::modules::module::{FeatureMatchingModule, LoopDetectionModule};
use crate::modules::orbmatcher::ORBMatcherTrait;
use crate::modules::sim3solver::Sim3Solver;
use crate::modules::{optimizer, orbmatcher};
use crate::registered_actors::{self, FEATURE_MATCHING_MODULE, FULL_MAP_OPTIMIZATION_MODULE, VISUALIZER};
use crate::{System, MapLock};
use crate::map::map::Id;
use crate::modules::imu::DVImu;

use super::local_mapping::LOCAL_MAPPING_PAUSE_SWITCH;
use super::messages::{IMUInitializedMsg, ShutdownMsg};

pub type KeyFrameAndPose = HashMap<Id, Sim3>;

pub static GBA_KILL_SWITCH: AtomicBool = AtomicBool::new(false); // mbStopGBA
static GBA_IDLE: AtomicBool = AtomicBool::new(true); // mbRunningGBA

pub struct LoopClosing {
    system: System,
    sensor: Sensor,

    map: MapLock,

    // Modules
    _imu: Option<DVImu>,
    loop_detection: Box<dyn LoopDetectionModule>,
}

impl Actor for LoopClosing {
    type MapRef = MapLock;

    fn new_actorstate(system: System, map: Self::MapRef) -> LoopClosing {

        LoopClosing {
            system,
            map,
            _imu: None, // TODO (IMU): ImuModule::new(None, None, sensor, false, false),
            sensor: SETTINGS.get(SYSTEM, "sensor"),
            loop_detection: registered_actors::new_loop_detection_module(),
        }
    }

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = LoopClosing::new_actorstate(system, map);

        let max_queue_size = actor.system.receiver_bound.unwrap_or(100);
        tracy_client::set_thread_name!("loop closing");

        'outer: loop {
            let message = actor.system.receive().unwrap();
            if message.is::<KeyFrameIdMsg>() {
                let msg = message.downcast::<KeyFrameIdMsg>().unwrap_or_else(|_| panic!("Could not downcast loop closing message!"));


                let queue_len = actor.system.queue_len();
                if queue_len > max_queue_size {
                    // Abort additional work if there are too many frames in the msg queue.
                    info!("Loop Closing dropped 1 frame, queue len: {}", queue_len);
                    continue;
                }

                if msg.kf_id == 0 {
                    // Don't add very first keyframe
                    continue;
                } else if actor.map.read().keyframes.get(&msg.kf_id).is_none() {
                    // KF deleted by map already. Shouldn't happen often, but it can
                    continue;
                }

                match actor.loop_closing(msg.kf_id) {
                    Ok(_) => {},
                    Err(e) => {
                        warn!("Loop closing failed: {}", e);
                    }
                }
            } else if message.is::<IMUInitializedMsg>() {
                todo!("IMU: Process message from local mapping");
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Loop Closing received unknown message type!");
            }
        }
    }


}

impl LoopClosing {
    fn loop_closing(&mut self, current_kf_id: Id) -> Result<(), Box<dyn std::error::Error>> {
        // Avoid that a keyframe can be erased while it is being process by this thread
        // TODO (design, fine-grained locking) would be great if we could just lock this keyframe
        self.map.write().keyframes.get_mut(&current_kf_id).unwrap().dont_delete = true;

        debug!("Loop closing working on kf {} (frame {})", current_kf_id, self.map.read().keyframes.get(&current_kf_id).unwrap().frame_id);

        // Detect loop candidates and check covisibility consistency
        match self.loop_detection.detect_loop(& self.map, current_kf_id) {
            Ok((merge_kf, loop_kf, scw, loop_mappoints, current_matched_points)) => {
                if merge_kf.is_some() {
                    info!("KF {}: Merge detected!", current_kf_id);
                }

                match (loop_kf, scw) {
                    (Some(loop_kf), Some(scw)) => {
                        info!("KF {}: Loop detected! with KF {}", current_kf_id, loop_kf);

                        match self.sensor.is_imu() {
                            true => {
                                todo!("IMU");
                                // Lines 235-258
                            },
                            false => {}
                        };

                        match self.correct_loop(current_kf_id, loop_kf, scw, loop_mappoints, current_matched_points) {
                            Ok(_) => {},
                            Err(e) => {
                                warn!("Loop correction failed: {}", e);
                            }
                        }

                        // Reset all variables
                        self.map.write().keyframes.get_mut(&loop_kf).unwrap().dont_delete = false;
                    },
                    _ => ()
                }
            },
            Err(e) => {
                warn!("Loop detection failed: {}", e);
            }
        }

        self.map.write().keyframes.get_mut(&current_kf_id).unwrap().dont_delete = false;
        thread::sleep(Duration::from_micros(5000));
        Ok(())
    }


    fn correct_loop(&mut self, current_kf_id: Id, loop_kf: Id, loop_scw: Sim3, loop_mappoints: Vec<Id>, current_matched_points: Vec<Option<Id>>) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("correct_loop");

        println!("CORRECT LOOP, Sim3: {:?}", loop_scw);

        set_switches(Switches::CorrectLoopBeginning);

        // Ensure current keyframe is updated
        self.map.write().update_connections(current_kf_id);

        // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        let (current_connected_kfs, mut corrected_sim3, mut non_corrected_sim3, twc) = {
            let lock = self.map.read();
            let current_kf = lock.keyframes.get(&current_kf_id).unwrap();

            let mut current_connected_kfs = current_kf.get_covisibility_keyframes(i32::MAX);
            current_connected_kfs.push(current_kf_id);

            let mut corrected_sim3 = KeyFrameAndPose::new();
            corrected_sim3.insert(current_kf_id, loop_scw);

            let mut non_corrected_sim3 = KeyFrameAndPose::new();
            non_corrected_sim3.insert(current_kf_id, current_kf.pose.into());

            let twc = current_kf.pose.inverse();

            (current_connected_kfs, corrected_sim3, non_corrected_sim3, twc)
        };

        let mut corrected_mp_references = HashMap::<Id, Id>::new(); // mnCorrectedReference in mappoints
        {
            let mut lock = self.map.write();

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            let current_kf = lock.keyframes.get_mut(&current_kf_id).unwrap();
            current_kf.pose = loop_scw.into();
            println!("Corrected current kf {} (frame {}): {:?}", current_kf.id, current_kf.frame_id, current_kf.pose);

            for connected_kf_id in &current_connected_kfs {
                let connected_kf = lock.keyframes.get_mut(connected_kf_id).unwrap();
                let tiw = connected_kf.pose;

                if connected_kf_id != &current_kf_id {
                    let tic = tiw * twc;
                    let sic: Sim3 = tic.into();

                    let corrected_siw = sic * loop_scw;
                    //Pose corrected with the Sim3 of the loop closure
                    corrected_sim3.insert(*connected_kf_id, corrected_siw);

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    connected_kf.pose = corrected_siw.into();

                    // Pose without correction
                    let original_siw: Sim3 = tiw.into();
                    non_corrected_sim3.insert(*connected_kf_id, original_siw);
                    println!("...corrected pose for kf {} (frame {}): {:?}", connected_kf_id, connected_kf.frame_id, connected_kf.pose);
                }
            }

            // Correct all MapPoints observed by current keyframe and neighbors, so that they align with the other side of the loop
            let mut corrected_by_kf = HashMap::<Id, Id>::new(); // mnCorrectedByKF
            for (kf_id, g2o_corrected_siw) in &corrected_sim3 {
                let g2o_corrected_swi = g2o_corrected_siw.inverse();
                let g2o_siw = non_corrected_sim3.get(kf_id).unwrap();
                let mappoints = {
                    let connected_kf = lock.keyframes.get(kf_id).unwrap();
                    connected_kf.get_mp_matches().clone()
                };
                let mut count = 0;
                for i in 0..mappoints.len() {
                    let mp_id = match mappoints.get(i).unwrap() {
                        Some((id, _)) => id,
                        None => continue
                    };
                    if corrected_by_kf.contains_key(mp_id) && *corrected_by_kf.get(mp_id).unwrap() == current_kf_id {
                        continue;
                    }

                    // Project with non-corrected pose and project back with corrected pose
                    {
                        let mp = lock.mappoints.get_mut(mp_id).unwrap();
                        let corrected_p3d_w = g2o_corrected_swi.map(&g2o_siw.map(&mp.position));
                        mp.position = corrected_p3d_w;
                        corrected_by_kf.insert(*mp_id, current_kf_id);
                    }

                    corrected_mp_references.insert(*mp_id, *kf_id);

                    let norm_and_depth = {
                        let mp = lock.mappoints.get(mp_id).unwrap();
                        mp.get_norm_and_depth(&lock)
                    };
                    if norm_and_depth.is_some() {
                        lock.mappoints.get_mut(mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                    } else {
                        error!("Mappoint {} has empty observations", mp_id);
                    }
                    count += 1;
                }

                // Make sure connections are updated
                lock.update_connections(*kf_id);
            }

            // Start Loop Fusion
            // Update matched map points and replace if duplicated
            let mut num_replaced = 0;
            let mut num_added = 0;
            for i in 0..current_matched_points.len() {
                if let Some(loop_mp_id) = current_matched_points[i] {
                    let curr_kf = lock.keyframes.get(&current_kf_id).unwrap();
                    match lock.mappoints.get(&loop_mp_id) {
                        None => {
                            if let Some((curr_mp_id, _is_outlier)) = curr_kf.get_mp_match(&(i as u32)) {
                                lock.replace_mappoint(curr_mp_id, loop_mp_id);
                                num_replaced += 1;
                            } else {
                                num_added += 1;
                                lock.add_observation(current_kf_id, loop_mp_id, i as u32, false);
                                let best_descriptor = lock.mappoints.get(&loop_mp_id)
                                    .and_then(|mp| mp.compute_distinctive_descriptors(&lock)).unwrap();
                                lock.mappoints.get_mut(&loop_mp_id)
                                    .map(|mp| mp.update_distinctive_descriptors(best_descriptor));
                            }
                        },
                        Some(_) => ()
                    };
                }
            }
            println!("Loop fusion, mappoints replaced {}, added {}", num_replaced, num_added);
        }

        // This is for testing the outcome of essential graph optimization
        // self.system.send(
        //     VISUALIZER, 
        //     Box::new(
        //         LoopClosureMapPointFusionMsg {
        //             mappoint_matches: self.current_matched_points.iter().filter(|m| m.is_some()).map(|m| m.unwrap()).collect(),
        //             loop_mappoints: loop_mappoints.clone(),
        //             keyframes_affected: corrected_sim3.keys().cloned().collect(),
        //             timestamp: self.map.read().keyframes.get(&current_kf_id).unwrap().timestamp
        //         }
        // ));

        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        self.search_and_fuse(&corrected_sim3, loop_mappoints)?;

        // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        let mut loop_connections: BTreeMap::<Id, HashSet<Id>> = BTreeMap::new();
        let mut test_unique_kfs: HashSet<Id> = HashSet::new();

        for kf_id in &current_connected_kfs {
            let mut map = self.map.write();
            let previous_neighbors = map.keyframes.get(kf_id).unwrap().get_covisibility_keyframes(i32::MAX);

            // Update connections. Detect new links.
            map.update_connections(*kf_id);

            let connected_kfs: HashSet<i32> = map.keyframes.get(kf_id).unwrap().get_connected_keyframes().iter().map(|(id, _)| *id).collect();
            test_unique_kfs.extend(connected_kfs.clone());
            loop_connections.insert(*kf_id, connected_kfs);

            for neighbor_id in previous_neighbors {
                loop_connections.get_mut(kf_id).unwrap().remove(&neighbor_id);
            }

            for neighbor_id in &current_connected_kfs {
                loop_connections.get_mut(kf_id).unwrap().remove(neighbor_id);
            }
        }

        // This is for debugging essential graph optimization
        // self.system.send(
        //     VISUALIZER, 
        //     Box::new(
        //         LoopClosureEssentialGraphMsg {
        //             relevant_keyframes: test_unique_kfs,
        //             mappoints: 
        //             timestamp: current_timestamp
        //         }
        // ));

        // Optimize graph
        optimizer::optimize_essential_graph(
            &self.map, loop_kf, current_kf_id,
            loop_connections, &non_corrected_sim3, &corrected_sim3, 
            corrected_mp_references,
            !self.sensor.is_mono()
        );

        // Add loop edge
        self.map.write().add_kf_loop_edges(loop_kf, current_kf_id);

        // Launch a new thread to perform Global Bundle Adjustment
        let mut map_copy = self.map.clone();
        let current_kf_id = current_kf_id;

        // This is for debugging GBA optimization
        // self.system.send(
        //     VISUALIZER, 
        //     Box::new(
        //         LoopClosureGBAMsg { 
        //             kf_id: current_kf_id,
        //             timestamp: self.map.read().keyframes.get(&current_kf_id).unwrap().timestamp
        //         }
        // ));

        thread::spawn(move || {
            run_gba(&mut map_copy, current_kf_id);
        });

        Ok(())
    }

    fn search_and_fuse(&mut self, corrected_poses_map: &HashMap<Id, Sim3>, loop_mappoints: Vec<Id>) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("search_and_fuse");

        for (kf_id, g2o_scw) in corrected_poses_map {
            let replace_points = FEATURE_MATCHING_MODULE.fuse_from_loop_closing(
                &kf_id, &g2o_scw, &loop_mappoints, &self.map, 4
            )?;
            let mut num_fused = 0;

            for i in 0..replace_points.len() {
                if let Some(mp_to_replace) = replace_points[i] {
                    let replace_with = loop_mappoints[i];
                    self.map.write().replace_mappoint(mp_to_replace, replace_with);
                    num_fused += 1;
                }
            }
            debug!("Search and fuse, for KF {}, fused {} mappoints. total candidates: {}", kf_id, num_fused, loop_mappoints.len());
        }
        Ok(())
    }
}


fn run_gba(map: &mut MapLock, loop_kf: Id) {
    // void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
    let _span = tracy_client::span!("run_gba_in_thread");
    info!("Starting Global Bundle Adjustment");

    set_switches(Switches::GbaBeginning);

    FULL_MAP_OPTIMIZATION_MODULE.optimize(map, 10, false, loop_kf);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree

    debug!("Global bundle adjustment finished. Updating map...");

    if GBA_KILL_SWITCH.load(Ordering::SeqCst) {
        set_switches(Switches::GbaDone);
        return;
    }

    set_switches(Switches::PostGBAUpdate);

    {
        let mut lock = map.write();
        // Correct keyframes starting at map first keyframe
        let mut kfs_to_check = vec![lock.initial_kf_id];
        let mut i = 0;
        let mut tcw_bef_gba: HashMap<Id, Pose> = HashMap::new();
        while i < kfs_to_check.len() {
            let curr_kf_id = kfs_to_check[i];
            let children = lock.keyframes.get(& curr_kf_id).unwrap().children.clone();

            let (curr_kf_pose_inverse, curr_kf_gba_pose) = {
                let curr_kf = lock.keyframes.get(&curr_kf_id).unwrap();
                (curr_kf.pose.inverse(), curr_kf.gba_pose.clone())
            };

            for child_id in & children {
                let child = lock.keyframes.get_mut(child_id).unwrap();
                if child.ba_global_for_kf != loop_kf {
                    let tchildc = child.pose * curr_kf_pose_inverse;
                    child.gba_pose = Some(tchildc * curr_kf_gba_pose.unwrap());
                    child.ba_global_for_kf = loop_kf;
                    println!("Add pose for child kf {}", child_id);
                }
                kfs_to_check.push(*child_id);
            }

            let kf = lock.keyframes.get_mut(&curr_kf_id).unwrap();
            tcw_bef_gba.insert(curr_kf_id, kf.pose);
            kf.pose = kf.gba_pose.unwrap().clone();
            println!("Update kf {} with pose {:?}", curr_kf_id, kf.pose);
            i += 1;
        }

        // Correct MapPoints
        let mps_to_update = {
            let mut mps_to_update = HashMap::new();
            for (id, mp) in &lock.mappoints {
                if mp.ba_global_for_kf == loop_kf {
                    // If optimized by Global BA, just update
                    mps_to_update.insert(*id, mp.gba_pose.unwrap());
                } else {
                    // Update according to the correction of its reference keyframe
                    let ref_kf = lock.keyframes.get(&mp.ref_kf_id).unwrap();

                    if ref_kf.ba_global_for_kf != loop_kf {
                        continue;
                    }

                    // Map to non-corrected camera
                    let tcw_bef_gba_for_ref_kf = tcw_bef_gba.get(&mp.ref_kf_id).unwrap();
                    let rcw = tcw_bef_gba_for_ref_kf.get_rotation();
                    let tcw = tcw_bef_gba_for_ref_kf.get_translation();
                    let xc = *rcw * *mp.position + *tcw;


                    // Backproject using corrected camera
                    let twc = ref_kf.pose.inverse();
                    let rwc = twc.get_rotation();
                    let twc = twc.get_translation();

                    mps_to_update.insert(*id, DVTranslation::new(*rwc * xc + *twc));
                }
            }
            mps_to_update
        };


        for (mp_id, gba_pose) in mps_to_update {
            lock.mappoints.get_mut(&mp_id).unwrap().position = gba_pose;
        }
    }

    set_switches(Switches::GbaDone);

    info!("Map updated!");
}

enum Switches {
    CorrectLoopBeginning,
    GbaBeginning,
    PostGBAUpdate,
    GbaDone,
}

fn set_switches(switches: Switches) {
    match switches {
        Switches::CorrectLoopBeginning => {
            // Avoid new keyframes are inserted while correcting the loop
            LOCAL_MAPPING_PAUSE_SWITCH.store(true, Ordering::SeqCst);

            // If a Global Bundle Adjustment is running, abort it
            if !GBA_IDLE.load(Ordering::SeqCst) {
                GBA_KILL_SWITCH.store(true, Ordering::SeqCst);
            }

            // Wait until Local Mapping has effectively stopped
            while !LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) {
                thread::sleep(Duration::from_millis(1));
            }
        },
        Switches::GbaBeginning => {
            // GBA is not idle
            GBA_IDLE.store(false, Ordering::SeqCst);
            // Reset kill switch so future loops of LC can set it if they need to
            GBA_KILL_SWITCH.store(false, Ordering::SeqCst);
            // Local mapping ok to run during GBA optimization
            LOCAL_MAPPING_PAUSE_SWITCH.store(false, Ordering::SeqCst);
        },
        Switches::PostGBAUpdate => {
            // Stop local mapping while updating the map with results of GBA optimization
            LOCAL_MAPPING_PAUSE_SWITCH.store(true, Ordering::SeqCst);

            while !LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) {
                thread::sleep(Duration::from_millis(1));
            }
        },
        Switches::GbaDone => {
            // GBA is not running, so reset GBA switches
            GBA_IDLE.store(true, Ordering::SeqCst);
            GBA_KILL_SWITCH.store(false, Ordering::SeqCst);
            // Release local mapping
            LOCAL_MAPPING_PAUSE_SWITCH.store(false, Ordering::SeqCst);
        },

    }
}
