// Loop closing implementation from orbslam2, not sure if it works correctly

// use core::system::Actor;
// use core::config::{SETTINGS, SYSTEM};
// use core::sensor::Sensor;
// use std::collections::{HashMap, HashSet};
// use std::sync::atomic::{AtomicBool, Ordering};
// use std::thread;
// use std::time::Duration;
// use log::{debug, info, warn};
// use crate::actors::local_mapping::LOCAL_MAPPING_IDLE;
// use crate::actors::messages::KeyFrameIdMsg;
// use crate::map::pose::{DVTranslation, Pose, Sim3};
// use crate::modules::sim3solver::Sim3Solver;
// use crate::modules::{optimizer, orbmatcher};
// use crate::registered_actors::{LOOP_CLOSING, VOCABULARY};
// use crate::{System, ReadWriteMap};
// use crate::map::map::Id;
// use crate::modules::imu::ImuModule;

// use super::local_mapping::LOCAL_MAPPING_PAUSE_SWITCH;
// use super::messages::{IMUInitializedMsg, ShutdownMsg};

// type ConsistentGroup = (Vec<Id>, i32);
// pub type KeyFrameAndPose = HashMap<Id, Sim3>;

// pub static GBA_KILL_SWITCH: AtomicBool = AtomicBool::new(false); // mbStopGBA
// static GBA_IDLE: AtomicBool = AtomicBool::new(false); // mbRunningGBA

// pub struct LoopClosing {
//     system: System,
//     sensor: Sensor,

//     map: ReadWriteMap,
//     _imu: Option<ImuModule>,

//     // Loop detector variables
//     current_kf_id: Id, // mpCurrentKF
//     matched_kf: Id, // mpMatchedKF
//     consistent_groups: Vec<ConsistentGroup>, // mvConsistentGroups
//     current_matched_points: HashMap<usize, Id>, // mvpCurrentMatchedPoints
//     enough_consistent_candidates: Vec<Id>, // mvpEnoughConsistentCandidates;
//     current_connected_kfs: Vec<Id>, // mvpCurrentConnectedKFs
//     loop_mappoints: Vec<Id>, // mvpLoopMapPoints
//     scw: Sim3, // mScw, mg2oScw

//     last_loop_kf_id: Id, // mLastLoopKFid

//     // Global BA
//     full_ba_idx: i32, // mnFullBAIdx
// }

// impl Actor for LoopClosing {
//     type MapRef = ReadWriteMap;

//     fn new_actorstate(system: System, map: Self::MapRef) -> LoopClosing {

//         LoopClosing {
//             system,
//             map,
//             _imu: None, // TODO (IMU): ImuModule::new(None, None, sensor, false, false),
//             sensor: SETTINGS.get(SYSTEM, "sensor"),
//             current_kf_id: -1,
//             full_ba_idx: 0,
//             consistent_groups: Vec::new(),
//             matched_kf: -1,
//             last_loop_kf_id: -1,
//             current_matched_points: HashMap::new(),
//             enough_consistent_candidates: Vec::new(),
//             current_connected_kfs: Vec::new(),
//             loop_mappoints: Vec::new(),
//             scw: Sim3::identity(),
//         }
//     }

//     fn spawn(system: System, map: Self::MapRef) {
//         let mut actor = LoopClosing::new_actorstate(system, map);
//         tracy_client::set_thread_name!("loop closing");

//         'outer: loop {
//             let message = actor.system.receive().unwrap();
//             if message.is::<KeyFrameIdMsg>() {
//                 let msg = message.downcast::<KeyFrameIdMsg>().unwrap_or_else(|_| panic!("Could not downcast loop closing message!"));

//                 if msg.kf_id == 0 {
//                     // Don't add very first keyframe
//                     continue;
//                 }

//                 actor.current_kf_id = msg.kf_id;
//                 match actor.loop_closing() {
//                     Ok(_) => {},
//                     Err(e) => {
//                         warn!("Loop closing failed: {}", e);
//                     }
//                 }
//                 println!();
//             } else if message.is::<IMUInitializedMsg>() {
//                 todo!("IMU: Process message from local mapping");
//             } else if message.is::<ShutdownMsg>() {
//                 break 'outer;
//             } else {
//                 warn!("Loop Closing received unknown message type!");
//             }
//         }
//     }


// }

// impl LoopClosing {
//     fn loop_closing(&mut self) -> Result<(), Box<dyn std::error::Error>> {
//         // Avoid that a keyframe can be erased while it is being process by this thread
//         // TODO (design, fine-grained locking) would be great if we could just lock this keyframe
//         self.map.write().keyframes.get_mut(&self.current_kf_id).unwrap().dont_delete = true;

//         debug!("Loop closing working on kf {} (frame {})", self.current_kf_id, self.map.read()?.keyframes.get(&self.current_kf_id).unwrap().frame_id);

//         // Detect loop candidates and check covisibility consistency
//         if self.detect_loop() {
//             debug!("KF {}: step 1 passed", self.current_kf_id);
//             // Add current keyframe to database
//             self.map.write().add_to_kf_database(self.current_kf_id);

//             // Compute similarity transformation [sR|t]
//             // In the stereo/RGBD case s=1
//             if self.compute_sim3()? {
//                 info!("KF {}: Loop detected! Correcting loop...", self.current_kf_id);
//                 // Perform loop fusion and pose graph optimization
//                 self.correct_loop()?;
//             }
//         } else {
//             // Add current keyframe to database
//             self.map.write().add_to_kf_database(self.current_kf_id);

//             // debug!("KF {}: No loop", self.current_kf_id);
//         }
//         self.map.write().keyframes.get_mut(&self.current_kf_id).unwrap().dont_delete = false;
//         thread::sleep(Duration::from_micros(5000));
//         Ok(())
//     }

//     fn detect_loop(&mut self) -> bool {
//         let _span = tracy_client::span!("detect_loop");
//         //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
//         if self.map.read()?.num_keyframes() <= 10 {
//             return false;
//         }

//         // Compute reference BoW similarity score
//         // This is the lowest score to a connected keyframe in the covisibility graph
//         // We will impose loop candidates to have a higher similarity than this
//         let min_score = {
//             let lock = self.map.read()?;
//             let connected_keyframes = lock.keyframes.get(&self.current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX);
//             print!("Connected KFs: ");

//             let mut min_score = 1.0;
//             for kf_id in connected_keyframes {
//                 let kf1 = lock.keyframes.get(&self.current_kf_id).unwrap();
//                 let kf2 = lock.keyframes.get(&kf_id).unwrap();

//                 let score = VOCABULARY.score(&kf1, &kf2);
//                 print!("{}:{}, ", kf2.id, score);
//                 if score < min_score {
//                     min_score = score;
//                 }
//             }
//             min_score
//         };
//         println!();

//         // Query the database imposing the minimum score
//         let candidate_kfs = self.map.read()?.kf_db_detect_loop_candidates(self.current_kf_id, min_score);

//         debug!("Candidate kfs: {:?}", candidate_kfs);

//         // If there are no loop candidates, just add new keyframe and return false
//         if candidate_kfs.is_empty() {
//             self.consistent_groups.clear();
//             debug!("No loop candidates");
//             return false;
//         }

//         // For each loop candidate check consistency with previous loop candidates
//         // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
//         // A group is consistent with a previous group if they share at least a keyframe
//         // We must detect a consistent loop in several consecutive keyframes to accept it
//         self.enough_consistent_candidates.clear();
//         let mut current_consistent_groups = Vec::new(); // vCurrentConsistentGroups
//         let mut consistent_group = vec![false; self.consistent_groups.len()]; // vbConsistentGroup

//         for cand_kf_id in candidate_kfs {
//             let map_read_lock = self.map.read()?;
//             let candidate_kf = map_read_lock.keyframes.get(&cand_kf_id).unwrap();
//             let mut candidate_group: Vec<Id> = candidate_kf.get_connected_keyframes().keys().cloned().collect();
//             candidate_group.push(cand_kf_id);

//             let mut enough_consistent = false;
//             let mut consistent_for_some_group = false;
//             for i in 0..self.consistent_groups.len() {
//                 let (previous_group, previous_consistency) = &self.consistent_groups[i];
//                 let mut consistent = false;
//                 for kf_id in &candidate_group {
//                     if previous_group.contains(kf_id) {
//                         consistent = true;
//                         consistent_for_some_group = true;
//                         break;
//                     }
//                 }

//                 if consistent {
//                     let current_consistency = previous_consistency + 1;
//                     if !consistent_group[i] {
//                         current_consistent_groups.push((candidate_group.clone(), current_consistency));
//                         consistent_group.push(true); //this avoid to include the same group more than once
//                     }
//                     if current_consistency >= SETTINGS.get::<i32>(LOOP_CLOSING, "covisibility_consistency_threshold") && !enough_consistent {
//                         self.enough_consistent_candidates.push(cand_kf_id);
//                         enough_consistent = true; //this avoid to insert the same candidate more than once
//                     }
//                 }
//             }

//             // If the group is not consistent with any previous group insert with consistency counter set to zero
//             if !consistent_for_some_group {
//                 current_consistent_groups.push((candidate_group, 0));
//             }
//         }

//         // Update Covisibility Consistent Groups
//         self.consistent_groups = current_consistent_groups;

//         debug!("Final consistent candidates: {:?}", self.enough_consistent_candidates);

//         return !self.enough_consistent_candidates.is_empty();
//     }

//     fn compute_sim3(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("compute_sim3");

//         // For each consistent loop candidate we try to compute a Sim3
//         let initial_candidates = self.enough_consistent_candidates.len();

//         // We compute first ORB matches for each candidate
//         // If enough matches are found, we setup a Sim3Solver

//         let mut discarded = vec![false; initial_candidates]; // vbDiscarded
//         let mut solvers = HashMap::new(); // vpSim3Solvers
//         let mut num_candidates = 0; // nCandidates, candidates with enough matches
//         let mut mappoint_matches: Vec<HashMap<u32, Id>> = vec![HashMap::new(); initial_candidates]; // vpMapPointMatches

//         {
//             for i in 0..initial_candidates {
//                 let candidate_kf_id = self.enough_consistent_candidates[i];

//                 mappoint_matches[i] = {
//                     let map_read_lock = self.map.read()?;
//                     let current_keyframe = map_read_lock.keyframes.get(&self.current_kf_id).unwrap();

//                     let keyframe = map_read_lock.keyframes.get(&candidate_kf_id).unwrap();
//                     let mappoint_matches = orbmatcher::search_by_bow_kf(current_keyframe, keyframe, true, 0.75)?;
//                     debug!("Searching for matches between {} (frame {}) and {} (frame {})", current_keyframe.id, current_keyframe.frame_id, keyframe.id, keyframe.frame_id);
//                     debug!("(Mappoint matches: {})", mappoint_matches.len());
//                     mappoint_matches
//                 };

//                 if mappoint_matches[i].len() < 20 {
//                     discarded[i] = true;
//                 } else {
//                     let mut sim3_solver = Sim3Solver::new(
//                         &self.map,
//                         self.current_kf_id, candidate_kf_id, &mappoint_matches[i], false,
//                     )?;
//                     sim3_solver.set_ransac_parameters(0.99, 20, 300);
//                     solvers.insert(i, sim3_solver);
//                     num_candidates += 1;
//                 }
//             }
//         }
//         let mut has_match = false;

//         println!("Compute sim3, initial candidates: {}", initial_candidates);

//         // Perform alternatively RANSAC iterations for each candidate
//         // until one is succesful or all fail
//         while num_candidates > 0 && !has_match {
//             for i in 0..initial_candidates {
//                 if discarded[i] {
//                     continue;
//                 }
//                 let kf_id = self.enough_consistent_candidates[i];

//                 // Perform 5 Ransac Iterations
//                 let solver = solvers.get_mut(&i).unwrap();
//                 let (no_more, sim3_result) = solver.iterate(5)?;

//                 // If Ransac reachs max. iterations discard keyframe
//                 if no_more {
//                     discarded[i] = true;
//                     num_candidates -= 1;
//                 }

//                 if sim3_result.is_none() {
//                     debug!("Sim3 result: None");
//                 }

//                 // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
//                 if sim3_result.is_some() {
//                     let (inliers, num_inliers) = sim3_result.unwrap();
//                     debug!("Sim3 result: {:?}", num_inliers);

//                     let mut mappoint_matches_copy = HashMap::new(); //vpMapPointMatches
//                     for j in 0..inliers.len() {
//                         if inliers[j] {
//                             mappoint_matches_copy.insert(j, *mappoint_matches[i].get(&(j as u32)).unwrap());
//                         }
//                     }
//                     // println!("Initial inliers: {}", num_inliers);

//                     let mut sim3 = solver.get_estimates();
//                     orbmatcher::search_by_sim3(
//                         &self.map, self.current_kf_id, kf_id, &mut mappoint_matches_copy, &sim3, 7.5
//                     );

//                     let num_inliers = optimizer::optimize_sim3(
//                         &self.map, self.current_kf_id, kf_id, &mut mappoint_matches_copy, &mut sim3, 10, false
//                     );

//                     debug!("Inliers: {}", num_inliers);
//                     // If optimization is succesful stop ransacs and continue
//                     if num_inliers >= 20 {
//                         has_match = true;
//                         self.matched_kf = kf_id;
//                         let lock = self.map.read()?;
//                         let kf = lock.keyframes.get(&kf_id).unwrap();

//                         let smw = Sim3 {
//                             pose: Pose::new(*kf.pose.get_translation(), *kf.pose.get_rotation()),
//                             scale: 1.0
//                         };
//                         self.scw = smw * sim3;

//                         self.current_matched_points = mappoint_matches_copy;
//                         break;
//                     }
//                 }
//             }
//         }

//         if !has_match {
//             for i in 0..initial_candidates {
//                 self.map.write().keyframes.get_mut(&self.enough_consistent_candidates[i]).unwrap().dont_delete = false;
//             }
//             return Ok(false);
//         }

//         // Retrieve MapPoints seen in Loop Keyframe and neighbors
//         let mut loop_connected_keyframes = self.map.read()?.keyframes.get(&self.matched_kf).unwrap().get_covisibility_keyframes(i32::MAX); // vpLoopConnectedKFs
//         loop_connected_keyframes.push(self.matched_kf);
//         self.loop_mappoints.clear();
//         let mut loop_point_for_kf = HashMap::new(); // mnLoopPointForKF
//         {
//             let map_read_lock = self.map.read()?;
//             for kf_id in loop_connected_keyframes {
//                 let keyframe = map_read_lock.keyframes.get(&kf_id).unwrap();
//                 let mappoints = keyframe.get_mp_matches();
//                 for mp in mappoints {
//                     match mp {
//                         Some((id, _)) => {
//                             let has_point = loop_point_for_kf.contains_key(id);
//                             if !has_point || (*loop_point_for_kf.get(id).unwrap() != self.current_kf_id) {
//                                 self.loop_mappoints.push(*id);
//                                 loop_point_for_kf.insert(*id, self.current_kf_id);
//                             }
//                         },
//                         None => continue
//                     }
//                 }
//             }
//         }

//         // Find more matches projecting with the computed Sim3
//         orbmatcher::search_by_projection_for_loop_detection(
//             &self.map, &self.current_kf_id, &self.scw, &self.loop_mappoints, &mut self.current_matched_points, 10
//         )?;

//         println!("Current matched points: {:?}", self.current_matched_points);

//         if self.current_matched_points.len()  >= 40 {
//             for i in 0..initial_candidates {
//                 if self.enough_consistent_candidates[i] != self.matched_kf {
//                     self.map.write().keyframes.get_mut(&self.enough_consistent_candidates[i]).unwrap().dont_delete = true;
//                 }
//             }
//             return Ok(true);
//         } else {
//             for i in 0..initial_candidates {
//                 self.map.write().keyframes.get_mut(&self.enough_consistent_candidates[i]).unwrap().dont_delete = false;
//             }
//             return Ok(false);
//         }

//     }

//     fn correct_loop(&mut self) -> Result<(), Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("correct_loop");

//         // Note: there's some really gross stuff in here to deal with mutable and immutable references to the map

//         // Send a stop signal to Local Mapping
//         // Avoid new keyframes are inserted while correcting the loop

//         {
//             // TODO (design, concurency) local mapping request stop
//             LOCAL_MAPPING_PAUSE_SWITCH.store(true, Ordering::SeqCst);
//             println!("Pausing local mapping");

//             // If a Global Bundle Adjustment is running, abort it
//             if !GBA_IDLE.load(Ordering::SeqCst) {
//                 self.full_ba_idx += 1;
//                 GBA_KILL_SWITCH.store(true, Ordering::SeqCst);
//             }

//             // Wait until Local Mapping has effectively stopped
//             while !LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) {
//                 println!("Waiting for local mapping to finish...");
//                 thread::sleep(Duration::from_millis(1000));
//             }
//         }

//         // Ensure current keyframe is updated
//         self.map.write().update_connections(self.current_kf_id);

//         // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
//         self.current_connected_kfs = self.map.read()?.keyframes.get(&self.current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX);
//         self.current_connected_kfs.push(self.current_kf_id);

//         let mut corrected_sim3 = KeyFrameAndPose::new();
//         let mut non_corrected_sim3 = KeyFrameAndPose::new();
//         corrected_sim3.insert(self.current_kf_id, self.scw.clone());
//         let twc = self.map.read()?.keyframes.get(&self.current_kf_id).unwrap().pose.inverse();

//         for connected_kf_id in &self.current_connected_kfs {
//             let map = self.map.read()?;
//             let connected_kf = map.keyframes.get(connected_kf_id).unwrap();
//             let pose = connected_kf.pose;

//             if connected_kf_id != &self.current_kf_id {
//                 let tic = pose * twc;
//                 let ric = tic.get_rotation();
//                 let tic = tic.get_translation();
//                 let g2o_sic = Sim3 { 
//                     pose: Pose::new(*tic, *ric), 
//                     scale: 1.0
//                 };
//                 let g2o_corrected_siw = g2o_sic * self.scw;
//                 //Pose corrected with the Sim3 of the loop closure
//                 corrected_sim3.insert(*connected_kf_id, g2o_corrected_siw);
//             }
//             let g2o_siw = Sim3 {
//                 pose: pose.clone(),
//                 scale: 1.0
//             };
//             // Pose without correction
//             non_corrected_sim3.insert(*connected_kf_id, g2o_siw);
//         }

//         // Correct all MapPoints observed by current keyframe and neighbors, so that they align with the other side of the loop
//         let mut mp_corrected_by_kf = HashMap::<Id, Id>::new(); // mpCorrectedByKF
//         let mut corrected_mp_references = HashMap::<Id, Id>::new(); // mnCorrectedReference in mappoints
//         for (kf_id, g2o_corrected_siw) in &corrected_sim3 {
//             let g2o_corrected_swi = g2o_corrected_siw.inverse();
//             let g2o_siw = non_corrected_sim3.get(kf_id).unwrap();
//             let mappoints = {
//                 let map = self.map.read()?;
//                 let connected_kf = map.keyframes.get(kf_id).unwrap();
//                 connected_kf.get_mp_matches().clone()
//             };
//             for i in 0..mappoints.len() {
//                 let mp_id = match mappoints.get(i).unwrap() {
//                     Some((id, _)) => id,
//                     None => continue
//                 };
//                 if mp_corrected_by_kf.contains_key(mp_id) && mp_corrected_by_kf.get(mp_id).unwrap() == kf_id {
//                     continue;
//                 }

//                 // Project with non-corrected pose and project back with corrected pose
//                 {
//                     let mut map = self.map.write();
//                     let mp = map.mappoints.get_mut(mp_id).unwrap();
//                     let p3d_w = mp.position;
//                     let corrected_p3d_w = g2o_corrected_swi.map(&g2o_siw.map(&p3d_w));
//                     mp.position = corrected_p3d_w;
//                     mp_corrected_by_kf.insert(*mp_id, *kf_id);
//                 }

//                 corrected_mp_references.insert(*mp_id, *kf_id);

//                 let norm_and_depth = {
//                     let map = self.map.read()?;
//                     let mp = map.mappoints.get(mp_id).unwrap();
//                     mp.get_norm_and_depth(&map)
//                 };
//                 if norm_and_depth.is_some() {
//                     self.map.write().mappoints.get_mut(mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
//                 }
//             }

//             // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
//             let new_t = *g2o_corrected_siw.pose.get_translation() * (1.0 / g2o_corrected_siw.scale); //[R t/s;0 1]
//             {
//                 let mut map = self.map.write();
//                 let keyframe = map.keyframes.get_mut(kf_id).unwrap();
//                 keyframe.pose = Pose::new(new_t, *g2o_corrected_siw.pose.get_rotation());

//                 // Make sure connections are updated
//                 map.update_connections(*kf_id);
//             }
//         }

//         // Start Loop Fusion
//         // Update matched map points and replace if duplicated
//         for (index, loop_mp_id) in &self.current_matched_points {
//             let mut map = self.map.write();
//             let curr_kf = map.keyframes.get(&self.current_kf_id).unwrap();
//             if curr_kf.has_mp_match_at_index(&(*index as u32)) {
//                 let curr_mp_id = curr_kf.get_mp_match(&(*index as u32));
//                 map.replace_mappoint(curr_mp_id, *loop_mp_id);
//             } else {
//                 map.add_observation(self.current_kf_id, *loop_mp_id, *index as u32, false);
//                 let best_descriptor = map.mappoints.get(&loop_mp_id)
//                     .and_then(|mp| mp.compute_distinctive_descriptors(&map)).unwrap();
//                 map.mappoints.get_mut(&loop_mp_id)
//                     .map(|mp| mp.update_distinctive_descriptors(best_descriptor));
//             }
//         }

//         // Project MapPoints observed in the neighborhood of the loop keyframe
//         // into the current keyframe and neighbors using corrected poses.
//         // Fuse duplications.
//         self.search_and_fuse(&corrected_sim3)?;

//         // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
//         let mut loop_connections: HashMap::<Id, HashSet<Id>> = HashMap::new();
//         for kf_id in &self.current_connected_kfs {
//             let mut map = self.map.write();
//             let previous_neighbors = {
//                 let kf = map.keyframes.get_mut(kf_id).unwrap();
//                 let previous_neighbors = kf.get_covisibility_keyframes(i32::MAX);
//                 loop_connections.insert(*kf_id, kf.get_connected_keyframes().iter().map(|(id, _)| *id).collect());
//                 previous_neighbors
//             };

//             // Update connections. Detect new links.
//             map.update_connections(*kf_id);
//             for neighbor_id in previous_neighbors {
//                 loop_connections.get_mut(kf_id).unwrap().remove(&neighbor_id);
//             }
//             for neighbor_id in &self.current_connected_kfs {
//                 loop_connections.get_mut(kf_id).unwrap().remove(neighbor_id);
//             }
//         }

//         // Optimize graph
//         optimizer::optimize_essential_graph(
//             &self.map, self.matched_kf, self.current_kf_id,
//                 loop_connections, &non_corrected_sim3, &corrected_sim3, 
//                 !self.sensor.is_mono()
//         );

//         // Add loop edge
//         self.map.write().add_kf_loop_edges(self.matched_kf, self.current_kf_id);

//         // Launch a new thread to perform Global Bundle Adjustment
//         let mut map_copy = self.map.clone();
//         let current_kf_id = self.current_kf_id;
//         thread::spawn(move || {
//             run_gba(&mut map_copy, current_kf_id);
//         });

//         LOCAL_MAPPING_PAUSE_SWITCH.store(false, Ordering::SeqCst);

//         self.last_loop_kf_id = self.current_kf_id;

//         Ok(())
//     }

//     fn search_and_fuse(&mut self, corrected_poses_map: &KeyFrameAndPose) -> Result<(), Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("search_and_fuse");

//         for (kf_id, g2o_scw) in corrected_poses_map {
//             let replace_points = orbmatcher::fuse_from_loop_closing(
//                 &kf_id, &g2o_scw, &self.loop_mappoints, &self.map, 3, 0.8
//             )?;

//             for (mp_to_replace, mp_id) in replace_points {
//                 self.map.write().replace_mappoint(mp_to_replace, mp_id);
//             }
//         }
//         Ok(())
//     }


// }


// fn run_gba(map: &mut ReadWriteMap, loop_kf: Id) {
//     // void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
//     let _span = tracy_client::span!("run_gba_in_thread");
//     info!("Starting Global Bundle Adjustment");

//     // TODO (design, concurrency) mbStopGBA called from within loop closing
//     if GBA_KILL_SWITCH.load(Ordering::SeqCst) {
//         return;
//     }

//     GBA_IDLE.store(false, Ordering::SeqCst);

//     optimizer::global_bundle_adjustment(map, 10, false, loop_kf);

//     // Update all MapPoints and KeyFrames
//     // Local Mapping was active during BA, that means that there might be new keyframes
//     // not included in the Global BA and they are not consistent with the updated map.
//     // We need to propagate the correction through the spanning tree

//     debug!("Global bundle adjustment finished. Updating map...");

//     if GBA_KILL_SWITCH.load(Ordering::SeqCst) {
//         return;
//     }

//     LOCAL_MAPPING_PAUSE_SWITCH.store(true, Ordering::SeqCst);
//     // Wait until Local Mapping has effectively stopped
//     while !LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) {
//         thread::sleep(Duration::from_millis(1000));
//     }

//     {
//         let mut lock = map.write();
//         // Correct keyframes starting at map first keyframe
//         let mut kfs_to_check = vec![lock.initial_kf_id];
//         let mut i = 0;
//         let mut tcw_bef_gba: HashMap<Id, Pose> = HashMap::new();
//         while i < kfs_to_check.len() {
//             let curr_kf_id = kfs_to_check[i];
//             let children = lock.keyframes.get(& curr_kf_id).unwrap().children.clone();

//             for child_id in & children {
//                 let kfs = lock.keyframes.get_many_mut([&curr_kf_id, child_id]).unwrap();
//                 if !kfs[0].ba_global_for_kf != loop_kf {
//                     let tchildc = kfs[0].pose * kfs[1].pose;
//                     kfs[1].gba_pose = Some(tchildc * kfs[0].gba_pose.unwrap());
//                     kfs[1].ba_global_for_kf = loop_kf;
//                     // debug!("Set gba pose: {} {:?}", kfs[1].id, kfs[1].gba_pose);
//                 }
//                 kfs_to_check.push(*child_id);
//             }

//             let kf = lock.keyframes.get_mut(&curr_kf_id).unwrap();
//             tcw_bef_gba.insert(curr_kf_id, kf.pose);
//             kf.pose = kf.gba_pose.unwrap().clone();
//             // debug!("Set gba pose: {} {:?}", kf.id, kf.gba_pose);

//             i += 1;
//         }

//         // Correct MapPoints
//         let mps_to_update = {
//             let mut mps_to_update = HashMap::new();
//             for (id, mp) in &lock.mappoints {
//                 if mp.ba_global_for_kf == loop_kf {
//                     // If optimized by Global BA, just update
//                     mps_to_update.insert(*id, mp.gba_pose.unwrap());
//                 } else {
//                     // Update according to the correction of its reference keyframe
//                     let kf = lock.keyframes.get(&mp.ref_kf_id).unwrap();

//                     if kf.ba_global_for_kf != loop_kf {
//                         continue;
//                     }

//                     // Map to non-corrected camera
//                     let rcw = tcw_bef_gba.get(&mp.ref_kf_id).unwrap().get_rotation();
//                     let tcw = tcw_bef_gba.get(&mp.ref_kf_id).unwrap().get_translation();
//                     let xc = *rcw * *mp.position + *tcw;

//                     // Backproject using corrected camera
//                     let twc = kf.pose.inverse();
//                     let rwc = twc.get_rotation();
//                     let twc = twc.get_translation();

//                     mps_to_update.insert(*id, DVTranslation::new(*rwc * xc + *twc));
//                 }
//             }
//             mps_to_update
//         };


//         for (mp_id, gba_pose) in mps_to_update {
//             lock.mappoints.get_mut(&mp_id).unwrap().position = gba_pose;
//         }
//     }

//     GBA_IDLE.store(true, Ordering::SeqCst);
//     GBA_KILL_SWITCH.store(false, Ordering::SeqCst);
//     // Loop closed. Release local mapping.
//     println!("Resuming local mapping");
//     LOCAL_MAPPING_PAUSE_SWITCH.store(false, Ordering::SeqCst);

//     info!("Map updated!");
// }