use core::system::Actor;
use core::config::{SETTINGS, SYSTEM};
use core::sensor::{FrameSensor, ImuSensor, Sensor};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;
use log::{debug, info, warn};
use opencv::core::KeyPointTraitConst;
use crate::actors::local_mapping::LOCAL_MAPPING_IDLE;
use crate::actors::messages::KeyFrameIdMsg;
use crate::map::pose::{DVTranslation, Pose, Sim3};
use crate::modules::sim3solver::Sim3Solver;
use crate::modules::{optimizer, orbmatcher};
use crate::{System, MapLock};
use crate::map::map::Id;
use crate::modules::imu::ImuModule;

use super::local_mapping::LOCAL_MAPPING_PAUSE_SWITCH;
use super::messages::{IMUInitializedMsg, ShutdownMsg};

pub type KeyFrameAndPose = HashMap<Id, Sim3>;

pub static GBA_KILL_SWITCH: AtomicBool = AtomicBool::new(false); // mbStopGBA
static GBA_IDLE: AtomicBool = AtomicBool::new(false); // mbRunningGBA

pub struct LoopClosing {
    system: System,
    sensor: Sensor,

    map: MapLock,
    _imu: Option<ImuModule>,

    // Loop detector variables
    loop_mappoints: HashMap<usize, Id>, // mvpLoopMapPoints
    loop_mps: HashMap<usize, Id>, // mvpLoopMPs
    current_matched_points: HashMap<usize, Id>, // mvpLoopMatchedMPs
    loop_slw: Sim3, // mg2oLoopSlw
    loop_matched_kf: Id, // mpLoopMatchedKF

    // removing as member variables
    // current_kf_id: Id, // mpCurrentKF
    // scw: Sim3, // mScw, mg2oScw

    // Information about prior keyframes
    num_coincidences: i32, // mnLoopNumCoincidences
    num_not_found: i32, // mnLoopNumNotFound
    last_current_kf: Id, // mpLoopLastCurrentKF
    last_corrected_loop_kf_id: Id, // mLastLoopKFid

    // Global BA
    full_ba_idx: i32, // mnFullBAIdx
}

impl Actor for LoopClosing {
    type MapRef = MapLock;

    fn new_actorstate(system: System, map: Self::MapRef) -> LoopClosing {

        LoopClosing {
            system,
            map,
            _imu: None, // TODO (IMU): ImuModule::new(None, None, sensor, false, false),
            sensor: SETTINGS.get(SYSTEM, "sensor"),
            full_ba_idx: 0,
            last_corrected_loop_kf_id: -1,
            current_matched_points: HashMap::new(),
            loop_mappoints: HashMap::new(),
            loop_matched_kf: -1,
            // scw: Sim3::identity(),
            num_coincidences: 0,
            loop_slw: Sim3::identity(),
            last_current_kf: -1,
            num_not_found: 0,
            loop_mps: HashMap::new(),
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
                }

                match actor.loop_closing(msg.kf_id) {
                    Ok(_) => {},
                    Err(e) => {
                        warn!("Loop closing failed: {}", e);
                    }
                }
                println!();
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
        match self.detect_common_regions(current_kf_id) {
            Ok((merge_kf, loop_kf, scw)) => {
                if merge_kf.is_some() {
                    info!("KF {}: Merge detected!", current_kf_id);
                }

                match (loop_kf, scw) {
                    (Some(loop_kf), Some(scw)) => {
                        info!("KF {}: Loop detected!", current_kf_id);

                        match self.sensor.is_imu() {
                            true => {
                                todo!("IMU");
                                // Lines 235-258
                            },
                            false => {}
                        };

                        self.loop_mappoints = self.loop_mps.clone(); // mvpLoopMapPoints = mvpLoopMPs;

                        println!("Doing correct loop with self.slw: rot {:?} trans {:?} scale {:?}", self.loop_slw.pose.get_rotation(), self.loop_slw.pose.get_translation(), self.loop_slw.scale);
                        match self.correct_loop(current_kf_id, loop_kf, self.loop_slw) {
                            Ok(_) => {},
                            Err(e) => {
                                warn!("Loop correction failed: {}", e);
                            }
                        }

                        // Reset all variables
                        self.map.write().keyframes.get_mut(&loop_kf).unwrap().dont_delete = false;
                        self.num_coincidences = 0;
                        self.loop_mps.clear();
                        self.current_matched_points.clear(); // mvpLoopMatchedMPs
                        self.loop_mps.clear(); // mvpLoopMPs
                        self.num_not_found = 0; // mnLoopNumNotFound
                    },
                    _ => ()
                }
            },
            Err(e) => {
                warn!("Loop detection failed: {}", e);
            }
        }

        self.last_current_kf = current_kf_id;
        self.map.write().keyframes.get_mut(&current_kf_id).unwrap().dont_delete = false;
        thread::sleep(Duration::from_micros(5000));
        Ok(())
    }

    fn detect_common_regions(&mut self, current_kf_id: Id) -> Result<(Option<Id>, Option<Id>, Option<Sim3>), Box<dyn std::error::Error>>{
        // bool LoopClosing::NewDetectCommonRegions
        let _span = tracy_client::span!("detect_common_regions");

        let mut loop_kf = None;
        let mut scw = None;
        let merge_kf = None;

        //If the map contains less than 12 KF
        if self.map.read().keyframes.len() < 12 {
            return Ok((merge_kf, loop_kf, scw));
        }

        // Check the last candidates with geometric validation
        // Loop candidates
        let mut loop_detected_in_kf = false; // bLoopDetectedInKF
        let mut loop_detected = false;
        if self.num_coincidences > 0 {
            debug!("Loop detection step 1: num coincidences > 0...");
            // Find from the last KF candidates
            let mut scw = {
                let tcl = self.map.read().keyframes.get(&current_kf_id).unwrap().pose * self.map.read().keyframes.get(&self.last_current_kf).unwrap().pose.inverse();
                let tcl_as_sim3 = Sim3::new(tcl.get_translation(), tcl.get_rotation(), 1.0);
                tcl_as_sim3 * self.loop_slw
            };
            println!("scw when loop num coincidences > 0: rot {:?} trans {:?} scale {:?}", scw.pose.get_rotation(), scw.pose.get_translation(), scw.scale);
            println!("current kf id: {}, last current kf: {}", current_kf_id, self.last_current_kf);

            match self.detect_and_reffine_sim3_from_last_kf(current_kf_id, self.loop_matched_kf, &mut scw)? {
                Some(scw) => {
                    loop_detected_in_kf = true;

                    self.num_coincidences += 1;
                    self.last_current_kf = current_kf_id;
                    self.loop_slw = scw;
                    self.map.write().keyframes.get_mut(&self.last_current_kf).unwrap().dont_delete = false;

                    loop_detected = self.num_coincidences >= 3;
                    self.num_not_found = 0;

                    debug!("...loop detected from detect_and_reffine_sim3_from_last_kf!");
                },
                None => {
                    loop_detected_in_kf = false;
                    self.num_not_found += 1;
                    if self.num_not_found >= 2 {
                        self.map.write().keyframes.get_mut(&self.last_current_kf).unwrap().dont_delete = false;
                        // self.map.write().keyframes.get_mut(&self.matched_kf).unwrap().dont_delete = false;
                        self.num_coincidences = 0;
                        self.current_matched_points.clear();
                        self.loop_mappoints.clear();
                        self.num_not_found = 0;
                    }
                }
            }
        }

        let merge_detected = false;
        {
            // TODO (multimaps)
            // For reference code look at " if(mnMergeNumCoincidences > 0) " in LoopClosing::NewDetectCommonRegions
        }

        if merge_detected || loop_detected {
            self.map.write().add_to_kf_database(current_kf_id);
            return Ok((merge_kf, loop_kf, scw));
        }

        // Extract candidates from the bag of words
        let (merge_bow_cand, loop_bow_cand) = match !merge_detected || !loop_detected_in_kf {
            true => {
                // Search in BoW
                self.map.read().detect_top_n_loop_candidates(current_kf_id, 3)
            },
            false => (Vec::new(), Vec::new())
        };

        // Check the BoW candidates if the geometric candidate list is empty
        // Loop candidates
        debug!("Loop detection step 2: detecting using bow");
        (loop_kf, scw) = match !loop_detected_in_kf && !loop_bow_cand.is_empty() {
            true => {
                    self.detect_common_regions_from_bow(
                        current_kf_id, 
                    &loop_bow_cand,
                )?
            },
            false => (None, None)
        };

        // Merge candidates
        if !merge_detected && !merge_bow_cand.is_empty() {
            todo!("multimaps");
        }

        self.map.write().add_to_kf_database(current_kf_id);

        if merge_detected || loop_kf.is_some() {
            return Ok((merge_kf, loop_kf, scw));
        }

        self.map.write().keyframes.get_mut(&current_kf_id).unwrap().dont_delete = false; 

        return Ok((None, None, None));
    }

    fn detect_and_reffine_sim3_from_last_kf(&mut self, current_kf_id: Id, loop_kf: Id, scw: &mut Sim3) -> Result<Option<Sim3>, Box<dyn std::error::Error>> {
        // bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let _span = tracy_client::span!("detect_and_reffine_sim3_from_last_kf");

        println!("Last matched kf: {:?}", loop_kf);
        let num_proj_matches = self.find_matches_by_projection(current_kf_id, scw, loop_kf)?;

        if num_proj_matches >= 30 {
            let mut scm = {
                let twm = self.map.read().keyframes.get(&current_kf_id).unwrap().pose.inverse();
                let twm_as_sim3 = Sim3::new(twm.get_translation(), twm.get_rotation(), 1.0);
                *scw * twm_as_sim3
            };

            let fixed_scale = match self.sensor {
                Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                    todo!("IMU");
                    // if !pCurrentKF->GetMap()->GetIniertialBA2()) { return false }
                },
                _ => false
            };
            let num_opt_matches = optimizer::optimize_sim3(
                &self.map, current_kf_id, loop_kf, &mut self.current_matched_points, &mut scm, 10, fixed_scale
            );

            if num_opt_matches > 50 {
                let num_proj_matches  = self.find_matches_by_projection(current_kf_id, scw, loop_kf)?;
                if num_proj_matches >= 100 {
                    return Ok(Some(*scw)); // todo is this right?
                }
            }
        }
        return Ok(None);
    }

     fn find_matches_by_projection(&mut self, current_kf_id: Id, scw: &mut Sim3, loop_kf: Id) -> Result<i32, Box<dyn std::error::Error>> {
        // int LoopClosing::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw, set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints, vector<MapPoint*> &vpMatchedMapPoints)
        let _span = tracy_client::span!("find_matches_by_projection");

        let num_covisibles = 10;
        let mut cov_kf_matched = self.map.read().keyframes.get(&loop_kf).unwrap().get_covisibility_keyframes(num_covisibles); // vpCovKFm
        let initial_cov = cov_kf_matched.len();
        cov_kf_matched.push(loop_kf);

        let mut check_kfs: HashSet<i32> = HashSet::from_iter(cov_kf_matched.iter().cloned()); // spCheckKFs
        let current_covisibles = self.map.read().keyframes.get(&current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX); // spCurrentCovisbles

        if (initial_cov as i32) < num_covisibles {
            for i in 0..initial_cov {
                let kfs = self.map.read().keyframes.get(&cov_kf_matched[i]).unwrap().get_covisibility_keyframes(num_covisibles);
                let mut num_inserted = 0;
                let mut j = 0;
                while j < kfs.len() && num_inserted < num_covisibles {
                    if !check_kfs.contains(&kfs[j]) && !current_covisibles.contains(&kfs[j]) {
                        check_kfs.insert(kfs[j]);
                        num_inserted += 1;
                    }
                    j += 1;
                }
                cov_kf_matched.extend(kfs);
            }
        }

        let mut sp_map_points = HashSet::new();
        self.loop_mappoints.clear(); // vpMapPoints
        self.current_matched_points.clear(); // vpMatchedMapPoints

        for keyframe in cov_kf_matched {
            let read = self.map.read();
            let kf = read.keyframes.get(&keyframe).unwrap();
            let mp_matches = kf.get_mp_matches();
            for index in 0..mp_matches.len() {
                if let Some((mp, _)) = mp_matches.get(index).unwrap() {
                    if !sp_map_points.contains(mp) {
                        sp_map_points.insert(*mp);
                        self.loop_mappoints.insert(index, *mp);
                    }
                }
            }
        }
        let num_matches = orbmatcher::search_by_projection_for_loop_detection(
            &self.map, &current_kf_id, &scw, &self.loop_mappoints, &mut self.current_matched_points, 3, 1.5
        )?;

        return Ok(num_matches);
    }

    fn detect_common_regions_from_last_kf(&mut self, current_kf_id: Id, scw: &mut Sim3, loop_kf: Id) -> Result<(bool, i32), Box<dyn std::error::Error>>{
        // bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let _span = tracy_client::span!("detect_common_regions_from_last_kf");

        println!("Final stage of loop detection, detect common regions from last kf. current kf id: {}, loop kf id: {}", current_kf_id, loop_kf);
        let num_proj_matches = self.find_matches_by_projection(current_kf_id, scw, loop_kf)?;
        if num_proj_matches >= 30 {
            return Ok((true, num_proj_matches));
        }
        return Ok((false, num_proj_matches));
    }

    fn detect_common_regions_from_bow(&mut self, current_kf_id: Id, bow_cands: &Vec<Id>) -> Result<(Option<Id>, Option<Sim3>), Box<dyn std::error::Error>> {
        // bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF2, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw, int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let _span = tracy_client::span!("detect_common_regions_from_bow");

        let bow_matches_threshold = 20; // nBoWMatches
        let bow_inliers_threshold = 15; // nBoWInliers
        let sim3_inliers_threshold = 20; // nSim3Inliers
        let proj_matches_threshold = 50; // nProjMatches
        let num_proj_opt_matches_threshold = 80; // nProjOptMatches

        let connected_keyframes = self.map.read().keyframes.get(&current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX); // spConnectedKeyFrames
        let num_covisibles = 10; // nNumCovisibles

        // Varibles to select the best number
        let mut best_matched_kf = -1; // pBestMatchedKF
        let mut num_best_matches_reproj = 0; // nBestMatchesReproj
        let mut best_num_coincidences = 0; // nBestNumCoincidences
        let mut best_scw = Sim3::identity(); // g2oBestScw
        let mut best_mappoints: HashMap<usize, Id> = HashMap::new(); // vpBestMapPoints
        let mut best_matched_mappoints: HashMap<usize, Id> = HashMap::new(); // vpBestMatchedMapPoints

        let num_candidates = bow_cands.len(); // nNumCandidates
        let mut vn_stage = vec![0; num_candidates]; // vnStage
        let mut vn_matches_stage = vec![0; num_candidates]; // vnMatchesStage

        debug!("... candidates are: {:?}", bow_cands);
        let mut index = 0;
        for kf_i_id in bow_cands {
            if self.map.read().keyframes.get(&kf_i_id).is_none() {
                continue;
            }

            // Current KF against KF with covisibles version
            let mut cov_kf = self.map.read().keyframes.get(&kf_i_id).unwrap().get_covisibility_keyframes(num_covisibles); // vpCovKFi
            if cov_kf.len() == 0 {
                cov_kf.push(*kf_i_id);
            } else {
                cov_kf.push(cov_kf[0]);
                cov_kf[0] = *kf_i_id;
            }

            let mut abort_by_near_kf = false;
            for i in 0..cov_kf.len() {
                let kf = cov_kf[i];
                if connected_keyframes.contains(&kf) {
                    abort_by_near_kf = true;
                    break;
                }
            }
            if abort_by_near_kf {
                debug!("Check BoW aborted because is close to the matched one");  
                continue;
            }

            let mut vvp_matched_mps = vec![]; // vvpMatchedMPs
            let mut sp_matched_mpi = HashSet::new(); // spMatchedMPi
            let mut num_bow_matches = 0; // numBoWMatches
            let mut vp_matched_mappoints = HashMap::new(); // vpMatchedPoints
            let mut keyframe_matched_mp = HashMap::new(); // vpKeyFrameMatchedMP
    
            let most_bow_matches_kf = kf_i_id; // pMostBoWMatchesKF
            let mut most_bow_num_matches = 0; // nMostBoWNumMatches

            {
                let read = self.map.read();
                let curr_kf = read.keyframes.get(&current_kf_id).unwrap();
                for j in 0..cov_kf.len() {
                    let matches = orbmatcher::search_by_bow_kf(
                        curr_kf,
                        read.keyframes.get(&cov_kf[j]).unwrap(),
                        true,
                        0.9
                    )?;
                    debug!("...Matches between {} (frame {}) and {} (frame {}): {}", current_kf_id, read.keyframes.get(&current_kf_id).unwrap().frame_id, cov_kf[j], read.keyframes.get(&cov_kf[j]).unwrap().frame_id, matches.len());
                    if matches.len() > most_bow_num_matches {
                        most_bow_num_matches = matches.len();
                    }
                    vvp_matched_mps.push(matches);
                }
            }

            println!("vvp_matched_mps: {:?}", vvp_matched_mps.len());
            println!("cov_kf: {:?}", cov_kf.len());

            for j in 0..cov_kf.len() {
                for (index, mp_id) in &vvp_matched_mps[j] {
                    if !sp_matched_mpi.contains(&mp_id) {
                        sp_matched_mpi.insert(mp_id);
                        num_bow_matches += 1;
                        vp_matched_mappoints.insert(*index as usize, *mp_id);
                        keyframe_matched_mp.insert(*index as usize, cov_kf[j]);
                    }
                }
            }

            if num_bow_matches >= bow_matches_threshold {
                debug!("... Num bow matches check passed with {} matches. Creating sim3 solve with kf {}", num_bow_matches, most_bow_matches_kf);

                // Geometric validation
                let fixed_scale = match self.sensor {
                    Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                        todo!("IMU");
                        // return !pCurrentKF->GetMap()->GetIniertialBA2();
                    },
                    Sensor(FrameSensor::Mono, ImuSensor::None) => false,
                    _ => true
                };

                let mut solver = Sim3Solver::new(
                    &self.map,
                    current_kf_id,
                    *most_bow_matches_kf,
                    &vp_matched_mappoints,
                    fixed_scale,
                    keyframe_matched_mp
                )?;

                let mut no_more = false;
                let mut sim3_result = None;
                while sim3_result.is_none() && !no_more {
                    (no_more, sim3_result) = solver.iterate(20)?;
                }

                match sim3_result {
                    Some((_inliers, num_inliers)) => {
                        debug!("...Sim3 solver check passed with {} geometrical inliers among {} BoW matches", num_inliers, bow_inliers_threshold);

                        // Match by reprojection
                        cov_kf.clear();
                        cov_kf = self.map.read().keyframes.get(&most_bow_matches_kf).unwrap().get_covisibility_keyframes(num_covisibles);
                        cov_kf.push(*most_bow_matches_kf);

                        let mut candidates = HashMap::new(); // vpMapPoints
                        let mut sp_map_points = HashSet::new(); // spMapPoints
                        for kf_id in cov_kf {
                            let read = self.map.read();
                            let mp_matches = read.keyframes.get(&kf_id).unwrap().get_mp_matches();
                            for index in 0..mp_matches.len() {
                                if let Some((mp_id, _)) = mp_matches[index] {
                                    if !sp_map_points.contains(&mp_id) {
                                        sp_map_points.insert(mp_id);
                                        candidates.insert(index, mp_id);
                                    }
                                }
                            }
                        }
                        println!("Candidate mappoints: {:?}", candidates);

                        let mut sim3 = solver.get_estimates(); // gScm
                        let smw = Sim3 {
                            pose: self.map.read().keyframes.get(&most_bow_matches_kf).unwrap().pose,
                            scale: 1.0
                        }; // gSmwco
                        let scw = sim3 * smw; // gScw // Similarity matrix of current from the world position
                        println!("sim3: rot {:?} trans {:?} scale {:?}", sim3.pose.get_rotation(), sim3.pose.get_translation(), sim3.scale);
                        println!("smw: rot {:?} trans {:?}", smw.pose.get_rotation(), smw.pose.get_translation());
                        println!("result scw: rot {:?} trans {:?}", scw.pose.get_rotation(), scw.pose.get_translation());

                        let mut matched_mps = HashMap::new(); // vpMatchedMP
                        let num_proj_matches = orbmatcher::search_by_projection_for_loop_detection(
                            &self.map, &current_kf_id, &scw, 
                            &candidates, &mut matched_mps,
                            8, 1.5
                        )?;
                        debug!("... first search by projection ... {} matches", num_proj_matches);

                        if num_proj_matches >= proj_matches_threshold {
                            // Optimize Sim3 transformation with every matches

                            let fixed_scale = match self.sensor {
                                Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                                    todo!("IMU");
                                    // if !pCurrentKF->GetMap()->GetIniertialBA2()) { return false }
                                },
                                _ => false
                            };


                            println!("vpMatchedMp: {:?}", matched_mps);
                            
                            let num_opt_matches = optimizer::optimize_sim3(
                                &self.map, current_kf_id, *kf_i_id, &mut matched_mps, &mut sim3, 10, fixed_scale
                            );

                            debug!("... matches based on sim3 optimization ... {} matches", num_opt_matches);

                            if num_opt_matches >= sim3_inliers_threshold {
                                let smw = Sim3 {
                                    pose: self.map.read().keyframes.get(&most_bow_matches_kf).unwrap().pose,
                                    scale: 1.0
                                }; // gSmw
                                let scw = sim3 * smw; // gScw // Similarity matrix of current from the world position

                                let mut matched_mps = HashMap::new(); // vpMatchedMP
                                let num_proj_opt_matches = orbmatcher::search_by_projection_for_loop_detection(
                                    &self.map, &current_kf_id, &scw, 
                                    &candidates, &mut matched_mps,
                                    5, 1.0
                                )?;
                                debug!("... second search by projection ... {} matches", num_proj_opt_matches);

                                if num_proj_opt_matches >= num_proj_opt_matches_threshold {
                                    let mut max_x = -1.0;
                                    let mut min_x = 1000000.0;
                                    let mut max_y = -1.0;
                                    let mut min_y = 1000000.0;
                                    let current_cov_kfs = {
                                        let read = self.map.read();
                                        let kf_i = read.keyframes.get(&kf_i_id).unwrap();
                                        for (_, mp_i_id) in &matched_mps {
                                            if let Some(mp_i) = read.mappoints.get(&mp_i_id) {
                                                let (index_left, _index_right) = mp_i.get_index_in_keyframe(*kf_i_id);
                                                if index_left >= 0 {
                                                    let kp = kf_i.features.get_keypoint(index_left as usize).0.pt();
                                                    if kp.x < min_x {
                                                        min_x = kp.x;
                                                    }
                                                    if kp.x > max_x {
                                                        max_x = kp.x;
                                                    }
                                                    if kp.y < min_y {
                                                        min_y = kp.y;
                                                    }
                                                    if kp.y > max_y {
                                                        max_y = kp.y;
                                                    }
                                                }
                                            }
                                        }

                                        read.keyframes.get(&current_kf_id).unwrap().get_covisibility_keyframes(num_covisibles)
                                    };

                                    let mut num_kfs = 0; // nNumKFs

                                    let mut j = 0;
                                    let current_kf_pose = self.map.read().keyframes.get(&current_kf_id).unwrap().pose;
                                    println!("Should be moving on here.. current_cov_kfs: {:?}", current_cov_kfs);
                                    while num_kfs < 3 && j < current_cov_kfs.len() {
                                        let mut sjw = {
                                            let kf_j_pose = self.map.read().keyframes.get(&current_cov_kfs[j]).unwrap().pose;
                                            let sjc = Sim3 {
                                                pose: kf_j_pose * current_kf_pose.inverse(),
                                                scale: 1.0
                                            };

                                            sjc * scw
                                        }; // gSjw

                                        println!("gscw: {:?} scale {:?}", scw.pose, scw.scale);
                                        println!("result sjw... rot: {:?}, trans: {:?}, scale {:?}", sjw.pose.get_rotation(), sjw.pose.get_translation(), sjw.scale);

                                        // bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j);
                                        let (valid, num_proj_matches_j) = self.detect_common_regions_from_last_kf(current_kf_id, &mut sjw, *most_bow_matches_kf)?;
                                        debug!("... detect common regions from last kf .. valid? {}, num matches: {}", valid, num_proj_matches_j);


                                        if valid {
                                            // let tc_cj = current_kf.pose * kf_j.pose.inverse();
                                            // let vector_dist = tc_cj.get_translation();
                                            num_kfs += 1;
                                        }
                                        j += 1;
                                    }

                                    if num_kfs < 3 {
                                        vn_stage[index] = 8;
                                        vn_matches_stage[index] = num_kfs;
                                    }

                                    println!("num_best_matches_reproj: {}, num_proj_opt_matches: {}", num_best_matches_reproj, num_proj_opt_matches);
                                    if num_best_matches_reproj < num_proj_opt_matches {
                                        num_best_matches_reproj = num_proj_opt_matches;
                                        best_num_coincidences = num_kfs;
                                        best_matched_kf = *kf_i_id;
                                        best_scw = scw;
                                        best_mappoints = candidates;
                                        best_matched_mappoints = matched_mps;
                                    }
                                }
                            }
                        }
                    },
                    None => ()
                }
            }
            index += 1;
        }

        if num_best_matches_reproj > 0 {
            self.last_current_kf = current_kf_id; // pLastCurrentKF = mpCurrentKF;
            self.num_coincidences = best_num_coincidences; // nNumCoincidences = nBestNumCoindicendes;
            self.map.write().keyframes.get_mut(&best_matched_kf).unwrap().dont_delete = true; // pMatchedKF2->SetNotErase();
            let matched_kf = Some(best_matched_kf);
            self.loop_mps = best_mappoints;  // vpMPs = vpBestMapPoints;
            self.current_matched_points = best_matched_mappoints; // vpMatchedMPs = vpBestMatchedMapPoints;
            self.loop_matched_kf = best_matched_kf; // pMatchedKF2 = pBestMatchedKF;
            self.loop_slw = best_scw; // g2oScw = g2oBestScw;

            debug!("...finally: best kf is {} with {} matches. Num coincidences: {}", best_matched_kf, num_best_matches_reproj, self.num_coincidences);

            if self.num_coincidences >= 3 {
                return Ok((matched_kf, Some(best_scw)))
            } else {
                return Ok((None, Some(best_scw)));
            }
        } else {
            let mut max_stage = -1;
            // let mut max_matched = 0;
            for i in 0..vn_stage.len() {
                if vn_stage[i] > max_stage {
                    max_stage = vn_stage[i];
                    // max_matched = vn_matches_stage[i];
                }
            }
        }

        return Ok((None, None));
    }


    //////////////////////////* everything below here should be ok 
    fn correct_loop(&mut self, current_kf_id: Id, loop_kf: Id, scw: Sim3) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("correct_loop");

        // Note: there's some really gross stuff in here to deal with mutable and immutable references to the map

        // Send a stop signal to Local Mapping
        // Avoid new keyframes are inserted while correcting the loop

        {
            // TODO (design, concurency) local mapping request stop
            LOCAL_MAPPING_PAUSE_SWITCH.store(true, Ordering::SeqCst);
            println!("Pausing local mapping");

            // If a Global Bundle Adjustment is running, abort it
            if !GBA_IDLE.load(Ordering::SeqCst) {
                self.full_ba_idx += 1;
                GBA_KILL_SWITCH.store(true, Ordering::SeqCst);
            }

            // Wait until Local Mapping has effectively stopped
            while !LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) {
                println!("Waiting for local mapping to finish...");
                thread::sleep(Duration::from_millis(1000));
            }
        }

        // Ensure current keyframe is updated
        self.map.write().update_connections(current_kf_id);

        // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        let mut current_connected_kfs = self.map.read().keyframes.get(&current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX);
        current_connected_kfs.push(current_kf_id);

        let mut corrected_sim3 = KeyFrameAndPose::new();
        let mut non_corrected_sim3 = KeyFrameAndPose::new();
        corrected_sim3.insert(current_kf_id, scw.clone());
        let twc = self.map.read().keyframes.get(&current_kf_id).unwrap().pose.inverse();

        for connected_kf_id in &current_connected_kfs {
            let map = self.map.read();
            let connected_kf = map.keyframes.get(connected_kf_id).unwrap();
            let pose = connected_kf.pose;

            if connected_kf_id != &current_kf_id {
                let tic = pose * twc;
                let ric = tic.get_rotation();
                let tic = tic.get_translation();
                let g2o_sic = Sim3 { 
                    pose: Pose::new(*tic, *ric), 
                    scale: 1.0
                };
                let g2o_corrected_siw = g2o_sic * scw;
                //Pose corrected with the Sim3 of the loop closure
                corrected_sim3.insert(*connected_kf_id, g2o_corrected_siw);
            }
            let g2o_siw = Sim3 {
                pose: pose.clone(),
                scale: 1.0
            };
            // Pose without correction
            non_corrected_sim3.insert(*connected_kf_id, g2o_siw);
        }

        // Correct all MapPoints observed by current keyframe and neighbors, so that they align with the other side of the loop
        let mut mp_corrected_by_kf = HashMap::<Id, Id>::new(); // mpCorrectedByKF
        let mut corrected_mp_references = HashMap::<Id, Id>::new(); // mnCorrectedReference in mappoints
        for (kf_id, g2o_corrected_siw) in &corrected_sim3 {
            let g2o_corrected_swi = g2o_corrected_siw.inverse();
            let g2o_siw = non_corrected_sim3.get(kf_id).unwrap();
            let mappoints = {
                let map = self.map.read();
                let connected_kf = map.keyframes.get(kf_id).unwrap();
                connected_kf.get_mp_matches().clone()
            };
            for i in 0..mappoints.len() {
                let mp_id = match mappoints.get(i).unwrap() {
                    Some((id, _)) => id,
                    None => continue
                };
                if mp_corrected_by_kf.contains_key(mp_id) && mp_corrected_by_kf.get(mp_id).unwrap() == kf_id {
                    continue;
                }

                // Project with non-corrected pose and project back with corrected pose
                {
                    let mut map = self.map.write();
                    let mp = map.mappoints.get_mut(mp_id).unwrap();
                    let p3d_w = mp.position;
                    let corrected_p3d_w = g2o_corrected_swi.map(&g2o_siw.map(&p3d_w));
                    mp.position = corrected_p3d_w;
                    mp_corrected_by_kf.insert(*mp_id, *kf_id);
                }

                corrected_mp_references.insert(*mp_id, *kf_id);

                let norm_and_depth = {
                    let map = self.map.read();
                    let mp = map.mappoints.get(mp_id).unwrap();
                    mp.get_norm_and_depth(&map)
                };
                if norm_and_depth.is_some() {
                    self.map.write().mappoints.get_mut(mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                }
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            let new_t = *g2o_corrected_siw.pose.get_translation() * (1.0 / g2o_corrected_siw.scale); //[R t/s;0 1]
            {
                let mut map = self.map.write();
                let keyframe = map.keyframes.get_mut(kf_id).unwrap();
                keyframe.pose = Pose::new(new_t, *g2o_corrected_siw.pose.get_rotation());

                // Make sure connections are updated
                map.update_connections(*kf_id);
            }
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (index, loop_mp_id) in &self.current_matched_points {
            let mut map = self.map.write();
            let curr_kf = map.keyframes.get(&current_kf_id).unwrap();
            if curr_kf.has_mp_match_at_index(&(*index as u32)) {
                let curr_mp_id = curr_kf.get_mp_match(&(*index as u32));
                map.replace_mappoint(curr_mp_id, *loop_mp_id);
            } else {
                map.add_observation(current_kf_id, *loop_mp_id, *index as u32, false);
                let best_descriptor = map.mappoints.get(&loop_mp_id)
                    .and_then(|mp| mp.compute_distinctive_descriptors(&map)).unwrap();
                map.mappoints.get_mut(&loop_mp_id)
                    .map(|mp| mp.update_distinctive_descriptors(best_descriptor));
            }
        }

        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        self.search_and_fuse(&corrected_sim3)?;

        // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        let mut loop_connections: HashMap::<Id, HashSet<Id>> = HashMap::new();
        for kf_id in &current_connected_kfs {
            let mut map = self.map.write();
            let previous_neighbors = {
                let kf = map.keyframes.get_mut(kf_id).unwrap();
                let previous_neighbors = kf.get_covisibility_keyframes(i32::MAX);
                loop_connections.insert(*kf_id, kf.get_connected_keyframes().iter().map(|(id, _)| *id).collect());
                previous_neighbors
            };

            // Update connections. Detect new links.
            map.update_connections(*kf_id);
            for neighbor_id in previous_neighbors {
                loop_connections.get_mut(kf_id).unwrap().remove(&neighbor_id);
            }
            for neighbor_id in &current_connected_kfs {
                loop_connections.get_mut(kf_id).unwrap().remove(neighbor_id);
            }
        }

        // Optimize graph
        optimizer::optimize_essential_graph(
            &self.map, loop_kf, current_kf_id,
                loop_connections, &non_corrected_sim3, &corrected_sim3, 
                !self.sensor.is_mono()
        );

        // Add loop edge
        self.map.write().add_kf_loop_edges(loop_kf, current_kf_id);

        // Launch a new thread to perform Global Bundle Adjustment
        let mut map_copy = self.map.clone();
        let current_kf_id = current_kf_id;
        thread::spawn(move || {
            run_gba(&mut map_copy, current_kf_id);
        });

        LOCAL_MAPPING_PAUSE_SWITCH.store(false, Ordering::SeqCst);

        self.last_corrected_loop_kf_id = current_kf_id;

        Ok(())
    }

    fn search_and_fuse(&mut self, corrected_poses_map: &KeyFrameAndPose) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("search_and_fuse");

        for (kf_id, g2o_scw) in corrected_poses_map {
            let replace_points = orbmatcher::fuse_from_loop_closing(
                &kf_id, &g2o_scw, &self.loop_mappoints, &self.map, 3, 0.8
            )?;

            for (mp_to_replace, mp_id) in replace_points {
                self.map.write().replace_mappoint(mp_to_replace, mp_id);
            }
        }
        Ok(())
    }


}


fn run_gba(map: &mut MapLock, loop_kf: Id) {
    // void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
    let _span = tracy_client::span!("run_gba_in_thread");
    info!("Starting Global Bundle Adjustment");

    // TODO (design, concurrency) mbStopGBA called from within loop closing
    if GBA_KILL_SWITCH.load(Ordering::SeqCst) {
        return;
    }

    GBA_IDLE.store(false, Ordering::SeqCst);

    optimizer::global_bundle_adjustment(map, 10, false, loop_kf);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree

    debug!("Global bundle adjustment finished. Updating map...");

    if GBA_KILL_SWITCH.load(Ordering::SeqCst) {
        return;
    }

    LOCAL_MAPPING_PAUSE_SWITCH.store(true, Ordering::SeqCst);
    // Wait until Local Mapping has effectively stopped
    while !LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) {
        thread::sleep(Duration::from_millis(1000));
    }

    {
        let mut lock = map.write();
        // Correct keyframes starting at map first keyframe
        let mut kfs_to_check = vec![lock.initial_kf_id];
        let mut i = 0;
        let mut tcw_bef_gba: HashMap<Id, Pose> = HashMap::new();
        while i < kfs_to_check.len() {
            let curr_kf_id = kfs_to_check[i];
            let children = lock.keyframes.get(& curr_kf_id).unwrap().children.clone();

            for child_id in & children {
                let kfs = lock.keyframes.get_many_mut([&curr_kf_id, child_id]).unwrap();
                if !kfs[0].ba_global_for_kf != loop_kf {
                    let tchildc = kfs[0].pose * kfs[1].pose;
                    kfs[1].gba_pose = Some(tchildc * kfs[0].gba_pose.unwrap());
                    kfs[1].ba_global_for_kf = loop_kf;
                    // debug!("Set gba pose: {} {:?}", kfs[1].id, kfs[1].gba_pose);
                }
                kfs_to_check.push(*child_id);
            }

            let kf = lock.keyframes.get_mut(&curr_kf_id).unwrap();
            tcw_bef_gba.insert(curr_kf_id, kf.pose);
            kf.pose = kf.gba_pose.unwrap().clone();
            // debug!("Set gba pose: {} {:?}", kf.id, kf.gba_pose);

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
                    let kf = lock.keyframes.get(&mp.ref_kf_id).unwrap();

                    if kf.ba_global_for_kf != loop_kf {
                        continue;
                    }

                    // Map to non-corrected camera
                    let rcw = tcw_bef_gba.get(&mp.ref_kf_id).unwrap().get_rotation();
                    let tcw = tcw_bef_gba.get(&mp.ref_kf_id).unwrap().get_translation();
                    let xc = *rcw * *mp.position + *tcw;

                    // Backproject using corrected camera
                    let twc = kf.pose.inverse();
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

    GBA_IDLE.store(true, Ordering::SeqCst);
    GBA_KILL_SWITCH.store(false, Ordering::SeqCst);
    // Loop closed. Release local mapping.
    println!("Resuming local mapping");
    LOCAL_MAPPING_PAUSE_SWITCH.store(false, Ordering::SeqCst);

    info!("Map updated!");
}