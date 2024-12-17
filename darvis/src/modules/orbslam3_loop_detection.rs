use core::{config::{SETTINGS, SYSTEM}, sensor::{FrameSensor, ImuSensor, Sensor}};
use std::collections::{HashMap, HashSet};

use log::debug;
use opencv::core::KeyPointTraitConst;

use crate::{map::{map::Id, pose::Sim3}, registered_actors::{self, FEATURE_MATCHING_MODULE}, MapLock};

use super::{module_definitions::LoopDetectionModule, optimizer, orbslam_matcher::ORBMatcherTrait, sim3solver::Sim3Solver};

pub struct ORBSLAM3LoopDetection {
    num_coincidences: i32,
    loop_mps: Vec<Id>, // mvpLoopMPs
    current_matched_points: Vec<Option<Id>>, // mvpLoopMatchedMPs
    loop_slw: Sim3, // mg2oLoopSlw
    loop_matched_kf: Id, // mpLoopMatchedKF

    num_not_found: i32, // mnLoopNumNotFound
    last_current_kf: Id, // mpLoopLastCurrentKF

    sensor: Sensor,

}

impl ORBSLAM3LoopDetection {
    pub fn new() -> ORBSLAM3LoopDetection {
        ORBSLAM3LoopDetection {
            current_matched_points: Vec::new(),
            loop_matched_kf: -1,
            sensor: SETTINGS.get(SYSTEM, "sensor"),
            num_coincidences: 0,
            loop_slw: Sim3::identity(),
            last_current_kf: -1,
            num_not_found: 0,
            loop_mps: Vec::new(),

        }
    }

    fn detect_and_reffine_sim3_from_last_kf(&mut self, map: &MapLock, current_kf_id: Id, loop_kf: Id, scw: &mut Sim3) -> Result<Option<(Sim3, Vec<Option<i32>>)>, Box<dyn std::error::Error>> {
        // bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let _span = tracy_client::span!("detect_and_reffine_sim3_from_last_kf");

        let mut matched_mappoints = Vec::new();
        let num_proj_matches = Self::find_matches_by_projection(&map, current_kf_id, scw, loop_kf, &mut self.loop_mps, &mut matched_mappoints)?;

        if num_proj_matches >= 30 {
            let mut scm = {
                let twm = map.read().keyframes.get(&current_kf_id).unwrap().get_pose().inverse();
                let twm_as_sim3 = Sim3::new(twm.get_translation(), twm.get_rotation(), 1.0);
                *scw * twm_as_sim3
            };

            let mut fixed_scale = ! matches!(self.sensor, Sensor(FrameSensor::Mono, ImuSensor::None));
            if matches!(self.sensor, Sensor(FrameSensor::Mono, ImuSensor::Some)) {
                if !map.read().imu_ba2 {
                    fixed_scale = false;
                }
            }
            println!("Fixed scale is {}", fixed_scale);

            let num_opt_matches = optimizer::optimize_sim3(
                &map, current_kf_id, loop_kf, &mut matched_mappoints, &mut scm, 10, fixed_scale
            );

            if num_opt_matches > 50 {
                let num_proj_matches  = Self::find_matches_by_projection(&map, current_kf_id, scw, loop_kf, &mut self.loop_mps, &mut matched_mappoints)?;
                if num_proj_matches >= 100 {
                    return Ok(Some((*scw, matched_mappoints)));
                }
            }
        }
        return Ok(None);
    }

    fn find_matches_by_projection(
        map: & MapLock, current_kf_id: Id, scw: &mut Sim3, loop_kf: Id, mappoints: &mut Vec<Id>, matched_mappoints: &mut Vec<Option<i32>>,
    ) -> Result<i32, Box<dyn std::error::Error>> {
        // int LoopClosing::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw, set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints, vector<MapPoint*> &vpMatchedMapPoints)
        let _span = tracy_client::span!("find_matches_by_projection");

        let num_covisibles = 10;
        let mut cov_kf_matched = map.read().keyframes.get(&loop_kf).unwrap().get_covisibility_keyframes(num_covisibles); // vpCovKFm
        let initial_cov = cov_kf_matched.len();
        cov_kf_matched.push(loop_kf);

        let mut check_kfs: HashSet<i32> = HashSet::from_iter(cov_kf_matched.iter().cloned()); // spCheckKFs
        let current_covisibles = map.read().keyframes.get(&current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX); // spCurrentCovisbles

        if (initial_cov as i32) < num_covisibles {
            for i in 0..initial_cov {
                let kfs = map.read().keyframes.get(&cov_kf_matched[i]).unwrap().get_covisibility_keyframes(num_covisibles);
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
        mappoints.clear(); // vpMapPoints
        matched_mappoints.clear(); // vpMatchedMapPoints
        matched_mappoints.resize_with(map.read().keyframes.get(&current_kf_id).unwrap().get_mp_matches().len(), || None);

        for keyframe in cov_kf_matched {
            let read = map.read();
            let kf = read.keyframes.get(&keyframe).unwrap();
            let mp_matches = kf.get_mp_matches();
            for index in 0..mp_matches.len() {
                if let Some((mp, _)) = mp_matches.get(index).unwrap() {
                    if !sp_map_points.contains(mp) {
                        sp_map_points.insert(*mp);
                        mappoints.push(*mp);
                    }
                }
            }
        }
        let num_matches = FEATURE_MATCHING_MODULE.search_by_projection_for_loop_detection2(
            &map, &current_kf_id, &scw, &mappoints,  matched_mappoints, 3, 1.5
        )?;

        return Ok(num_matches);
    }

    fn detect_common_regions_from_bow(&mut self, map: &MapLock, current_kf_id: Id, bow_cands: &Vec<Id>) -> Result<(Option<Id>, Option<Sim3>), Box<dyn std::error::Error>> {
        // bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF2, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw, int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let _span = tracy_client::span!("detect_common_regions_from_bow");

        let bow_matches_threshold = 20; // nBoWMatches
        let bow_inliers_threshold = 15; // nBoWInliers
        let sim3_inliers_threshold = 20; // nSim3Inliers
        let proj_matches_threshold = 50; // nProjMatches
        let num_proj_opt_matches_threshold = 80; // nProjOptMatches

        let connected_keyframes = map.read().keyframes.get(&current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX); // spConnectedKeyFrames
        let num_covisibles = 10; // nNumCovisibles

        // Varibles to select the best number
        let mut best_matched_kf = -1; // pBestMatchedKF
        let mut num_best_matches_reproj = 0; // nBestMatchesReproj
        let mut best_num_coincidences = 0; // nBestNumCoincidences
        let mut best_scw = Sim3::identity(); // g2oBestScw
        let mut best_mappoints: Vec<Id> = Vec::new(); // vpBestMapPoints
        let mut best_matched_mappoints: Vec<Option<Id>> = Vec::new(); // vpBestMatchedMapPoints

        let num_candidates = bow_cands.len(); // nNumCandidates
        let mut vn_stage = vec![0; num_candidates]; // vnStage
        let mut vn_matches_stage = vec![0; num_candidates]; // vnMatchesStage

        let num_curr_kf_matches = map.read().keyframes.get(&current_kf_id).unwrap().get_mp_matches().len(); 

        let mut index = 0;
        for kf_i_id in bow_cands {
            if map.read().keyframes.get(&kf_i_id).is_none() {
                continue;
            }

            // Current KF against KF with covisibles version
            let mut cov_kf = map.read().keyframes.get(&kf_i_id).unwrap().get_covisibility_keyframes(num_covisibles); // vpCovKFi
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
                let read = map.read();
                let curr_kf = read.keyframes.get(&current_kf_id).unwrap();
                for j in 0..cov_kf.len() {
                    let matches = FEATURE_MATCHING_MODULE.search_by_bow_with_keyframe(
                        curr_kf,
                        read.keyframes.get(&cov_kf[j]).unwrap(),
                        true,
                        0.9
                    )?;
                    // println!("...Matches between {} (frame {}) and {} (frame {}): {}", current_kf_id, read.keyframes.get(&current_kf_id).unwrap().frame_id, cov_kf[j], read.keyframes.get(&cov_kf[j]).unwrap().frame_id, matches.len());
                    if matches.len() > most_bow_num_matches {
                        most_bow_num_matches = matches.len();
                    }
                    vvp_matched_mps.push(matches);
                }
            }

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
                // Geometric validation
                let mut fixed_scale = ! matches!(self.sensor, Sensor(FrameSensor::Mono, ImuSensor::None));
                if matches!(self.sensor, Sensor(FrameSensor::Mono, ImuSensor::Some)) {
                    if !map.read().imu_ba2 {
                        fixed_scale = false;
                    }
                }
                println!("Fixed scale is {}", fixed_scale);

                let mut solver = Sim3Solver::new(
                    &map,
                    current_kf_id,
                    *most_bow_matches_kf,
                    &vp_matched_mappoints,
                    fixed_scale,
                    keyframe_matched_mp
                )?;
                solver.set_ransac_parameters(0.99, bow_inliers_threshold, 300);

                let mut no_more = false;
                let mut sim3_result = None;
                while sim3_result.is_none() && !no_more {
                    (no_more, sim3_result) = solver.iterate(20)?;
                }

                match sim3_result {
                    Some((_inliers, _num_inliers)) => {
                        // Match by reprojection
                        cov_kf.clear();
                        cov_kf = map.read().keyframes.get(&most_bow_matches_kf).unwrap().get_covisibility_keyframes(num_covisibles);
                        cov_kf.push(*most_bow_matches_kf);

                        let mut candidates = Vec::new(); // vpMapPoints
                        let mut sp_map_points = HashSet::new(); // spMapPoints
                        let mut kfs_for_candidates = Vec::new(); // vpKeyFrames
                        for kf_id in cov_kf {
                            let read = map.read();
                            let mp_matches = read.keyframes.get(&kf_id).unwrap().get_mp_matches();
                            for index in 0..mp_matches.len() {
                                if let Some((mp_id, _)) = mp_matches[index] {
                                    if !sp_map_points.contains(&mp_id) {
                                        sp_map_points.insert(mp_id);
                                        candidates.push(mp_id);
                                        kfs_for_candidates.push(kf_id);
                                    }
                                }
                            }
                        }

                        let mut scm = solver.get_estimates(); // gScm
                        let smw = map.read().keyframes.get(&most_bow_matches_kf).unwrap().get_pose().into(); // gSmw
                        let scw = scm * smw; // gScw // Similarity matrix of current from the world position

                        let mut matched_mps = vec![None; num_curr_kf_matches]; // vpMatchedMP
                        let (num_proj_matches, _matched_kfs) = FEATURE_MATCHING_MODULE.search_by_projection_for_loop_detection1(
                            &map, &current_kf_id, &scw, 
                            &candidates, &kfs_for_candidates, &mut matched_mps,
                            8, 1.5
                        )?;

                        if num_proj_matches >= proj_matches_threshold {
                            // Optimize Sim3 transformation with every matches

                            let mut fixed_scale = ! matches!(self.sensor, Sensor(FrameSensor::Mono, ImuSensor::None));
                            if matches!(self.sensor, Sensor(FrameSensor::Mono, ImuSensor::Some)) {
                                if !map.read().imu_ba2 {
                                    fixed_scale = false;
                                }
                            }
                            println!("Fixed scale is {}", fixed_scale);


                            let num_opt_matches = optimizer::optimize_sim3(
                                &map, current_kf_id, *kf_i_id, &mut matched_mps, &mut scm, 10, fixed_scale
                            );

                            if num_opt_matches >= sim3_inliers_threshold {
                                let smw = map.read().keyframes.get(&most_bow_matches_kf).unwrap().get_pose().into(); // gSmw
                                let scw = scm * smw; // gScw // Similarity matrix of current from the world position

                                let mut matched_mps = vec![None; num_curr_kf_matches]; // vpMatchedMP
                                let num_proj_opt_matches = FEATURE_MATCHING_MODULE.search_by_projection_for_loop_detection2(
                                    &map, &current_kf_id, &scw, 
                                    &candidates, &mut matched_mps,
                                    5, 1.0
                                )?;

                                if num_proj_opt_matches >= num_proj_opt_matches_threshold {
                                    let mut max_x = -1.0;
                                    let mut min_x = 1000000.0;
                                    let mut max_y = -1.0;
                                    let mut min_y = 1000000.0;
                                    let current_cov_kfs = {
                                        let read = map.read();
                                        let kf_i = read.keyframes.get(&kf_i_id).unwrap();
                                        for matched_mp in &matched_mps {
                                            if let Some(mp_i_id) = matched_mp {
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
                                        }

                                        read.keyframes.get(&current_kf_id).unwrap().get_covisibility_keyframes(num_covisibles)
                                    };

                                    let mut num_kfs = 0; // nNumKFs

                                    let mut j = 0;
                                    let current_kf_pose = map.read().keyframes.get(&current_kf_id).unwrap().get_pose();
                                    while num_kfs < 3 && j < current_cov_kfs.len() {
                                        let mut sjw = {
                                            let kf_j_pose = map.read().keyframes.get(&current_cov_kfs[j]).unwrap().get_pose();
                                            let sjc = Sim3 {
                                                pose: kf_j_pose * current_kf_pose.inverse(),
                                                scale: 1.0
                                            };

                                            sjc * scw
                                        }; // gSjw

                                        let (valid, _num_proj_matches_j) = self.detect_common_regions_from_last_kf(map, current_kf_id, &mut sjw, *most_bow_matches_kf, &mut candidates)?;

                                        if valid {
                                            num_kfs += 1;
                                        }
                                        j += 1;
                                    }

                                    if num_kfs < 3 {
                                        vn_stage[index] = 8;
                                        vn_matches_stage[index] = num_kfs;
                                    }

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
            map.write().keyframes.get_mut(&best_matched_kf).unwrap().dont_delete = true; // pMatchedKF2->SetNotErase();
            self.loop_mps = best_mappoints;  // vpMPs = vpBestMapPoints;
            self.current_matched_points = best_matched_mappoints; // vpMatchedMPs = vpBestMatchedMapPoints;
            self.loop_matched_kf = best_matched_kf; // pMatchedKF2 = pBestMatchedKF;
            self.loop_slw = best_scw; // g2oScw = g2oBestScw;

            if self.num_coincidences >= 3 {
                return Ok((Some(best_matched_kf), Some(best_scw)))
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

    fn detect_common_regions_from_last_kf(&mut self, map: &MapLock, current_kf_id: Id, scw: &mut Sim3, loop_kf: Id, mappoints: &mut Vec<Id>) -> Result<(bool, i32), Box<dyn std::error::Error>>{
        // bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let _span = tracy_client::span!("detect_common_regions_from_last_kf");

        let mut matched_mappoints = Vec::new();
        let num_proj_matches = Self::find_matches_by_projection(&map, current_kf_id, scw, loop_kf, mappoints, &mut matched_mappoints)?;
        if num_proj_matches >= 30 {
            return Ok((true, num_proj_matches));
        }
        return Ok((false, num_proj_matches));
    }

}

impl LoopDetectionModule for ORBSLAM3LoopDetection {
    fn detect_loop(&mut self, map: &MapLock, current_kf_id: Id) -> Result<(Option<Id>, Option<Id>, Option<Sim3>, Vec<Id>, Vec<Option<Id>>), Box<dyn std::error::Error>>{
        // bool LoopClosing::NewDetectCommonRegions
        let _span = tracy_client::span!("detect_common_regions");

        let mut loop_kf = None;
        let mut scw = None;
        let merge_kf = None;

        //If the map contains less than 12 KF
        if map.read().keyframes.len() < 12 {
            map.write().add_to_kf_database(current_kf_id);
            return Ok((None, None, None, vec![], vec![]));
        }

        // Check the last candidates with geometric validation
        // Loop candidates
        let mut loop_detected_in_kf = false; // bLoopDetectedInKF
        let mut loop_detected = false;
        if self.num_coincidences > 0 {
            // Find from the last KF candidates
            let mut scw = {
                let tcl = map.read().keyframes.get(&current_kf_id).unwrap().get_pose() * map.read().keyframes.get(&self.last_current_kf).unwrap().get_pose().inverse();
                let tcl_as_sim3: Sim3 = tcl.into();
                tcl_as_sim3 * self.loop_slw
            };

            match self.detect_and_reffine_sim3_from_last_kf(map, current_kf_id, self.loop_matched_kf, &mut scw)? {
                Some((scw, current_matched_points)) => {
                    loop_detected_in_kf = true;

                    self.num_coincidences += 1;
                    map.write().keyframes.get_mut(&self.last_current_kf).unwrap().dont_delete = false;
                    self.last_current_kf = current_kf_id;
                    self.loop_slw = scw;
                    self.current_matched_points = current_matched_points;

                    loop_detected = self.num_coincidences >= 3;
                    self.num_not_found = 0;
                },
                None => {
                    loop_detected_in_kf = false;
                    self.num_not_found += 1;
                    if self.num_not_found >= 2 {
                        map.write().keyframes.get_mut(&self.last_current_kf).unwrap().dont_delete = false;
                        // map.write().keyframes.get_mut(&self.matched_kf).unwrap().dont_delete = false;
                        self.num_coincidences = 0;
                        self.current_matched_points.clear();
                        self.loop_mps.clear();
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
            map.write().add_to_kf_database(current_kf_id);
            // Reset all variables
            let loop_mps_save = self.loop_mps.clone();
            let current_matched_points_save = self.current_matched_points.clone();
            self.num_coincidences = 0;
            self.current_matched_points.clear(); // mvpLoopMatchedMPs
            self.loop_mps.clear(); // mvpLoopMPs
            self.num_not_found = 0; // mnLoopNumNotFound
            return Ok((merge_kf, loop_kf, scw, loop_mps_save, current_matched_points_save));
        }

        // Extract candidates from the bag of words
        let (merge_bow_cand, loop_bow_cand) = match !merge_detected || !loop_detected_in_kf {
            // Search in BoW
            true => map.read().detect_top_n_loop_candidates(current_kf_id, 3),            false => (Vec::new(), Vec::new())
        };

        // Check the BoW candidates if the geometric candidate list is empty
        // Loop candidates
        (loop_kf, scw) = match !loop_detected_in_kf && !loop_bow_cand.is_empty() {
            true => self.detect_common_regions_from_bow(map, current_kf_id,  &loop_bow_cand,)?,
            false => (None, None)
        };

        // Merge candidates
        if !merge_detected && !merge_bow_cand.is_empty() {
            todo!("multimaps");
        }

        map.write().add_to_kf_database(current_kf_id);

        if merge_detected || loop_kf.is_some() {
            // Reset all variables
            let loop_mps_save = self.loop_mps.clone();
            let current_matched_points_save = self.current_matched_points.clone();
            self.num_coincidences = 0;
            self.current_matched_points.clear(); // mvpLoopMatchedMPs
            self.loop_mps.clear(); // mvpLoopMPs
            self.num_not_found = 0; // mnLoopNumNotFound
            return Ok((merge_kf, loop_kf, scw, loop_mps_save, current_matched_points_save));
        }

        map.write().keyframes.get_mut(&current_kf_id).unwrap().dont_delete = false; 

        return Ok((None, None, None, vec![], vec![]));
    }

}